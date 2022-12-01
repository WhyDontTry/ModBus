#include "modbus.h"
#include <stdarg.h>

/** Конфигурирование экземпляров ModBus **/
/*** Параметры ***
** address: Адрес устройства
** frameType: Режим протокола
** sendHandler: Внешний интерфейс для передачи данных, например, привязка к функции отправки через последовательный порт, входящие параметры(uint8_t* buff, size_t buffLen), параметры включают указатель данных и длину данных
***/
void ModBus_setup( ModBus_parameter* ModBus_para, ModBus_Setting_T setting)
{
	ModBus_para->m_address = setting.address;
	ModBus_para->m_modeType = setting.frameType;
	ModBus_para->m_receiveFrameBufferLen = 0;
	ModBus_para->m_sendFramesN = 0;
	ModBus_para->m_nextFrameIndex = 1; // Порядковый номер пакета, начиная с 1

	//ModBus_para->m_receiveBufferTmpLen = 0;
	ModBus_para->m_pBeginReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	ModBus_para->m_hasDetectedBufferStart = 0;

	ModBus_para->m_registerCount = 0;
	if (setting.register_access_limit > 0 && setting.register_access_limit <= MODBUS_REGISTER_LIMIT)
	{
		ModBus_para->m_registerAcessLimit = setting.register_access_limit;
	}
	else
	{
		ModBus_para->m_registerAcessLimit = MODBUS_REGISTER_LIMIT;
	}

	if (setting.baudRate == 0)
	{
		setting.baudRate = MODBUS_DEFAULT_BAUD;
	}
	ModBus_para->m_receiveTimeout = 4000u * 8u / setting.baudRate + 2u;
	ModBus_para->m_sendTimeout = ((ModBus_para->m_registerAcessLimit * 4u + 20u) * 2000u + 7000u) * 8u / setting.baudRate + 5u;

	ModBus_para->m_lastReceivedTime = ModBus_para->m_lastSentTime = millis();

	ModBus_para->m_faston = 0; // Быстрый режим по умолчанию отключен, чтобы гарантировать, что инструкции могут выполняться по порядку во время инициализации

	ModBus_para->m_SendHandler = setting.sendHandler;

#ifdef MODBUS_MASTER // Master

#endif

#ifdef MODBUS_SLAVE // Slave
	ModBus_para->m_GetRegisterHandler = NULL;
	ModBus_para->m_SetRegisterHandler = NULL;
#endif

}

/** Установка скорости отправки и приема данных **/
/*** Параметры ***
** baud: Скорость передачи и приема данных
** Примечание: Можно не устанавливать, используйте тайм-аут по умолчанию (скорость передачи данных 9600 совместима, скорость передачи данных выше 9600 можно не устанавливать, ниже необходимо установить).
***/
void ModBus_setBitRate(ModBus_parameter* ModBus_para, uint32_t baud)
{
	if (baud > 0)
	{
		ModBus_para->m_receiveTimeout = 4000u * 8u / baud + 2u;
		ModBus_para->m_sendTimeout = ((ModBus_para->m_registerAcessLimit * 4u + 20u) * 2000u + 7000u) * 8u / baud + 15u;
	}
}

/** Установка тайм-аута приема **/
/*** Параметры ***
** receiveTimeout: Время ожидания следующего байта при приеме, определяется скоростью последовательного порта
** sendTimeout: Тайм-аут для ожидания обратного кадра после передачи, определяется скоростью последовательного порта
** Примечание: Можно не устанавливать, используя тайм-аут по умолчанию.
***/
void ModBus_setTimeout(ModBus_parameter* ModBus_para, uint32_t receiveTimeout, uint32_t sendTimeout)
{
	if (receiveTimeout > 0)
		ModBus_para->m_receiveTimeout = receiveTimeout;
	if (sendTimeout > 0)
		ModBus_para->m_sendTimeout = sendTimeout;
}

// В режиме RTU генерируется контрольная сумма CRC, которая добавляется в конец данных.
// Calculate CRC for outcoming buffer
// and place it to end.
// return total length
static size_t GenCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;

	buff[len++] = lo;
	buff[len++] = hi;
	return len;
}

// В режиме RTU контрольная сумма CRC вычисляется, учитывая начальное значение
// Calculate CRC fro incoming buffer
// Return 1 - if CRC is correct, overwise return 0
static uint8_t CheckCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len - 2; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;
	if ((buff[len - 2] == lo) &&
		(buff[len - 1] == hi))
	{
		return 1;
	}
#ifdef _UNIT_TEST
	printf("CRC Check ERROR\n");
#endif // _UNIT_TEST
	return 0;
}

// В режиме ASCII генерируется контрольная сумма LRC, которая добавляется в конец данных.
static size_t GenLRC(uint8_t* buff, size_t len)
{
	size_t i = len;
	uint8_t uchLRC = 0; /* Инициализация байта LRC */
	while (i--)
		uchLRC += *buff++; /* Накапливать*/

	uchLRC = ((uint8_t)(-((char)uchLRC)));
	*buff = uchLRC;
	return len + 1;
}

// В режиме ASCII проверяется контрольная сумма LRC,
static uint8_t CheckLRC(uint8_t* buff, size_t len)
{
	uint8_t uchLRC = 0; /* Инициализация байта LRC */
	uint8_t LRC1 = buff[--len];
	while (len--)
		uchLRC += *buff++; /* Накапливать*/

	uchLRC = ((uint8_t)(-((char)uchLRC)));
	if (LRC1 == uchLRC)
		return 1;
#ifdef _UNIT_TEST
	printf("LRC Check ERROR\n");
#endif // _UNIT_TEST
	return 0;
}

// В режиме ASCII входящая строка преобразуется в двоичные байты
static size_t char2bin(uint8_t* buff, size_t len)
{
	size_t binInd = 0;
	for (size_t i = 0; i < len; i++)
	{
		uint8_t bin = 0, chr = buff[i];
		if ((chr >= '0') && (chr <= '9'))
		{
			bin = (uint8_t)(chr - '0');
		}
		else if ((chr >= 'A') && (chr <= 'F'))
		{
			bin = (uint8_t)(chr - 'A' + 0x0A);
		}
		if (i % 2 == 0)
		{
			buff[binInd] = bin;
		}
		else
		{
			buff[binInd] = (buff[binInd] << 4) + bin;
			binInd++;
		}
	}
	return binInd;
}

// Режим ASCII, двоичное преобразование байтов в строку, для отправки
static size_t bin2char_s(uint8_t* buff, size_t len, size_t maxLen)
{
	size_t binLen = len * 2;
	size_t ret = binLen;
	if (ret > maxLen)
	{
		return 0;
	}
	while (len--)
	{
		for (int i = 0; i < 2; i++)
		{
			uint8_t bin = (buff[len] >> 4 * i) & 0x0F;
			char chr = 0;
			if ((bin >= 0) && (bin <= 9))
			{
				chr = (char)(bin + '0');
			}
			else if ((bin >= 0x0A) && (bin <= 0x0F))
			{
				chr = (char)(bin - 0x0A + 'A');
			}
			buff[--binLen] = chr;
		}
	}
	return ret;
}

static MODBUS_FRAME_T* addFrame(ModBus_parameter* ModBus_para)
{
	MODBUS_FRAME_T* pFrame;
	if (ModBus_para->m_sendFramesN >= MODBUS_WAITFRAME_N)
	{
		memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (ModBus_para->m_sendFramesN - 1) * sizeof(MODBUS_FRAME_T));
		pFrame = ModBus_para->m_sendFrames + (ModBus_para->m_sendFramesN - 1);
	}
	else
	{
		pFrame = ModBus_para->m_sendFrames + (ModBus_para->m_sendFramesN++);
	}
	pFrame->index = ModBus_para->m_nextFrameIndex++;
	if (ModBus_para->m_nextFrameIndex == 0) // Номер инструкции не равен 0
	{
		ModBus_para->m_nextFrameIndex = 1;
	}
	pFrame->size = 0;
	pFrame->responseHandler = NULL;
	pFrame->time = millis();
	MODBUS_DELAY_DEBUG(("Frame Len %d\n", ModBus_para->m_sendFramesN));
	return pFrame;
}


// Получение байтовых данных по протоколу ModBus, обычно вызывается в функциях прерывания (например, прерывание приема последовательного порта).
void ModBus_readbyteFromOuter(ModBus_parameter* ModBus_para, uint8_t receiveduint8_t)
{
#ifdef _UNIT_TEST
	printf("address %02x read uint8_t: %02x\n", ModBus_para->m_address, receiveduint8_t);
#endif // _UNIT_TEST

	/*** Значение ModBus_para->m_pBeginReceiveBufferTmp не может быть изменено внутри этой функции!!!!!!!!!!!!!
	**** Значение ModBus_para->m_pEndReceiveBufferTmp не может быть изменено вне этой функции!!!!!!!!!!!!!!!
	**** Избегайте конфликтов при записи в память и обеспечивайте целостность данных***/
	*ModBus_para->m_pEndReceiveBufferTmp = receiveduint8_t;
	if (ModBus_para->m_pEndReceiveBufferTmp >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE - 1)
	{
		ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	}
	else
	{
		ModBus_para->m_pEndReceiveBufferTmp++;
	}
	if (ModBus_para->m_pEndReceiveBufferTmp == ModBus_para->m_pBeginReceiveBufferTmp)
	{
		ModBus_para->m_pEndReceiveBufferTmp--;
		if (ModBus_para->m_pEndReceiveBufferTmp < ModBus_para->m_receiveBufferTmp)
		{
			ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp + (MODBUS_BUFFER_SIZE - 1);
		}
	}
	ModBus_para->m_lastReceivedTime = millis();
}

void ModBus_fastMode(ModBus_parameter* ModBus_para, uint8_t faston)
{
	ModBus_para->m_faston = faston;
}


// Проверка входящих пакетов, возвращает 1, если есть достоверные данные, в противном случае возвращает 0
static uint8_t ModBus_detectFrame(ModBus_parameter* ModBus_para, size_t* restSize)
{
	size_t i = 0, j = 0;
	uint8_t* pEnd, *pBegin;
	size_t lenBufferTmp;
	uint8_t frameSize = 0;

#ifdef MODBUS_MASTER
	if (ModBus_para->m_sendFramesN > 0)
	{
		frameSize = ModBus_para->m_sendFrames[0].responseSize;
	}
#endif

	pEnd = ModBus_para->m_pEndReceiveBufferTmp;
	pBegin = ModBus_para->m_pBeginReceiveBufferTmp; // volatile переменные должны быть присвоены энергонезависимым переменным, прежде чем ими можно будет манипулировать, иначе существует вероятность неполноты данных
	lenBufferTmp = pEnd - pBegin;
	if (pEnd < pBegin)
	{
		lenBufferTmp = (size_t)MODBUS_BUFFER_SIZE - (pBegin - pEnd);
	}
	*restSize = 0;

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
	{
		if (lenBufferTmp == 0)
		{
			return 0;
		}
		if (!ModBus_para->m_hasDetectedBufferStart)
		{// Определение начальных символов
			for (i = 0; i < lenBufferTmp; i++, pBegin++)
			{
				if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
				{
					pBegin = ModBus_para->m_receiveBufferTmp;
				}
				if (*pBegin == ':') // Обнаружен начальный символ
				{
					ModBus_para->m_hasDetectedBufferStart = 1;
					i++;
					pBegin++;
					if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
					{
						pBegin = ModBus_para->m_receiveBufferTmp;
					}
					break;
				}
			}
		}
		if (ModBus_para->m_hasDetectedBufferStart)
		{// Обнаружение конечных символов
			for (j = i; j < lenBufferTmp; j++, pBegin++)
			{
				if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
				{
					pBegin = ModBus_para->m_receiveBufferTmp;
				}
				if (*pBegin == '\r') // Обнаружен завершающий символ
				{
					break;
				}
				ModBus_para->m_receiveFrameBuffer[ModBus_para->m_receiveFrameBufferLen++] = *pBegin;
			}
		}
		else // Если начальный символ не обнаружен, полученные данные являются ненормальными
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd; // Отбросить данные между pBegin и pEnd и сохранить новые данные, добавленные после pEnd, поскольку новые данные могут быть получены во время вышеуказанного процесса
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (j == lenBufferTmp) // Если закрывающий символ не обнаружен, вернитесь для продолжения приема
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			return 0;
		}
		else if (j + 1 == lenBufferTmp) // Если символ возврата каретки является последним символом, то символ возврата каретки сохраняется для дальнейшего приема
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pBegin;;
			return 0;
		}
		pBegin++;
		if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
		{
			pBegin = ModBus_para->m_receiveBufferTmp;
		}
		if (*pBegin != '\n') // Исключение при получении данных, если следующий символ не является новой строкой
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}

		ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
		ModBus_para->m_receiveFrameBufferLen = char2bin(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen);
		if (ModBus_para->m_receiveFrameBuffer[0] != ModBus_para->m_address)
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!CheckLRC(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen)) // Если проверка не проходит
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}

		//MODBUS_DEBUG(("ModBus rec: %*s\n", ModBus_para->m_receiveFrameBufferLen, ModBus_para->m_receiveFrameBuffer));

		ModBus_para->m_receiveFrameBufferLen--; // Удаление контрольной суммы
		ModBus_para->m_hasDetectedBufferStart = 0;

		break;
	}
	case RTU:
	{
		uint8_t isTimeout = 0;
		if (lenBufferTmp == 0) // Данные не получены из-за тайм-аута приема
		{
			isTimeout = 1;
		}
		if (!ModBus_para->m_hasDetectedBufferStart)
		{// Определение начального байта
			for (i = 0; i < lenBufferTmp; i++, pBegin++)
			{
				if (*pBegin == ModBus_para->m_address) // Адрес обнаружен
				{
					ModBus_para->m_hasDetectedBufferStart = 1;
					ModBus_para->m_receiveFrameBuffer[ModBus_para->m_receiveFrameBufferLen++] = *pBegin;
					i++;
					pBegin++;
					break;
				}
			}
		}
		if (ModBus_para->m_hasDetectedBufferStart)
		{
			// Копирование всех данных из временного буфера в буфер данных приема
			size_t newSize = lenBufferTmp - i;
			if (ModBus_para->m_receiveFrameBufferLen + newSize > MODBUS_BUFFER_SIZE)
			{
				newSize = MODBUS_BUFFER_SIZE - ModBus_para->m_receiveFrameBufferLen;
			}
			if (pBegin <= pEnd)
			{
				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, pBegin, newSize);
			}
			else
			{
				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, pBegin, 
				        (size_t)MODBUS_BUFFER_SIZE - (pBegin - ModBus_para->m_receiveBufferTmp));

				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen + (size_t)MODBUS_BUFFER_SIZE - (pBegin - ModBus_para->m_receiveBufferTmp), 
				        (void*)ModBus_para->m_receiveBufferTmp, pEnd - ModBus_para->m_receiveBufferTmp);
			}
			ModBus_para->m_receiveFrameBufferLen += newSize;
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
		}
		else // Если начальный символ не обнаружен, полученные данные являются ненормальными
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!(isTimeout // Тайм-аут приема
			|| frameSize > 0 && ModBus_para->m_receiveFrameBufferLen >= frameSize // Достаточное количество пакетов данных
			|| ModBus_para->m_receiveFrameBufferLen >= MODBUS_BUFFER_SIZE)) // Буфер заполнен
		{
			// Прием не завершен, вернитесь для продолжения приема данных
			return 0;
		}
		if (ModBus_para->m_receiveFrameBufferLen < 2) // Получение тайм-аута и недостаточного количества данных является ненормальным
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!CheckCRC16(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen)) // Если проверка не проходит
		{
			if (frameSize > 0 && frameSize < ModBus_para->m_receiveFrameBufferLen)  // Если длина данных больше, чем m_responseFrameLen, попробуйте получить их с длиной m_responseFrameLen
			{
				if (!CheckCRC16(ModBus_para->m_receiveFrameBuffer, frameSize)) // Если проверка не проходит, это не тайм-аут или буфер заполнен, затем вернитесь, чтобы продолжить прием
				{
					if (isTimeout || ModBus_para->m_receiveFrameBufferLen >= MODBUS_BUFFER_SIZE)
						ModBus_para->m_receiveFrameBufferLen = 0;
					return 0;
				}

				*restSize = ModBus_para->m_receiveFrameBufferLen - frameSize;// Объем сохраняемых данных
				ModBus_para->m_receiveFrameBufferLen = frameSize;
			}
			else
			{
				ModBus_para->m_receiveFrameBufferLen = 0;
				return 0;
			}
		}
		ModBus_para->m_receiveFrameBufferLen--; // Удаление контрольной суммы
		ModBus_para->m_hasDetectedBufferStart = 0;
		break;
	}
	default:
		ModBus_para->m_receiveFrameBufferLen = 0;
		return 0;
		break;
	}

	return 1;
}


#ifdef MODBUS_MASTER
/** Чтение регистра **/
/*** Параметры ***
** address: Адрес первого регистра
** count: Количество считываемых регистров
** GetReponseHandler: Функция обратного вызова для чтения результатов, входящие параметры(uint16_t* buff, uint16_t buffLen)
** Возвращает серийный номер команды (больше 0), так что в функции обратного вызова можно определить, какая команда завершена, и не может быть отправлена обратно в 0
***/
uint8_t ModBus_getRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t count, void(*GetReponseHandler)(uint16_t*, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = READ_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = GetReponseHandler;
	pFrame->address = address;
	pFrame->count = count;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // Адрес устройства
	pFrame->data[pFrame->size++] = READ_REGISTER; // Код функции - чтение регистров
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // Старший байт адреса первого регистра
	pFrame->data[pFrame->size++] = address & 0x0FF; // Младший байт адреса первого регистра
	pFrame->data[pFrame->size++] = (count >> 8) & 0x0FF; // Старший байт количества запрашиваемых регистров
	pFrame->data[pFrame->size++] = count & 0x0FF; // Младший байт количества запрашиваемых регистров
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // Исключение начальных символов
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // Конечный символ
		pFrame->data[pFrame->size++] = '\n'; // Конечный символ
		pFrame->responseSize = 11 + 4 * count; // Количество байт, которые должны быть в ответном кадре
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 5 + 2 * count; // Количество байт, которые должны быть в ответном кадре
		break;
	default:
		break;
	}

	return pFrame->index;
}

/** Запись одного регистра **/
/*** Параметры ***
** address: Адрес регистра
** data: Данные для записи
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают первый адрес и количество регистров
** Возвращает серийный номер инструкции, чтобы определить, какая команда была завершена в функции обратного вызова
***/
uint8_t ModBus_setRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data, void(*SetReponseHandler)(uint16_t, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = WRITE_SINGLE_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = SetReponseHandler;
	pFrame->address = address;
	pFrame->count = 1;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // Адрес устройства
	pFrame->data[pFrame->size++] = WRITE_SINGLE_REGISTER; // Код функции - запись одного регистра
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // Старший байт адреса регистра
	pFrame->data[pFrame->size++] = address & 0x0FF; // Младший байт адреса первого регистра
	pFrame->data[pFrame->size++] = (data >> 8) & 0x0FF; // Старший байт записываемых данных
	pFrame->data[pFrame->size++] = data & 0x0FF; // Младший байт записываемых данных
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // Исключение стартовых символов
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // Конечный символ
		pFrame->data[pFrame->size++] = '\n'; // Конечный символ
		pFrame->responseSize = 17; // Количество байт, которые должны быть в ответном кадре
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 8; // Количество байт, которые должны быть в ответном кадре
		break;
	default:
		break;
	}

	return pFrame->index;
}

/** Запись нескольких регистров **/
/*** Параметры ***
** address: Адрес первого регистра
** data: Данные для записи
** count: Количество записываемых регистров
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают в себя первый адрес и количество регистров
** Возврат 0 означает успешную отправку, возврат 1 означает занято, но не отправлено
***/
uint8_t ModBus_setRegisters(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count, void(*SetReponseHandler)(uint16_t, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = WRITE_MULTI_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = SetReponseHandler;
	pFrame->address = address;
	pFrame->count = count;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // Адрес устройства
	pFrame->data[pFrame->size++] = WRITE_MULTI_REGISTER; // Код функции, запись в несколько регистров
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // Зарегистрируйте первый адрес старшим битом
	pFrame->data[pFrame->size++] = address & 0x0FF; // Зарегистрируйте первый адрес младший бит
	pFrame->data[pFrame->size++] = (count >> 8) & 0x0FF; // Количество регистров высокого уровня
	pFrame->data[pFrame->size++] = count & 0x0FF; // Количество регистров низкого уровня
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->data[pFrame->size++] = count * 4; // Количество байт данных
		break;
	case RTU:
		pFrame->data[pFrame->size++] = count * 2; // Количество байт данных
		break;
	default:
		break;
	}
	if (count > ModBus_para->m_registerAcessLimit || pFrame->size + 2 * count + 2 > MODBUS_BUFFER_SIZE) // Если максимальный объем данных превышен, они не отправляются, а сразу вызывается функция обратного вызова
	{
		if (SetReponseHandler)
		{
			ModBus_para->m_sendFramesN--;
			(*(SetReponseHandler))(address, 0);
		}
		return 0;
	}
	for (uint16_t i = 0; i < count; i++)
	{
		pFrame->data[pFrame->size++] = (data[i] >> 8) & 0x0FF; // Высокий уровень данных
		pFrame->data[pFrame->size++] = data[i] & 0x0FF; // Низкие данные
	}
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // Исключение начальных символов
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // Конечный символ
		pFrame->data[pFrame->size++] = '\n'; // Конечный символ
		pFrame->responseSize = 17; // Возвращает количество байт, требуемое для фрейма
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 8; // Возвращает количество байт, требуемое для фрейма
		break;
	default:
		break;
	}

	return pFrame->index;
}


// Конец приема данных, обработка данных, возврат 1, если существуют действительные данные, в противном случае возврат 0
static uint8_t ModBus_parseReveivedBuff(ModBus_parameter* ModBus_para)
{
	size_t restSize;
	MODBUS_FRAME_T* pFrame = NULL;
	if (ModBus_para->m_sendFramesN > 0)
	{
		pFrame = ModBus_para->m_sendFrames;
	}
	else // Если возвратный кадр не ожидается, данные не обрабатываются
	{
		ModBus_para->m_pBeginReceiveBufferTmp = ModBus_para->m_pEndReceiveBufferTmp;
		ModBus_para->m_receiveFrameBufferLen = 0;
		return 0;
	}

	if (!ModBus_detectFrame(ModBus_para, &restSize))
	{
		return 0;
	}

	MODBUS_DELAY_DEBUG(("Frame Delay %d\n", millis() - pFrame->time));
	// Код функции суждения
	switch (ModBus_para->m_receiveFrameBuffer[1])
	{
	case READ_REGISTER:
	{
		uint8_t count = ModBus_para->m_receiveFrameBuffer[2];
		MODBUS_DEBUG(("ModBus read reg response\n"));
		if (count % 2 != 0 || pFrame->type != READ_REGISTER || count != pFrame->count * 2) // Ненормальные данные
		{
			// Сохраненные необработанные данные
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}
		count >>= 1; // Разделить на 2
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = ModBus_para->m_registerAcessLimit;
		}
		for (size_t i = 0; i < count; i++)
		{
			ModBus_para->m_registerData[i] = (((uint16_t)ModBus_para->m_receiveFrameBuffer[3 + (i << 1)]) << 8) + ModBus_para->m_receiveFrameBuffer[4 + (i << 1)];
		}
		ModBus_para->m_registerCount = count;

		// Функция обратного вызова
		if (pFrame->responseHandler)
		{
			(*(GetReponseHandler_T)(pFrame->responseHandler))(ModBus_para->m_registerData, count);
		}
		break;
	}
	case WRITE_SINGLE_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t data = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		uint16_t dataSent;
		MODBUS_DEBUG(("ModBus write 0x%04x %d response\n", address, data));
		if (ModBus_para->m_modeType == ASCII)
		{
			uint8_t data[4];
			memcpy(data, pFrame->data + 9, 4);
			char2bin(data, 4);
			dataSent = ((uint16_t)data[0] << 8) + data[1];
		}
		else if (ModBus_para->m_modeType == RTU)
		{
			dataSent = (pFrame->data[4] << 8) + pFrame->data[5];
		}
		if (pFrame->type != WRITE_SINGLE_REGISTER || address != pFrame->address || dataSent != data) // Ненормальные данные
		{
			// Сохраненные необработанные данные
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}

		// Функция обратного вызова
		if (pFrame->responseHandler)
		{
			(*(SetReponseHandler_T)(pFrame->responseHandler))(address, 1);
		}
		break;
	}
	case WRITE_MULTI_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		MODBUS_DEBUG(("ModBus write 0x%04x %d regs response\n", address, count));
		if (pFrame->type != WRITE_MULTI_REGISTER || address != pFrame->address || count != pFrame->count) // Ненормальные данные
		{
			// Сохраненные необработанные данные
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}

		// Функция обратного вызова
		if (pFrame->responseHandler)
		{
			(*(SetReponseHandler_T)(pFrame->responseHandler))(address, count);
		}
		break;
	}
	default:
		memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
		ModBus_para->m_receiveFrameBufferLen = restSize;
		return 0;
		break;
	}

	memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
	ModBus_para->m_receiveFrameBufferLen = restSize;

	// Удалить возвращенную команду
	memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (--ModBus_para->m_sendFramesN) * sizeof(MODBUS_FRAME_T));
	ModBus_para->m_waitingResponse = 0;

	return 1;
}

static void sendFrame_loop(ModBus_parameter* ModBus_para)
{
	uint32_t now = millis();
	if (ModBus_para->m_waitingResponse && now - ModBus_para->m_lastSentTime < ModBus_para->m_sendTimeout || ModBus_para->m_sendFramesN == 0) // Время ожидания обратного кадра не истекло, или нет данных для отправки
	{
		return;
	}
	if (ModBus_para->m_waitingResponse && now - ModBus_para->m_lastSentTime >= ModBus_para->m_sendTimeout) // Ожидание тайм-аута возвратного кадра
	{
		MODBUS_FRAME_T* pFrame = ModBus_para->m_sendFrames;
		MODBUS_DELAY_DEBUG(("Frame Timeout %d\n", millis() - pFrame->time));
		if (pFrame->responseHandler) // Вызывается обратный вызов, передайте параметр (0,0)
		{
			switch (pFrame->type)
			{
			case READ_REGISTER:
				(*(GetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			case WRITE_SINGLE_REGISTER:
				(*(SetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			case WRITE_MULTI_REGISTER:
				(*(SetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			default:
				break;
			}
		}

		memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (--ModBus_para->m_sendFramesN) * sizeof(MODBUS_FRAME_T)); // Удаление отправленных пакетов
		ModBus_para->m_waitingResponse = 0;
	}
	if (!ModBus_para->m_waitingResponse && ModBus_para->m_sendFramesN > 0) // Если вы не ждете обратного кадра, а пакет должен быть отправлен, отправьте
	{
		MODBUS_FRAME_T* pFrame = ModBus_para->m_sendFrames;
		if (ModBus_para->m_faston) // В случае быстрого режима выполняется только последняя команда
		{
			ModBus_para->m_sendFrames[0] = ModBus_para->m_sendFrames[ModBus_para->m_sendFramesN - 1];
			ModBus_para->m_sendFramesN = 1;
		}
		if (ModBus_para->m_SendHandler != NULL)
		{
			(*ModBus_para->m_SendHandler)(pFrame->data, pFrame->size);
			ModBus_para->m_waitingResponse = 1;
			ModBus_para->m_lastSentTime = millis();
		}
	}
}

void ModBus_Master_loop(ModBus_parameter* ModBus_para)
{
	uint32_t now = millis();

	if (ModBus_para->m_pBeginReceiveBufferTmp != ModBus_para->m_pEndReceiveBufferTmp)
	{
		ModBus_parseReveivedBuff(ModBus_para); // Обработка входящих данных
	}
	if (now - ModBus_para->m_lastReceivedTime > ModBus_para->m_receiveTimeout) // Таймаут приема, обработка данных и сброс
	{
		ModBus_parseReveivedBuff(ModBus_para); // Обработка входящих данных
		ModBus_para->m_receiveFrameBufferLen = 0;
		ModBus_para->m_lastReceivedTime = millis();
	}

	sendFrame_loop(ModBus_para);
}
#endif

#ifdef MODBUS_SLAVE

void ModBus_attachRegisterHandler(ModBus_parameter* ModBus_para, size_t(*GetRegisterHandler)(uint16_t, uint16_t, uint16_t*), size_t(*SetRegisterHandler)(uint16_t, uint16_t, uint16_t*))
{
	ModBus_para->m_GetRegisterHandler = GetRegisterHandler;
	ModBus_para->m_SetRegisterHandler = SetRegisterHandler;
}

/** Кадр возврата регистра чтения **/
/*** Параметры ***
** address: Адрес первого регистра
** count: Количество считываемых регистров
** GetReponseHandler: Функция обратного вызова результата чтения, входящие параметры(uint16_t* buff, uint16_t buffLen)
***/
static void ModBus_getRegister_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint8_t count)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // Адрес устройства
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = READ_REGISTER; // Код функции, чтение регистров

	if (count > ModBus_para->m_registerAcessLimit || ModBus_para->m_sendFrameBufferLen + 2 * count + 3 > MODBUS_BUFFER_SIZE) // Если максимальный объем данных превышен
	{
		count = 0;
	}

	count = (uint8_t)(*(ModBus_para->m_GetRegisterHandler))(address, count, ModBus_para->m_registerData);
	ModBus_para->m_registerCount = count;
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = count * 2; // Количество байт = количество считываемых регистров * 2
	for (uint16_t i = 0; i < count; i++)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (ModBus_para->m_registerData[i] >> 8) & 0x0FF; // Старший байт регистра
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_registerData[i] & 0x0FF; // Младший байт регистра
	}
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // Исключение начальных символов
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // Конечный символ
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // Конечный символ
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

/** Запись одиночного регистра кадр возврата **/
/*** Параметры ***
** address: Адрес регистра
** data: Данные для записи
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают в себя первый адрес и количество регистров
***/
static void ModBus_setRegister_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // Адрес устройства
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = WRITE_SINGLE_REGISTER; // Функциональный код, считанный регистр

	if ((*(ModBus_para->m_SetRegisterHandler))(address, 1, &data) == 0) // Если происходит ошибка записи, данные инвертируются и возвращаются, чтобы хост мог определить
	{
		data = ~data;
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (address >> 8) & 0x0FF; // Регистрация первого адреса высокого уровня
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = address & 0x0FF; // Регистрировать первый младший адрес
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (data >> 8) & 0x0FF; // Высокий уровень данных
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = data & 0x0FF; // Низкие данные

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // Исключение начальных символо
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // Конечный символ
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // Конечный символ
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

/** Запись кадра возврата нескольких регистров **/
/*** Параметры ***
** address: Адрес первого регистра
** data: Данные для записи
** count: Количество записываемых регистров
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают в себя первый адрес и количество регистров
***/
static void ModBus_setRegisters_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // Адрес устройства
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = WRITE_MULTI_REGISTER; // Коды функций, запись регистров

	count = (uint16_t)(*(ModBus_para->m_SetRegisterHandler))(address, count, data);
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (address >> 8) & 0x0FF; // Высокий уровень первого адреса
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = address & 0x0FF; // Низкий уровень первого адреса
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (count >> 8) & 0x0FF; // высокий номер
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = count & 0x0FF; // низкий номер

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // Исключение начальных символов
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // Конечный символ
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // Конечный символ
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

// Конец приема данных, обработка данных, возвращает 1, если существуют действительные данные, в противном случае возвращает 0
static uint8_t ModBus_parseReveivedBuff_Slave(ModBus_parameter* ModBus_para)
{
	size_t restSize;
	if (!ModBus_detectFrame(ModBus_para, &restSize))
	{
		return 0;
	}

	// Коды функций ModBus
	switch (ModBus_para->m_receiveFrameBuffer[1])
	{
	case READ_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = 0;
		}
		ModBus_getRegister_Slave(ModBus_para, address, (uint8_t)count);
		break;
	}
	case WRITE_SINGLE_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t data = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		ModBus_setRegister_Slave(ModBus_para, address, data);
		break;
	}
	case WRITE_MULTI_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		//uint8_t size = ModBus_para->m_receiveFrameBuffer[6];
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = 0;
		}
		for (uint16_t i = 0; i < count; i++)
		{
			ModBus_para->m_registerData[i] = ((uint16_t)(*(ModBus_para->m_receiveFrameBuffer + 7 + i * 2)) << 8) + (uint16_t)(*(ModBus_para->m_receiveFrameBuffer + 8 + i * 2));
		}
		ModBus_para->m_registerCount = count;
		ModBus_setRegisters_Slave(ModBus_para, address, ModBus_para->m_registerData, count);
		break;
	}
	default:
		assert(0);
		memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
		ModBus_para->m_receiveFrameBufferLen = restSize;
		return 0;
		break;
	}
	memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
	ModBus_para->m_receiveFrameBufferLen = restSize;
	return 1;
}

void ModBus_Slave_loop(ModBus_parameter* ModBus_para)
{
	uint32_t now = millis();
	
	if (ModBus_para->m_pBeginReceiveBufferTmp != ModBus_para->m_pEndReceiveBufferTmp)
	{
		ModBus_parseReveivedBuff_Slave(ModBus_para); // Обработка входящих данных
	}
	if (now - ModBus_para->m_lastReceivedTime > ModBus_para->m_receiveTimeout) // Таймаут приема, обработка данных и сброс
	{
		ModBus_parseReveivedBuff_Slave(ModBus_para); // Обработка входящих данных
		ModBus_para->m_receiveFrameBufferLen = 0;
	}
}
#endif

#ifdef _UNIT_TEST
#include <string.h>
#include <stdio.h>
#include <windows.h>
ModBus_parameter modBus_master_test, modBus_slave_test;
uint32_t t = 0;
uint32_t millis()
{
	return t;
}

static void OutputData_master(uint8_t* data, size_t len)
{
	int t = millis();
	switch (modBus_master_test.m_modeType)
	{
	case ASCII:
	{
		char strtmp[1000];
		assert(len < 1000);
		strncpy(strtmp, data, len);
		strtmp[len] = 0;
		printf("master send: %s\n", strtmp);
		break;
	}
	case RTU:
	{
		char strtmp[1000];
		for (size_t i = 0; i < len; i++)
		{
			sprintf(strtmp + i * 2, "%02x", data[i]);
		}

		printf("master send: %s\n", strtmp);

		break;
	}
	default:
		assert(0);
		break;
	}

	for (size_t i = 0; i < len; i++)
	{
		ModBus_readbyteFromOuter(&modBus_slave_test, data[i]);
	}
}

static void OutputData_slave(uint8_t* data, size_t len)
{
	switch (modBus_slave_test.m_modeType)
	{
	case ASCII:
	{
		char strtmp[1000];
		assert(len < 1000);
		strncpy(strtmp, data, len);
		strtmp[len] = 0;
		printf("slave send: %s\n", strtmp);
		break;
	}
	case RTU:
	{
		char strtmp[1000];
		for (size_t i = 0; i < len; i++)
		{
			sprintf(strtmp + i * 2, "%02x", data[i]);
		}

		printf("slave send: %s\n", strtmp);
		break;
	}
	default:
		assert(0);
		break;
	}

	for (int i = 0; i < len; i++)
	{
		ModBus_readbyteFromOuter(&modBus_master_test, data[i]);
	}
}

uint16_t g_registerData[50];
uint16_t g_address = 0, g_count = 0;

static size_t getReg(uint16_t address, uint16_t n, uint16_t* data)
{
	for (uint16_t i = 0; i < n; i++)
	{
		data[i] = g_registerData[address + i];
	}
	return n;
}

static size_t setReg(uint16_t address, uint16_t n, uint16_t* data)
{
	for (uint16_t i = 0; i < n; i++)
	{
		g_registerData[address + i] = data[i];
	}
	return n;
}

void master_printReg(uint16_t* data, uint16_t count)
{
	char strtmp[1000];
	printf("count= %u\n", count);
	printf("g_count= %u\n", g_count);
	assert(count == g_count);
	for (uint16_t i = 0; i < count; i++)
	{
		sprintf(strtmp + i * 4, "%04x", data[i]);
	}
	printf("register data: %s\n", strtmp);
}

void master_printSetReg(uint16_t address, uint16_t count)
{
	assert(address == g_address);
	assert(count == g_count);
	printf("set register: address %d, count %d\n", address, count);
}

void unit_test()
{
	// Конфигурация хоста
	ModBus_Setting_T modbusSetting;
	modbusSetting.address = 0x01;
	modbusSetting.baudRate = 9600;
	modbusSetting.frameType = RTU;
	modbusSetting.register_access_limit = 5;
	modbusSetting.sendHandler = OutputData_master;
	ModBus_setup(&modBus_master_test, modbusSetting);
	ModBus_setTimeout(&modBus_master_test, 5, 5);

	// Конфигурация ведомого устройства
	modbusSetting.address = 0x01;
	modbusSetting.baudRate = 9600;
	modbusSetting.frameType = RTU;
	modbusSetting.register_access_limit = 5;
	modbusSetting.sendHandler = OutputData_slave;
	ModBus_setup(&modBus_slave_test, modbusSetting);
	ModBus_setTimeout(&modBus_slave_test, 5, 5);
	ModBus_attachRegisterHandler(&modBus_slave_test, getReg, setReg);

	// Инициализация виртуальных регистров
	for (int i = 0; i < 10; i++)
	{
		g_registerData[i] = -i;
	}

	for (int i = 0; i < 1000; i++)
	{
		// Тестовый регистр чтения
		g_address = 0;
		g_count = 5;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// Тестовая запись одного регистра
		g_address = 2;
		g_count = 1;
		ModBus_setRegister(&modBus_master_test, g_address, 0x0005, master_printSetReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// Тестовый регистр чтения
		g_address = 1;
		g_count = 3;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// Тестовая запись нескольких регистров
		{
			uint16_t data[] = { 1,2,3,4 };
			g_address = 0;
			g_count = 4;
			ModBus_setRegisters(&modBus_master_test, g_address, data, g_count, master_printSetReg);
		}
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// Тестовый регистр чтения
		g_address = 50;
		g_count = 5;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		printf("Master loop 1\n");
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		printf("Slave loop\n");
		ModBus_Slave_loop(&modBus_slave_test);
		printf("Master loop 2\n");
		ModBus_Master_loop(&modBus_master_test);
	}

}

#endif // _UNIT_TEST
