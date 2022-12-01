#ifndef MOTECMODBUS_H_
#define MOTECMODBUS_H_
/**** Протокол ModBus master/slave ****
** Поддержка двух режимов ASCII / RTU
** Может быть создано несколько экземпляров
** Регистры чтения и записи используют неблокирующую функцию и получают результат путем привязки функции обратного вызова.
** Как использовать:
**** 1.Master
****** Вызов конфигурации ModBus_setup
****** Вызовите ModBus_readbyteFromOuter в функции прерывания приема последовательного порта
****** Циклический вызов ModBus_Master_loop
****** Вызовите ModBus_getRegister для считывания значения регистров целевого устройства
****** Вызовите ModBus_setRegister для записи одного регистра целевого устройства
****** Вызовите ModBus_setRegisters для записи нескольких регистров целевого устройства
**** 2.Slave
****** Вызов параметров конфигурации ModBus_setup
****** Вызовите ModBus_attachRegisterHandler, чтобы привязать функцию для получения и установки регистров
****** Вызов ModBus_Slave_loop в цикле
**** 3.Официальное описание функции см. в следующей части внешнего интерфейса.
*/

// Используя протокол master/slave, можно использовать их одновременно
#define MODBUS_MASTER
//#define MODBUS_SLAVE

#define _UNIT_TEST
//#define DEBUG
//#define _DELAY_DEBUG

#ifdef _UNIT_TEST

#ifndef MODBUS_MASTER
#define MODBUS_MASTER
#endif // !MODBUS_MASTER

#ifndef MODBUS_SLAVE
#define MODBUS_SLAVE
#endif // !MODBUS_SLAVE

#endif // _UNIT_TEST

#ifdef DEBUG
#define MODBUS_DEBUG(x) print_controlled x
#else
#define MODBUS_DEBUG(x)  
#endif // DEBUG

#ifdef _DELAY_DEBUG
#define MODBUS_DELAY_DEBUG(x) print_controlled x
#else
#define MODBUS_DELAY_DEBUG(x)  
#endif // DEBUG

#define MODBUS_REGISTER_LIMIT 6 // Максимальное количество регистров чтения и записи одновременно
#define MODBUS_BUFFER_SIZE ((MODBUS_REGISTER_LIMIT)*2 + 20) // Максимальная длина пакета данных (длина пакета данных для записи нескольких регистров)
#define MODBUS_WAITFRAME_N 5  // Максимальное количество кэшей команд
#define MODBUS_DEFAULT_BAUD 9600 // Скорость передачи и приема данных по умолчанию, 9600 Бит/с

#include <assert.h>
#include <stdint.h>
#include <string.h>

// TODO: функция для получения системного времени в миллисекундах.
uint32_t millis();

typedef enum {
	ASCII,
	RTU
} MODBUS_MODE_TYPE;

typedef enum {
	READ_REGISTER = 0x03,
	WRITE_SINGLE_REGISTER = 0x06,
	WRITE_MULTI_REGISTER = 0x10,
} MODBUS_FUNCTION_TYPE;

typedef struct _MODBUS_SETTING_T { // Тип для конфигурации экземпляра ModBus
	uint8_t address; // Адрес целевого устройства
	MODBUS_MODE_TYPE frameType; // Режим работы: ASCII/RTU
	uint32_t baudRate; // Скорость передачи данных, например 9600 или 115200 и т.д.
	uint8_t register_access_limit; // Максимальное количество регистров чтения/записи одновременно
	void(*sendHandler)(uint8_t*, size_t); // Функция, используемая для отправки данных, параметры функции: (uint8_t* data, size_t size), data - адрес массива данных, size - его размер
} ModBus_Setting_T;

typedef struct _MODBUS_FRAME_T {
	uint8_t index; // Номер команды
	uint8_t data[MODBUS_BUFFER_SIZE + 2]; // Данные, выделенные двумя дополнительными байтами для безопасности
	uint8_t size; // Размер данных
	MODBUS_FUNCTION_TYPE type; // Тип команды
	uint32_t time; // Время начала выполнения команды
	void* responseHandler; // Указатель на функцию обратного вызова в конце выполнения инструкции
	uint8_t responseSize; // Длина возвращаемого кадра
	uint16_t address; // Адрес регистра доступа
	uint8_t count; // Количество регистров доступа
} MODBUS_FRAME_T;

typedef void(*GetReponseHandler_T)(uint16_t*, uint16_t); // Тип указателя функции обратного вызова регистра чтения, параметры функции возврата: (первый адрес буфера значений регистра, количество регистров)
typedef void(*SetReponseHandler_T)(uint16_t, uint16_t); // Тип указателя функции обратного вызова регистра записи, параметры функции обратного вызова: (адрес регистра, количество записей)

typedef struct __MODBUS_Parameter {
	uint8_t m_address; // Адрес Slave устройства
	MODBUS_MODE_TYPE m_modeType; // Режим работы протокола: ASCII / RTU
	uint8_t m_receiveFrameBuffer[MODBUS_BUFFER_SIZE + 2]; // Получение пакетов, выделение двух дополнительных байтов для безопасности
	size_t m_receiveFrameBufferLen;  // Количество принятых байтов данных

	volatile uint8_t m_receiveBufferTmp[MODBUS_BUFFER_SIZE + 2]; // Временно хранящиеся данные приема, так как эта переменная изменяется функцией прерывания, поэтому используйте круговой доступ, чтобы избежать изменения этой переменной вне функции прерывания
	volatile uint8_t* m_pBeginReceiveBufferTmp; // Начальное положение области циклического доступа
	volatile uint8_t* m_pEndReceiveBufferTmp; // Следующая позиция в конце круговой зоны доступа
	uint8_t m_hasDetectedBufferStart;

	uint16_t m_registerData[MODBUS_REGISTER_LIMIT + 2]; // Данные регистров чтения кэша
	uint16_t m_registerCount;
	uint8_t m_registerAcessLimit;

	volatile uint32_t m_lastReceivedTime; // Момент последнего получения байта данных
	uint32_t m_lastSentTime; // Момент последней отправки данных
	uint32_t m_receiveTimeout; // Установка таймаута ожидания приема следующего символа
	uint32_t m_sendTimeout; // Установака тайм-аута для ожидания обратного кадра

	uint8_t m_faston; // Включение или выключение быстрого режима

	void(*m_SendHandler)(uint8_t*, size_t); // Функция отправки данных, используется для передачи данных на внешние устройства

#ifdef MODBUS_MASTER // Master
	MODBUS_FRAME_T m_sendFrames[MODBUS_WAITFRAME_N]; // Очередь отправки пакетов
	size_t m_sendFramesN; // Длина очереди отправляемых пакетов
	uint8_t m_nextFrameIndex; // Порядковый номер следующего пакета
	uint8_t m_waitingResponse; // Ожидание ответного кадра
#endif // MODBUS_MASTER

#ifdef MODBUS_SLAVE // Slave
	uint8_t m_sendFrameBuffer[MODBUS_BUFFER_SIZE];
	uint8_t m_sendFrameBufferLen;

	size_t(*m_GetRegisterHandler)(uint16_t, uint16_t, uint16_t*); // Функция считывания регистра, параметры функции (первый адрес регистра, количество регистров, считанные данные), возвращает количество успешных считываний
	size_t(*m_SetRegisterHandler)(uint16_t, uint16_t, uint16_t*); // Установить функцию регистра, параметры функции (адрес регистра, количество записей, записанные данные), вернуть количество успешных установок
#endif // MODBUS_SLAVE


} ModBus_parameter;

/************ Внешний интерфейс BEGIN ***********/
void ModBus_setup(ModBus_parameter* ModBus_para, ModBus_Setting_T setting); // Конфигурирование экземпляров ModBus
void ModBus_readbyteFromOuter(ModBus_parameter* ModBus_para, uint8_t receiveduint8_t); // Передача байтовых данных в протокол ModBus
void ModBus_fastMode(ModBus_parameter* ModBus_para, uint8_t faston); // Следует ли включать режим быстрой команды, быстрый режим не кэширует инструкцию, выключение быстрого режима может гарантировать выполнение инструкции, но может возникнуть задержка

/** Настройка скорости отправки и приема данных **/
/*** Параметры ***
** baud: Скорость передачи и приема данных
** Примечание: Можно не устанавливать, используйте тайм-аут по умолчанию (скорость передачи данных 9600 совместима, скорость передачи данных выше 9600 можно не устанавливать, ниже необходимо установить).
***/
void ModBus_setBitRate(ModBus_parameter* ModBus_para, uint32_t baud);

/** Установка тайм-аута приема **/
/*** Параметры ***
** receiveTimeout: Время ожидания следующего байта при приеме, определяется скоростью последовательного порта
** sendTimeout: Тайм-аут для ожидания обратного кадра после передачи, определяется скоростью последовательного порта
** Примечание: Можно не устанавливать, используйте тайм-аут по умолчанию (скорость передачи данных 9600 совместима, скорость передачи данных выше 9600 можно не устанавливать, ниже необходимо установить).
***/
void ModBus_setTimeout(ModBus_parameter* ModBus_para, uint32_t receiveTimeout, uint32_t sendTimeout);


#ifdef MODBUS_MASTER // ModBus Master
// Функция Master-цикла
void ModBus_Master_loop(ModBus_parameter* ModBus_para);

/** Чтение регистров(-а) **/
/*** Параметры ***
** address: Адрес первого регистра
** count: Количество регистров для чтения
** GetReponseHandler: Функция обратного вызова для чтения результатов, входящие параметры(uint16_t* buff, uint16_t buffLen),неудачное считывание входящих параметров (0,0)
** Возвращает серийный номер команды (больше 0), чтобы определить, какая команда была выполнена в функции обратного вызова, и возвращает 0, если она не может быть отправлена.
***/
uint8_t ModBus_getRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t count, void(*GetReponseHandler)(uint16_t*, uint16_t));

/** Запись одного регистра **/
/*** Параметры ***
** address: Адрес первого регистра
** data: Данные для записи
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают в себя первый адрес и количество регистров, а время ожидания команды записи передается в параметрах (0,0).
** Возвращает серийный номер команды (больше 0), чтобы определить, какая команда была выполнена в функции обратного вызова, и возвращает 0, если она не может быть отправлена.
***/
uint8_t ModBus_setRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data, void(*SetReponseHandler)(uint16_t, uint16_t));

/** Запись нескольких регистров **/
/*** Параметры ***
** address: Адрес первого регистра
** data: Данные для записи
** count: Количество записываемых регистров
** SetReponseHandler: Функция обратного вызова результата записи, входящие параметры(uint16_t address, uint16_t count), параметры включают в себя первый адрес и количество регистров, а время ожидания команды записи передается в параметрах (0,0).
** Возвращает серийный номер команды (больше 0), чтобы определить, какая команда была выполнена в функции обратного вызова, и возвращает 0, если она не может быть отправлена.
***/
uint8_t ModBus_setRegisters(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count, void(*SetReponseHandler)(uint16_t, uint16_t));

#endif


#ifdef MODBUS_SLAVE // ModBus Slave
// Функция Slave-цикла
void ModBus_Slave_loop(ModBus_parameter* ModBus_para);

// Функция чтения и записи регистров ведомого устройства
void ModBus_attachRegisterHandler(ModBus_parameter* ModBus_para, size_t(*GetRegisterHandler)(uint16_t, uint16_t, uint16_t*), size_t(*SetRegisterHandler)(uint16_t, uint16_t, uint16_t*));

#endif
/**************** Внешний интерфейс END ***************/

#endif
