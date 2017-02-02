;Програмный проект для лабораторного измерительного стенда на МК ATmega328P

.INCLUDE "m328Pdef.inc"

;Переменные (RAMEND-71...RAMEND-8, 64 байта)
.equ	U_RX_H	= RAMEND-71			;Адреса указателей USART
.equ	U_RX_L	= RAMEND-70
.equ	UTXADH	= RAMEND-69			;Вторая функция - адрес указателя буфера ADC (ADC и TX UART не могут быть использованы одновременно)
.equ	UTXADL	= RAMEND-68

.equ	U_RCNT	= RAMEND-67			;Счетчики байт для прм/прд USART
.equ	UTCNTH	= RAMEND-66
.equ	UTCNTL	= RAMEND-65

.equ	WDTCFG	= RAMEND-65			;Последняя сохранённая конфигурация WDT

.equ	SPITRH	= RAMEND-64			;Адрес указателя прм/прд SPI
.equ	SPITRL	= RAMEND-63

.equ	SPITCT	= RAMEND-62			;Счетчик байт для прд SPI
.equ	SPIRCT	= RAMEND-61			;Счетчик байт для прм SPI

.equ	ADSCTH	= RAMEND-60			;Счетчик семплов ADC
.equ	ADSCTL	= RAMEND-59

;Форма работы с устройством по USART:
;Для обращения к устройству используется посылка из шести байт (даже когда команда не содержит
;аргументов - количество пересылаемых байт равно шести). Если принято меньше (больше) информации,
;в ответ на посылку будет отправлен байт 0xEB, то есть, неправильное число байт (error: byte).
;Если первый байт в посылке (команда) не содержит одного из нижеперечисленных переходов, будет
;послан ответ, состоящий из байта 0xEC (error: comand).
;1 байт - команда. Допустимые (принимаемые) команды:
;	- 0b00000010, 0b00000011 - сконфигурировать осциллограф, 0 или 1 канал соответственно
;	- 0b00000100 - сконфигурировать синтезатор частоты
;	- 0b00001000, 0b00011000 - сконфигурировать ШИМ-генератор, выкл. или вкл. соответственно
;	- 0b00100000 - ping. В ответ будет послан байт 0xAA
;	- 0b01000000 - сбросить настройки DDS- и ШИМ-генераторов
;Для конфигурирования осциллографа аргументы носят следующее содержание:
;1 байт - коэффициент усиления для заданного канала (0...255)
;2:3 байты (2 байт старший) - количество измеряемых семплов (0...512)
;4:5 байты (4 байт старший) - множитель интервала измерения (15...65535).  T=3,2*N (мкс)
;Для конфигурирования синтезатора частоты:
;1 байт - коэффициент усиления масштабирующего усилителя (0...255)
;2:3 байты (2 байт старший) - режим работы DDS-генератора. Параметры из даташита (по умолчанию 0x0000)
;4:5 байты (4 байт старший) - коєфициент деления для настройки частоты. F=20*10^6*N/2^28
;Для конфигурирования ШИМ-генератора:
;1 байт - число для канала OCR0A (0...255)
;2 байт - число для канала OCR0B (0...255)

;Буферы для обработчтка команд
.equ	COMAND	= RAMEND-1103			;Команда
.equ	ARG1	= RAMEND-1102			;Аргументы
.equ	ARG2	= RAMEND-1101
.equ	ARG3	= RAMEND-1100
.equ	ARG4	= RAMEND-1099
.equ	ARG5	= RAMEND-1098

;Использование регистров ввода-вывода под флаги:
;GPIOR0:
;0 бит - передать данные по USART
;1 бит - передаются данные по USART
;2 бит - принимаются данные по USART
;3 бит - данные по USART приняты
;4 бит - используется SPI
;5 бит - принимаются данные по SPI
;6 бит - передача по SPI завершена
;7 бит - передать данные по SPI
;GPIOR1:
;0 бит - запустить измерение ADC
;1 бит - вызвать обработчик приёмника USART (для отладки)
;2 бит - 
;3 бит - 
;4 бит - 
;5 бит - 
;6 бит - 
;7 бит - 

;Векторы прерываний
	rjmp	RESET				;Reset Handler
	reti
	reti	;EXT_INT0			;IRQ0 Handler
	reti
	reti	;EXT_INT1			;IRQ1 Handler
	reti
	reti	;PCINT0				;PCINT0 Handler
	reti
	reti	;PCINT1				;PCINT1 Handler
	reti
	reti	;PCINT2				;PCINT2 Handler
	reti
	rjmp	WDT				;Watchdog Timer Handler
	reti
	reti	;TIM2_COMPA			;Timer2 Compare A Handler
	reti
	reti	;TIM2_COMPB			;Timer2 Compare B Handler
	reti
	reti	;TIM2_OVF			;Timer2 Overflow Handler
	reti
	reti	;TIM1_CAPT			;Timer1 Capture Handler
	reti
	rjmp	TIM1_COMPA			;Timer1 Compare A Handler
	reti
	reti	;TIM1_COMPB			;Timer1 Compare B Handler
	reti
	reti	;TIM1_OVF			;Timer1 Overflow Handler
	reti
	reti	;TIM0_COMPA			;Timer0 Compare A Handler
	reti
	reti	;TIM0_COMPB			;Timer0 Compare B Handler
	reti
	reti	;TIM0_OVF			;Timer0 Overflow Handler
	reti
	rjmp	SPI_STC				;SPI Transfer Complete Handler
	reti
	rjmp	USART_RXC			;USART, RX Complete Handler
	reti
	reti	;USART_UDRE			;USART, UDR Empty Handler
	reti
	rjmp	USART_TXC			;USART, TX Complete Handler
	reti
	reti	;ADC				;ADC Conversion Complete Handler
	reti
	reti	;EE_RDY				;EEPROM Ready Handler
	reti
	reti	;ANA_COMP			;Analog Comparator Handler
	reti
	reti	;TWI				;2-wire Serial Interface Handler
	reti
	reti	;SPM_RDY			;Store Program Memory Ready Handler
	reti

;*******************ОБРАБОТЧИКИ ПРЕРЫВАНИЙ*******************

;Обработчик прерывания от WDT (содержимое использовано для отладки)
WDT:	lds	r17,	WDTCFG			;Восстановить прежнюю конфигурацию WDT
	ldi	r16,	0b00011000
	sts	WDTCSR,	r16
	sts	WDTCSR,	r17

	cbi	GPIOR0,	2
	sbi	GPIOR0,	3
reti

;Обработчик прерывания по совпадению TCNT1 с константой
TIM1_COMPA:
	cli

	lds	r17,	ADCSRA			;r17 в этом обработчике - стратегически важная хрень!

	sbrs	r17,	4			;Если нет флага прерывания от АЦП - запустить
	rjmp	stcnv

	ldi	r16,	0b00000010		;Триггер (для отладки)
	out	PINC,	r16

	lds	r23,	UTXADL			;Проверка указателя
	lds	r22,	UTXADH
	cp	r23,	zl
	cpc	r22,	zh
	in	r16,	SREG
	sbrc	r16,	1
	rjmp	cmeas
	lds	zl,	UTXADL			;Копирование указателя из буфера
	lds	zh,	UTXADH

cmeas:	lds	r27,	ADSCTH			;Счетчик семплов
	lds	r26,	ADSCTL
	lds	r25,	UTCNTH			;Счетчик байт (для UART)
	lds	r24,	UTCNTL

	cp	r26,	r0
	cpc	r27,	r0
	breq	adcend

	sbiw	r26,	1			;Количество семплов

	cpi	r25,	0x04
	breq	adcend

	adiw	r24,	2			;Количество байт

	sts	ADSCTH,	r27
	sts	ADSCTL,	r26
	sts	UTCNTH,	r25
	sts	UTCNTL,	r24

	lds	r19,	ADCL			;Сохранить данные
	lds	r18,	ADCH
	st	z+,	r18
	st	z+,	r19

	sts	UTXADL,	zl			;Сохранить значение указателя
	sts	UTXADH,	zh

	ori	r17,	0b01010000		;Начать преобразование и убрать флаг
	sts	ADCSRA,	r17
reti
;Отключение АЦП и таймера
adcend:	ori	r17,	0b00010000		;Убрать флаг прерывания
	sts	ADCSRA,	r17

	ldi	r16,	0b00001000		;Остановить таймер
	sts	TCCR1B,	r16

	sbi	GPIOR0,	0			;Флаг на передачу данных по UART

	sts	ADSCTH,	r0			;Записать нули в счетчик семплов
	sts	ADSCTL,	r0
reti
;Инициализация указателя
stcnv:	ldi	zl,	low(RAMEND-1095)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1095)
	sts	UTXADL,	zl
	sts	UTXADH,	zh

	in	r16,	GPIOR1			;Убрать флаг "начать измерение"
	andi	r16,	0b11111110
	out	GPIOR1,	r16

	sts	TCNT1H,	r0			;Обнулить таймер
	sts	TCNT1L,	r0
	ldi	r16,	0b00001011		;f = fclk/64, CTC mode

	ori	r17,	0b01000000		;Начать преобразование
	sts	ADCSRA,	r17

	sts	TCCR1B,	r16
reti

;Обработчик прерывания по успешной передаче информации по SPI
zpotrx:	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1119)
	sts	SPITRL,	zl
	sts	SPITRH,	zh

	cbi	GPIOR0,	7			;Убрать флаг "передать данные"

	lds	r15,	SPITCT			;Если счетчик пуст - выйти
	tst	r15
	breq	spiex

	sbi	GPIOR0,	4			;Поставить флаг "используется SPI"

spiiu:	lds	zl,	SPITRL
	lds	zh,	SPITRH
SPI_STC:
	cli

	sbis	GPIOR0,	4			;Если SPI не используется - настроить указатель
	rjmp	zpotrx

	lds	r16,	SPITRL			;Проверка указателя
	lds	r17,	SPITRH
	cp	r16,	zl
	cpc	r17,	zh
	brne	spiiu

	lds	r15,	SPITCT			;Если счетчик пуст - выйти с изменением состояния CS
	tst	r15
	breq	spioff
	dec	r15
	sts	SPITCT,	r15

	in	r17,	SPDR			;Прочитать данные с буфера

	ld	r16,	z			;Передать данные
	out	SPDR,	r16

	sbis	GPIOR0,	5			;Если данные принимаются - записать принятую информацию в буфер
	rjmp	spit

	lds	r16,	SPIRCT			;Если буфр заполнен - выйти с изменением состояния CS
	cpi	r16,	8
	breq	spioff

	inc	r16
	sts	SPIRCT,	r16

	st	z+,	r17

spit:	sbis	GPIOR0,	5			;Инкрементирование указателя вручную, если данные передаются
	adiw	z,	1

	sts	SPITRL,	zl			;Сохранить значение указателя
	sts	SPITRH,	zh
reti
spioff:	sbi	GPIOR0,	6			;Передача завершена

spiex:	in	r16,	PIND
	ori	r16,	0b10011100
	out	PORTD,	r16

	cbi	GPIOR0,	4			;Очистить флаг
reti

;Обработчик прерывания по успешному приёму информации по USART
zporx:	ldi	zl,	low(RAMEND-1103)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1103)
	sts	U_RX_L,	zl
	sts	U_RX_H,	zh

	lds	r16,	WDTCSR			;Сохранить конфигурацию WDT
	sts	WDTCFG,	r16
	ldi	r16,	0b00011000		;Interrupt mode, 64 ms
	sts	WDTCSR,	r16
	ldi	r16,	0b01000000
	sts	WDTCSR,	r16

	clr	r16				;Очистить счетчик принятых байт
	sts	U_RCNT,	r16

	sbi	GPIOR0,	2

usriu:	lds	zl,	U_RX_L
	lds	zh,	U_RX_H
USART_RXC:
	wdr					;Сброс таймера

	sbis	GPIOR0,	2			;Если данные не принимались - настроить указатель
	rjmp	zporx

	lds	r16,	U_RX_L			;Проверка указателя
	lds	r17,	U_RX_H
	cp	r16,	zl
	cpc	r17,	zh
	brne	usriu

	lds	r16,	UDR0			;Забрать полученную информацию с буфера

	lds	r17,	U_RCNT			;Проверка заполненности буфера
	cpi	r17,	8
	breq	rxexi
	inc	r17				;Инкрементировать счетчик принятых байт
	sts	U_RCNT,	r17

	st	z+,	r16			;Сохранить полученную информацию в SRAM

	sts	U_RX_L,	zl			;Сохранить значение указателя
	sts	U_RX_H,	zh
rxexi:	reti

;Обработчик прерывания по успешной передаче информации по USART
zpotx:	ldi	zl,	low(RAMEND-1095)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1095)
	sts	UTXADL,	zl
	sts	UTXADH,	zh

	cbi	GPIOR0,	0			;Убрать флаг "передать данные"
	sbi	GPIOR0,	1			;Поставить флаг "передаются данные"

ustiu:	lds	zl,	UTXADL
	lds	zh,	UTXADH
USART_TXC:
	cli

	lds	r16,	UCSR0A			;Если буфер не пуст - выйти
	sbrs	r16,	UDRE0
	rjmp	txexi

	sbis	GPIOR0,	1			;Если USART не используется - настроить указатель
	rjmp	zpotx

	lds	r16,	UTXADL			;Проверка указателя
	lds	r17,	UTXADH
	cp	r16,	zl
	cpc	r17,	zh
	brne	ustiu

	lds	r25,	UTCNTH			;Обработка счетчика байт
	lds	r24,	UTCNTL

	cp	r24,	r0
	cpc	r25,	r0
	breq	txexif

	sbiw	r24,	1

	sts	UTCNTH,	r25
	sts	UTCNTL,	r24

	ld	r16,	z+			;Отправить информацию в буфер
	sts	UDR0,	r16

	sts	UTXADL,	zl			;Сохранить значение указателя
	sts	UTXADH,	zh
txexi:	reti
txexif:	cbi	GPIOR0,	1			;Убрать флаг "передаются данные"
reti

RESET:	;Инициализация стека
	ldi	r16,	low(RAMEND)
	out	SPL,	r16
	ldi	r16,	high(RAMEND)
	out	SPH,	r16

	;Инициализация портов ввода-вывода
	ldi	r16,	0b00101100		;PB6 - ext. KXO, PB5 - SCK, PB4 - MISO, PB3 - MOSI, PB2 - SCK
	out	DDRB,	r16
	ldi	r16,	0
	out	PORTB,	r16

	ldi	r16,	0b00000010		;PC6 - RESET, pull-up, PC1 - out
	out	DDRC,	r16
	ldi	r16,	0b01000000		
	out	PORTC,	r16

	ldi	r16,	0b11111110		;PD7, PD4...PD2 - slave select, PD6...PD5 - PWM, PD1 - TXD, PD0 - RXD
	out	DDRD,	r16
	ldi	r16,	0b10011100
	out	PORTD,	r16

	;Инициализация USART
	ldi	r16,	0			;Cкорость передачи данных 57.6 kBaud/s (для тактовой 20 МГц)
	sts	UBRR0H,	r16
	ldi	r16,	21
	sts	UBRR0L,	r16

	ldi	r16,	0b11011000		;(1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)
	sts	UCSR0B,	r16

	;Инициализация SPI
	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16

	ldi	r16,	0			;Запретить удвоение скорости прд данных
	out	SPSR,	r16

	;Настройка режима сна (idle)
	ldi	r16,	1
	out	SMCR,	r16

	;Обнуление всех счетчиков
	ldi	r16,	0
	mov	r0,	r16
	sts	U_RCNT,	r0
	sts	UTCNTL,	r0
	sts	UTCNTH,	r0
	sts	SPITCT,	r0
	sts	SPIRCT,	r0
	sts	ADSCTL,	r0
	sts	ADSCTH,	r0

	sei

	;Инициализация регуляторов коэф. усиления
	;AD9833
	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	0			;5k
	st	z+,	r16

	ldi	r16,	1			;Счетчик
	sts	SPITCT,	r16

	cbi	PORTD,	4
	rcall	SPI_STC

	rcall	wspifl

	;ADС6
	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	254			;390
	st	z+,	r16

	ldi	r16,	1			;Счетчик
	sts	SPITCT,	r16

	cbi	PORTD,	2
	rcall	SPI_STC

	rcall	wspifl

	;ADС7
	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса в указатель
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	254			;390
	st	z+,	r16

	ldi	r16,	1			;Счетчик
	sts	SPITCT,	r16

	cbi	PORTD,	3
	rcall	SPI_STC

	rcall	wspifl


	;Инициализация AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16

	;Reset, B28, Sleep1, Sleep12 = 1
	ldi	zl,	low(dbwrs*2)		;Адрес на флеш
	ldi	zh,	high(dbwrs*2)

	ldi	yl,	low(RAMEND-1119)	;Адрес в SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwf0-dbwrs)*2)	;Количество байт
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	;freq0, freq1, phase0, phase1
	ldi	zl,	low(dbwf0*2)		;Адрес на флеш
	ldi	zh,	high(dbwf0*2)

	ldi	yl,	low(RAMEND-1119)	;Адрес в SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwst-dbwf0)*2)	;Количество байт
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	;B28, HLB, Sleep1, Sleep12 = 0, freg0, phreg0, sinus mode
	ldi	zl,	low(dbwst*2)		;Адрес на флеш
	ldi	zh,	high(dbwst*2)

	ldi	yl,	low(RAMEND-1119)	;Адрес в SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwrd-dbwst)*2)	;Количество байт
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16

	;Инициализация ADC
	ldi	r16,	0b00000110		;Источник сигнала - ADC6
	sts	ADMUX,	r16

	ldi	r16,	0;b00000101		;Trigger source - OCR0B
	sts	ADCSRB,	r16

	ldi	r16,	0b10000110		;ADEN = 1, f=fclk/64=312500 (Hz)
	sts	ADCSRA,	r16

	;Инициализация TCNT0
	ldi	r16,	0b10100011		;Fast PWM mode, OC0A, OC0B non-inverting mode
	out	TCCR0A,	r16

	ldi	r16,	0			;Почистить компараторы (на всякий случай)
	out	OCR0A,	r16
	out	OCR0B,	r16

	;Инициализация TCNT1
	ldi	r16,	0
	sts	TCCR1A,	r16
	sts	TCCR1C,	r16

	ldi	r16,	0b00000010
	sts	TIMSK1,	r16

;*******************ГЛАВНЫЙ ЦИКЛ*******************

	wdr
main:	sbic	GPIOR0,	0
	rcall	USART_TXC

	sbic	GPIOR0,	3
	rcall	comhdl

	in	r1,	GPIOR1
	sbrc	r1,	0
	rcall	TIM1_COMPA

	sbrc	r1,	1
	rcall	USART_RXC

	sbic	GPIOR0,	7
	rcall	SPI_STC
rjmp	main

;*******************ПОДПРОГРАММЫ*******************

;Подпрограмма для обработки полученных по UART команд
comhdl:	cli

	cbi	GPIOR0,	3			;Убрать флаг "Данные приняты"

	lds	r16,	U_RCNT
	cpi	r16,	6
	brne	erradr				;Если число принятых байт не 6 - ошибка

	lds	r15,	COMAND
	sbrc	r15,	1
	rjmp	osclsc
	sbrc	r15,	2
	rjmp	wavgen
	sbrc	r15,	3
	rjmp	pwmgen
	sbrc	r15,	5
	rjmp	pinpon
	sbrc	r15,	6
	rjmp	slpcom

errcom:	ldi	zl,	low(RAMEND-1095)	;Загрузка адреса передатчика
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xEC			;Ошибка команды
	st	z,	r16

	ldi	r16,	1			;Байты
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
ret
erradr:	ldi	zl,	low(RAMEND-1095)	;Загрузка адреса передатчика
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xEB			;Ошибка кол-ва байт в команде
	st	z,	r16

	ldi	r16,	1			;Байты
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
reti
;Настройка осциллографа
osclsc:	sbrc	r15,	0			;Определение канала измерения
	rjmp	adc7
	;Настройка на канал ADC6
	ldi	r16,	0b00000110
	sts	ADMUX,	r16
	rjmp	adcset				;Пропустить настройку на канал ADC7
	;Настройка на канал ADC7
adc7:	ldi	r16,	0b00000110
	sts	ADMUX,	r16
	;Настройка коэффициента усиления
adcset:	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса буфера SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG1
	st	z,	r16			;Константа

	ldi	r16,	1
	sts	SPITCT,	r16			;Счетчик байт

	sbrc	r15,	0
	rjmp	ad7cs
	cbi	PORTD,	2			;ADC6 RDAC
	rjmp	ad6cs
ad7cs:	cbi	PORTD,	3			;ADC7 RDAC
ad6cs:	rcall	SPI_STC				;Отправить байт на RDAC
	rcall	wspifl
	cli

	;Загрузка счетчика семплов ADC
	lds	r16,	ARG2
	sts	ADSCTH,	r16			;Старшая часть
	lds	r16,	ARG3
	sts	ADSCTL,	r16			;Младшая часть

	;Определение периода измерения
	lds	r16,	ARG4			;Старшая часть
	lds	r17,	ARG5			;Младшая часть
	sts	OCR1AH,	r16			;Хитрая последовательность записи - младшая часть за старшей.
	sts	OCR1AL,	r17
	in	r16,	GPIOR1			;Поставить флаг "начать измерение"
	ori	r16,	0b00000001
	out	GPIOR1,	r16
reti
;Настройка DDS-генератора
wavgen:	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса буфера SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG1
	st	z+,	r16			;Константа

	ldi	r16,	1
	sts	SPITCT,	r16			;Счетчик байт

	cbi	PORTD,	4			;AD9833 RDAC
	rcall	SPI_STC
	rcall	wspifl
	cli

	;Изменение конфигурации SPI для работы с AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16

	;Настройка режима работы AD9833
	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса буфера SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG2
	st	z+,	r16
	lds	r16,	ARG3
	st	z+,	r16

	ldi	r16,	2
	sts	SPITCT,	r16			;Счетчик байт

	cbi	PORTD,	7			;Настройка режима работы
	rcall	SPI_STC
	rcall	wspifl
	cli

	;Настройка частоты AD9833
	ldi	zl,	low(RAMEND-1119)	;Загрузка адреса буфера SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG4
	ori	r16,	0x40			;Запись частоты в freq0
	st	z+,	r16
	lds	r16,	ARG5
	st	z+,	r16

	ldi	r16,	2
	sts	SPITCT,	r16			;Счетчик байт

	cbi	PORTD,	7			;Настройка частоты
	rcall	SPI_STC
	rcall	wspifl
	cli

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16
reti
;Настройка PWM-генератора
pwmgen:	sbrs	r15,	4			;Если 4-ый бит в командде присутствует - включить генератор
	rjmp	pwmoff

	ldi	r16,	0b00000111		;Почистить флаги перед обновлением
	out	TIFR0,	r16

	lds	r16,	ARG1			;Значения для обоих каналов
	out	OCR0A,	r16
	lds	r16,	ARG2
	out	OCR0B,	r16

	ldi	r16,	0			;Очистить таймер перед запуском
	out	TCNT0,	r16
	ldi	r16,	0b00000010		;ftcnt=fclk/8
	out	TCCR0B,	r16
reti
pwmoff:	ldi	r16,	0			;Отключить тактирование
	out	TCCR0B,	r16
reti
;Отклик
pinpon:	ldi	zl,	low(RAMEND-1095)	;Загрузка адреса передатчика
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xAA
	st	z,	r16

	ldi	r16,	1			;Байты
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
reti
;Сбросить все настройки
slpcom:	ldi	r16,	0			;Отключить тактирование TCNT0
	out	TCCR0B,	r16

	;Изменение конфигурации SPI для работы с AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16
	;B28, HLB, Sleep1, Sleep12 = 0, freg0, phreg0, sinus mode
	ldi	zl,	low(dbwst*2)		;Адрес на флеш
	ldi	zh,	high(dbwst*2)

	ldi	yl,	low(RAMEND-1119)	;Адрес в SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwrd-dbwst)*2)	;Количество байт
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC
	rcall	wspifl
	cli

	;Записать в freq0 значение 0 Гц
	ldi	zl,	low(RAMEND-1119)	;Адрес в SRAM
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	0x40			;0 Гц
	st	z+,	r16
	st	z+,	r0

	ldi	r16,	1			;Счетчик байт
	sts	SPITCT,	r16

	cbi	PORTD,	7
	rcall	SPI_STC
	rcall	wspifl
	cli

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16
reti

;Подпрограмма для задержки по флагу от SPI
wspifl:	sleep					;Спать

	sbis	GPIOR0,	6			;Проверка состояния флага
	rjmp	wspifl

	cbi	GPIOR0,	6			;Если передача завершена - очистить флаг
ret

;Копировалка массивов из флэша в СОЗУ
fscopy:	dec	r14				;Декрементировать счетчик

	lpm	r16,	z+			;Копирование данные
	st	y+,	r16

	tst	r14				;Условный переход
	brne	fscopy
ret

;*******************МАССИВЫ*******************

dbwrs:	.db	0x21,	0xC0			;Reset, B28, Sleep1, Sleep12 = 1
dbwf0:	.db	0x40,	0x00,	0x40,	0x00	;freg0 = 0 Hz (0)
dbwf1:	.db	0xB4,	0x6E,	0x80,	0x00	;freg1 = 1000 Hz (13422)
dbwp0:	.db	0xD0,	0x00			;phase0 = 0 deg (0)
dbwp1:	.db	0xF4,	0x00			;phase1 = pi/2 deg (1024)
dbwst:	.db	0x00,	0x00			;B28, HLB, Sleep1, Sleep12 = 0, freg0, phreg0, sinus mode
dbwrd:	.db	0x48,	0x65,	0x6c,	0x6c	;Hello, world!
	.db	0x6f,	0x2c,	0x20,	0x77
	.db	0x6f,	0x72,	0x6c,	0x64
	.db	0x21,	0x0d,	0x0a,	0x00
dbend:
