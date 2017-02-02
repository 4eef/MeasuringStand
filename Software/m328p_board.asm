;���������� ������ ��� ������������� �������������� ������ �� �� ATmega328P

.INCLUDE "m328Pdef.inc"

;���������� (RAMEND-71...RAMEND-8, 64 �����)
.equ	U_RX_H	= RAMEND-71			;������ ���������� USART
.equ	U_RX_L	= RAMEND-70
.equ	UTXADH	= RAMEND-69			;������ ������� - ����� ��������� ������ ADC (ADC � TX UART �� ����� ���� ������������ ������������)
.equ	UTXADL	= RAMEND-68

.equ	U_RCNT	= RAMEND-67			;�������� ���� ��� ���/��� USART
.equ	UTCNTH	= RAMEND-66
.equ	UTCNTL	= RAMEND-65

.equ	WDTCFG	= RAMEND-65			;��������� ���������� ������������ WDT

.equ	SPITRH	= RAMEND-64			;����� ��������� ���/��� SPI
.equ	SPITRL	= RAMEND-63

.equ	SPITCT	= RAMEND-62			;������� ���� ��� ��� SPI
.equ	SPIRCT	= RAMEND-61			;������� ���� ��� ��� SPI

.equ	ADSCTH	= RAMEND-60			;������� ������� ADC
.equ	ADSCTL	= RAMEND-59

;����� ������ � ����������� �� USART:
;��� ��������� � ���������� ������������ ������� �� ����� ���� (���� ����� ������� �� ��������
;���������� - ���������� ������������ ���� ����� �����). ���� ������� ������ (������) ����������,
;� ����� �� ������� ����� ��������� ���� 0xEB, �� ����, ������������ ����� ���� (error: byte).
;���� ������ ���� � ������� (�������) �� �������� ������ �� ����������������� ���������, �����
;������ �����, ��������� �� ����� 0xEC (error: comand).
;1 ���� - �������. ���������� (�����������) �������:
;	- 0b00000010, 0b00000011 - ���������������� �����������, 0 ��� 1 ����� ��������������
;	- 0b00000100 - ���������������� ���������� �������
;	- 0b00001000, 0b00011000 - ���������������� ���-���������, ����. ��� ���. ��������������
;	- 0b00100000 - ping. � ����� ����� ������ ���� 0xAA
;	- 0b01000000 - �������� ��������� DDS- � ���-�����������
;��� ���������������� ������������ ��������� ����� ��������� ����������:
;1 ���� - ����������� �������� ��� ��������� ������ (0...255)
;2:3 ����� (2 ���� �������) - ���������� ���������� ������� (0...512)
;4:5 ����� (4 ���� �������) - ��������� ��������� ��������� (15...65535).  T=3,2*N (���)
;��� ���������������� ����������� �������:
;1 ���� - ����������� �������� ��������������� ��������� (0...255)
;2:3 ����� (2 ���� �������) - ����� ������ DDS-����������. ��������� �� �������� (�� ��������� 0x0000)
;4:5 ����� (4 ���� �������) - ��������� ������� ��� ��������� �������. F=20*10^6*N/2^28
;��� ���������������� ���-����������:
;1 ���� - ����� ��� ������ OCR0A (0...255)
;2 ���� - ����� ��� ������ OCR0B (0...255)

;������ ��� ����������� ������
.equ	COMAND	= RAMEND-1103			;�������
.equ	ARG1	= RAMEND-1102			;���������
.equ	ARG2	= RAMEND-1101
.equ	ARG3	= RAMEND-1100
.equ	ARG4	= RAMEND-1099
.equ	ARG5	= RAMEND-1098

;������������� ��������� �����-������ ��� �����:
;GPIOR0:
;0 ��� - �������� ������ �� USART
;1 ��� - ���������� ������ �� USART
;2 ��� - ����������� ������ �� USART
;3 ��� - ������ �� USART �������
;4 ��� - ������������ SPI
;5 ��� - ����������� ������ �� SPI
;6 ��� - �������� �� SPI ���������
;7 ��� - �������� ������ �� SPI
;GPIOR1:
;0 ��� - ��������� ��������� ADC
;1 ��� - ������� ���������� �������� USART (��� �������)
;2 ��� - 
;3 ��� - 
;4 ��� - 
;5 ��� - 
;6 ��� - 
;7 ��� - 

;������� ����������
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

;*******************����������� ����������*******************

;���������� ���������� �� WDT (���������� ������������ ��� �������)
WDT:	lds	r17,	WDTCFG			;������������ ������� ������������ WDT
	ldi	r16,	0b00011000
	sts	WDTCSR,	r16
	sts	WDTCSR,	r17

	cbi	GPIOR0,	2
	sbi	GPIOR0,	3
reti

;���������� ���������� �� ���������� TCNT1 � ����������
TIM1_COMPA:
	cli

	lds	r17,	ADCSRA			;r17 � ���� ����������� - ������������� ������ �����!

	sbrs	r17,	4			;���� ��� ����� ���������� �� ��� - ���������
	rjmp	stcnv

	ldi	r16,	0b00000010		;������� (��� �������)
	out	PINC,	r16

	lds	r23,	UTXADL			;�������� ���������
	lds	r22,	UTXADH
	cp	r23,	zl
	cpc	r22,	zh
	in	r16,	SREG
	sbrc	r16,	1
	rjmp	cmeas
	lds	zl,	UTXADL			;����������� ��������� �� ������
	lds	zh,	UTXADH

cmeas:	lds	r27,	ADSCTH			;������� �������
	lds	r26,	ADSCTL
	lds	r25,	UTCNTH			;������� ���� (��� UART)
	lds	r24,	UTCNTL

	cp	r26,	r0
	cpc	r27,	r0
	breq	adcend

	sbiw	r26,	1			;���������� �������

	cpi	r25,	0x04
	breq	adcend

	adiw	r24,	2			;���������� ����

	sts	ADSCTH,	r27
	sts	ADSCTL,	r26
	sts	UTCNTH,	r25
	sts	UTCNTL,	r24

	lds	r19,	ADCL			;��������� ������
	lds	r18,	ADCH
	st	z+,	r18
	st	z+,	r19

	sts	UTXADL,	zl			;��������� �������� ���������
	sts	UTXADH,	zh

	ori	r17,	0b01010000		;������ �������������� � ������ ����
	sts	ADCSRA,	r17
reti
;���������� ��� � �������
adcend:	ori	r17,	0b00010000		;������ ���� ����������
	sts	ADCSRA,	r17

	ldi	r16,	0b00001000		;���������� ������
	sts	TCCR1B,	r16

	sbi	GPIOR0,	0			;���� �� �������� ������ �� UART

	sts	ADSCTH,	r0			;�������� ���� � ������� �������
	sts	ADSCTL,	r0
reti
;������������� ���������
stcnv:	ldi	zl,	low(RAMEND-1095)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1095)
	sts	UTXADL,	zl
	sts	UTXADH,	zh

	in	r16,	GPIOR1			;������ ���� "������ ���������"
	andi	r16,	0b11111110
	out	GPIOR1,	r16

	sts	TCNT1H,	r0			;�������� ������
	sts	TCNT1L,	r0
	ldi	r16,	0b00001011		;f = fclk/64, CTC mode

	ori	r17,	0b01000000		;������ ��������������
	sts	ADCSRA,	r17

	sts	TCCR1B,	r16
reti

;���������� ���������� �� �������� �������� ���������� �� SPI
zpotrx:	ldi	zl,	low(RAMEND-1119)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1119)
	sts	SPITRL,	zl
	sts	SPITRH,	zh

	cbi	GPIOR0,	7			;������ ���� "�������� ������"

	lds	r15,	SPITCT			;���� ������� ���� - �����
	tst	r15
	breq	spiex

	sbi	GPIOR0,	4			;��������� ���� "������������ SPI"

spiiu:	lds	zl,	SPITRL
	lds	zh,	SPITRH
SPI_STC:
	cli

	sbis	GPIOR0,	4			;���� SPI �� ������������ - ��������� ���������
	rjmp	zpotrx

	lds	r16,	SPITRL			;�������� ���������
	lds	r17,	SPITRH
	cp	r16,	zl
	cpc	r17,	zh
	brne	spiiu

	lds	r15,	SPITCT			;���� ������� ���� - ����� � ���������� ��������� CS
	tst	r15
	breq	spioff
	dec	r15
	sts	SPITCT,	r15

	in	r17,	SPDR			;��������� ������ � ������

	ld	r16,	z			;�������� ������
	out	SPDR,	r16

	sbis	GPIOR0,	5			;���� ������ ����������� - �������� �������� ���������� � �����
	rjmp	spit

	lds	r16,	SPIRCT			;���� ���� �������� - ����� � ���������� ��������� CS
	cpi	r16,	8
	breq	spioff

	inc	r16
	sts	SPIRCT,	r16

	st	z+,	r17

spit:	sbis	GPIOR0,	5			;����������������� ��������� �������, ���� ������ ����������
	adiw	z,	1

	sts	SPITRL,	zl			;��������� �������� ���������
	sts	SPITRH,	zh
reti
spioff:	sbi	GPIOR0,	6			;�������� ���������

spiex:	in	r16,	PIND
	ori	r16,	0b10011100
	out	PORTD,	r16

	cbi	GPIOR0,	4			;�������� ����
reti

;���������� ���������� �� ��������� ����� ���������� �� USART
zporx:	ldi	zl,	low(RAMEND-1103)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1103)
	sts	U_RX_L,	zl
	sts	U_RX_H,	zh

	lds	r16,	WDTCSR			;��������� ������������ WDT
	sts	WDTCFG,	r16
	ldi	r16,	0b00011000		;Interrupt mode, 64 ms
	sts	WDTCSR,	r16
	ldi	r16,	0b01000000
	sts	WDTCSR,	r16

	clr	r16				;�������� ������� �������� ����
	sts	U_RCNT,	r16

	sbi	GPIOR0,	2

usriu:	lds	zl,	U_RX_L
	lds	zh,	U_RX_H
USART_RXC:
	wdr					;����� �������

	sbis	GPIOR0,	2			;���� ������ �� ����������� - ��������� ���������
	rjmp	zporx

	lds	r16,	U_RX_L			;�������� ���������
	lds	r17,	U_RX_H
	cp	r16,	zl
	cpc	r17,	zh
	brne	usriu

	lds	r16,	UDR0			;������� ���������� ���������� � ������

	lds	r17,	U_RCNT			;�������� ������������� ������
	cpi	r17,	8
	breq	rxexi
	inc	r17				;���������������� ������� �������� ����
	sts	U_RCNT,	r17

	st	z+,	r16			;��������� ���������� ���������� � SRAM

	sts	U_RX_L,	zl			;��������� �������� ���������
	sts	U_RX_H,	zh
rxexi:	reti

;���������� ���������� �� �������� �������� ���������� �� USART
zpotx:	ldi	zl,	low(RAMEND-1095)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1095)
	sts	UTXADL,	zl
	sts	UTXADH,	zh

	cbi	GPIOR0,	0			;������ ���� "�������� ������"
	sbi	GPIOR0,	1			;��������� ���� "���������� ������"

ustiu:	lds	zl,	UTXADL
	lds	zh,	UTXADH
USART_TXC:
	cli

	lds	r16,	UCSR0A			;���� ����� �� ���� - �����
	sbrs	r16,	UDRE0
	rjmp	txexi

	sbis	GPIOR0,	1			;���� USART �� ������������ - ��������� ���������
	rjmp	zpotx

	lds	r16,	UTXADL			;�������� ���������
	lds	r17,	UTXADH
	cp	r16,	zl
	cpc	r17,	zh
	brne	ustiu

	lds	r25,	UTCNTH			;��������� �������� ����
	lds	r24,	UTCNTL

	cp	r24,	r0
	cpc	r25,	r0
	breq	txexif

	sbiw	r24,	1

	sts	UTCNTH,	r25
	sts	UTCNTL,	r24

	ld	r16,	z+			;��������� ���������� � �����
	sts	UDR0,	r16

	sts	UTXADL,	zl			;��������� �������� ���������
	sts	UTXADH,	zh
txexi:	reti
txexif:	cbi	GPIOR0,	1			;������ ���� "���������� ������"
reti

RESET:	;������������� �����
	ldi	r16,	low(RAMEND)
	out	SPL,	r16
	ldi	r16,	high(RAMEND)
	out	SPH,	r16

	;������������� ������ �����-������
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

	;������������� USART
	ldi	r16,	0			;C������� �������� ������ 57.6 kBaud/s (��� �������� 20 ���)
	sts	UBRR0H,	r16
	ldi	r16,	21
	sts	UBRR0L,	r16

	ldi	r16,	0b11011000		;(1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)
	sts	UCSR0B,	r16

	;������������� SPI
	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16

	ldi	r16,	0			;��������� �������� �������� ��� ������
	out	SPSR,	r16

	;��������� ������ ��� (idle)
	ldi	r16,	1
	out	SMCR,	r16

	;��������� ���� ���������
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

	;������������� ����������� ����. ��������
	;AD9833
	ldi	zl,	low(RAMEND-1119)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	0			;5k
	st	z+,	r16

	ldi	r16,	1			;�������
	sts	SPITCT,	r16

	cbi	PORTD,	4
	rcall	SPI_STC

	rcall	wspifl

	;AD�6
	ldi	zl,	low(RAMEND-1119)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	254			;390
	st	z+,	r16

	ldi	r16,	1			;�������
	sts	SPITCT,	r16

	cbi	PORTD,	2
	rcall	SPI_STC

	rcall	wspifl

	;AD�7
	ldi	zl,	low(RAMEND-1119)	;�������� ������ � ���������
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	254			;390
	st	z+,	r16

	ldi	r16,	1			;�������
	sts	SPITCT,	r16

	cbi	PORTD,	3
	rcall	SPI_STC

	rcall	wspifl


	;������������� AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16

	;Reset, B28, Sleep1, Sleep12 = 1
	ldi	zl,	low(dbwrs*2)		;����� �� ����
	ldi	zh,	high(dbwrs*2)

	ldi	yl,	low(RAMEND-1119)	;����� � SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwf0-dbwrs)*2)	;���������� ����
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	;freq0, freq1, phase0, phase1
	ldi	zl,	low(dbwf0*2)		;����� �� ����
	ldi	zh,	high(dbwf0*2)

	ldi	yl,	low(RAMEND-1119)	;����� � SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwst-dbwf0)*2)	;���������� ����
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	;B28, HLB, Sleep1, Sleep12 = 0, freg0, phreg0, sinus mode
	ldi	zl,	low(dbwst*2)		;����� �� ����
	ldi	zh,	high(dbwst*2)

	ldi	yl,	low(RAMEND-1119)	;����� � SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwrd-dbwst)*2)	;���������� ����
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC

	rcall	wspifl

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16

	;������������� ADC
	ldi	r16,	0b00000110		;�������� ������� - ADC6
	sts	ADMUX,	r16

	ldi	r16,	0;b00000101		;Trigger source - OCR0B
	sts	ADCSRB,	r16

	ldi	r16,	0b10000110		;ADEN = 1, f=fclk/64=312500 (Hz)
	sts	ADCSRA,	r16

	;������������� TCNT0
	ldi	r16,	0b10100011		;Fast PWM mode, OC0A, OC0B non-inverting mode
	out	TCCR0A,	r16

	ldi	r16,	0			;��������� ����������� (�� ������ ������)
	out	OCR0A,	r16
	out	OCR0B,	r16

	;������������� TCNT1
	ldi	r16,	0
	sts	TCCR1A,	r16
	sts	TCCR1C,	r16

	ldi	r16,	0b00000010
	sts	TIMSK1,	r16

;*******************������� ����*******************

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

;*******************������������*******************

;������������ ��� ��������� ���������� �� UART ������
comhdl:	cli

	cbi	GPIOR0,	3			;������ ���� "������ �������"

	lds	r16,	U_RCNT
	cpi	r16,	6
	brne	erradr				;���� ����� �������� ���� �� 6 - ������

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

errcom:	ldi	zl,	low(RAMEND-1095)	;�������� ������ �����������
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xEC			;������ �������
	st	z,	r16

	ldi	r16,	1			;�����
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
ret
erradr:	ldi	zl,	low(RAMEND-1095)	;�������� ������ �����������
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xEB			;������ ���-�� ���� � �������
	st	z,	r16

	ldi	r16,	1			;�����
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
reti
;��������� ������������
osclsc:	sbrc	r15,	0			;����������� ������ ���������
	rjmp	adc7
	;��������� �� ����� ADC6
	ldi	r16,	0b00000110
	sts	ADMUX,	r16
	rjmp	adcset				;���������� ��������� �� ����� ADC7
	;��������� �� ����� ADC7
adc7:	ldi	r16,	0b00000110
	sts	ADMUX,	r16
	;��������� ������������ ��������
adcset:	ldi	zl,	low(RAMEND-1119)	;�������� ������ ������ SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG1
	st	z,	r16			;���������

	ldi	r16,	1
	sts	SPITCT,	r16			;������� ����

	sbrc	r15,	0
	rjmp	ad7cs
	cbi	PORTD,	2			;ADC6 RDAC
	rjmp	ad6cs
ad7cs:	cbi	PORTD,	3			;ADC7 RDAC
ad6cs:	rcall	SPI_STC				;��������� ���� �� RDAC
	rcall	wspifl
	cli

	;�������� �������� ������� ADC
	lds	r16,	ARG2
	sts	ADSCTH,	r16			;������� �����
	lds	r16,	ARG3
	sts	ADSCTL,	r16			;������� �����

	;����������� ������� ���������
	lds	r16,	ARG4			;������� �����
	lds	r17,	ARG5			;������� �����
	sts	OCR1AH,	r16			;������ ������������������ ������ - ������� ����� �� �������.
	sts	OCR1AL,	r17
	in	r16,	GPIOR1			;��������� ���� "������ ���������"
	ori	r16,	0b00000001
	out	GPIOR1,	r16
reti
;��������� DDS-����������
wavgen:	ldi	zl,	low(RAMEND-1119)	;�������� ������ ������ SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG1
	st	z+,	r16			;���������

	ldi	r16,	1
	sts	SPITCT,	r16			;������� ����

	cbi	PORTD,	4			;AD9833 RDAC
	rcall	SPI_STC
	rcall	wspifl
	cli

	;��������� ������������ SPI ��� ������ � AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16

	;��������� ������ ������ AD9833
	ldi	zl,	low(RAMEND-1119)	;�������� ������ ������ SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG2
	st	z+,	r16
	lds	r16,	ARG3
	st	z+,	r16

	ldi	r16,	2
	sts	SPITCT,	r16			;������� ����

	cbi	PORTD,	7			;��������� ������ ������
	rcall	SPI_STC
	rcall	wspifl
	cli

	;��������� ������� AD9833
	ldi	zl,	low(RAMEND-1119)	;�������� ������ ������ SPI
	ldi	zh,	high(RAMEND-1119)

	lds	r16,	ARG4
	ori	r16,	0x40			;������ ������� � freq0
	st	z+,	r16
	lds	r16,	ARG5
	st	z+,	r16

	ldi	r16,	2
	sts	SPITCT,	r16			;������� ����

	cbi	PORTD,	7			;��������� �������
	rcall	SPI_STC
	rcall	wspifl
	cli

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16
reti
;��������� PWM-����������
pwmgen:	sbrs	r15,	4			;���� 4-�� ��� � �������� ������������ - �������� ���������
	rjmp	pwmoff

	ldi	r16,	0b00000111		;��������� ����� ����� �����������
	out	TIFR0,	r16

	lds	r16,	ARG1			;�������� ��� ����� �������
	out	OCR0A,	r16
	lds	r16,	ARG2
	out	OCR0B,	r16

	ldi	r16,	0			;�������� ������ ����� ��������
	out	TCNT0,	r16
	ldi	r16,	0b00000010		;ftcnt=fclk/8
	out	TCCR0B,	r16
reti
pwmoff:	ldi	r16,	0			;��������� ������������
	out	TCCR0B,	r16
reti
;������
pinpon:	ldi	zl,	low(RAMEND-1095)	;�������� ������ �����������
	ldi	zh,	high(RAMEND-1095)

	ldi	r16,	0xAA
	st	z,	r16

	ldi	r16,	1			;�����
	sts	UTCNTL,	r16

	sbi	GPIOR0,	0
reti
;�������� ��� ���������
slpcom:	ldi	r16,	0			;��������� ������������ TCNT0
	out	TCCR0B,	r16

	;��������� ������������ SPI ��� ������ � AD9833
	ldi	r16,	0b11011001		;SPIE, SPI = 1, MSB first, Master mode, CPOL = 1, CPHA = 0, fosc/16
	out	SPCR,	r16
	;B28, HLB, Sleep1, Sleep12 = 0, freg0, phreg0, sinus mode
	ldi	zl,	low(dbwst*2)		;����� �� ����
	ldi	zh,	high(dbwst*2)

	ldi	yl,	low(RAMEND-1119)	;����� � SRAM
	ldi	yh,	high(RAMEND-1119)

	ldi	r16,	((dbwrd-dbwst)*2)	;���������� ����
	mov	r14,	r16
	sts	SPITCT,	r16

	rcall fscopy

	cbi	PORTD,	7
	rcall	SPI_STC
	rcall	wspifl
	cli

	;�������� � freq0 �������� 0 ��
	ldi	zl,	low(RAMEND-1119)	;����� � SRAM
	ldi	zh,	high(RAMEND-1119)

	ldi	r16,	0x40			;0 ��
	st	z+,	r16
	st	z+,	r0

	ldi	r16,	1			;������� ����
	sts	SPITCT,	r16

	cbi	PORTD,	7
	rcall	SPI_STC
	rcall	wspifl
	cli

	ldi	r16,	0b11010001		;SPIE, SPI = 1, MSB first, Master mode, CPOL, CPHA = 0, fosc/16
	out	SPCR,	r16
reti

;������������ ��� �������� �� ����� �� SPI
wspifl:	sleep					;�����

	sbis	GPIOR0,	6			;�������� ��������� �����
	rjmp	wspifl

	cbi	GPIOR0,	6			;���� �������� ��������� - �������� ����
ret

;����������� �������� �� ����� � ����
fscopy:	dec	r14				;���������������� �������

	lpm	r16,	z+			;����������� ������
	st	y+,	r16

	tst	r14				;�������� �������
	brne	fscopy
ret

;*******************�������*******************

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
