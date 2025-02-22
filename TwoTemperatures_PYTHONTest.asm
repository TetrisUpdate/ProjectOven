; 76E003 ADC test program: Reads channel 7 on P1.1, pin 14
; This version uses the LM4040 voltage reference connected to pin 6 (P1.7/AIN0)

$NOLIST
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;

CLK               EQU 16600000 ; Microcontroller system frequency in Hz
BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD EQU (0x10000-(CLK/TIMER0_DENOM))

SAMPLES_PER_DISPLAY equ 150
REFRESHES_PER_SECOND equ 15 ;Does not work properly for high number of samples/display, actual refreshes/sec is less than value given if samples/display is higher than around 255
TIMER0_DENOM equ (SAMPLES_PER_DISPLAY*REFRESHES_PER_SECOND)

ORG 0x0000
	ljmp main

;                     1234567890123456    <- This helps determine the location of the counter
test_message:     db 'Current Temp.:', 0
value_message:    db 'Deg. C', 0
cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3
SSR_BOX equ P0.5


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; These register definitions needed by 'math32.inc'
DSEG at 30H
x:   ds 4
y:   ds 4
bcd: ds 5
VAL_LM4040: ds 2

StoreMeasurements: ds 4
TempStore: ds 2
MeasurementCounter: ds 2
SamplesPerDisplay: ds 2
TimePerSample: ds 1
LastMeasurement: ds 4
StoreThermocouple: ds 4
FinalLM335: ds 4

BSEG
mf: dbit 1

$NOLIST
$include(math32.inc)
$LIST

Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10000001 ; Activate AIN0 and AIN7 analog inputs
	orl ADCCON1, #0x01 ; Enable ADC
	
	ret
	
wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD)
	mov	TL0,#low(TIMER0_RELOAD)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret

; We can display a number any way we want.  In this case with
; four decimal places.
Display_formated_BCD:
	Set_Cursor(2, 1)
	Display_BCD(bcd+2)
	Display_BCD(bcd+1)
	Display_char(#'.')
	Display_BCD(bcd+0)
	Display_char(#0xDF)
	Display_char(#'C')
	Set_Cursor(2,1)
	Display_char(#' ')
	ret

Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret

main:
	mov sp, #0x7f
	lcall Init_All
    lcall LCD_4BIT

	mov MeasurementCounter+0, #1
	mov MeasurementCounter+1, #0

	mov TimePerSample, #1

	mov SamplesPerDisplay+0, #low(SAMPLES_PER_DISPLAY)
	mov SamplesPerDisplay+1, #high(SAMPLES_PER_DISPLAY)
	
	mov LastMeasurement+0, #0
	mov LastMeasurement+1, #0
	mov LastMeasurement+2, #0
	mov LastMeasurement+3, #0
    
    ; initial messages in LCD
	Set_Cursor(1, 1)
    Send_Constant_String(#test_message)
	ljmp Forever

SendBCD:

	mov a, bcd+2
	anl a, #0x0F ; Isolate ones place
	add a, #'0' ; Convert value to ASCII
	lcall SendSerial
	
	mov a, bcd+1
	anl a, #0xF0 ; Isolate tens place
	swap a ; Put high nibble into lower nibble
	add a, #'0' ; Convert value to ASCII
	lcall SendSerial

	mov a, bcd+1
	anl a, #0x0F ; Isolate ones place
	add a, #'0' ; Convert value to ASCII
	lcall SendSerial

	mov a, #'.'
	lcall SendSerial

	mov a, bcd+0
	anl a, #0xF0 ; Isolate 0.1 place
	swap a ; Put high nibble into lower nibble
	add a, #'0' ; Convert value to ASCII
	lcall SendSerial

	mov a, bcd+0
	anl a, #0x0F ; Isolate 0.01 place
	add a, #'0' ; Convert value to ASCII
	lcall SendSerial

	mov a, #'\n'
	lcall SendSerial

	mov a, #'\r'
	lcall SendSerial

	ret

SendSerial:
	clr TI
	mov SBUF, a
	jnb TI, $
	ret

Forever:

	; Read the 4.096V LM4040 voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VAL_LM4040+0, R0
	mov VAL_LM4040+1, R1

	; Read the signal connected to AIN7 (This reads the LM335)
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40959) ; The MEASURED voltage reference: 4.0959V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LM4040 value
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32


	mov y+0, StoreMeasurements+0
	mov y+1, StoreMeasurements+1
	mov y+2, StoreMeasurements+2
	mov y+3, StoreMeasurements+3

	lcall add32
	
	mov StoreMeasurements+0, x+0
	mov StoreMeasurements+1, x+1
	mov StoreMeasurements+2, x+2
	mov StoreMeasurements+3, x+3
	


	; Read the signal connected to AIN4 (this reads analog signal of the OP07/Thermocouple)
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x04 ; Select channel 4
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40959) ; The MEASURED voltage reference: 4.0959V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LM4040 value
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32

	Load_y(1000000)
    lcall mul32

    Load_y(243)
    lcall div32

    Load_y(4100)
    lcall div32


	mov y+0, StoreThermocouple+0
	mov y+1, StoreThermocouple+1
	mov y+2, StoreThermocouple+2
	mov y+3, StoreThermocouple+3

	lcall add32
	
	mov StoreThermocouple+0, x+0
	mov StoreThermocouple+1, x+1
	mov StoreThermocouple+2, x+2
	mov StoreThermocouple+3, x+3

	mov R2, TimePerSample
	lcall waitms

	;Checks if enough measurements have been taken
	
	dec MeasurementCounter+0
	mov a, MeasurementCounter+0
	cjne a, #0xFF, CheckHigh
	dec MeasurementCounter+1
	CheckHigh:
	mov a, MeasurementCounter+0
	orl a, MeasurementCounter+1
	jz DisplayValue
	ljmp EndForever
	
	;Divides stored measurements by number of measurements taken
DisplayValue:	
	Load_y(0)
	
	mov x+0, StoreMeasurements+0
	mov x+1, StoreMeasurements+1
	mov x+2, StoreMeasurements+2
	mov x+3, StoreMeasurements+3
	
	mov a, SamplesPerDisplay+0
	mov y+0, a
	mov MeasurementCounter+0, a

	mov a, SamplesPerDisplay+1
	mov y+1, a
	mov MeasurementCounter+1, a	
	
	lcall div32
	
	;Load_y(100)
	;lcall mul32

	load_y(27300)
	lcall sub32
	
	mov FinalLM335+0, x+0 ;Stores final LM335 value in degrees C
	mov FinalLM335+1, x+1
	mov FinalLM335+2, x+2
	mov FinalLM335+3, x+3
	
	;Does same as above, but for the thermocouple
	
	Load_y(0)
	
	mov x+0, StoreThermocouple+0
	mov x+1, StoreThermocouple+1
	mov x+2, StoreThermocouple+2
	mov x+3, StoreThermocouple+3
	
	mov a, SamplesPerDisplay+0
	mov y+0, a
	mov MeasurementCounter+0, a

	mov a, SamplesPerDisplay+1
	mov y+1, a
	mov MeasurementCounter+1, a	
	
	lcall div32
	
	;Load_y(100)
	;lcall mul32 ;Note that the final thermocouple value in degrees C is now stored in x
	
	;Load_y(0)
	;mov y+0, FinalLM335+0
	;mov y+1, FinalLM335+1
	;mov y+2, FinalLM335+2
	;mov y+3, FinalLM335+3

	load_y(2200)

	lcall add32 ; Puts the final temperature value into x
	
	
	; Convert to BCD and display
	lcall hex2bcd
	lcall Display_formated_BCD

	lcall SendBCD

	mov StoreMeasurements+0, #0
	mov StoreMeasurements+1, #0
	mov StoreMeasurements+2, #0
	mov StoreMeasurements+3, #0
	
	mov StoreThermocouple+0, #0
	mov StoreThermocouple+1, #0
	mov StoreThermocouple+2, #0
	mov StoreThermocouple+3, #0
	
	mov FinalLM335+0, #0
	mov FinalLM335+1, #1
	mov FinalLM335+2, #2
	mov FinalLM335+3, #3


EndForever:
	mov x+0, #0
	mov x+1, #0
	mov x+2, #0
	mov x+3, #0
	ljmp Forever

END
	