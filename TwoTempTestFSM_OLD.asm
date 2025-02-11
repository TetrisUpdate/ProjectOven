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
TIMER2_RATE   EQU 100; 100Hz, for a timer tick of 10ms
TIMER2_RELOAD EQU ((65536-(CLK/TIMER2_RATE)))

SAMPLES_PER_DISPLAY equ 300 
REFRESHES_PER_SECOND equ 45 ;Does not work properly for high number of samples/display, actual refreshes/sec is less than value given if samples/display is higher than around 255
TIMER0_DENOM equ (SAMPLES_PER_DISPLAY*REFRESHES_PER_SECOND)

ORG 0x0000
	ljmp main

org 0x002B
	ljmp Timer2_ISR

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
SSR_BOX equ P0.4


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; These register definitions needed by 'math32.inc'
DSEG at 30H
x:   ds 4
y:   ds 4
bcd: ds 5
VAL_LM4040: ds 2

state: ds 1
StoreMeasurements: ds 4
TempStore: ds 2
MeasurementCounter: ds 2
SamplesPerDisplay: ds 2
TimePerSample: ds 1
LastMeasurement: ds 4
StoreThermocouple: ds 4
CurrentTemp: ds 4
FinalLM335: ds 4

FinalTemp: ds 4

Count1ms: ds 2
pwm_counter: ds 1
pwm: ds 1

; oven settings
temp_soak: ds 1 
time_soak: ds 1 
temp_refl: ds 1 
time_refl: ds 1

seconds:  ds 1

BSEG
mf: dbit 1

start: dbit 1
temp_state1: dbit 1 ; ramp to soak
time_state2: dbit 1 ; preheat/soak
temp_state3: dbit 1 ; ramp to peak
time_state4: dbit 1 ; reflow
temp_state5: dbit 1 ; cooling if temp less than 60

$NOLIST
$include(math32.inc)
$LIST

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;


;Use a flag for when the display should update with the current state/time (not needed for temperature updates, done automatically)
;If flag is set to true, some code in the "Forever" loop should run some code that updates the display (could be done in a separate subroutine, "Forever" calls it)
; 



Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	orl T2MOD, #0x80 ; Enable timer 2 autoreload
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
	; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Enable the timer and interrupts
	orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    setb TR2  ; Enable timer 2
	ret

Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1

	;PWM for reflow oven controller
	inc pwm_counter
	clr c
	mov a, pwm
	subb a, pwm_counter ; If pwm_counter <= pwm then c=1
	cpl c
	mov SSR_BOX, c

	mov a, pwm_counter
	cjne a, #100, Timer2_ISR_done
	mov pwm_counter, #0
	inc seconds ; It is super easy to keep a seconds count here
	; setb s_flag (don't know what to do with this)


Inc_Done:
	; Check if half second has passed
	
	mov a, Count1ms+0
	cjne a, #low(1000), State_0 ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), State_0
	

	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb half_seconds_flag ; Let the main program know half second had passed



State_0:
	cjne state, #0, State_1
	mov pwm, #0
	cjne start, #1, Timer2_ISR_done
	mov state, #1
	ljmp Timer2_ISR_done
	
State_1:
	cjne state, #1, State_2
	mov pwm, #100
	cjne temp_state1, #0, Timer2_ISR_done ; temp_lt_150 should be compared to zero b/c the bit will be set to 0 when the temp is greater than 150, which is when we want to transition to the next state
	mov seconds, #0
	mov state, #2

State_2: ;transition to state three if more than 60 seconds have passed
	cjne state, #2, State_3
	mov pwm, #20
	cjne time_state2, #0, Timer2_ISR_done
	mov seconds, #0		 	
	mov state, #3

State_3: 
	cjne state, #3, State_4 ; check if state = 3, if not, move to state_4
	mov pwm, #100 ; set pwm to 100%
	cjne temp_state3, #0, Timer2_ISR_done ; 
	mov seconds, #0
	mov state, #4

State_4:
	cjne state, #4, State_5
	mov pwm, #20
	cjne time_state4, #0, Timer2_ISR_done
	mov seconds, #0
	mov state, #5

State_5:
	cjne state, #5, Timer2_ISR_done
	mov pwm, #0
	cjne temp_state5, #1, Timer2_ISR_done
	mov seconds, #0
	mov state, #0

State_error:
	mov state, #0
	
Timer2_ISR_done:
	pop psw
	pop acc
	reti

Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00

	lcall Timer2_Init
	
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
	Display_char(#' ') ; Used to remove 0, better solution is to change  x40959 to x409590 (in "Forever")
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

	mov state, #0

	mov start, #0
	mov temp_lt_150, #0
	mov sec_lt_60, #0
	mov temp_lt_220, #0
	mov temp_gt_250, #0
	mov sec_lt_45, #0
	mov temp_gt_60, #0
	
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
	
	Load_y(0)
	mov y+0, FinalLM335+0
	mov y+1, FinalLM335+1
	mov y+2, FinalLM335+2
	mov y+3, FinalLM335+3

	
	lcall add32 ; Puts the final temperature value into x

	mov FinalTemp+0, x+0
	mov FinalTemp+1, x+1
	mov FinalTemp+2, x+2
	mov FinalTemp+3, x+3

	;CHANGE THESE SO THAT WE MOVE THE VARIABLES INTO Y ONCE THEYRE SET UP, NOT JUST MOVING CONSTANTS INTO Y
	Load_y(100)

	mov x+0, temp_soak
	mov x+1, #0
	mov x+2, #0
	mov x+3, #0
	lcall mul32 ; Multiplies by 100 so we can compare with FinalTemp properly
	mov y+0, FinalTemp+0
	mov y+1, FinalTemp+1 ; Move final temperature measurement into y
	mov y+2, FinalTemp+2
	mov y+3, FinalTemp+3
	lcall x_gteq_y
	mov temp_gt_150, mf ; If temp_soak >= FinalTemp, = 1

	Load_y(100)
	mov x+0, temp_refl
	mov x+1, #0
	mov x+2, #0
	mov x+3, #0
	lcall mul32 ; Multiplies by 100 so we can compare with FinalTemp properly, stores it in x
	mov y+0, FinalTemp+0
	mov y+1, FinalTemp+1 ; Move final temperature measurement into y
	mov y+2, FinalTemp+2
	mov y+3, FinalTemp+3
	lcall x_gteq_y
	mov temp_state1, mf ; If temp_refl >= FinalTemp, = 1

	mov x+0, FinalTemp+0
	mov x+1, FinalTemp+1 ; Move final temperature measurement into x
	mov x+2, FinalTemp+2
	mov x+3, FinalTemp+3

	Load_y(25000) ;if temp greater than 250, temp_gt_250 = 1
	lcall x_gteq_y
	mov temp_state3, mf ; Temp > Soak

	Load_y(6000) ;if temp greater than 60, temp_gt_60 = 1
	lcall x_gteq_y
	mov temp_state5, mf

; NOTE: Could change this to be more optimal and use less math32
; Store #100 in y, then call div32, then store this value in a variable
; Then, we can do the comparisons *in the timer* *without using math32*, using subb and the "c" flag
; This could also allow us to make a subroutine that simply does the comparison using R0/R1, simplifying the code


	; Convert to BCD and display
	lcall hex2bcd
	lcall Display_formated_BCD
	lcall SendBCD

; Might be able to use clr instead of moving #0 to each byte
; Check if its possible, and replace if it is to simplify code

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
	
