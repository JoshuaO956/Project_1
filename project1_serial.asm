; 76E003 ADC test program: Reads channel 7 on P1.1, pin 14
; This version uses an LED as voltage reference connected to pin 6 (P1.7/AIN0)

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

; LED voltage: 2.0523

CLK               EQU 16600000 ; Microcontroller system frequency in Hz
BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))

ORG 0x0000
	ljmp main

;                     1234567890123456    <- This helps determine the location of the counter
test_message:     db 'Temp= xx.xx degC', 0
cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3

$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; These register definitions needed by 'math32.inc'
DSEG at 30H
x:   ds 4
y:   ds 4
bcd: ds 6
VLED_ADC: ds 2
roomtemp: ds 2
vout: ds 2

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
	mov	TH0, #high(TIMER0_RELOAD_1MS)
	mov	TL0,#low(TIMER0_RELOAD_1MS)
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
	Set_Cursor(2, 10)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	Set_Cursor(2, 10)
	Display_char(#'=')
	ret
	
Display_formated_BCD2:
	Set_Cursor(1, 6)
	Display_BCD(bcd+3)
	Send_BCD(bcd+3)
	Display_BCD(bcd+2)
	Send_BCD(bcd+2)
	Display_char(#'.')
	mov a, #'.'
	lcall putchar
	Display_BCD(bcd+1)
	Send_BCD(bcd+1)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	ret
	
Display_formated_BCD3:
	Set_Cursor(2, 1)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	Set_Cursor(2, 1)
	ret
	
Display_formated_BCD4:
	Set_Cursor(2, 1)
	Display_BCD(bcd+3)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	Set_Cursor(2, 1)
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
	
	
; Configure the serial port and baud rate
InitSerialPort:

	push AR0
	push AR1
	; Configure all the pins for biderectional I/O
	mov	P3M1,#0x00
	mov	P3M2,#0x00
	mov	P1M1,#0x00
	mov	P1M2,#0x00
	mov	P0M1,#0x00
	mov	P0M2,#0x00
	
    ; Since the reset button bounces, we need to wait a bit before
    ; sending messages, otherwise we risk displaying gibberish!
    mov R1, #200
    mov R0, #104
    djnz R0, $   ; 4 cycles->4*60.285ns*104=25us
    djnz R1, $-4 ; 25us*200=5.0ms

    ; Now we can proceed with the configuration of the serial port
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD
	setb TR1
    ret

; Send a character using the serial port
putchar:
    jnb TI, putchar ; jump (to putchar) if bit not set (0) (keep looping and checking, if 1, NEXT LINE; send byte to SBUF
    clr TI
    mov SBUF, a
    ret

; Send a constant-zero-terminated string using the serial port
SendString:
    clr A
    movc A, @A+DPTR
    jz SendStringDone
    lcall putchar
    inc DPTR
    sjmp SendString
SendStringDone:
	pop AR1
	pop AR0
    ret
 
;Hello_World:
;    DB  'Hello, World!', '\r', '\n', 0
;mov SP, #7FH ; Set the stack pointer to the begining of idata


main:
	mov sp, #0x7f
	lcall Init_All
    lcall LCD_4BIT
    
    ; initial messages in LCD
	Set_Cursor(1, 1)
    Send_Constant_String(#test_message)
    
Forever:

	; Read the 2.08V LED voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VLED_ADC+0, R0
	mov VLED_ADC+1, R1
	
	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(20570) ; The MEASURED LED voltage: 2.074V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LED value
	mov y+0, VLED_ADC+0
	mov y+1, VLED_ADC+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32
	
	lcall hex2bcd
	lcall Display_formated_BCD
	
	; FOR TESTING PURPOSES ; SAVE THE VOLTAGE FROM LM355 IN VOUT 
	; (AS IF IT IS THE THERMOCOUPLE VOUT (MAX 4.36, EXPECTED 2.99)
	mov vout+0, x+0
	mov vout+1, x+1
	
	; continue with room temperature conversion from LM355
	Load_y(27300) ; put 2.73V in y ; 2.73 is a CONVERSION CONSTANT
	lcall sub32 ; x - y stored in x ; x = 2.98 - 2.73
	Load_y(100)	; 
	lcall mul32 ; 100 * x
	
	lcall hex2bcd
	lcall Display_formated_BCD3
	
	; HERE WE CONVERT THE THERMOCOUPLE VOLTAGE TO TEMPERATURE
	; read the thermocouple temperature from vout connected to pin 20 and store it in variable tcpltemp for now
	;anl ADCCON0, #0xF0
	;orl ADCCON0, #0x05 ; AIN5, channel 5
	;lcall Read_ADC ; vout stored in x
	
	mov x+0, vout+0
	mov x+1, vout+1
	mov x+2, #0
	mov x+3, #0
	Load_y(57) ; INVERSE OF (425(R1/R2) * 41x10^-6(slope)) = (0.017425)^-1 = 57.388, input 57
	lcall mul32 
	Load_y(230000)
	lcall add32 ; vout * (0.017425)^-1 = tcpl temp stored in x
	
	; final temp stored in x; send to putty
	lcall hex2bcd
	lcall Display_formated_BCD2
	
	; Wait 500 ms between conversions
	mov R2, #250
	lcall waitms
	mov R2, #250
	lcall waitms
	mov R2, #250
	lcall waitms
	mov R2, #250
	lcall waitms	
	
	lcall InitSerialPort ; Initializes port where information will be sent to computer
	
	ljmp Forever
END