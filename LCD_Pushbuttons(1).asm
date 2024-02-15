; N76E003 LCD_Pushbuttons.asm: Reads muxed push buttons using one input

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
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))
TIMER2_RATE   EQU 500     ; 500Hz, sets the BCD counter to work in seconds
TIMER2_RELOAD EQU ((65536-(CLK/TIMER2_RATE)))


ORG 0x0000
	ljmp main

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
;org 0x000B
;	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti

; Timer/Counter 2 overflow interrupt vector
org 0x002B
	ljmp Timer2_ISR




;                     1234567890123456    <- This helps determine the location of the counter
title_1:          db 'Temp:     D    s', 0
title_2:          db 'Temp:     D    C', 0
Lower:            db 's   ,   r   ,   ', 0
Initial_Message:  db 'Timer:          ', 0



cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3
UPDOWN equ P1.6
;Pin for start/stop
START_STOP_BUTTON    equ P1.6
PWM_OUT    EQU P1.0


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$include(math32.inc)
$include(troubleshooter.inc)
$LIST

BSEG
; These five bit variables store the value of the pushbuttons after calling 'LCD_PB' below
PB0:               dbit 1
PB1:               dbit 1
PB2:               dbit 1
PB3:               dbit 1
PB4:			   dbit 1
s_flag: dbit 1 
mf: dbit 1



dseg at 0x30

FSM_state:	ds 1
temp: 		ds 1
temp_time_mode: ds 1
soak_temp:		ds 1
reflow_temp:	ds 1
soak_time:		ds 1
reflow_time:	ds 1
Count1ms:       ds 2 ; Used to determine when half second has passed
Timer_counter:  ds 1 ; The Timer counter incrememted in the ISR and displayed in the main loop
x:        ds 4
y:        ds 4
bcd:      ds 4
VLED_ADC: ds 2
roomtemp: ds 4
vout:     ds 2
Val_test: ds 4
Val_temp: ds 4
pwm_counter: ds 1 ; Free running counter 0, 1, 2, ..., 100, 0
pwm: ds 1 ; pwm percentage
seconds:      ds 1 ; a seconds counter attached to Timer 2 ISR
Temperature: ds 1
On_Off: ds 1


CSEG
Init_All:
	; Configure all the pins for biderectional I/O
	mov P3M1, #0x00
    mov P3M2, #0x00
    mov P1M1, #0x00
    mov P1M2, #0x00
    mov P0M1, #0x00
    mov P0M2, #0x00

    ; Configure timer 1 for baud rate
    orl CKCON, #0x10 ; Timer 1 as baud rate generator
    orl PCON, #0x80  ; Double baud rate
    mov SCON, #0x52
    anl T3CON, #0b11011111
    anl TMOD, #0x0F   ; Clear configuration bits for timer 1
    orl TMOD, #0x20   ; Timer 1 Mode 2
    mov TH1, #TIMER1_RELOAD
    setb TR1

    ; Configure timer 0 for delay functions
    clr TR0 ; Stop timer 0
    orl CKCON, #0x08 ; Timer 0 using system clock
    orl TMOD, #0x01  ; Timer 0 in Mode 1: 16-bit timer
    anl TMOD, #0xF0  ; Clear configuration bits for timer

    ; Initialize ADC pins as inputs
    orl P1M1, #0b10000010
    anl P1M2, #0b01111101

    ; Initialize and start the ADC
    anl ADCCON0, #0xF0
    orl ADCCON0, #0x07 ; Select channel 7
    mov AINDIDS, #0x00 ; Disable all analog inputs
    orl AINDIDS, #0b10000001 ; Activate AIN0 and AIN7 analog inputs
    orl ADCCON1, #0x01 ; Enable ADC

	setb EA ; Enable global interrupts

ret

Timer2_Init:
	; Initialize timer 2 for periodic interrupts
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	mov T2MOD, #0b1010_0000 ; Enable timer 2 autoreload, and clock divider is 16
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
	; Init the free running 10 ms counter to zero
	mov pwm_counter, #0
	; Enable the timer and interrupts
	orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    setb TR2  ; Enable timer 2
ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	push psw
	push acc

	inc pwm_counter
	clr c
	mov a, pwm
	subb a, pwm_counter ; If pwm_counter <= pwm then c=1
	cpl c
	mov PWM_OUT, c
	
	mov a, pwm_counter
	cjne a, #100, Timer2_ISR_done
	mov pwm_counter, #0
	inc seconds ; It is super easy to keep a seconds count here
	setb s_flag

	ret



Timer2_ISR_done:
	pop acc
	pop psw
	reti

	
	ret

; Eight bit number to display passed in 'a'
; Sends result to LCD

SendToLCD:
    mov b, #100      ; Load 100 into register B
    div ab           ; Divide A by B, quotient in A, remainder in B

    ; Convert hundreds to ASCII
    orl a, #0x30     ; Convert to ASCII
    lcall ?WriteData  ; Send to LCD

    mov a, b         ; Move remainder (tens + units) into A
    mov b, #10       ; Load 10 into B
    div ab           ; Divide A by B, quotient in A, remainder in B

    ; Convert tens to ASCII
    orl a, #0x30     ; Convert to ASCII
    lcall ?WriteData  ; Send to LCD

    mov a, b         ; Move remainder (units) into A

    ; Convert units to ASCII
    orl a, #0x30     ; Convert to ASCII
    lcall ?WriteData  ; Send to LCD

    ret              ; Return from subroutine
	
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

Display_formated_BCD2:
	Set_Cursor(1, 7)
	Display_BCD(bcd+1)
	Send_BCD(bcd+1)
	Display_BCD(bcd+0)
	Send_BCD(bcd+0)
	Display_char(#'C')
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	ret

Average_ADC:
    Load_x(0)
    mov R5, #100
    Sum_loop0:
        anl ADCCON0, #0xF0
        orl ADCCON0, #0x05
        lcall Read_ADC
        mov y+3, #0
        mov y+2, #0
        mov y+1, R1
        mov y+0, R0
        lcall add32
        djnz R5, Sum_loop0
    load_y(100)
    lcall div32
    ret

Read_ADC:
    clr ADCF
    setb ADCS ; Start ADC conversion
    jnb ADCF, $ ; Wait for conversion complete

    ; Read ADC result
    mov A, ADCRL
    anl A, #0x0F
    mov R0, A
    mov A, ADCRH
    swap A
    anl A, #0x0F
    mov R1, A
    anl A, #0xF0
    orl A, R0
    mov R0, A
    ret

InitSerialPort:
    
    ; Wait for reset button debounce
    mov R1, #200
    mov R0, #104
    debounce_loop:
        djnz R0, debounce_loop ; Delay
        djnz R1, debounce_loop ; Delay

    ; Configure serial port
    orl CKCON, #0x10 ; Timer 1 as baud rate generator
    orl PCON, #0x80  ; Double baud rate
    mov SCON, #0x52
    anl T3CON, #0b11011111
    anl TMOD, #0x0F   ; Clear configuration bits for timer 1
    orl TMOD, #0x20   ; Timer 1 Mode 2
    mov TH1, #TIMER1_RELOAD
    setb TR1
    ret

putchar:
    jnb TI, $ ; Wait for TI to be ready
    clr TI
    mov SBUF, A ; Send character
    ret

SendString:
    clr A
    movc A, @A+DPTR
    jz SendStringDone
    lcall putchar
    inc DPTR
    sjmp SendString
SendStringDone:
    ret


LCD_PB:
	; Set variables to 1: 'no push button pressed'
	setb PB0
	setb PB1
	setb PB2
	setb PB3
	setb PB4
	; The input pin used to check set to '1'
	setb P1.5
	
	; Check if any push button is pressed
	clr P0.0
	clr P0.1
	clr P0.2
	clr P0.3
	clr P1.3
	jb P1.5, LCD_PB_Done

	; Debounce
	mov R2, #50
	lcall waitms
	jb P1.5, LCD_PB_Done

	; Set the LCD data pins to logic 1
	setb P0.0
	setb P0.1
	setb P0.2
	setb P0.3
	setb P1.3
	
	; Check the push buttons one by one
	clr P1.3
	mov c, P1.5
	mov PB4, c
	setb P1.3

	clr P0.0
	mov c, P1.5
	mov PB3, c
	setb P0.0
	
	clr P0.1
	mov c, P1.5
	mov PB2, c
	setb P0.1
	
	clr P0.2
	mov c, P1.5
	mov PB1, c
	setb P0.2
	
	clr P0.3
	mov c, P1.5
	mov PB0, c
	setb P0.3

	;jnb PB0, PB0_Pressed
	jnb PB1, PB1_Pressed
	jnb PB2, PB2_Pressed
	jnb PB3, PB3_Pressed
	jnb PB4, PB4_Pressed
	ret

LCD_PB_Done:	
	ret


;PB0_Pressed:
	
	;if temp_time_mode = 1, set temp, otherwise set time

	;mov a,temp_time_mode
	;cjne a, #0x01, set_temp
	;mov temp_time_mode, #0x00
	;Set_Cursor(1, 1)
   ; Send_Constant_String(#title_1)
	;ret

	;set_temp:
	;	mov temp_time_mode, #0x01
	;	Set_Cursor(1, 1)
	;	Send_Constant_String(#title_2)
	;ret
   
PB1_Pressed:
	mov a, temp_time_mode
	cjne a, #0x01, dec_time_2
	mov a, reflow_temp
    Dec a
    mov reflow_temp, a
	ret

	dec_time_2:
	mov a, reflow_time
    Dec a
    mov reflow_time, a
    ret

PB2_Pressed:

	mov a, temp_time_mode
	cjne a, #0x01, inc_time_2 
	mov a, reflow_temp
    Inc a
    mov reflow_temp, a
	ret

	inc_time_2:
	mov a, reflow_time
    Inc a
    mov reflow_time, a
    ret

PB3_Pressed:

	mov a, temp_time_mode
	cjne a, #0x01, dec_time 
	mov a, soak_temp
    Dec a
    mov soak_temp, a
	ret

	dec_time:
	mov a, soak_time
    Dec a
    mov soak_time, a
    ret

PB4_Pressed:

	mov a, temp_time_mode
	cjne a, #0x01, inc_time 
	mov a, soak_temp
    Inc a
    mov soak_temp, a
	ret

	inc_time:
	mov a, soak_time
    Inc a
    mov soak_time, a
    ret
    

Display_PushButtons_LCD:
	Set_Cursor(2, 1)
	mov a, #'0'
	mov c, PB4
	addc a, #0
    lcall ?WriteData	
	mov a, #'0'
	mov c, PB3
	addc a, #0
    lcall ?WriteData	
	mov a, #'0'
	mov c, PB2
	addc a, #0
    lcall ?WriteData	
	mov a, #'0'
	mov c, PB1
	addc a, #0
    lcall ?WriteData	
	mov a, #'0'
	mov c, PB0
	addc a, #0
    lcall ?WriteData	
	ret

	
main:
	mov sp, #0x7f
	mov Timer_counter, #0x00
	mov On_Off, #0x01

    lcall Init_All
    lcall LCD_4BIT
	lcall InitSerialPort
  

	mov soak_temp, #150
	mov reflow_temp, #230
	mov soak_time, #0x45
	mov reflow_time, #0x30
	mov temp_time_mode, #0x00
	mov On_Off, #0x00
	mov seconds, #0x00
    
    ; initial messages in LCD
	;Set_Cursor(1, 1)
    ;Send_Constant_String(#title_1)
	Set_Cursor(2, 1)
    Send_Constant_String(#Lower)
	
Forever:

	;Check for START_STOP_BUTTON press
    setb START_STOP_BUTTON
    jb START_STOP_BUTTON,  loop_after
    ;Wait_Milli_Seconds(#1) ; Wait and check again
    jb START_STOP_BUTTON, loop_after

	START_STOP_BUTTON_PRESSED_LABEL: jnb START_STOP_BUTTON, START_STOP_BUTTON_PRESSED_LABEL
    ; Button is pressed

	;Check if switch is on/off

	mov a, On_off
	cjne a, #0x01, start_the_timer

	mov On_Off, #0x00
	sjmp loop_after

start_the_timer:
	mov On_Off, #0x01
	sjmp loop_after


loop_after:

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
	Load_y(27300) ; put 2.73V in y ; 2.73 is a CONVERSION CONSTANT
	lcall sub32 ; x - y stored in x ; x = 2.98 - 2.73
	Load_y(100)	; 
	lcall mul32 ; 100 * x
	
	; WE HAVE ROOMTEMP
	
	mov roomtemp+0, x+0
	mov roomtemp+1, x+1
	mov roomtemp+2, x+2
	mov roomtemp+3, x+3
	
	;-----------------------------------------------------------------------------------------
	; HERE WE CONVERT THE THERMOCOUPLE VOLTAGE TO TEMPERATURE
	; read the thermocouple temperature from vout connected to pin 20 and store it in variable tcpltemp for now
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x05 ; AIN5, channel 5
	lcall Read_ADC
	mov x+0, R0
	mov x+1, R1
	mov x+2, #0
	mov x+3, #0
	
	Load_y(49436)
	lcall mul32
	Load_y(4095)
	lcall div32
	Load_y(100)
	lcall mul32 ; At this point the voltage is in microvolts
	Load_y(425)
	lcall div32
	Load_y(41)
	lcall div32
	
	;mov y+0, roomtemp+0
	;mov y+1, roomtemp+1
	;mov y+2, roomtemp+2
	;mov y+3, roomtemp+3
	Load_y(22)
	lcall add32

	mov Temperature, x+0
	
	lcall hex2bcd
	lcall Display_formated_BCD2
	
	; Wait 500 ms between conversions
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	
	lcall LCD_PB
	
	mov a, soak_temp
	Set_Cursor(2, 2)
	lcall SendToLCD

	mov a, soak_time
	Set_Cursor(2, 6)
	lcall SendToLCD
    
    mov a, reflow_temp
	Set_Cursor(2, 10)
	lcall SendToLCD

	mov a, reflow_time
	Set_Cursor(2, 14)
	lcall SendToLCD

	Set_Cursor(1, 2)
	Display_BCD(On_Off)

	ljmp Forever
	
END
	
