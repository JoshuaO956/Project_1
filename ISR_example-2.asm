; ISR_example.asm: a) Increments/decrements a BCD variable every half second using
; an ISR for timer 2; b) Generates a 2kHz square wave at pin P1.7 using
; an ISR for timer 0; and c) in the 'main' loop it displays the variable
; incremented/decremented using the ISR for timer 2 on the LCD.  Also resets it to 
; zero if the 'CLEAR' push button connected to P1.5 is pressed.
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

CLK           EQU 16600000 ; Microcontroller system frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/TIMER0_RATE)))
TIMER2_RATE   EQU 500     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/TIMER2_RATE)))

CLEAR_BUTTON  equ P1.5
UPDOWN        equ P1.6
SOUND_OUT     equ P1.2
HOUR		  equ P0.5
MINUTE		  equ P3.0
SECOND		  equ P1.7
DAY_NIGHT	  equ P1.0
SET_ALARM	  equ P1.5

; Reset vector
org 0x0000
    ljmp main

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

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

; In the 8051 we can define direct access variables starting at location 0x30 up to location 0x7F
dseg at 0x30
uH:			  ds 1
uM:			  ds 1
uS:			  ds 1
DN:			  ds 1
alarm_flag:	  ds 1
ampm:		  ds 1
minutes:	  ds 1
hours:		  ds 1
Count1ms:     ds 2 ; Used to determine when half second has passed
BCD_counter:  ds 1 ; The BCD counter incrememted in the ISR and displayed in the main loop
alarm:		  ds 1
;amFlag:		  ds 1
;hourFlag:	  ds 1
;minFlag:	  ds 1
;secFlag:	  ds 1

; In the 8051 we have variables that are 1-bit in size.  We can use the setb, clr, jb, and jnb
; instructions with these variables.  This is how you define a 1-bit variable:
bseg
half_seconds_flag: dbit 1 ; Set to one in the ISR every time 500 ms had passed

cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3

$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

;                     1234567890123456    <- This helps determine the location of the counter
Initial_Message:  db 'Time  hh:mm:ssAM', 0
pm_message: db 'PM', 0
am_message: db 'AM', 0
alarm_message: db 'Alarm hh:mm:ssAM', 0
alarm_on_message: db 'Y', 0
alarm_off_message: db ' ', 0

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 0                     ;
;---------------------------------;
Timer0_Init:
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz wave at pin SOUND_OUT   ;
;---------------------------------;
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	; Timer 0 doesn't have 16-bit auto-reload, so
	clr TR0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	setb TR0
	push acc
	push psw
	
	mov a, alarm_flag
    cjne a, #1, salta ;
   	mov alarm_flag, a
    
    mov a, ampm
	cjne a, DN, salta ;
	mov ampm, a
	
	mov a, hours
	cjne a, uH, salta ;
	mov hours, a
	
	mov a, minutes
	cjne a, uM, salta ;
	mov minutes, a
	
	;mov a, BCD_counter
	;cjne a, uS, salta ;
	;mov BCD_counter, a
	
	; if all met, sound alarm
	cpl SOUND_OUT ; Connect speaker the pin assigned to 'SOUND_OUT'!

salta:	
	pop psw
	pop acc
	reti

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;
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

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1

Inc_Done:
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(500), Timer2_ISR_done ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(500), Timer2_ISR_done
	
	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb half_seconds_flag ; Let the main program know half second had passed
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Increment the BCD counter
	mov a, BCD_counter
	jnb UPDOWN, Timer2_ISR_decrement
	add a, #0x01
	sjmp Timer2_ISR_da
Timer2_ISR_decrement:
	add a, #0x99 ; Adding the 10-complement of -1 is like subtracting 1.
Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov BCD_counter, a
	
Timer2_ISR_done:
	pop psw
	pop acc
	reti
	
stop_t0:
	clr TR0
	clr ET0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	clr TF0
	ret

;---------------------------------;
; Main program. Includes hardware ;
; initialization and 'forever'    ;
; loop.                           ;
;---------------------------------;
main:
	; Initialization
    mov SP, #0x7F
    mov P0M1, #0x00
    mov P0M2, #0x00
    mov P1M1, #0x00
    mov P1M2, #0x00
    mov P3M2, #0x00
    mov P3M2, #0x00
          
    lcall Timer0_Init
    lcall Timer2_Init
    setb EA   ; Enable Global interrupts
    lcall LCD_4BIT
    ; For convenience a few handy macros are included in 'LCD_4bit.inc':
    
    ; initialize main clock
	Set_Cursor(1, 1)
    Send_Constant_String(#Initial_Message)
    setb half_seconds_flag
	mov BCD_counter, #0x00
	mov minutes, #0x00
	mov hours, #0x12
	mov ampm, #0 ; 0 for AM, 1 for PM
	
	; initialize alarm settings and display
	Set_Cursor(2, 1)
	Send_Constant_String(#alarm_message)
	mov uH, #0x00
	mov uM, #0x00
	mov uS,	#0x00
	mov DN, #0 ; 0 for AM, 1 for PM
	mov alarm_flag, #0
	
; logic for setting user input through pushbuttons
; SETTING USER INPUT: INCLUDES ALL THE SET HOURS, MINUTES, SECONDS, AND AMPM BRANCHES
; ALARM HOUR LOGIC	
set_hours:
	; HOUR LOGIC - USER INPUT
	setb P0.5 ; Before using as input...
	jb P0.5, set_mins
	Wait_Milli_Seconds(#1) ; Wait and check again
	jb P0.5, set_mins
	; Wait for the button to be released ; IF PRESSED IT WILL PROCEED TO NEXT LINE
	
hour_pressed: jnb P0.5, hour_pressed
	mov a, uH
	add a, #1
	da a
	mov uH, a
	mov a, uH
	cjne a, #0x13, set_mins
    mov a, #0x01
    mov uH, a
    
; ALARM MINUTE LOGIC 
set_mins:
	; HOUR LOGIC - USER INPUT
	setb P3.0 ; Before using as input...
	jb P3.0, set_secs
	Wait_Milli_Seconds(#1) ; Wait and check again
	jb P3.0, set_secs

mins_pressed: jnb P3.0, mins_pressed
	mov a, uM 
	add a, #1
	da a
	mov uM, a
	mov a, uM
	cjne a, #0x60, set_secs
    mov a, #0x01
    mov uM, a
    
; ALARM SECOND LOGIC 
set_secs:
	setb P1.7 ; Before using as input...
	jb P1.7, set_am_or_pm
	Wait_Milli_Seconds(#1) ; Wait and check again
	jb P1.7, set_am_or_pm

secs_pressed: jnb P1.7, secs_pressed
	mov a, uS 
	add a, #1
	da a
	mov uS, a
	mov a, uS
	cjne a, #0x60, set_am_or_pm
	mov a, #0x01
    mov uS, a
    
; setting am or pm 
set_am_or_pm:
	setb P1.0 ; Before using as input...
	jb P1.0, userDisp
	Wait_Milli_Seconds(#1) ; Wait and check again
	jb P1.0, userDisp 

ampm_pressed: jnb P1.0, ampm_pressed
	mov a, DN
	cjne a, #1, ampm_reset ; hours have been pressed, but not ovf yet so proceed to checking minutes 
	Set_Cursor(2, 15)
    Send_Constant_String(#am_message)	
    mov a, #0
	mov DN, a
	ljmp userDisp
	
ampm_reset:
    Set_Cursor(2, 15)
    Send_Constant_String(#pm_message)
    mov DN, #1
    
userDisp:
	Set_Cursor(2, 7)
	Display_BCD(uH)
	
	Set_Cursor(2, 10)
	Display_BCD(uM)
	
	Set_Cursor(2, 13)
	Display_BCD(uS)
	
on_off:
	setb P1.5 ; Before using as input...
	jb P1.5, loop
	Wait_Milli_Seconds(#1) ; Wait and check again
	jb P1.5, loop 
	
alarm_off: jnb P1.5, alarm_off
	mov a, alarm_flag
	cjne a, #1, alarm_on ; hours have been pressed, but not ovf yet so proceed to checking minutes 
	Set_Cursor(2, 6)
    Send_Constant_String(#alarm_off_message)	
    ;mov alarm, #0
    mov a, #0
	mov alarm_flag, a
	ljmp loop
	
alarm_on:
    Set_Cursor(2, 6)
    Send_Constant_String(#alarm_on_message)
    mov alarm_flag, #1
    
if_alarm_on:
	mov a, alarm_flag
	cjne a, #0, loop
	lcall stop_t0
	lcall Timer0_Init

loop:
	jb CLEAR_BUTTON, loop_a  ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb CLEAR_BUTTON, loop_a  ; if the 'CLEAR' button is not pressed skip
	jnb CLEAR_BUTTON, $		; Wait for button release.  The '$' means: jump to same instruction.
	; A valid press of the 'CLEAR' button has been detected, reset the BCD counter.
	; But first stop timer 2 and reset the milli-seconds counter, to resync everything.
	clr TR2                 ; Stop timer 2
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Now clear the BCD counter
	mov BCD_counter, a
	setb TR2                ; Start timer 2
	sjmp loop_b
	
; MAIN CLOCK LOGIG - INCLUDES LOOP A,B,C,C2,D,AMPM_CHECK
labelhour:
	ljmp set_hours	
loop_a:
	jnb half_seconds_flag, labelhour
    
loop_b: ;secs
    clr half_seconds_flag
    Set_Cursor(1, 7)
    Display_BCD(hours)
    Set_Cursor(1, 10)
    Display_BCD(minutes)
    Set_Cursor(1, 13)
    Display_BCD(BCD_counter)
    
    mov a, BCD_counter
    cjne a, #0x60, labelloop
    ;
    mov a, minutes
	add a, #1
	da a
	mov minutes, a
    ;inc minutes
    mov BCD_counter, #0
    mov a, hours
	cjne a, #0x12, label5
	sjmp loop_c
labelloop:
	ljmp loop
label5:
	ljmp loop_c2

loop_c: ; minutes
    Set_Cursor(1, 7)
    Display_BCD(hours)
    Set_Cursor(1, 10)
    Display_BCD(minutes)
    Set_Cursor(1, 13)
    Display_BCD(BCD_counter)
    
    mov a, minutes
    cjne a, #0x60, label1
    mov hours, #0x01
    mov minutes, #0
    mov BCD_counter, #0
    sjmp loop_d ; hours
label1:
	ljmp loop_b ; sec 


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
loop_d:
	Set_Cursor(1, 7)
    Display_BCD(hours)
    Set_Cursor(1, 10)
    Display_BCD(minutes)
    Set_Cursor(1, 13)
    Display_BCD(BCD_counter)
    
    mov a, hours
    cjne a, #0x12, label2; IF NOT 12, GO TO loop_c2  ; ;  until hours get to 12, keep incrementing hours; make a c2, that instead of resetting hours to 0, increments it. \
	;; keep the hours at 12, just change from AM times...  11:59... 12:00:00PM
	mov minutes, #0
    mov BCD_counter, #0
	sjmp ampm_check
label2:
	ljmp loop_c2 ; mins
		
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ampm_check: ; when loop_d goes to this function, it is still in AM the first time around, but next time loop_d goes to this function
			; we have to check the ampm flags to determine if we are still in AM or PM
    Set_Cursor(1, 7)
    Display_BCD(hours)
    Set_Cursor(1, 10)
    Display_BCD(minutes)
    Set_Cursor(1, 13)
    Display_BCD(BCD_counter)
	
	; Display AM/PM at position (1, 15)
	Set_Cursor(1, 15)
    mov a, ampm
    cjne a, #0, am_display
    ; if 1, proceed
    mov ampm, #1
    Send_Constant_String(#pm_message)
    sjmp label6 ; Skip display_am
label6:
	ljmp loop_b; if we switched to pm, go to loop_b as all time will be reset to 12:00:00 
				; and PM will stay as were not calling on the send initial string function
am_display:
    Set_Cursor(1, 15)
    mov ampm, #0
    Send_Constant_String(#am_message); 
    ljmp loop_b


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
loop_c2: ;minutes
    Set_Cursor(1, 7)
    Display_BCD(hours)
    Set_Cursor(1, 10)
    Display_BCD(minutes)
    Set_Cursor(1, 13)
    Display_BCD(BCD_counter)
    
    mov a, minutes
    cjne a, #0x60, label3
    ;
	mov a, hours
	add a, #1
	da a
	mov hours, a
	;inc hours
    mov minutes, #0
    mov BCD_counter, #0
    sjmp label4

label3:
	ljmp loop_b ; sec
label4:
	ljmp loop_d ; hour



END