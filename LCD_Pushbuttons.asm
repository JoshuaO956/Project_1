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
title_1:          db 'Temp:   Dc     s', 0
title_2:          db 'Temp    Dc     C', 0
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
START_STOP_BUTTON    equ P1.0


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

BSEG
; These five bit variables store the value of the pushbuttons after calling 'LCD_PB' below
PB0:               dbit 1
PB1:               dbit 1
PB2:               dbit 1
PB3:               dbit 1
PB4:			   dbit 1
half_seconds_flag: dbit 1 ; Set to one in the ISR every time 500 ms had passed

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
On_Off:         ds 1



CSEG
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

	jnb PB0, PB0_Pressed
	jnb PB1, PB1_Pressed
	jnb PB2, PB2_Pressed
	jnb PB3, PB3_Pressed
	jnb PB4, PB4_Pressed

LCD_PB_Done:	
	ret

PB0_Pressed:
	
	;if temp_time_mode = 1, set temp, otherwise set time

	mov a,temp_time_mode
	cjne a, #0x01, set_temp
	mov temp_time_mode, #0x00
	Set_Cursor(1, 1)
    Send_Constant_String(#title_1)
	ret

	set_temp:
		mov temp_time_mode, #0x01
		Set_Cursor(1, 1)
		Send_Constant_String(#title_2)
	ret
   
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
    
Start_Stop_Button_Pressed:
	
	;Check for Start_Stop_Button press
    setb P1.0
    jb P1.0,  Start_Stop_Button_NOT_PRESSED_LABEL
    ;Wait_Milli_Seconds(#1) ; Wait and check again
    jb P1.0, Start_Stop_Button_NOT_PRESSED_LABEL

	Start_Stop_Button_PRESSED_LABEL: jnb P1.0, Start_Stop_Button_PRESSED_LABEL
    ; Button is pressed

    mov a, On_Off     ; Check if timer is running
    cjne a, #0x01, Start_Timer  ; If not running, start the timer
    mov On_Off, #0x00  ; Stop the timer
    ret

Start_Timer:
    mov On_Off, #0x01   ; Start the timer
	lcall Timer2_Init
	setb EA  
    ret

Start_Stop_Button_NOT_PRESSED_LABEL:
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

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.

	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw

	mov a, On_Off  
	cjne a, #0x01, stop_timer 
	sjmp start_the_timer

stop_timer:
	mov Timer_counter, #0x00
	clr TF2
	ljmp Timer2_ISR_done

start_the_timer:
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
	mov a, Timer_counter
	jnb UPDOWN, Timer2_ISR_decrement
	add a, #0x01
	sjmp Timer2_ISR_da
Timer2_ISR_decrement:
	add a, #0x99 ; Adding the 10-complement of -1 is like subtracting 1.
Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov Timer_counter, a
	
Timer2_ISR_done:
	pop psw
	pop acc
	reti

	
main:
	mov sp, #0x7f
	mov On_Off, #0x00 ;Off is 0, On is 1
	mov Timer_counter, #0x00

	lcall Init_All
    lcall LCD_4BIT
	lcall Timer2_Init
    setb EA   ; Enable Global interrupts
	setb half_seconds_flag

	mov soak_temp, #0x00
	mov reflow_temp, #0x00
	mov soak_time, #0x00
	mov reflow_time, #0x00
	mov temp_time_mode, #0x00
    
    ; initial messages in LCD
	Set_Cursor(1, 1)
    Send_Constant_String(#title_1)
	Set_Cursor(2, 1)
    Send_Constant_String(#Lower)
	
Forever:

FSM_loop: ;main loop for the finite state machine
	mov a, FSM_state

FSM_state0: ;loop for the start state
	cjne a, #0, FSM_state1
	mov pwm, #0
	jnb	On_Off , FSM_state0_done
	mov FSM_state, #1
FSM_state0_done:
	ljmp FSM_loop_done


FSM_state1: ;loop for soak to ramp
	cjne a, #1, FSM_state2
	clr C
	mov pwm, #100
	mov Timer_counter, #0
	mov a, soak_temp
	subb a, temp
	jnc FSM_state1_done
	mov FSM1_state, #2
FSM_state1_done:
	ljmp FSM_loop_done


FSM_state2: ;loop for pre-heat/soak
	cjne a, #2, FSM_state3
	clr C
	mov pwm, #20
	mov a, soak_time
	subb a, Timer_counter
	jnc FSM_state2_done
	mov FSM_state, #3
FSM_state2_done:
	ljmp FSM_loop_done

FSM_state3: ;loop for ramp to peak
	cjne a, #3, FSM_state4
	clr C
	mov pwm, #100
	mov a, reflow_temp
	subb a, temp
	jnc FSM_state3_done
	mov FSM_state, #4
FSM_state3_done:
	ljmp FSM_loop_done

FSM_state4: ;loop for reflow
	cjne a, #4, FSM_state5
	clr C
	mov pwm, #20
	mov a, reflow_time
	subb a, Timer_counter
	jnc FSM_state4_done
	mov FSM_state, #5
FSM_state4_done:
	ljmp FSM_loop_done

FSM_state5: ;loop for cooling
	cjne a, #5, FSM_state6
	clr C
	mov pwm, #0
	mov a, #60
	subb a, temp
	jc FSM_state3_done
	mov FSM_state, #4
FSM_state5_done:
	ljmp FSM_loop_done

FSM_loop_done:	;fsm is done

	lcall LCD_PB
	;lcall Display_PushButtons_LCD
	lcall Start_Stop_Button_Pressed
	
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


	;Set_Cursor(1, 12)
	;Display_BCD(Timer_counter)
	
	
	ljmp Forever
	
END
	
