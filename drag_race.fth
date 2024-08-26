\ ukmarsbot_forth
\ (c) 2022 Rob Probin 
\ MIT License, see LICENSE file
\ 
\ BRIEF: loads forth files into the robot.
\ 
\ NOTE1: This file loads the files into the robot to make a complete robot.
\        The order is important.
\ 
\ NOTE2: This isn't standard Forth - include and require are used by the terminal program. 
\        Alternatively, you can do it automatically. 

\ ==========================================================
\ Basic IO Stuff 
\ ==========================================================

\ -IOBase
marker -IOBase

decimal
\ PORTB
37 constant PORTB   \ Port B Data Register
36 constant DDRB    \ Port B Data Direction Register
35 constant PINB    \ Port B Input Pins
\ PORTC
40 constant PORTC   \ Port C Data Register
39 constant DDRC    \ Port C Data Direction Register
38 constant PINC    \ Port C Input Pins
\ PORTD
43 constant PORTD   \ Port D Data Register
42 constant DDRD    \ Port D Data Direction Register
41 constant PIND    \ Port D Input Pins


\ Use:
\ PORTD 7 defPIN: PD7  ( define portD pin #7)
: defPIN: ( PORTx bit# --- <word> | <word> --- mask port)
    create
        1 swap lshift c, c,           \ compile PORT and min mask
    does>
        dup c@          \ push pin mask
        swap 1+ c@      \ push PORT
  ;


\ Turn a port pin on, dont change the others.
: high ( pinmask portadr -- )
    mset
  ;
\ Turn a port pin off, dont change the others.
: low ( pinmask portadr -- )
    mclr
  ;

\ Only for PORTx bits, 
\ because address of DDRx is one less than address of PORTx.
 
\ Set DDRx so its corresponding pin is output.
: output ( pinmask portadr -- )
    1- high
  ;
\ Set DDRx so its corresponding pin is input.
: input  ( pinmask portadr -- )   
    1- low
  ;

\ read the pins masked as input
: pin@  ( pinmask portaddr -- fl )
    2- mtst \ select PINx register as input
    if      true
    else    false   then
  ;

\ ==========================================================
\ Board setup
\ ==========================================================

\ -ukmarsBoard
marker -ukmarsBoard

flash
\ Arduino Pin 13 = PB13 = Onboard LED
\ (Rob's board has a buzzer connected to this pin as well)
PORTB 5 defPIN: LED
\ PORTD 3 defPIN: BUTTON1
\ PORTC 5 defPIN: BUTTON3
\ PORTC 0 defPIN: BUTTON4
\ PORTB 3 defPIN: BUZZER

PORTD 7 defPIN: LDIR
PORTB 0 defPIN: RDIR
PORTB 1 defPIN: LMOTOR
PORTB 2 defPIN: RMOTOR
ram

: init.ports ( --)
    LED output
    LDIR output
    RDIR output
    LMOTOR output
    RMOTOR output

  ;

\ motor raw test code
\ LMOTOR high
\ LMOTOR low
\ LDIR high
\ LDIR low


: test_buttons ( -- )
    init.ports

    begin
        \ BUTTON1 pin@ 0= if ." Button1" cr then
        \ BUTTON3 pin@ 0= if ." Button3" cr then
        \ BUTTON4 pin@ 0= if ." Button4" cr then
        LED high
        100 ms
        LED low
        100 ms
        LED high
        100 ms
        LED low
        500 ms
        
    key? until
  ;

\ ==========================================================
\ Analogue to digital conversion Reading 
\ ==========================================================

\ On ATmega based boards (UNO, Nano, Mini, Mega), it takes about 
\ 100 microseconds (0.0001 s) to read an analog input, so the 
\ maximum reading rate is about 10,000 times a second.

marker -ADCBase

hex
7e constant DIDR0   \ digital input disable
7c constant ADMUX
7b constant ADCSRB  \ defs to free running, no mux enable
7a constant ADCSRA  \ 
79 constant ADCH    \ analogue reading high byte
78 constant ADCL    \ analogue reading low byte

20 constant ADLAR   \ left shift result (for easy 8 bit)
40 ADLAR + constant _ADMUX  \ 40 is Vref=Vcc

: analog.init ( -- )
    _ADMUX ADMUX mset
\ AVR ADC must be clocked at a frequency between 50 and 200kHz. 
\ So we need to set proper prescaller bits so that the scaled 
\ system clock would fit in this range. As our AVR is clocked 
\ at 16MHz, we will use 128 scaling factors by setting ADPS0, 
\ ADPS1, and ADPS2 bits in the ADCSRA register. 
\ This gives 16000000/128=125kHz of ADC clock.
\ /128 = value 7

    87 ADCSRA mset \ 80 = ADC enable
;

: analogRead8 ( ch -- u )
    0f and _ADMUX + ADMUX c!   \ select channel
    40 ADCSRA mset      \ start conv
    begin 40 ADCSRA mtst while repeat \ wait

    \ return result
    ADCH c@ \ for 16 bit we'd read ADCL first, and have ADLAR=0
;

decimal



\ ==========================================================
\ PWM for motor control
\ ==========================================================

0 constant PWM_488_HZ
1 constant PWM_3906_HZ
2 constant PWM_31250_HZ
$81 constant TCCR1B
$02 constant CS11
$01 constant CS10

: motor_pwm_freq ( freq -- )
  dup PWM_31250_HZ = if
    \ Divide by 1. frequency = 31.25 kHz;
    CS11 TCCR1B mclr
    CS10 TCCR1B mset
    drop
  else
    PWM_3906_HZ = if
      \ Divide by 8. frequency = 3.91 kHz;
      CS11 TCCR1B mset
      CS10 TCCR1B mclr
    else \ PWM_488_HZ or anything else
      \ Divide by 64. frequency = 488Hz;
      CS11 TCCR1B mset
      CS10 TCCR1B mset
    then
  then
;

$80 constant TCCR1A \ 
$80 constant COM1A1 \ bit 7 of TCCR1A
$20 constant COM1B1 \ bit 5 of TCCR1A
$02 constant WGM11  \ bit 1 of TCCR1A
$01 constant WGM10  \ bit 0 of TCCR1A

$88 constant OCR1A  \ actually OCR1AL, since we are in 8 bit mode
$8a constant OCR1B  \ actually OCR1BL

\ disable PWM from pin
: -pwm ( mask port -- )
  drop 
  2 = if
    COM1A1 TCCR1A mclr
  else
    COM1B1 TCCR1A mclr
  then
;

: pwm! ( mask port val  -- )    
    dup 254 > if
        drop
        2dup -pwm
        high exit
    then
    dup 1 < if
        drop
        2dup -pwm
        low exit
    then
    \ must be an intermediate value, so we need PWM
    \ Arduino D9 and D10 are controlled by TCCR1B
    \ D9 = PB1 = OC1A
    \ D10 = PB2 = OC1B
    \ Both are timer1 pins

    \ put timer 1 in 8-bit phase correct pwm mode
    WGM10 TCCR1A mset

    nip  \ drop port number, leave port mask under val
    swap \ bring mask to top of stack
    2 = if
        \  connect pwm to pin on timer 1, channel A (OC1A, D9, PB1)
        COM1A1 TCCR1A mset
        OCR1A c! \ set pwm duty
    else
        \ connect pwm to pin on timer 1, channel B
        COM1B1 TCCR1A mset
        OCR1B c! \ set pwm duty
    then
;

: lmotor! ( n -- ) \ -255 to 255
  LDIR output
  LMOTOR output

  dup 0< if
    abs
    LDIR high
  else
    LDIR low
  then
  >r LMOTOR r> pwm!
;

: rmotor! ( n -- ) \ -255 to 255
  negate \ make motor go opposite direction  
  RDIR output
  RMOTOR output

  dup 0< if
    abs
    RDIR high
  else
    RDIR low
  then
  >r RMOTOR r> pwm!
;

: mstop ( -- )
  0 rmotor!
  0 lmotor!
;



\ ==========================================================
\ 
\ ==========================================================



