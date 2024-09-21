\ Hardware base for ukmarsbot style robots
\ (c) 2022-2024 Rob Probin 
\ MIT License, see LICENSE file
\ 
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
7b constant ADCSRB  \ defaults to free running, no mux enable
7a constant ADCSRA  \ 
79 constant ADCH    \ analogue reading high byte
78 constant ADCL    \ analogue reading low byte
64 constant PRR     \ power reduction register

20 constant ADLAR   \ left shift result (for easy 8 bit)
40 ADLAR + constant _ADMUX  \ 40 is Vref=Vcc


: analog.init ( -- )
    80 PRR mclr \ ensure PRR is set to disable for ADC

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
\  Robot Configuration
\ ==========================================================

\ ukmarsbot_forth
\ (c) 2022-2023 Rob Probin 
\ MIT License, see LICENSE file
\ 
\ BRIEF: Robot configuration

\ Based on  ukmars mazerunner core

marker -config

decimal 

\ NOTICE: Flashforth is a 16 bit forth, you need to use a double cell to get 32 bit (2constant)
\ We could have used some sort of floating point support, like the C/C++ version of UKMARS
\ but we decided to use integer maths to make installing FlashForth on the Nano simpler.

\ Robot Specific Constants
\ ========================

\ These allow you to select the forward count direction based on wiring, and orientation
\ encoder polarity is either 0 or -1 and is used to account for reversal of the encoder phases
-1 constant ENC_LEFT_POL
0  constant ENC_RIGHT_POL

\ The robot is likely to have wheels of different diameters and that must be
\ compensated for if the robot is to reliably drive in a straight line.
\ Negative makes robot curve to left
3 constant ROTATION_BIAS

\ these are in micrometers = µm
40000 constant WHEEL_DIA    \ µm, adjust on test (normal 
                            \ ukmarsbot wheels are 32000)
20 constant ENCODER_PULSES \ per motor rev
11400 constant GEAR_RATIO   \ x1000 ... 11:1 gear ratio (actual is 11.4:1)
50000 constant MOUSE_RADIUS \ µm  (@TODO Fix me for drag race robot)

\ NOTES ABOUT SCALING
\ If your wheel diameter exceeds 65.535 mm, then you need to alter the scaling below.
\ If your gear ratio exceeds 65.535:1 then you need to alter the scaling below.
\ For UK mars bot, the radius will never go beyond 65mm :-)

\ ---------------------------------------------------

\ 500,0 fconstant LOOP_FREQ

\ --------------------------------------------------------------------------------------

\ General Calculated values from Constants
\ ========================================
\ Unsigned division. ( ud u1 -- ud.quot ) 32-bit/16-bit to 32-bit
: ud/ ud/mod rot drop ;
: ?swap if swap then ; 

\ 31416 constant PI        \ x10000 good enough for us
3142 constant PI        \ x1000 good enough for us
WHEEL_DIA PI um* 2constant WHEEL_CIRCUM \ nanometers
\ ENCODER_PULSES GEAR_RATIO um* 2constant PULSES/REV

\ MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
\ MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);

\ this the basic µm (micrometers) per pulse value
WHEEL_CIRCUM GEAR_RATIO ud/ ENCODER_PULSES ud/ drop dup

\ Finally assign the UM/COUNT for each side.
\ NOTICE: 2/ gives us an asymetric result, which allows us to leverage the bottom bit
\ usually this is about 428
ROTATION_BIAS negate 2/ + constant UM/COUNT_LEFT
ROTATION_BIAS 2/ + constant UM/COUNT_RIGHT

\ ---------------------
\ DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

\ first calculate half circumference in µm, typically around 116239 (116.239 mm)
\ divide by 10 to make it fit into u16  ( = 11625). So this is 10 times too big for um.
MOUSE_RADIUS PI um* 10000 ud/ drop

\ M_DIFF = Meter difference, deg/m is typically around 774 (1 deg = 0.774mm)
\ We compensate for the 10/ above by reducing one of the 1000's to 100.
90 1000 um* 100 ud* rot ud/ drop constant DEG/M_DIFF

\ UM/COUNT_LEFT . 
\ UM/COUNT_RIGHT .
\ DEG/M_DIFF . 




\ ==========================================================
\  Encoders
\ ==========================================================

\ ukmarsbot_forth
\ (c) 2022 Rob Probin 
\ MIT License, see LICENSE file
\ 
\ BRIEF: Deals with the encoders

\ Based on  ukmars mazerunner core
\ from ukmarsbot.forth

\ 0encoders
\ -encoders
marker -encoders

\ Variables
\ =========

\ None of the variables in this file should be directly available to the rest
\ of the code without a guard to ensure atomic access. 

2variable my_distance
2variable my_angle

\ the change in distance or angle in the last tick.
variable fwd_change
variable rot_change

\ internal use only to track encoder input edges
variable left_count
variable right_count


\ define the pins
\ ===============

flash
PORTD 2 defPIN: ENC_LEFT_CLK
PORTD 3 defPIN: ENC_RIGHT_CLK
PORTD 4 defPIN: ENC_LEFT_B
PORTD 5 defPIN: ENC_RIGHT_B
ram

$69 constant EICRA
$3d constant EIMSK

\ Interrupt Service routines
\ ==========================

\ The ISR will respond to the XOR-ed pulse train from the encoder - i.e. 
\ both rising and falling interrupts

\ These are called on quite a high freqency for the ATmega328, so we 
\ need to be careful we only do the minimum possible. The calculation
\ work is done by a slower interrupt - like a 2ms timer interrupt.

\ left input change - interrupt service routine
variable l_oldA
variable l_oldB
: left_isr
    ENC_LEFT_B pin@
    ENC_LEFT_CLK pin@ over xor \ fix the A-xor-B input to A only.
    2dup
    \ we need to check the ENCODER_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    swap l_oldA @ xor ( newB newA newB newA -- newB newA newA newB^oldA )
    swap l_oldB @ xor (  -- newB newA newB^oldA newA^oldB )
    - ENC_LEFT_POL ?negate ( -- newB newA delta )
    left_count +!   \ add delta onto counter

    \ store for next interrupt
    l_oldA !
    l_oldB !
;i

\ right input change - interrupt service routine
variable r_oldA
variable r_oldB
: right_isr
    ENC_RIGHT_B pin@
    ENC_RIGHT_CLK pin@ over xor \ fix the A-xor-B input to A only.
    2dup
    \ we need to check the ENCODER_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    swap r_oldA @ xor ( newB newA newB newA -- newB newA newA newB^oldA )
    swap r_oldB @ xor (  -- newB newA newB^oldA newA^oldB )
    - ENC_RIGHT_POL ?negate ( -- newB newA delta )
    right_count +!   \ add delta onto counter

    \ store for next interrupt
    r_oldA !
    r_oldB !
;i


\ General Words
\ =============

: enc_reset
    \ atomic block
    di
    0 left_count !
    0 right_count !
    0 my_distance !
    0 my_angle !
    ei
;

\ this update function should be called from the periodic timer interrupt
\ The rate should be LOOP_FREQ.
\ 
: enc_update
    right_count left_count
    \ Make sure values don't change while being read. Be quick.
    di
        @ swap @
        0 left_count !
        0 right_count !
    ei
    ( left_count right_count )

    \ calculate the change in micrometers
    \ notice: if the count is about 153, depending on the size of UM_COUNT_x
    \ then this could overflow 16 bit.
    \ We get about 2357 counts per meter, so at 1 m/s we get 2357 counts
    \ ... for 2ms (500Hz) 
    \ that would be 4 or 5 counts. This seems safe to keep in a u16.
    UM/COUNT_RIGHT *
    swap
    UM/COUNT_LEFT *
    ( right left -- )

    \ forward is sum
    \ rotation is difference
    2dup
    + 2/ fwd_change !
    - DEG/M_DIFF * rot_change !

    \ update cumulatives figures
    my_distance @ fwd_change @  m+ my_distance 2!
    my_angle @ rot_change @ m+ my_angle 2!
;

: enc_setup
    0 l_oldA !
    0 l_oldB !
    0 r_oldA !
    0 r_oldB !
    ENC_LEFT_CLK input
    ENC_LEFT_B input
    ENC_RIGHT_CLK input
    ENC_RIGHT_B input
    enc_reset

    \ attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), callback_left, CHANGE);
    \ attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), callback_right, CHANGE);
    di
        %00000101 EICRA mset \ set interupt triggers (rising or falling edges = CHANGE)
        %00000011 EIMSK mset \ Enable Int0 & Int1
        ['] left_isr $0002 int! \ Int0
        ['] right_isr $0003 int! \ Int1
     \   ['] enc_update is systick_update
    ei
;


: 0encoders
  di
   %00000011 EIMSK mclr \ disable Int0 & Int1
  \ ['] systick_dummy is systick_update
  ei
;

\ these access the variables in a safe manner
: distance ( -- distance distance )
    my_distance di 2@ ei
;

\ : speed ( -- speed )
\    my_speed di @ ei
\    LOOP_FREQ *
\ ;

: omega ( -- omega )
    rot_change di @ ei
    LOOP_FREQ *
;

: fwd_change@  ( -- distance )
    fwd_change di @ ei
;

: rot_change@ ( -- distance )
    rot_change di @ ei
;

: angle ( -- angle angle )
    my_angle di 2@ ei
;


