\ Drag race code for robots based on the ukmarsbot platform
\ Written in Forth for the ATmega328P
\
\ (c) 2022-2024 Peter Probin and Rob Probin 
\ MIT License, see LICENSE file
\ 
\ BRIEF: loads forth files into the robot.
\ 
\ NOTE1: This file loads the files into the robot to make a complete robot.
\        The order is important.
\ 
\ NOTE2: #include and #require isn't standard Forth - include and require are 
\        used by the terminal program. We use Zeptocom.js to load the files into 
\        the robot.
\        Alternatively, you can do it manually.

\ #include hw_base.fth

\ ==========================================================
\ Stubs for main run loop
\ ==========================================================
marker -run

: not 0= ;

flash
\ left green LED is LED5 and is on D11 = PB3
PORTB 3 defPIN: LLED
\ right yellow LED is LED4 and is on D6 = PD6
PORTD 6 defPIN: RLED
\ Three three IR LEDs are on D12 = PB4
PORTB 4 defPIN: IRLED

ram

: toggle  ( pinmask portadr -- )
  2dup mtst
  if
    mclr
  else
    mset
  then
;

: ir_on ( -- )
  IRLED high
;

: ir_off ( -- )
  IRLED low
;

: init.ports2 ( -- )
  init.ports
  LLED output
  LLED low
  RLED output
  RLED low

  IRLED output
  IRLED low
;

variable mark_left
variable mark_right
variable left_side
variable right_side
variable pos_offset
variable old_pos_offset
variable sw_but
variable bat_raw

\ switch decode
\ levels are: 
\   251 = button pressed
\   163 = no switches, no buttons

\   163 = 1_ 2_ 3_ 4_ (all off)
\   160 = 1_ 2_ 3_ 4^ (4 on)
\   156 = 1_ 2_ 3^ 4_ (3 on)
\   152 = 1_ 2_ 3^ 4^ (3+4 on)
\   146 = 1_ 2^ 3_ 4_ (2 on)
\   141 = 1_ 2^ 3_ 4^ (2+4 on)
\   135 = 1_ 2^ 3^ 4_ (2+3 on)
\   129 = 1_ 2^ 3^ 4^ (2+3+4 on)

\   115 = 1^ 2_ 3_ 4_ (1 on)
\   108 = 1^ 2_ 3_ 4^ (1 on)
\    97 = 1^ 2_ 3^ 4_ (1+3 on)
\    87 = 1^ 2_ 3^ 4^ (1+3+4 on)
\    69 = 1^ 2^ 3_ 4_ (1+2 on)
\    54 = 1^ 2^ 3_ 4^ (1+2+4 on)
\    33 = 1^ 2^ 3^ 4_ (1+2+3 on) (or 34)
\    13 = 1^ 2^ 3^ 4^ (all on)

flash
create swtable 
  207 c, 162 c, 158 c, 154 c, 
  149 c, 144 c, 138 c, 132 c, 
  122 c, 112 c, 103 c, 92 c, 
  78 c, 61 c, 43 c, 23 c, 
  -1 c,
ram

\ sw returns -1 for button pressed, or 0-15 for switch position
\ sw is 0 for all switches off, 15 for all switches on
\ sw is 1 for switch 4 on, 2 for switch 3 on, 4 for switch 2 on, 8 for switch 1 on
\ i.e. value reads like binary of the switch
: >sw ( -- n )
  swtable
  begin 
    dup c@ sw_but @ 
    > while
    1+
  repeat
  swtable -
  1-
  dup 16 = if 1- then
;

: scan_sensor ( ch -- n )
  dup analogRead8
  ir_on
  swap analogRead8
  ir_off
  ( dark light ... light is smaller than dark )
  -
;

: um*256/ ( n1 n2 -- n3 )
  um* 8 lshift swap 8 rshift +
;


\ my blackboard painted board with 19mm white tape
\ marker left 4 69 
\ marker right 1 64 
\ line left 16 103 
\ line right 2 74 

eeprom
4 value minML
69 value maxML

1 value minMR
64 value maxMR

16 value minLL
103 value maxLL

2 value minLR
74 value maxLR
ram

variable scaleLL
variable scaleLR

eeprom 30000 value pos_max ram

variable 1/8_normal

: setscale
  \ considered making 30000 into 65536, but if 
  \ we get a reading bigger than maxLL or maxLR
  \ then we overflow the 16 bit number. 
  \ 
  \ Also pos_offset is signed...
  pos_max maxLL minLL - 1 max u/ scaleLL !
  pos_max maxLR minLR - 1 max u/ scaleLR !
  ." Scale " scaleLL @ . scaleLR @ . cr
  
  pos_max 8 / 1/8_normal !
  ." 1/8 "  1/8_normal @ . cr
;

: <1/8 ( n -- flag )
  1/8_normal @ <
;

: setpos
  \ work out a left side vs. right side shift
  left_side @ minLL - 0 max scaleLL @ *
  right_side @ minLR - 0 max scaleLR @ *
  ( left_normalised right_normalised )
  2dup
  - 
  \ right is positive, left is negative
  \ we store the difference in pos_offset ... 
  pos_offset !
  
  
  <1/8 if
    \ right is less than 1/8
    <1/8 if 
        \ both left and right is less than 1/8
        \ we need to use the previous state
        old_pos_offset @ 0< if
          \ previous was left
          pos_max negate pos_offset !
        else
          \ previous was right
          pos_max pos_offset !
        then
    else
      \ only right is less than 1/8
      \ we assume left - which is negative
      pos_max negate pos_offset !
    then
  else  
    \ right > 1/8
    <1/8 if 
      \ only left is less than 1/8
      \ right is > 1/8 - so we assume right
      pos_max pos_offset !
    else
      \ both left or right are great than 1/8
      \ this is in the middle of the tape - and the difference
      \ will give us a good result
      \ we can leave pos_offset as is - the difference
      \ between left and right
    then
  then
  pos_offset @ old_pos_offset !
;


: scan_IR
  3 scan_sensor mark_left !
  2 scan_sensor left_side !
  1 scan_sensor right_side !
  0 scan_sensor mark_right !
  setpos
;
: scansw 
  6 analogRead8 sw_but !
;

: scanbat
  \ read the battery
  7 analogRead8 bat_raw !
;

: bat_mV ( -- mV )
  \ 255 = 5v at input. The potential divider is /2
  \ to get to voltage in mV
  \ (ADC / 255) * 5000 * 2
  \ we approximate 255 to 256
  bat_raw @
  10000 um*256/ 
;

: mV>ADC ( mV -- ADC )
  \ mV = (ADC / 255) * 5000 * 2
  \ ADC = mV / 10000 * 255
  \ 
  \ example 9v = 9000
  \ ADC = 9000 / 10000 * 255
  \ ADC = 229.5

  \ Unsigned 16x16 to 32 bit multiply. ( u1 u2 — ud )
  255 um*
  \ Unsigned division. ( ud u1 — u.rem u.quot ) 32-bit/16-bit to 16-bit .. Divide ud by u1
  10000 um/mod nip
;


: scan_ADCs
  scan_IR
  scansw
  scanbat
;

: bat 
  scanbat 
  bat_mV . ." mV" cr 
;

: rawread2 
  0 analogRead8 . 
  1 analogRead8 . 
  2 analogRead8 . 
  3 analogRead8 . 
  ir_on 0 analogRead8 .
  1 analogRead8 . 
  2 analogRead8 . 
  3 analogRead8 . 
  ir_off
; 

: .sens ( -- )
  ." << " left_side @ . ."  >> " right_side @ . 
  ."  L " mark_left @ . ."  R " mark_right @ .
  ." POS " pos_offset @ . 
  ."  SW " sw_but @ . ." ["  >sw . ." ]" 
  ." V " bat_mV .
  ."    | " rawread2
  cr
;

: WatchSens ( -- ) 
  init.ports2
  analog.init
  setscale

  begin
    scan_ADCs
    .sens
    500 ms
  key? until
  key drop
;

: button
  scansw
  >sw -1 =
;


variable flash_time
variable led_last

: LED_flash ( type -- )
  ticks led_last @ -
  flash_time @ > if
    dup 0= if
      LED toggle
    else
      LLED toggle
      RLED toggle
    then
    flash_time @ led_last +!
  then
  drop
;

: flash! ( n-- )
  flash_time !
  ticks led_last !
;


0 constant BUTTON_UP
-1 constant BUTTON_DOWN

: wait_button ( state --)
    500 flash!

    0 \ debounce count
    begin
        5 ms \ debounce
        over button = if
          1+
        else
          drop 0
        then
        0 LED_flash
        key? if 2drop quit then
    dup 10 = until
    2drop
;

eeprom 40 value llevel ram
eeprom 40 value rlevel ram

: left* ( -- result )
  mark_left @ llevel > 
;

: right* ( -- result )
  mark_right @ rlevel > 
;

: marker* ( -- result )
  left* right* or
;
: nomarker* ( -- result )
  marker* 0=
;

\ : line* ( selected -- result )
\ ;


variable speed
variable steering
variable start_time

20 constant MIN_SPEED \ starts going about 25
eeprom 255 value MAX_SPEED ram  \ pwm value

\ 200 constant ACCEL_TIME  \ in milliseconds
eeprom 500 value ACCEL_TIME ram \ in milliseconds

\ so we go from MIN_SPEED to 255 in ACCELERATION_TIME ms

: start_speed ( -- )
  MIN_SPEED speed !
  ticks start_time !
;

: top_speed ( -- flag )
  speed @ MAX_SPEED =
;

\ Over ACCELERATION_TIME ms we go from MIN to 255
\ each millisecond we need to add ((255-MIN) / ACCELERATION_TIME) to the speed.
\
\ Assume t is (ticks - start_time) in milliseconds
\ so at time t we have:
\         speed = MIN + t * ((255-MIN) / ACCELERATION_TIME)
\ speed = MIN + (t * (255-MIN) / ACCELERATION_TIME)
: acc_calc ( t -- speed )
  MAX_SPEED MIN_SPEED - 
  um*         \ Unsigned 16x16 to 32 bit multiply. ( u1 u2 — ud )
  ACCEL_TIME
  \ Unsigned division. ( ud u1 — u.rem u.quot ) 32-bit/16-bit to 16-bit
  um/mod nip
  MIN_SPEED +
;

\ 6v is minimum battery voltage we run at!

6000 mV>ADC constant bat_low_ADC

: bat_flash
  LLED low RLED low
  begin
    LED high 100 ms LED low 100 ms
    LED high 100 ms LED low 100 ms
    600 ms
  key? until
  key drop
;

variable bat_low_flag

: battery_check ( --  )
  bat_raw @ bat_low_ADC < if
    true bat_low_flag !

    \ stop the robot
    mstop
    ." *** Bat low" cr
    bat_flash
    ." *** Bat low" cr
  then
;

: do_acceleration ( -- )
  top_speed if
    \ already at top speed
    exit
  then
  ticks start_time @ -  \ calculate t
  ACCEL_TIME min  \ cap at ACCEL_TIME
  acc_calc speed !
;

variable time5ms

: init5ms
  ticks 5 + time5ms !
;

eeprom
-100 value minsteer
100 value maxsteer
256 value 1/Kp
ram

400 constant arrSize
\ allocate 800 bytes for the debug array
create DBGarray arrSize 2* allot
variable myOffset

: clrDATA
  arrSize for $aa DBGarray r@ + c! next
  0 myOffset !
;

: strDATA ( n -- )
  myOffset @ arrSize < if
    myOffset @ DBGarray + !
    2 myOffset +!
  else
    drop
  then
;

: showDATA
  hex
  DBGarray arrSize dump
  decimal
;

: 5ms? 
  ticks time5ms @ -  \ calculate t
  0 > 
  dup if init5ms then
;


: do_steering
  \ PID controller, with just the proportional term
  pos_offset @ 1/Kp /    \ Pout = Kp e(t) + P0
  minsteer max maxsteer min steering !
  steering @ 0< if
    RLED high
    LLED low
  else steering @ 0 > if
    RLED low
    LLED high
  then then
;

: steertest
  init.ports2
  analog.init
  setscale

  begin
    scan_IR
    do_steering
    ." (" left_side @ . ." <> " right_side @ . ." )" 
    pos_offset @ . steering @ . cr
    333 ms
  key? until
  key drop
;

\ we need to scale the speed based on the battery voltage
variable MinV/NowV

\ we want to calculate the ratio of the minimum voltage to the current voltage
: CalcPWMScale ( -- )
  \ we want to scale the PWM so that the robot goes at the same speed at any battery voltage
  \ we want to scale the PWM so that the robot goes at the same speed at any battery voltage
  \ if the battery is 6v, then bat_raw = bat_low_ADC
  \ if the battery is 9v, then bat_raw = 1.5 * bat_low_ADC
  \ if the battery is 9v then we only want 2/3 of the PWM value
  \                 (equivalent of 6v / 9v)
  \ PWM_out = PWM_in * ( bat_low_ADC / bat_raw )
  \ here we calculate ((bat_low*65536)/current)

  \ um* = Unsigned 16x16 to 32 bit multiply. ( u1 u2 — ud )
  0 bat_low_ADC \ bat_low_ADC 65536 um*  
  \ um/mod = Unsigned division. ( ud u1 — u.rem u.quot ) 32-bit/16-bit to 16-bit .. Divide ud by u1 
  bat_raw @ um/mod nip 
  
  MinV/NowV !
;

\ we scale the PWM drive based on the battery voltage
\ we want to scale the PWM so that the robot goes at the same speed at any battery voltage
: scale_PWM ( PWM_in -- PWM_out )
  \ PWM goes from 0 to 255 (usually not negative for us)
  \ 
  \ bat_raw - ADC reading of the battery taken every 20ms
  \ bat_low_ADC - a voltage relating to 6v
  \ so if the battery is 6v, then bat_raw = bat_low_ADC
  \ if the battery is 9v, then bat_raw = 1.5 * bat_low_ADC
  \ if the battery is 9v then we only want 2/3 of the PWM value
  \                 (equivalent of 6v / 9v)
  \
  \ PWM_out = PWM_in * ( bat_low_ADC / bat_raw )
  
  MinV/NowV @
  \ to avoid a each loop divide, we can multiply by 256 and divide by 256
  um*   \ Unsigned 16x16 to 32 bit multiply. ( u1 u2 — ud )
;


: set_motors
  steering @ 0< if
    \ negative is left, so we want to move right 
    \ which means we slow the right motor
    speed @              scale_PWM lmotor!
    speed @ steering @ + scale_PWM rmotor!
  else
    \ positive is right, so we want to move left
    \ which means we slow the left motor
    speed @ steering @ - scale_PWM lmotor!
    speed @              scale_PWM rmotor!
  then
;

: wait_mark
  \ check for either sensor to be triggered
  500 flash!
  begin 
    scan_IR
    nomarker*
  while 
    1 LED_flash
  repeat
;

: wait_nomark
  \ wait for sensor to clear
  100 flash!
  begin 
    scan_IR
    marker*
  while
    1 LED_flash
  repeat
;

\ ==========================================================
\ Main loop
\ ==========================================================

\ Overall Algorithm - Drag race robot
\ ===================================
\ 
\ * Wait for a button press
\ * Wait for left or right front sensor to be triggered
\ * Start the motors and accelerate the PWM from 0 to 255 over the period of 200 ms
\ * Keep the motors running until the robot detects the end of the track using left or right sensor
\ * Watch the line following sensors to keep the robot on track
\ *   If the robot goes off track, slow left/right motor as appropriate to bring it back on track - using a PID controller
\ *   Initially just the PID controller in proportional mode only, and see how it goes
\ * Stop the motors (PWM 0)

variable max_t
variable loopst

: Xtime
  ticks dup loopst @  -
  max_t @ max max_t !
  loopst !
;

variable min_mleft
variable max_mleft

variable min_mright
variable max_mright

variable min_lleft
variable max_lleft

variable min_lright
variable max_lright



: minmax ( min-addr value max-addr -- )
  2dup @ max swap !
  ( min-addr value ) 
  over @ min swap !
;

: calibrate
  50 flash!
  255 min_lleft  ! 0 max_lleft !
  255 min_lright ! 0 max_lright !
  255 min_mleft  !  0 max_mleft !
  255 min_mright !  0 max_mright !

  ." Calibrate" cr

  begin
    scan_IR
    min_lleft  left_side  @ max_lleft  minmax
    min_lright right_side @ max_lright minmax
    min_mleft  mark_left  @ max_mleft  minmax
    min_mright mark_right @ max_mright minmax
    5 ms

    1 LED_flash

    key? 
    BUTTON_DOWN button =
    or
  until
  key? if
    key drop
  else
    \ print and store min and max
    ." marker left " min_mleft @ . max_mleft @ . cr
    ." marker right " min_mright @ . max_mright @ . cr
    ." line left " min_lleft @ . max_lleft @ . cr
    ." line right " min_lright @ . max_lright @ . cr

    min_lleft @ to minLL
    max_lleft @ to maxLL
    min_lright @ to minLR
    max_lright @ to maxLR
    min_mleft @ to minML
    max_mleft @ to maxML
    min_mright @ to minMR
    max_mright @ to maxMR

    \ now set the marker trip levels
    maxML minML - 2/ to llevel
    maxMR minMR - 2/ to rlevel
    ." Marker levels " llevel . rlevel . cr
    setscale
  then
;



variable runT
variable batStart
variable batEnd

: show_run
  ." Run time " runT @ . cr
  ." Max loop t " max_t @ . cr
  ." Bat Start " batStart @ . ." mV" cr 
  ." Bat End " batEnd @ . ." mV" cr 
  showDATA
;

variable 5msPhase
\ count up 5ms ticks to 20ms
: Count5ms ( -- )
  5msPhase @ 1+ 
  3 and 5msPhase !
;

variable cycle_after

: race

  ." RACE: Waiting for marker trigger" cr

  wait_mark
  wait_nomark
  key? if exit then

  ." Run" cr

  LED high
  LLED low
  RLED low
  clrDATA

  setscale
  scanbat bat_mV batStart !

  0 max_t !
  start_speed
  ticks runT !
  ticks loopst !
  init5ms
  0 5msPhase !
  false cycle_after !

  begin 
    scan_IR
    do_acceleration

    \ tasks to do every 5ms
    5ms? if
      Count5ms
      \ 5ms tasks
      do_steering
      true cycle_after !
    else
        \ do this the loop after do_steering
      cycle_after @ if
          false cycle_after !

          5msPhase 1 = if
            \ 20ms tasks
            pos_offset @ strDATA
            steering @ strDATA
            speed @ strDATA
          then
          5msPhase 2 = if
            scanbat
          then
          5msPhase 3 = if
            CalcPWMScale
            battery_check
          then
      then
    then

    bat_low_flag not if
      \ set motors every loop
      set_motors
    then

    \ while we are accelerating we ignore the left and right sensors
    \ so we don't get triggered while crossing the start line
    marker* top_speed and \ marker at top speed should end run
    \ top_speed \ debug line 
    key? or
    bat_low_flag @ or
    Xtime
  until

  ticks runT @ - runT !
  scanbat bat_mV batEnd !

  \ run done  - stop
  mstop
  LED high
  show_run
;


: 1run
  init.ports2
  analog.init
  PWM_31250_HZ motor_pwm_freq

  setscale

  LLED low
  RLED low
  LED low

  ." Waiting for button press" cr



  BUTTON_UP wait_button
  BUTTON_DOWN wait_button
  BUTTON_UP wait_button
  key? if exit then

  >sw 8 = if
    calibrate
  then
  >sw 4 = if
    steertest
  then
  >sw 0 = if
    race
  then
;

: run 
  false bat_low_flag !

  \enc_setup
  begin
    battery_check
    bat_low_flag not if
      1run
    then
  key? until
  key drop
  \ 0encoders
  LLED low
  RLED low
  LED low
;


\ @TODO - Fix steering
\ @TODO - Fastest and slowest loop times
\ @TODO - Optimise acceleration time

' run is turnkey

.free
