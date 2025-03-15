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
variable sw_but
variable bat_mV

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

: setscale
  \ considered making 30000 into 65536, but if 
  \ we get a reading bigger than maxLL or maxLR
  \ then we overflow the 16 bit number. 
  \ 
  \ Also pos_offset is signed...
  pos_max maxLL minLL - 1 max u/ scaleLL !
  pos_max maxLR minLR - 1 max u/ scaleLR !
  ." Scale " scaleLL @ . scaleLR @ . cr
;

: setpos
  \ work out a left side vs. right side shift
  left_side @ minLL - 0 max scaleLL @ *
  right_side @ minLR - 0 max scaleLR @ *
  ( left_normalised right_normalised )
  - 
  pos_offset !
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
  7 analogRead8 
  \ 255 = 5v at input. The potential divider is /2
  \ to get to voltage in mV
  \ (ADC / 255) * 5000 * 2
  \ we approximate 255 to 256
  10000 um*256/ bat_mV !
;
: scan_ADCs
  scan_IR
  scansw
  scanbat
;

: bat 
  scanbat 
  bat_mV @ . ." mV" cr 
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
  ." V " bat_mV @ .
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

: do_acceleration ( -- )
  top_speed if
    \ already at top speed
    exit
  then
  ticks start_time @ -  \ calculate t
  ACCEL_TIME min  \ cap at ACCEL_TIME
  acc_calc speed !
;

variable st_time

: init_steer 
  ticks 5 + st_time !
;

eeprom
-100 value minsteer
100 value maxsteer
256 value 1/Kp
ram

400 constant arrSize
create DBGarray arrSize allot
variable myOffset

: clrDATA
  arrSize for $aa DBGarray r@ + c! next
  0 myOffset !
;

: strDATA ( n -- )
  myOffset @ arrSize < if
    myOffset @ DBGarray + !
    1 myOffset +!
  else
    drop
  then
;

: showDATA
  hex
  DBGarray arrSize dump
  decimal
;

: do_steering
  ticks st_time @ -  \ calculate t
  0 > if
    init_steer
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

    steering @ strDATA

  then
;

: steertest
  init.ports2
  analog.init
  init_steer
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


: set_motors
  steering @ 0< if
    \ negative is left, so we want to move right 
    \ which means we slow the right motor
    speed @ lmotor!
    speed @ steering @ + rmotor!
  else
    \ positive is right, so we want to move left
    \ which means we slow the left motor
    speed @ steering @ - lmotor!
    speed @ rmotor!
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
  scanbat bat_mV @ batStart !

  0 max_t !
  start_speed
  ticks runT !
  ticks loopst !
  begin 
    scan_IR
    do_acceleration
    do_steering
    set_motors
    speed @ strDATA
    \ while we are accelerating we ignore the left and right sensors
    \ so we don't get triggered while crossing the start line
    marker* top_speed and \ marker at top speed should end run
    \ top_speed \ debug line 
    key? or
    Xtime
  until

  ticks runT @ - runT !
  scanbat bat_mV @ batStart !

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
  \enc_setup
  begin
    1run
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