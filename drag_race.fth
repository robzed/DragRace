\ ukmarsbot_forth
\ (c) 2022-2024 Rob Probin 
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
\   152 = 1_ 2_ 3^ 4^ (1 on)
\   146 = 1_ 2^ 3_ 4_ (2 on)
\   141 = 1_ 2^ 3_ 4^ (1 on)
\   135 = 1_ 2^ 3^ 4_ (2+3 on)
\   129 = 1_ 2^ 3^ 4^ (1 on)
\   115 = 1^ 2_ 3_ 4_ (1 on)
\   108 = 1^ 2_ 3_ 4^ (1 on)
\    97 = 1^ 2_ 3^ 4_ (1+3 on)
\    87 = 1^ 2_ 3^ 4^ (1 on)
\    69 = 1^ 2^ 3_ 4_ (1+2 on)
\    54 = 1^ 2^ 3_ 4^ (1 on)
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


: scan_IR
  3 scan_sensor mark_left !
  2 scan_sensor left_side !
  1 scan_sensor right_side !
  0 scan_sensor mark_right !
  \ work out a left side vs. right side shift
  left_side @ right_side @ - pos_offset !
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

eeprom 128 value llevel ram
eeprom 128 value rlevel ram

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

: do_steering
  ticks st_time @ -  \ calculate t
  0 > if
    init_steer
    \ PID controller, with just the proportional term
    pos_offset @ 2/    \ Pout = Kp e(t) + P0
    -20 max 20 min steering !
    steering @ 0< if
      RLED high
      LLED low
    else steering @ 0 > if
      RLED low
      LLED high
    then then
  then
;

: steertest
  init.ports2
  analog.init
  init_steer
  begin
    scan_IR
    do_steering
    pos_offset @ . steering @ . cr
    333 ms
  key? until
  key drop
;


: set_motors
  steering @ 0< if
    speed @ steering @ negate - lmotor!
    speed @ rmotor!
  else
    speed @ lmotor!
    speed @ steering @ negate - rmotor!
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


: 1run
  init.ports2
  analog.init
  PWM_31250_HZ motor_pwm_freq

  LLED low
  RLED low
  LED low

  ." Waiting for button press" cr

  BUTTON_UP wait_button
  BUTTON_DOWN wait_button
  BUTTON_UP wait_button
  key? if exit then

  ." Waiting for marker trigger" cr

  wait_mark
  wait_nomark
  key? if exit then

  ." Run" cr

  LED high
  LLED low
  RLED low

  start_speed
  begin 
    scan_IR
    do_acceleration
    do_steering
    set_motors
    \ while we are accelerating we ignore the left and right sensors
    \ so we don't get triggered while crossing the start line
    marker* top_speed and \ marker at top speed should end run
    \ top_speed \ debug line 
    key? or
  until

  \ run done  - stop
  mstop
  LED high
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
\ @TODO - how do we learn the sensor levels? 
\ @TODO - Fastest and slowest loop times
\ @TODO - Optimise acceleration time

' run is turnkey