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

: scan_ADCs
;

0 constant BUTTON_UP
1 constant BUTTON_DOWN

: wait_button ( state --)
    0 \ debounce count
    begin
        scan_ADCs
        \ adc_button @
        \ button pin@
        5 ms \ debounce
    dup 10 = until
;

: sensor ( selected -- result )
  drop 0
;
1 constant left
2 constant right
4 constant line

: LED_flash ( time -- )
  drop
;

variable current_speed
variable steering_output

: top_speed ( -- flag )
  true
;

: do_steering
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
  init.ports
  LED low

  BUTTON_UP wait_button
  BUTTON_DOWN wait_button
  BUTTON_UP wait_button

  \ check for either sensor to be triggered
  begin 
    scan_ADCs
    left sensor right sensor or 0=
  while 
    250 LED_flash
  repeat

  \ wait for sensor to clear
  begin 
    scan_ADCs
    left sensor right sensor or
  while
    100 LED_flash
  repeat
  
  
  LED low
  0 current_speed !
  begin 
    scan_ADCs
    left sensor right sensor or 0=
    \ while we are accelerating we ignore the left and right sensors
    \ so we don't get triggered while crossing the start line
    top_speed and
  while
    do_steering
  repeat

  \ run done  - stop
  mstop
;

: run 
  \enc_setup
  begin
    1run
  key? until
  \ 0encoders
;

