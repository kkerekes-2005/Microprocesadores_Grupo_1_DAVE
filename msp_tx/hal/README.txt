hal/ - hardware abstraction layer
=================================

This folder is already on the compiler include path (-I${PROJECT_ROOT}/hal) and
starts empty on purpose. Copy here the HAL files you need, one peripheral at a
time, from the edumkiilib example that uses them (they live in
libs/edumkiilib/hal/):

  hal_lcd.c/.h              SPI and control pins of the LCD
  hal_i2c.c/.h              I2C bus (OPT3001 light, TMP006 temperature)
  hal_uart.c/.h             backchannel UART over the USB debug port
  hal_digital_input.c/.h    EDUMKII push-buttons
  hal_digital_output.c/.h   LaunchPad LEDs and RGB LED driven as plain GPIO
  hal_adc.c/.h              analogue inputs, in three variants with the same
                            interface: adc/joystick (2 channels),
                            adc/kxtc9_2050 (3) and adc/all (5, both at once)
  hal_pwm.c/.h              timer outputs, in three variants with the same
                            interface: pwm/buzzer, pwm/rgb and pwm/all

Only ONE variant of each HAL can be in a project: they are different files with
the same name.

In main(), every HAL is used in two steps: <HAL>_init_gpio() BEFORE
PMM_unlockLPM5(), to mux its pins, and <HAL>_config() after it.

Note: hal_digital_output and hal_pwm share pins P3.4-P3.6 (the RGB LED). If you
copy both, call the PWM init_gpio last so that its muxing wins on those pins,
and drive the RGB LED through the PWM side.
