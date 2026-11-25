drivers/ - one driver per BoosterPack device
============================================

This folder is already on the compiler include path
(-I${PROJECT_ROOT}/drivers) and starts empty on purpose. Copy here the drivers
you need, one device at a time, from the edumkiilib example that uses them
(they live in libs/edumkiilib/drivers/):

  crystalfontz.c/.h   LCD (needs hal_lcd, and GRLIB for drawing)
  switch.c/.h         push-buttons S1/S2      (needs hal_digital_input)
  led.c/.h            RGB LED, in two variants: led/rgb_digout (on/off, needs
                      hal_digital_output) and led/rgb_pwm (0-255, needs hal_pwm)
  buzzer.c/.h         buzzer                  (needs hal_pwm)
  joystick.c/.h       joystick                (needs hal_adc)
  kxtc9_2050.c/.h     accelerometer           (needs hal_adc)
  opt3001.c/.h        light sensor            (needs hal_i2c)
  tmp006.c/.h         infrared temperature    (needs hal_i2c)
  cli.c/.h            command line over the backchannel UART (needs hal_uart)

Each driver needs its HAL in hal/ as well. The ones that talk to a device
(CRYSTALFONTZ, TMP006, OPT3001...) are initialized in main() after the HAL
config calls and after global interrupts are enabled.
