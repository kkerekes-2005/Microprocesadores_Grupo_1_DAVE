#ifndef DRIVERS_LED_H_
#define DRIVERS_LED_H_

#include <stdint.h>
#include <stdbool.h>

#define CYCLE_OFF   0
#define CYCLE_RED   1
#define CYCLE_GREEN 2
#define CYCLE_BLUE  3

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Sets the EDUMKII RGB LED color (on/off per channel).
//!
//! DIGITAL variant: each channel is switched on or off as a digital output
//! (HAL_DIGOUT), with no intermediate intensities. For 0-255 intensity use
//! the rgb_pwm driver variant instead.
//!
//! \param r red channel (true = on).
//! \param g green channel (true = on).
//! \param b blue channel (true = on).
//!
//! \return None.
//
//*****************************************************************************
void LED_set_rgb(bool r,bool g,bool b);

//*****************************************************************************
//
//! \brief Advances the RGB LED through the red -> green -> blue sequence.
//!
//! Each call moves to the next color of the cycle. Useful as a visual
//! heartbeat.
//!
//! \return None.
//
//*****************************************************************************
void LED_cycle_rgb();

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_LED_H_ */
