#ifndef DRIVERS_BUZZER_H_
#define DRIVERS_BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Turns the buzzer on with a fixed 1 kHz tone.
//!
//! Delegates to the PWM HAL (Timer B0, output TB0.6 on P3.7) with a 50%
//! duty cycle.
//!
//! \return None.
//
//*****************************************************************************
void BUZ_sound_on();

//*****************************************************************************
//
//! \brief Turns the buzzer off.
//!
//! Sets the duty cycle of the buzzer PWM output to 0.
//!
//! \return None.
//
//*****************************************************************************
void BUZ_sound_off();

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BUZZER_H_ */
