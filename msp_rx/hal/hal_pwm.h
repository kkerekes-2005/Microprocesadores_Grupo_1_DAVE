#ifndef HAL_HAL_PWM_H_
#define HAL_HAL_PWM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Configures the buzzer pin as a PWM output.
//!
//! P3.7 = TB0.6 (primary function). Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_PWM_init_gpio();

//*****************************************************************************
//
//! \brief Initializes Timer B0 with the buzzer silent.
//!
//! Call AFTER PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_PWM_config();

//*****************************************************************************
//
//! \brief Makes the buzzer sound at the given frequency.
//!
//! The tone is the PWM frequency: the TB0 period is set to 16 MHz / freq
//! with a 50% duty cycle. With freq = 0 the buzzer goes silent.
//!
//! \param freq frequency in Hz (0 = silence; practical minimum ~250 Hz).
//!
//! \return None.
//
//*****************************************************************************
void HAL_PWM_update_buzzer_freq(int freq);

//*****************************************************************************
//
//! \brief Rewrites the Timer B0 configuration from the current state.
//!
//! Internal HAL function (single writer of TB0); normally not called from
//! the application.
//!
//! \return None.
//
//*****************************************************************************
void HAL_PWM_update_tb0();

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_PWM_H_ */
