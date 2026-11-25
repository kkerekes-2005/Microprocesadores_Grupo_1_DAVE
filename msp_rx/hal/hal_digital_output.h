#ifndef HAL_HAL_DIGITAL_OUTPUT_H_
#define HAL_HAL_DIGITAL_OUTPUT_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Configures the EDUMKII RGB LED pins and the LaunchPad LEDs as
//! outputs.
//!
//! RGB: R=P3.6, G=P3.5, B=P3.4. LaunchPad: LED1=P1.0, LED2=P1.1.
//! \b Warning: the RGB pins are the same ones used by the PWM HAL; in a
//! given project drive the RGB either with this HAL or with the PWM one,
//! not both. Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_init_gpio();

//*****************************************************************************
//
//! \brief Drives all outputs low (LEDs off).
//!
//! Call AFTER PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_config();

//*****************************************************************************
//
//! \brief Turns the RED channel of the RGB LED (P3.6) on or off.
//!
//! \param status true = on, false = off.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_set_R(bool status);

//*****************************************************************************
//
//! \brief Turns the GREEN channel of the RGB LED (P3.5) on or off.
//!
//! \param status true = on, false = off.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_set_G(bool status);

//*****************************************************************************
//
//! \brief Turns the BLUE channel of the RGB LED (P3.4) on or off.
//!
//! \param status true = on, false = off.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_set_B(bool status);

//*****************************************************************************
//
//! \brief Turns the LaunchPad LED1 (P1.0) on or off.
//!
//! \param status true = on, false = off.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_set_LED1(bool status);

//*****************************************************************************
//
//! \brief Turns the LaunchPad LED2 (P1.1) on or off.
//!
//! \param status true = on, false = off.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGOUT_set_LED2(bool status);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_DIGITAL_OUTPUT_H_ */
