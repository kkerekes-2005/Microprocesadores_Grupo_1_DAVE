#ifndef HAL_DIGITAL_INPUT_H_
#define HAL_DIGITAL_INPUT_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Flag set to true by the port ISR on a falling edge on S1 (EDUMKII, P4.3).
//! Consumed by the buttons driver (switch.c).
extern volatile bool flag_edumkii_S1;
//! Equivalent flag for S2 (EDUMKII, P4.2).
extern volatile bool flag_edumkii_S2;

//*****************************************************************************
//
//! \brief Configures the EDUMKII button pins as inputs.
//!
//! S1 on P4.3 and S2 on P4.2. Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGIN_init_gpio();

//*****************************************************************************
//
//! \brief Configures the button interrupts (falling edge).
//!
//! Enables the port interrupt of S1 and S2 and clears the flags. The port 4
//! ISR sets flag_edumkii_S1/S2. Call AFTER PMM_unlockLPM5() and enable the
//! global interrupts (GIE) afterwards.
//!
//! \return None.
//
//*****************************************************************************
void HAL_DIGIN_config();

#ifdef __cplusplus
}
#endif

#endif /* HAL_DIGITAL_INPUT_H_ */
