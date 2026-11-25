#ifndef DRIVERS_SWITCHES_H_
#define DRIVERS_SWITCHES_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Checks whether the EDUMKII BoosterPack S1 button has been pressed.
//!
//! The driver keeps a software state machine (pressed / released) with an
//! independent 10 ms debounce window per edge. The press edge is latched by
//! the port ISR, so quick taps are never lost regardless of how fast the
//! main loop polls; the release is detected by level with its own debounce,
//! so release bounces never produce phantom presses. Reports exactly one
//! event per physical press (no auto-repeat while held). Call periodically
//! from the main loop (polling).
//!
//! \return true exactly once per press.
//
//*****************************************************************************
bool SWITCH_get_edumkii_S1();

//*****************************************************************************
//
//! \brief Checks whether the EDUMKII BoosterPack S2 button has been pressed.
//!
//! Same mechanism (ISR-latched edge + software state machine with per-edge
//! debounce) as SWITCH_get_edumkii_S1().
//!
//! \return true exactly once per press.
//
//*****************************************************************************
bool SWITCH_get_edumkii_S2();

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SWITCHES_H_ */
