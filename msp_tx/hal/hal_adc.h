#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Latest conversion of the accelerometer X axis (channel A12, 0-4095).
extern uint16_t acc_x_raw;
//! Latest conversion of the accelerometer Y axis (channel A13, 0-4095).
extern uint16_t acc_y_raw;
//! Latest conversion of the accelerometer Z axis (channel A14, 0-4095).
extern uint16_t acc_z_raw;

//*****************************************************************************
//
//! \brief Configures the accelerometer pins as analog inputs.
//!
//! X on P3.0/A12, Y on P3.1/A13, Z on P3.2/A14 (ternary function).
//! Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_ADC_init_gpio();

//*****************************************************************************
//
//! \brief Configures the ADC12_B in 3-channel sequence mode.
//!
//! MEM0=A12, MEM1=A13 and MEM2=A14, 12 bits, with an interrupt at the end
//! of the sequence. The ISR stores the results in acc_x/y/z_raw.
//! Call AFTER PMM_unlockLPM5() and enable GIE afterwards.
//!
//! \return None.
//
//*****************************************************************************
void HAL_ADC_config();

//*****************************************************************************
//
//! \brief Triggers one conversion of the sequence and waits for completion.
//!
//! On return, the *_raw variables hold fresh values.
//!
//! \return None.
//
//*****************************************************************************
void HAL_ADC_trigger_single_conversion();

//*****************************************************************************
//
//! \brief Starts automatically repeated conversions of the sequence.
//!
//! The *_raw variables are refreshed continuously via ISR.
//!
//! \return None.
//
//*****************************************************************************
void HAL_ADC_trigger_continuous_conversion();

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H_ */
