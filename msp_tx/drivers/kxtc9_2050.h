#ifndef DRIVERS_KXTC9_2050_H_
#define DRIVERS_KXTC9_2050_H_

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Returns the X-axis acceleration in g.
//!
//! Converts the latest ADC conversion (channel A12, refreshed by calling
//! HAL_ADC_trigger_single_conversion()) using the offset (VDD/2) and the
//! sensitivity (660 mV/g) from the KXTC9-2050 datasheet.
//!
//! \return X acceleration in g (approx. -2.5 .. +2.5).
//
//*****************************************************************************
float KXTC9_get_accel_x();

//*****************************************************************************
//
//! \brief Returns the Y-axis acceleration in g.
//!
//! Same as KXTC9_get_accel_x() for the Y axis (channel A13).
//!
//! \return Y acceleration in g.
//
//*****************************************************************************
float KXTC9_get_accel_y();

//*****************************************************************************
//
//! \brief Returns the Z-axis acceleration in g.
//!
//! Same as KXTC9_get_accel_x() for the Z axis (channel A14). With the board
//! lying flat it should read approximately +1 g.
//!
//! \return Z acceleration in g.
//
//*****************************************************************************
float KXTC9_get_accel_z();

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_KXTC9_2050_H_ */
