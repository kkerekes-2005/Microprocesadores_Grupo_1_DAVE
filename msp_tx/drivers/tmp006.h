#ifndef HAL_TMP006_H_
#define HAL_TMP006_H_

// I2C SLAVE ADDRESS
#define TMP006_SLAVE_ADDRESS    0x40

// REGISTERS
#define TMP006_P_VOBJ           0x00
#define TMP006_P_TABT           0x01
#define TMP006_WRITE_REG        0x02
#define TMP006_P_MAN_ID         0xFE
#define TMP006_P_DEVICE_ID      0xFF

// CONFIG REGISTER FLAGS
#define TMP006_RST              0x8000
#define TMP006_POWER_DOWN       0x0000
#define TMP006_POWER_UP         0x7000
#define TMP006_CR_4             0x0000
#define TMP006_CR_2             0x0200
#define TMP006_CR_1             0x0400
#define TMP006_CR_0_5           0x0600
#define TMP006_CR_0_25          0x0800
#define TMP006_EN               0x0100
#define TMP006_DRDY             0x0080

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Configures the TMP006 temperature sensor (I2C).
//!
//! Powers the sensor up and sets the conversion rate. Call once after
//! configuring the I2C HAL and before reading the temperature.
//!
//! \return None.
//
//*****************************************************************************
void TMP006_config(void);

//*****************************************************************************
//
//! \brief Reads the temperature in degrees Celsius.
//!
//! Reads the sensor die temperature (infrared thermopile) and converts it
//! as described in the datasheet.
//!
//! \return Temperature in degrees C.
//
//*****************************************************************************
float TMP006_get_temperature(void);

//*****************************************************************************
//
//! \brief Reads the sensor device identifier.
//!
//! Useful to verify the I2C communication.
//!
//! \return Contents of the DEVICE_ID register.
//
//*****************************************************************************
int TMP006_read_device_id(void);

//*****************************************************************************
//
//! \brief Reads the raw object voltage (VOBJ register).
//!
//! \return Contents of the VOBJ register.
//
//*****************************************************************************
int TMP006_read_object_voltage(void);

//*****************************************************************************
//
//! \brief Reads the raw ambient temperature (TABT register).
//!
//! \return Contents of the TABT register (die temperature, unconverted).
//
//*****************************************************************************
int TMP006_read_ambient_temperature(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_TMP006_H_ */
