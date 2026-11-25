#ifndef HAL_I2C_H_
#define HAL_I2C_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Configures the pins of the EDUMKII sensors I2C bus.
//!
//! SDA and SCL of eUSCI_B2. Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
void HAL_I2C_init_gpio(void);

//*****************************************************************************
//
//! \brief Configures eUSCI_B2 as I2C master.
//!
//! Call AFTER PMM_unlockLPM5(). The TMP006 and OPT3001 sensors share this
//! bus (each driver selects its own slave address).
//!
//! \return None.
//
//*****************************************************************************
void HAL_I2C_config(void);

//*****************************************************************************
//
//! \brief Reads a 16-bit register from the selected I2C slave.
//!
//! Writes the register pointer and reads two bytes (MSB first).
//!
//! \param pointer is the register address inside the slave.
//!
//! \return The 16-bit value read.
//
//*****************************************************************************
uint16_t HAL_I2C_read16(unsigned char pointer);

//*****************************************************************************
//
//! \brief Writes a 16-bit register in the selected I2C slave.
//!
//! \param pointer is the register address inside the slave.
//! \param writeByte is the 16-bit value to write.
//!
//! \return None.
//
//*****************************************************************************
void HAL_I2C_write16(unsigned char pointer, unsigned int writeByte);

//*****************************************************************************
//
//! \brief Selects the I2C slave address to talk to.
//!
//! Call before read16/write16 when switching sensors (e.g. between
//! TMP006_SLAVE_ADDRESS and OPT3001_SLAVE_ADDRESS).
//!
//! \param slaveAdr is the 7-bit slave address.
//!
//! \return None.
//
//*****************************************************************************
void HAL_I2C_setslave(unsigned int slaveAdr);

#ifdef __cplusplus
}
#endif

#endif /* HAL_I2C_H_ */
