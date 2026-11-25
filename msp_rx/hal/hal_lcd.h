#ifndef __HAL_MSP_EXP430FR5994_CRYSTALFONTZLCD_H_
#define __HAL_MSP_EXP430FR5994_CRYSTALFONTZLCD_H_


#include <stdint.h>
#include "driverlib.h"
//*****************************************************************************
//
// User Configuration for the LCD Driver
//
//*****************************************************************************

// System clock speed (in Hz)
#define LCD_SYSTEM_CLOCK_SPEED                 16000000
// SPI clock speed (in Hz)
#define LCD_SPI_CLOCK_SPEED                    16000000

// Ports from MSP430 connected to LCD
#define LCD_SCK_PORT          GPIO_PORT_P5
#define LCD_SCK_PIN_FUNCTION  GPIO_PRIMARY_MODULE_FUNCTION
#define LCD_MOSI_PORT         GPIO_PORT_P5
#define LCD_MOSI_PIN_FUNCTION GPIO_PRIMARY_MODULE_FUNCTION
#define LCD_RST_PORT          GPIO_PORT_P5
#define LCD_CS_PORT           GPIO_PORT_P8
#define LCD_DC_PORT           GPIO_PORT_P4

// Pins from MSP432 connected to LCD
#define LCD_SCK_PIN           GPIO_PIN2
#define LCD_MOSI_PIN          GPIO_PIN0
#define LCD_RST_PIN           GPIO_PIN3
#define LCD_CS_PIN            GPIO_PIN3
#define LCD_DC_PIN            GPIO_PIN1

// Definition of USCI base address to be used for SPI communication
#define LCD_EUSCI_BASE        EUSCI_B1_BASE

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Sends a COMMAND byte to the LCD controller over SPI.
//!
//! Drives the D/C line to command and transmits over eUSCI_B1. Used by the
//! crystalfontz driver; normally not called from the application.
//!
//! \param command is the ST7735 command to send.
//!
//! \return None.
//
//*****************************************************************************
extern void HAL_LCD_writeCommand(uint8_t command);

//*****************************************************************************
//
//! \brief Sends a DATA byte to the LCD controller over SPI.
//!
//! Drives the D/C line to data and transmits over eUSCI_B1. Used by the
//! crystalfontz driver; normally not called from the application.
//!
//! \param data is the data byte to send.
//!
//! \return None.
//
//*****************************************************************************
extern void HAL_LCD_writeData(uint8_t data);

//*****************************************************************************
//
//! \brief Configures the LCD pins (SPI + control lines).
//!
//! SCK=P5.2, MOSI=P5.0 (eUSCI_B1), RST=P5.3, CS=P8.3, D/C=P4.1.
//! Call BEFORE PMM_unlockLPM5().
//!
//! \return None.
//
//*****************************************************************************
extern void HAL_LCD_init_gpio(void);

//*****************************************************************************
//
//! \brief Configures eUSCI_B1 as SPI master for the LCD.
//!
//! Call AFTER PMM_unlockLPM5() and before CRYSTALFONTZ_init().
//!
//! \return None.
//
//*****************************************************************************
extern void HAL_LCD_config(void);

#ifdef __cplusplus
}
#endif

#define HAL_LCD_delay(x)      __delay_cycles(x * 16)

#endif /* HAL_MSP_EXP430FR5994_CRYSTALFONTZ128X128_ST7735_H_ */
