#ifndef __CRYSTALFONTZLCD_H__
#define __CRYSTALFONTZLCD_H__

#include <driverlib.h>
#include <grlib.h>
#include <stdint.h>

// LCD Screen Dimensions
#define LCD_VERTICAL_MAX                   128
#define LCD_HORIZONTAL_MAX                 128

// LCD Screen Orientation Modes
#define LCD_ORIENTATION_UP    0
#define LCD_ORIENTATION_LEFT  1
#define LCD_ORIENTATION_DOWN  2
#define LCD_ORIENTATION_RIGHT 3

// ST7735 LCD controller Command Set
#define CM_NOP             0x00
#define CM_SWRESET         0x01
#define CM_RDDID           0x04
#define CM_RDDST           0x09
#define CM_SLPIN           0x10
#define CM_SLPOUT          0x11
#define CM_PTLON           0x12
#define CM_NORON           0x13
#define CM_INVOFF          0x20
#define CM_INVON           0x21
#define CM_GAMSET          0x26
#define CM_DISPOFF         0x28
#define CM_DISPON          0x29
#define CM_CASET           0x2A
#define CM_RASET           0x2B
#define CM_RAMWR           0x2C
#define CM_RGBSET          0x2d
#define CM_RAMRD           0x2E
#define CM_PTLAR           0x30
#define CM_MADCTL          0x36
#define CM_COLMOD          0x3A
#define CM_SETPWCTR        0xB1
#define CM_SETDISPL        0xB2
#define CM_FRMCTR3         0xB3
#define CM_SETCYC          0xB4
#define CM_SETBGP          0xb5
#define CM_SETVCOM         0xB6
#define CM_SETSTBA         0xC0
#define CM_SETID           0xC3
#define CM_GETHID          0xd0
#define CM_SETGAMMA        0xE0
#define CM_MADCTL_MY       0x80
#define CM_MADCTL_MX       0x40
#define CM_MADCTL_MV       0x20
#define CM_MADCTL_ML       0x10
#define CM_MADCTL_BGR      0x08
#define CM_MADCTL_MH       0x04

#ifdef __cplusplus
extern "C" {
#endif

//! grlib display descriptor: connects TI's generic graphics library with
//! this LCD. Use it in Graphics_initContext(&ctx, &g_sCRYSTALFONTZ).
extern Graphics_Display g_sCRYSTALFONTZ;

//*****************************************************************************
//
//! \brief Initializes the ST7735 controller of the Crystalfontz 128x128 LCD.
//!
//! Controller startup sequence over SPI (reset, power-up, color format).
//! Requires a previous call to HAL_LCD_config(). After this call the grlib
//! context can be created with Graphics_initContext().
//!
//! \return None.
//
//*****************************************************************************
void CRYSTALFONTZ_init(void);

//*****************************************************************************
//
//! \brief Defines the controller drawing window (draw frame).
//!
//! Support function used by the grlib driver; normally not called directly
//! from the application.
//!
//! \param x0 first column.
//! \param y0 first row.
//! \param x1 last column.
//! \param y1 last row.
//!
//! \return None.
//
//*****************************************************************************
void CRYSTALFONTZ_set_draw_frame(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

//*****************************************************************************
//
//! \brief Sets the screen orientation.
//!
//! \param orientation is the desired orientation.
//!        Valid values are: \b LCD_ORIENTATION_UP, \b LCD_ORIENTATION_LEFT,
//!        \b LCD_ORIENTATION_DOWN, \b LCD_ORIENTATION_RIGHT.
//!
//! \return None.
//
//*****************************************************************************
void CRYSTALFONTZ_set_orientation(uint8_t orientation);

#ifdef __cplusplus
}
#endif

#endif /* __CRYSTALFONTZLCD_H__ */
