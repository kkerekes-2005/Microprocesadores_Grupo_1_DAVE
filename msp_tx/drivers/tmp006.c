#include <hal/hal_i2c.h>
#include <drivers/tmp006.h>
#include "math.h"


void TMP006_config(void)
{
    // Specify slave address for TMP006
    HAL_I2C_setslave(TMP006_SLAVE_ADDRESS);

    // Reset TMP006
    HAL_I2C_write16(TMP006_WRITE_REG, TMP006_RST);

    // Wait for reset
    __delay_cycles(10000);

    // Power-up and re-enable device
    HAL_I2C_write16(TMP006_WRITE_REG, TMP006_POWER_UP | TMP006_CR_2);
}

int TMP006_read_device_id(void)
{
    // Specify slave address for TMP006
    HAL_I2C_setslave(TMP006_SLAVE_ADDRESS);

    // Return content of register
    return HAL_I2C_read16(TMP006_P_DEVICE_ID);
}

int TMP006_read_object_voltage(void)
{
    // Specify slave address for TMP006
    HAL_I2C_setslave(TMP006_SLAVE_ADDRESS);

    // Return content of register (signed 16-bit value)
    return (int16_t)HAL_I2C_read16(TMP006_P_VOBJ);
}

int TMP006_read_ambient_temperature(void)
{
    // Specify slave address for TMP006
    HAL_I2C_setslave(TMP006_SLAVE_ADDRESS);

    // Return significant bits of register. The cast keeps the sign for
    // temperatures below 0 C (a plain uint16 shift would lose it).
    return ((int16_t)HAL_I2C_read16(TMP006_P_TABT)) / 4;
}


#define TMP006_B0   -2.94e-5
#define TMP006_B1   -5.7e-7
#define TMP006_B2   4.63e-9
#define TMP006_C2   13.4
#define TMP006_TREF 298.15
#define TMP006_A2   -1.678e-5
#define TMP006_A1   1.75e-3
#define TMP006_S0   6.4e-14

float TMP006_get_temperature(void)
{
    // Get object voltage
    float Vobj = TMP006_read_object_voltage() * 156.25e-9; // 156.25 nV per LSB

    // Get ambient temperature
    float Tdie = TMP006_read_ambient_temperature() * 31.25e-3   // 31.25 mC per LSB
            + 273.15;                                           // Convert to Kelvin

    // Calculate object temperature
    float tdie_tref = Tdie - TMP006_TREF;
    float S = TMP006_S0 * (1 +
            TMP006_A1 * tdie_tref +
            TMP006_A2 * pow(tdie_tref,2));

    float Vos = TMP006_B0 +
            TMP006_B1 * tdie_tref +
            TMP006_B2 * pow(tdie_tref,2);

    float fVobj = (Vobj - Vos) + TMP006_C2 * (Vobj - Vos) * (Vobj - Vos);

    float Tobj = sqrt(sqrt(pow(Tdie,4) + fVobj / S))
            - 273.15;                                   // Kelvin -> *C

    return Tobj;
}

