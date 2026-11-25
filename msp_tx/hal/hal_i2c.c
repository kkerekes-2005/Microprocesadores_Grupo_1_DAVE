#include <msp430.h>
#include <driverlib.h>

// The I2C transactions in this HAL are implemented at register level with
// EVERY wait bounded by a timeout. The blocking DriverLib multi-byte API is
// not used on purpose: its internal polls are unbounded, so a dead bus (no
// BoosterPack pull-ups, or SCL stuck low) would hang the program forever.
// On any timeout the eUSCI is reset (UCSWRST) so the next call starts clean,
// and reads return 0.
#define I2C_TIMEOUT 50000u

// I2C Master Configuration Parameter
EUSCI_B_I2C_initMasterParam i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        16000000,                               // SMCLK = 16MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100 kHz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

void HAL_I2C_init_gpio()
{
    // Select I2C function for I2C_SCL(P7.1)
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P7,
            GPIO_PIN1,
            GPIO_PRIMARY_MODULE_FUNCTION);

    // Select I2C function for I2C_SDA(P7.0 )
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P7,
            GPIO_PIN0,
            GPIO_PRIMARY_MODULE_FUNCTION);
}


void HAL_I2C_config(void)
{
    // Initialize USCI_B2 and I2C Master to communicate with slave devices
    EUSCI_B_I2C_initMaster(EUSCI_B2_BASE, &i2cConfig);

    // Disable I2C module to make changes
    EUSCI_B_I2C_disable(EUSCI_B2_BASE);

    // Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B2_BASE);
}


// Waits (bounded) until the given UCB2IFG flags are set. Returns false on
// timeout, resetting the eUSCI so the bus state machine starts clean again.
static bool i2c_wait_flags(unsigned int flags)
{
    unsigned int timeout = I2C_TIMEOUT;
    while (!(UCB2IFG & flags) && --timeout);
    if (timeout == 0)
    {
        UCB2CTLW0 |= UCSWRST;       // abort: reset the eUSCI state machine
        UCB2CTLW0 &= ~UCSWRST;
        return false;
    }
    return true;
}

// Waits (bounded) until the given UCB2CTLW0 bits clear (UCTXSTT/UCTXSTP).
static bool i2c_wait_ctl_clear(unsigned int bits)
{
    unsigned int timeout = I2C_TIMEOUT;
    while ((UCB2CTLW0 & bits) && --timeout);
    if (timeout == 0)
    {
        UCB2CTLW0 |= UCSWRST;
        UCB2CTLW0 &= ~UCSWRST;
        return false;
    }
    return true;
}


uint16_t HAL_I2C_read16(unsigned char pointer)
{
    uint8_t content_MSB = 0;
    uint8_t content_LSB = 0;

    // --- Write phase: send the register pointer ---
    UCB2CTLW0 |= UCTR | UCTXSTT;            // transmitter + START
    if (!i2c_wait_flags(UCTXIFG0)) return 0;
    UCB2TXBUF = pointer;
    if (!i2c_wait_flags(UCTXIFG0)) return 0;
    UCB2CTLW0 |= UCTXSTP;                   // STOP
    if (!i2c_wait_ctl_clear(UCTXSTP)) return 0;

    // --- Read phase: receive two bytes (MSB first) ---
    UCB2CTLW0 &= ~UCTR;                     // receiver
    UCB2CTLW0 |= UCTXSTT;                   // START (sends slave address)
    if (!i2c_wait_flags(UCRXIFG0)) return 0;
    content_MSB = UCB2RXBUF;
    UCB2CTLW0 |= UCTXSTP;                   // STOP after the next byte
    if (!i2c_wait_flags(UCRXIFG0)) return 0;
    content_LSB = UCB2RXBUF;
    if (!i2c_wait_ctl_clear(UCTXSTP)) return 0;

    // Mount full content word
    return (content_MSB << 8) | content_LSB;
}


void HAL_I2C_write16 (unsigned char pointer, unsigned int writeByte)
{
    UCB2CTLW0 |= UCTR | UCTXSTT;            // transmitter + START
    if (!i2c_wait_flags(UCTXIFG0)) return;
    UCB2TXBUF = pointer;
    if (!i2c_wait_flags(UCTXIFG0)) return;
    UCB2TXBUF = (unsigned char)(writeByte >> 8);    // MSB
    if (!i2c_wait_flags(UCTXIFG0)) return;
    UCB2TXBUF = (unsigned char)(writeByte & 0xFF);  // LSB
    if (!i2c_wait_flags(UCTXIFG0)) return;
    UCB2CTLW0 |= UCTXSTP;                   // STOP
    i2c_wait_ctl_clear(UCTXSTP);
}


void HAL_I2C_setslave(unsigned int slaveAdr)
{
    // Specify I2C slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B2_BASE,
        slaveAdr);

    // Enable and clear the interrupt flag
    EUSCI_B_I2C_clearInterrupt(EUSCI_B2_BASE,
        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}
