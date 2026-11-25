#include <msp430.h>
#include <driverlib.h>
#include <hal/hal_uart.h>

/* UART hacia el CC1310, SOLO TRANSMISIÓN: eUSCI_A3, P6.0 = UCA3TXD */

void HAL_UART_init_gpio(void)
{
    // P6.0 = UCA3TXD (función PRIMARIA). Llamar ANTES de PMM_unlockLPM5().
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P6,
            GPIO_PIN0,
            GPIO_PRIMARY_MODULE_FUNCTION);
}

void HAL_UART_config(void)
{
    EUSCI_A_UART_initParam uartConfig = {0};
    uartConfig.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    uartConfig.clockPrescalar    = 8;       // 115200 baudios con SMCLK = 16 MHz
    uartConfig.firstModReg       = 10;
    uartConfig.secondModReg      = 247;
    uartConfig.parity            = EUSCI_A_UART_NO_PARITY;
    uartConfig.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
    uartConfig.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
    uartConfig.uartMode          = EUSCI_A_UART_MODE;
    uartConfig.overSampling      = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    EUSCI_A_UART_init(EUSCI_A3_BASE, &uartConfig);     // configurar (módulo en reset)
    EUSCI_A_UART_enable(EUSCI_A3_BASE);                // sacar de reset
    // Sin interrupción de recepción: este nodo solo transmite
}

void HAL_UART_tx_byte(uint8_t txByte)
{
    EUSCI_A_UART_transmitData(EUSCI_A3_BASE, txByte);  // espera a TXBUF libre y escribe
}

void HAL_UART_tx_array(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        HAL_UART_tx_byte(data[i]);
    }
}
