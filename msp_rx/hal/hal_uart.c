#include <msp430.h>
#include <driverlib.h>
#include <hal/hal_uart.h>

/* Nodo_Rx:
 *   eUSCI_A3, P6.1 = UCA3RXD  <- CC1310 (recepción por interrupción, buffer circular)
 *   eUSCI_A0, P2.0 = UCA0TXD  -> PC / PuTTY (backchannel USB de la LaunchPad)
 */

#define RX_BUF_LEN  64u                     // potencia de 2: caben 4 tramas de 16 bytes
#define RX_MASK     (RX_BUF_LEN - 1u)

static volatile uint8_t  rx_buf[RX_BUF_LEN];
static volatile uint8_t  rx_head = 0;       // posición donde escribe la ISR
static volatile uint8_t  rx_tail = 0;       // posición donde lee el main
static volatile uint16_t rx_overflow = 0;   // bytes perdidos por buffer lleno


// Mismos parámetros para las dos UART: 115200 8N1 con SMCLK = 16 MHz
static void uart_115200(uint16_t base)
{
    EUSCI_A_UART_initParam cfg = {0};
    cfg.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    cfg.clockPrescalar    = 8;              // 16 MHz / 16 / 115200 = 8,68
    cfg.firstModReg       = 10;             // UCBRF
    cfg.secondModReg      = 247;            // UCBRS = 0xF7 (tabla del User's Guide)
    cfg.parity            = EUSCI_A_UART_NO_PARITY;
    cfg.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
    cfg.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
    cfg.uartMode          = EUSCI_A_UART_MODE;
    cfg.overSampling      = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    EUSCI_A_UART_init(base, &cfg);          // configurar (módulo en reset)
    EUSCI_A_UART_enable(base);              // sacar de reset
}


void HAL_UART_init_gpio(void)
{
    // P6.1 = UCA3RXD (función primaria): entrada desde el TX del CC1310
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6,
            GPIO_PIN1,
            GPIO_PRIMARY_MODULE_FUNCTION);

    // P2.0 = UCA0TXD, P2.1 = UCA0RXD (función secundaria): backchannel USB hacia el PC
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P2,
            GPIO_PIN0,
            GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P2,
            GPIO_PIN1,
            GPIO_SECONDARY_MODULE_FUNCTION);
}


void HAL_UART_config(void)
{
    // CC1310 -> MSP430: recepción por interrupción
    uart_115200(EUSCI_A3_BASE);
    EUSCI_A_UART_clearInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // MSP430 -> PC: solo transmisión, sin interrupciones
    uart_115200(EUSCI_A0_BASE);
}


// ---------------- CC1310 (eUSCI_A3) ----------------
bool HAL_UART_CC_data_available(void)
{
    return rx_head != rx_tail;
}

uint8_t HAL_UART_CC_rx_byte(void)
{
    uint8_t b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1u) & RX_MASK;
    return b;
}

uint16_t HAL_UART_CC_overflows(void)
{
    return rx_overflow;
}

#pragma vector = EUSCI_A3_VECTOR
__interrupt void EUSCI_A3_ISR(void)
{
    switch (__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG)) {
    case USCI_UART_UCRXIFG: {
        uint8_t b    = UCA3RXBUF;           // leer RXBUF borra el flag
        uint8_t next = (rx_head + 1u) & RX_MASK;
        if (next != rx_tail) {              // hay sitio en el buffer
            rx_buf[rx_head] = b;
            rx_head = next;
        } else {
            rx_overflow++;                  // buffer lleno: se pierde el byte
        }
        break;
    }
    default:
        break;
    }
}


// ---------------- PC / PuTTY (eUSCI_A0) ----------------
void HAL_UART_PC_tx_byte(uint8_t txByte)
{
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, txByte);  // espera a TXBUF libre y escribe
}

