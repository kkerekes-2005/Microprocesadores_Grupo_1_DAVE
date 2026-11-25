#ifndef HAL_HAL_UART_H_
#define HAL_HAL_UART_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART hacia el CC1310, SOLO TX: eUSCI_A3, P6.0 = TX, 115200 8N1 (SMCLK = 16 MHz)

// Configura P6.0 como UCA3TXD. Llamar ANTES de PMM_unlockLPM5().
void HAL_UART_init_gpio(void);

// Configura eUSCI_A3 a 115200 8N1. Llamar DESPUÉS de PMM_unlockLPM5().
void HAL_UART_config(void);

// Envía un byte (bloqueante: espera a que el buffer de TX esté libre).
void HAL_UART_tx_byte(uint8_t txByte);

// Envía len bytes seguidos (por ejemplo, una trama completa).
void HAL_UART_tx_array(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_UART_H_ */
