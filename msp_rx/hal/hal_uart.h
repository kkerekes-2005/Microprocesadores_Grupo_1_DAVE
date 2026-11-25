#ifndef HAL_HAL_UART_H_
#define HAL_HAL_UART_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Nodo_Rx: dos UART, 115200 8N1 con SMCLK = 16 MHz
 *   eUSCI_A3 (P6.1 = RX)  <- CC1310: recepción de tramas por interrupción
 *   eUSCI_A0 (P2.0 = TX)  -> PC (PuTTY, por el USB de la LaunchPad): trazas
 */

// Configura los pines de las dos UART. Llamar ANTES de PMM_unlockLPM5().
void HAL_UART_init_gpio(void);

// Configura eUSCI_A3 y eUSCI_A0 y activa la interrupción de RX del CC1310.
// Llamar DESPUÉS de PMM_unlockLPM5() y de configurar los relojes.
void HAL_UART_config(void);

// ---- CC1310 (eUSCI_A3, solo recepción) ----
// true si hay bytes recibidos pendientes de leer.
bool HAL_UART_CC_data_available(void);

// Saca un byte del buffer de recepción (llamar solo si hay datos).
uint8_t HAL_UART_CC_rx_byte(void);

// Bytes perdidos porque el buffer estaba lleno (para depurar).
uint16_t HAL_UART_CC_overflows(void);

// ---- PC / PuTTY (eUSCI_A0, solo transmisión) ----
// Envía un byte (bloqueante: espera a que el buffer de TX esté libre).
void HAL_UART_PC_tx_byte(uint8_t txByte);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HAL_UART_H_ */

