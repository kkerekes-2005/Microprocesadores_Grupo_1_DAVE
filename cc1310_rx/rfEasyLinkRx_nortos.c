/*
 *  ======== rfEasyLinkRx_nortos.c ========
 *  CC1310 receptor: radio (EasyLink, sincrono) -> UART hacia el MSP430 (Nodo_Rx).
 *
 *  Recibe las tramas de 14 bytes del coche, comprueba longitud, cabecera y CRC-8,
 *  y reenvia al MSP430 16 bytes: trama (14) + RSSI (1) + CRC-8 del enlace UART (1).
 *
 *  LEDs (segun Board.h de este proyecto):
 *      Board_PIN_LED1 = VERDE (DIO7)
 *      Board_PIN_LED2 = ROJO  (DIO6)
 *
 *  Secuencia de arranque:
 *      verde x1 = LEDs OK (este firmware es el que corre)
 *      verde x2 = UART abierta
 *      verde x3 = radio lista (EasyLink, frecuencia y filtro de direccion)
 *      rojo x2 repetido = fallo en UART_open
 *      rojo x3 repetido = fallo en EasyLink_init
 *      rojo x4 repetido = frecuencia no valida
 *      rojo x5 repetido = fallo en el filtro de direccion
 *
 *  En funcionamiento:
 *      VERDE hace toggle con cada trama valida reenviada al MSP430 (latido).
 *      ROJO fijo  = timeout: no llega nada (sin senal del coche).
 *      ROJO toggle = llego un paquete invalido o hubo error de radio (no se reenvia).
 *
 *  Prueba de UART sin radio: con ENVIAR_PRUEBA_UART = 1 manda una trama valida
 *  de 16 bytes al arrancar. Si Nodo_Rx la muestra en PuTTY y en el LCD,
 *  el tramo CC1310 -> MSP430 esta bien.
 */


/* Application header files */
#include "smartrf_settings/smartrf_settings.h"
#include "Board.h"

/* Standard C Libraries */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART.h>
#include <ti/devices/DeviceFamily.h>

/* EasyLink API Header files */
#include "easylink/EasyLink.h"


/***** Configuracion *****/
#define FREQ_HZ             863000000UL     /* igual que Radio_Tx */
#define RX_ADDR             0xAA            /* igual que DST_ADDR de Radio_Tx */
#define RX_TIMEOUT_MS       2000            /* > 1 s entre tramas periodicas del coche */
#define ENVIAR_PRUEBA_UART  0               /* 1 = manda trama de prueba al arrancar */

/* Trama de radio (contrato con Nodo_Tx / Radio_Tx) */
#define TRAMA_HDR           0xA5
#define TRAMA_LEN           14
#define OFF_CRC             13              /* CRC-8 de los bytes 0..12 */

/* Trama por UART hacia Nodo_Rx (contrato con Nodo_Rx) */
#define UART_LEN            16
#define OFF_RSSI            14              /* RSSI en dBm (int8) */
#define OFF_CRC_UART        15              /* CRC-8 de los bytes 0..14 */


/* Pin driver */
static PIN_Handle pinHandle;
static PIN_State  pinState;

PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Tabla del filtro de direccion: solo se aceptan paquetes dirigidos a RX_ADDR */
static uint8_t addrFilter[1] = { RX_ADDR };


/***** Utilidades *****/
static void ledSet(PIN_Id pin, uint8_t v)
{
    PIN_setOutputValue(pinHandle, pin, v);
}

static void ledToggle(PIN_Id pin)
{
    PIN_setOutputValue(pinHandle, pin, !PIN_getOutputValue(pin));
}

static void blink(PIN_Id pin, uint8_t n, uint32_t ms)
{
    while (n--) {
        ledSet(pin, 1);
        usleep(ms * 1000);
        ledSet(pin, 0);
        usleep(ms * 1000);
    }
}

/* Parada de error: rojo parpadea 'code' veces, pausa, repite */
static void fatal(uint8_t code)
{
    while (1) {
        blink(Board_PIN_LED2, code, 200);
        usleep(1000000);
    }
}

/* CRC-8, polinomio 0x07: identica a la de Nodo_Tx, Radio_Tx y Nodo_Rx */
static uint8_t crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0x00, i;
    while (n--) {
        crc ^= *d++;
        for (i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* Comprueba que el paquete es una trama del coche: longitud, cabecera y CRC */
static bool trama_valida(const uint8_t *p, uint8_t len)
{
    return (len == TRAMA_LEN) &&
           (p[0] == TRAMA_HDR) &&
           (crc8(p, OFF_CRC) == p[OFF_CRC]);
}

/* Reenvia al MSP430: trama (14) + RSSI (1) + CRC-8 de todo lo anterior (1) */
static void enviar_al_msp(UART_Handle uart, const uint8_t *trama, int8_t rssi)
{
    uint8_t out[UART_LEN];
    memcpy(out, trama, TRAMA_LEN);
    out[OFF_RSSI]     = (uint8_t)rssi;
    out[OFF_CRC_UART] = crc8(out, OFF_CRC_UART);
    UART_write(uart, out, UART_LEN);
}


/***** main thread *****/
void *mainThread(void *arg0)
{
    UART_Handle uart;
    UART_Params uartParams;

    /* --- LEDs --- */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL) { while (1); }
    ledSet(Board_PIN_LED1, 0);
    ledSet(Board_PIN_LED2, 0);
    blink(Board_PIN_LED1, 1, 150);              /* verde x1 */

    /* --- UART hacia el MSP430 (115200 8N1, binaria) --- */
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode  = UART_DATA_BINARY;
    uartParams.readDataMode   = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho       = UART_ECHO_OFF;
    uartParams.baudRate       = 115200;         /* igual que HAL_UART_config() de Nodo_Rx */

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) { fatal(2); }             /* rojo x2 */
    blink(Board_PIN_LED1, 2, 150);              /* verde x2 */

#if ENVIAR_PRUEBA_UART
    {
        /* Trama valida de prueba: coche 1, periodica, seq 0, ax=0, ay=0,
         * az=+1000 mg, T=23,45 C, sin flags. Se envia con RSSI = -40 dBm. */
        uint8_t test[TRAMA_LEN] = {
            TRAMA_HDR, 0x01, 0x01, 0x00,        /* cabecera, ID, tipo, secuencia */
            0x00, 0x00,                         /* ax = 0 */
            0x00, 0x00,                         /* ay = 0 */
            0xE8, 0x03,                         /* az = 1000 (0x03E8, little-endian) */
            0x29, 0x09,                         /* T  = 2345 (0x0929, little-endian) */
            0x00,                               /* flags */
            0x00                                /* CRC (se calcula abajo) */
        };
        test[OFF_CRC] = crc8(test, OFF_CRC);
        usleep(500000);                         /* deja arrancar al MSP */
        enviar_al_msp(uart, test, -40);
    }
#endif

    /* --- Radio --- */
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success) {
        fatal(3);                               /* rojo x3 */
    }
    if (EasyLink_setFrequency(FREQ_HZ) != EasyLink_Status_Success) {
        fatal(4);                               /* rojo x4 */
    }
    /* Filtro de direccion explicito: 1 direccion de 1 byte (RX_ADDR) */
    if (EasyLink_enableRxAddrFilter(addrFilter, 1, 1) != EasyLink_Status_Success) {
        fatal(5);                               /* rojo x5 */
    }
    blink(Board_PIN_LED1, 3, 150);              /* verde x3 */

    EasyLink_RxPacket rxPacket;

    while (1) {
        memset(&rxPacket, 0, sizeof(rxPacket));
        rxPacket.absTime   = 0;
        rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT_MS);

        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success) {
            if (trama_valida(rxPacket.payload, rxPacket.len)) {
                enviar_al_msp(uart, rxPacket.payload, rxPacket.rssi);
                ledSet(Board_PIN_LED2, 0);      /* hay senal: rojo apagado */
                ledToggle(Board_PIN_LED1);      /* verde: latido por trama valida */
            } else {
                ledToggle(Board_PIN_LED2);      /* rojo: paquete invalido, no se reenvia */
            }
        }
        else if (result == EasyLink_Status_Rx_Timeout) {
            ledSet(Board_PIN_LED1, 0);
            ledSet(Board_PIN_LED2, 1);          /* rojo fijo: sin senal del coche */
        }
        else {
            ledToggle(Board_PIN_LED2);          /* rojo: error de radio */
        }
    }
}

