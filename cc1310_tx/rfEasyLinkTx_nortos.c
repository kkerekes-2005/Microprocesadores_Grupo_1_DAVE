/*
 * ======== rfEasyLinkTx_nortos.c ========
 * CC1310 transmisor: puente UART (MSP430) -> radio
 *
 * Recibe por UART las tramas de 14 bytes de Nodo_Tx, comprueba su CRC-8
 * y las reenvia por radio a Radio_Rx sin modificarlas.
 */

#include "smartrf_settings/smartrf_settings.h"
#include "Board.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART.h>
#include <ti/devices/DeviceFamily.h>
#include <ti/devices/cc13x0/driverlib/cpu.h>

#include "easylink/EasyLink.h"

/***** Configuracion *****/
#define MODO_DIAG        2              /* 0 = Solo Radio, 1 = Solo UART, 2 = Puente Completo */
#define FREQ_HZ          863000000UL    /* Frecuencia en Hz (863 MHz), la asignada al grupo */
#define TX_POWER_DBM     10             /* Potencia fija: necesaria para estimar distancia por RSSI */
#define DST_ADDR         0xAA           /* Direccion de destino para filtro Rx */

/* Trama: mismo contrato que Nodo_Tx y Nodo_Rx
    [0] cabecera 0xA5    [1] ID coche    [2] tipo (1 periodica, 2 evento)
    [3] secuencia        [4-5] ax mg     [6-7] ay mg     [8-9] az mg  (int16, little-endian)
    [10-11] temp en centesimas de C (int16, little-endian)
    [12] flags           [13] CRC-8 (polinomio 0x07) de los bytes 0..12          */
#define TRAMA_HDR        0xA5
#define TRAMA_LEN        14
#define OFF_CRC          13

#ifndef Board_PIN_LED1
  #ifdef CC1310_LAUNCHXL_PIN_GLED
    #define Board_PIN_LED1 CC1310_LAUNCHXL_PIN_GLED
  #else
    #define Board_PIN_LED1 Board_PIN_GLED
  #endif
#endif

#ifndef Board_PIN_LED2
  #ifdef CC1310_LAUNCHXL_PIN_RLED
    #define Board_PIN_LED2 CC1310_LAUNCHXL_PIN_RLED
  #else
    #define Board_PIN_LED2 Board_PIN_RLED
  #endif
#endif

#ifndef Board_UART0
  #ifdef CC1310_LAUNCHXL_UART0
    #define Board_UART0 CC1310_LAUNCHXL_UART0
  #endif
#endif

static PIN_Handle pinHandle;
static PIN_State  pinState;

PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Utilidades *****/

/* CPUdelay tarda 3 ciclos por iteracion: 48 MHz / 3 = 16000 iteraciones por ms */
static void delay_ms(uint32_t ms)
{
    CPUdelay(16000 * ms);
}

static void ledSet(PIN_Id pin, uint8_t v)
{
    PIN_setOutputValue(pinHandle, pin, v);
}

/* ledToggle solo se compila en Modo 1 para evitar warning #179-D */
#if (MODO_DIAG == 1)
static void ledToggle(PIN_Id pin)
{
    PIN_setOutputValue(pinHandle, pin, !PIN_getOutputValue(pin));
}
#endif

static void blink(PIN_Id pin, uint8_t n, uint32_t ms)
{
    while (n--) {
        ledSet(pin, 1);
        delay_ms(ms);
        ledSet(pin, 0);
        delay_ms(ms);
    }
}

static void fatal(uint8_t code)
{
    while (1) {
        blink(Board_PIN_LED2, code, 200);
        delay_ms(1000);
    }
}

/* Estas funciones solo se usan en los modos con radio (0 y 2) */
#if (MODO_DIAG != 1)
static void blinkBoth(uint8_t n, uint32_t ms)
{
    while (n--) {
        ledSet(Board_PIN_LED1, 1);
        ledSet(Board_PIN_LED2, 1);
        delay_ms(ms);
        ledSet(Board_PIN_LED1, 0);
        ledSet(Board_PIN_LED2, 0);
        delay_ms(ms);
    }
}

/* CRC-8, polinomio 0x07: identica a la de Nodo_Tx (deteccion de errores) */
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

static bool radioSend(const uint8_t *buf)
{
    EasyLink_TxPacket txPacket;
    memset(&txPacket, 0, sizeof(txPacket));
    memcpy(txPacket.payload, buf, TRAMA_LEN);
    txPacket.len        = TRAMA_LEN;
    txPacket.dstAddr[0] = DST_ADDR;
    txPacket.absTime    = 0;                /* transmitir inmediatamente */
    return (EasyLink_transmit(&txPacket) == EasyLink_Status_Success);
}
#endif

/***** Hilo Principal *****/
void *mainThread(void *arg0)
{
    /* --- LEDs --- */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL) { while (1); }
    ledSet(Board_PIN_LED1, 0);
    ledSet(Board_PIN_LED2, 0);

    /* Verde 1 vez = LEDs OK */
    blink(Board_PIN_LED1, 1, 150);

    /* --- UART (solo si se usa) --- */
#if (MODO_DIAG != 0)
    UART_Handle uart;
    UART_Params up;

    UART_init();
    UART_Params_init(&up);
    up.writeDataMode  = UART_DATA_BINARY;
    up.readDataMode   = UART_DATA_BINARY;
    up.readReturnMode = UART_RETURN_FULL;
    up.readEcho       = UART_ECHO_OFF;
    up.baudRate       = 115200;             /* igual que HAL_UART_config() de Nodo_Tx */
    uart = UART_open(Board_UART0, &up);
    if (uart == NULL) { fatal(2); }         /* rojo x2 = fallo UART */
    blink(Board_PIN_LED1, 2, 150);          /* verde x2 = UART OK */
#endif

    /* --- Radio (solo si se usa) --- */
#if (MODO_DIAG != 1)
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success) {
        fatal(3);                           /* rojo x3 = fallo EasyLink */
    }
    if (EasyLink_setFrequency(FREQ_HZ) != EasyLink_Status_Success) {
        fatal(4);                           /* rojo x4 = frecuencia no valida */
    }
    if (EasyLink_setRfPower(TX_POWER_DBM) != EasyLink_Status_Success) {
        fatal(5);                           /* rojo x5 = potencia no valida */
    }
    blink(Board_PIN_LED1, 3, 150);          /* verde x3 = radio OK */
#endif

    delay_ms(500);

    /* ============================ MODO 0: SOLO RADIO ============================ */
#if (MODO_DIAG == 0)
    {
        /* Trama de prueba: coche 1, periodica, ax=0, ay=0, az=+1000 mg, T=23,45 C */
        uint8_t trama[TRAMA_LEN] = {
            TRAMA_HDR, 0x01, 0x01, 0x00,    /* cabecera, ID, tipo, secuencia */
            0x00, 0x00,                     /* ax = 0 */
            0x00, 0x00,                     /* ay = 0 */
            0xE8, 0x03,                     /* az = 1000 (0x03E8, little-endian) */
            0x29, 0x09,                     /* T  = 2345 (0x0929, little-endian) */
            0x00,                           /* flags */
            0x00                            /* CRC (se calcula abajo) */
        };

        while (1) {
            trama[OFF_CRC] = crc8(trama, OFF_CRC);
            if (radioSend(trama)) {
                blink(Board_PIN_LED1, 1, 40);   /* verde = enviado */
            } else {
                blinkBoth(1, 100);              /* rojo+verde = error TX radio */
            }
            trama[3]++;                         /* secuencia: el Rx puede detectar perdidas */
            delay_ms(1000);
        }
    }
#endif

    /* ============================ MODO 1: SOLO UART ============================ */
#if (MODO_DIAG == 1)
    {
        uint8_t b;
        while (1) {
            UART_read(uart, &b, 1);
            if (b == TRAMA_HDR) {
                ledToggle(Board_PIN_LED1);      /* 1 toggle por trama */
            } else {
                ledSet(Board_PIN_LED2, 1);      /* rojo = llego un byte que no es cabecera */
                ledSet(Board_PIN_LED2, 0);      /* pulso muy corto, casi invisible */
            }
        }
    }
#endif

    /* ============================ MODO 2: PUENTE COMPLETO ============================ */
#if (MODO_DIAG == 2)
    {
        uint8_t rx[TRAMA_LEN];
        uint8_t idx = 0;
        uint8_t b, k;

        while (1) {
            UART_read(uart, &b, 1);

            if (idx == 0 && b != TRAMA_HDR) {
                continue;                       /* esperando cabecera */
            }
            rx[idx++] = b;
            if (idx < TRAMA_LEN) {
                continue;                       /* trama incompleta */
            }

            if (crc8(rx, OFF_CRC) == rx[OFF_CRC]) {
                idx = 0;
                if (radioSend(rx)) {
                    blink(Board_PIN_LED1, 1, 30);   /* verde = trama reenviada por radio */
                } else {
                    blink(Board_PIN_LED2, 1, 100);  /* rojo = fallo TX radio */
                }
                continue;
            }

            /* CRC incorrecto: puede que se haya perdido un byte y que la
             * siguiente trama ya haya empezado dentro de rx[]. Buscamos la
             * siguiente cabecera y conservamos lo recibido desde ahi. */
            blinkBoth(1, 30);
            for (k = 1; k < TRAMA_LEN && rx[k] != TRAMA_HDR; k++);
            idx = TRAMA_LEN - k;
            memmove(rx, &rx[k], idx);
        }
    }
#endif
}
