// MAIN Nodo_Rx_MSP430  -  D.A.V.E. Central (aseguradora)
#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <grlib.h>

#include <drivers/crystalfontz.h>
#include <drivers/led.h>
#include <drivers/switch.h>
#include <drivers/buzzer.h>
#include <hal/hal_uart.h>
#include <hal/hal_lcd.h>
#include <hal/hal_digital_output.h>
#include <hal/hal_digital_input.h>
#include <hal/hal_pwm.h>

// ================== Configuracion ===================
#define TICK_HZ            100u              // tick de 10 ms
#define T_SIN_SENAL        (3u * TICK_HZ)    // 3 s sin tramas válidas -> enlace perdido
#define T_DESTELLO         (TICK_HZ / 10)    // destello azul 100 ms por trama recibida
#define T_SIRENA           (10u * TICK_HZ)   // la sirena suena como mucho 10 s
#define T_SIRENA_SEMI      (TICK_HZ / 4)     // 250 ms encendida / 250 ms apagada
#define T_PARPADEO         (TICK_HZ / 2)     // periodo del parpadeo rojo: 500 ms

#define UMBRAL_IMPACTO_MG  1800              // mismo criterio que Nodo_Tx
#define UMBRAL_TEMP_CC     4500              // 45,00 ºC

// Estimación de distancia por RSSI (modelo log-distancia)
//   d = D_REF * 10^((RSSI_REF - RSSI) / (10 * n))
// CALIBRAR: medir el RSSI con el coche a D_REF_M metros y ponerlo en RSSI_REF_DBM
#define RSSI_REF_DBM       (-40)             // RSSI medido a D_REF_M
#define D_REF_M            1.0f
#define N_PROPAG           2.5f              // 2 = espacio libre; 2.5-3.5 en interiores

/* =============== Trama de radio (igual que en Nodo_Tx) ===============
    [0] cabecera 0xA5    [1] ID coche    [2] tipo (1 periódica, 2 evento)
    [3] secuencia        [4-5] ax mg    [6-7] ay mg     [8-9] az mg  (int16, little-endian)
    [10-11] temp en centésimas de ºC (int16, little-endian)
    [12] flags: bit0 impacto, bit1 temp alta, bit2 SOS, bit3 error sensor T
    [13] CRC-8 (polinomio 0x07) de los bytes 0..12

   Por UART, Radio_Rx añade:
    [14] RSSI en dBm (int8)
    [15] CRC-8 de los bytes 0..14 (protege el salto UART, incluido el RSSI)
*/
#define TRAMA_SYNC      0xA5
#define TIPO_PERIODICA  0x01
#define TIPO_EVENTO     0x02
#define FLAG_IMPACTO    0x01
#define FLAG_TEMP_ALTA  0x02
#define FLAG_SOS        0x04
#define FLAG_TEMP_ERR   0x08
#define OFF_CRC         13
#define OFF_RSSI        14
#define OFF_CRC_UART    15
#define UART_LEN        16

typedef struct {
    uint8_t id;
    uint8_t tipo;
    uint8_t seq;
    uint8_t flags;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp_cc;
    int8_t  rssi;
} trama_t;

// Ordenadas por prioridad: una alarma más grave sustituye a una menos grave
typedef enum {
    ALARMA_NINGUNA = 0,
    ALARMA_INCENDIO,
    ALARMA_SOS,
    ALARMA_ACCIDENTE
} alarma_t;


// ========== DECLARACIÓN FUNCIONES ==================
static void Clocks_init(void);
void TimerA_init(void);
static void tarea_tick(void);
static void rx_byte(uint8_t b);
static void procesar_trama(const uint8_t *r);
static uint8_t crc8(const uint8_t *d, uint8_t n);
static int16_t get_i16(const uint8_t *p);
static uint16_t estimar_distancia_dm(int8_t rssi);
static void buzzer_on(void);
static void buzzer_off(void);
static void pc_str(const char *s);
static void pc_hex(uint8_t b);
static void lcd_linea(uint8_t fila, const char *txt, uint32_t color);
static void LCD_init(void);
static void LCD_dibujar_pantalla(void);
static void LCD_enlace(bool ok);
static void LCD_datos(const trama_t *t, uint16_t amag);
static void LCD_alarma(const trama_t *t);


// ==================== VARIABLES ==============
volatile bool flagTick = false;
static uint16_t aclk_hz = 32768;

static bool     enlace_ok   = false;
static bool     primera     = true;        // aún no ha llegado ninguna trama
static uint8_t  ult_seq     = 0;
static uint16_t perdidas    = 0;           // tramas perdidas (huecos en la secuencia)
static uint16_t errores     = 0;           // tramas con CRC incorrecto
static uint16_t t_sin_senal = 0;
static uint16_t t_destello  = 0;
static uint16_t t_sirena    = 0;
static alarma_t alarma      = ALARMA_NINGUNA;
static uint16_t dist_dm     = 0;           // distancia estimada en decímetros

Graphics_Context g_sContext;


// =================== RELOJES =============
static void Clocks_init(void)
{
    FRAMCtl_A_configureWaitStateControl(FRAMCTL_A_ACCESS_TIME_CYCLES_1); // 16 MHz -> 1 wait state
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4);                          // DCO = 16 MHz
    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // ACLK desde el cristal de 32768 Hz (el VLO varía mucho entre placas)
    CS_setExternalClockSource(32768, 0);
    if (CS_turnOnLFXTWithTimeout(CS_LFXT_DRIVE_3, 1000000)) {
        CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
        aclk_hz = 32768;
    }
    else {      // si no arranca el cristal, seguimos con el VLO
        CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);
        aclk_hz = 9400;
    }
}


// ===================== UTILIDADES DE TRAMA ====================
// CRC-8, polinomio 0x07: idéntica a la de Nodo_Tx (detección de errores)
static uint8_t crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0x00;
    uint8_t i;
    while (n--) {
        crc ^= *d++;
        for (i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Inversa de put_i16 de Nodo_Tx: little-endian explícito
static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// Modelo log-distancia: cuanto más cae el RSSI respecto a la referencia, más lejos
static uint16_t estimar_distancia_dm(int8_t rssi)
{
    float d = D_REF_M * powf(10.0f, (float)(RSSI_REF_DBM - rssi) / (10.0f * N_PROPAG));
    if (d > 999.0f) d = 999.0f;
    return (uint16_t)(d * 10.0f + 0.5f);
}


// ===================== RECEPCIÓN ====================
// Monta la trama byte a byte; si el CRC falla, se resincroniza en la siguiente cabecera
static void rx_byte(uint8_t b)
{
    static uint8_t buf[UART_LEN];
    static uint8_t idx = 0;
    uint8_t k;

    if (idx == 0 && b != TRAMA_SYNC)
        return;                             // esperando cabecera
    buf[idx++] = b;
    if (idx < UART_LEN)
        return;                             // trama incompleta

    bool ok_uart  = (crc8(buf, OFF_CRC_UART) == buf[OFF_CRC_UART]);
    bool ok_coche = (crc8(buf, OFF_CRC) == buf[OFF_CRC]);

    if (ok_uart && ok_coche) {
        idx = 0;
        procesar_trama(buf);
        return;
    }

    // Trama corrupta: la mostramos para depurar y buscamos la siguiente cabecera
    errores++;
    pc_str(ok_coche ? "ERROR CRC enlace UART:" : "ERROR CRC trama coche:");
    for (k = 0; k < UART_LEN; k++) { pc_str(" "); pc_hex(buf[k]); }
    pc_str("\r\n");

    for (k = 1; k < UART_LEN && buf[k] != TRAMA_SYNC; k++);
    idx = UART_LEN - k;
    memmove(buf, &buf[k], idx);
}


static void procesar_trama(const uint8_t *r)
{
    static bool fuego_antes = false;
    trama_t t;
    char l[128];

    // 1. Desempaquetar (mismos offsets que put_i16 en Nodo_Tx)
    t.id      = r[1];
    t.tipo    = r[2];
    t.seq     = r[3];
    t.ax      = get_i16(&r[4]);
    t.ay      = get_i16(&r[6]);
    t.az      = get_i16(&r[8]);
    t.temp_cc = get_i16(&r[10]);
    t.flags   = r[12];
    t.rssi    = (int8_t)r[OFF_RSSI];

    // 2. Tramas perdidas: huecos en el número de secuencia
    if (!primera) {
        uint8_t salto = (uint8_t)(t.seq - ult_seq);
        if (salto > 1 && salto < 50)        // salto enorme = el coche se ha reiniciado
            perdidas += salto - 1;
    }
    primera = false;
    ult_seq = t.seq;

    // 3. Enlace vivo
    t_sin_senal = 0;
    t_destello  = T_DESTELLO;
    if (!enlace_ok) {
        enlace_ok = true;
        LCD_enlace(true);
        pc_str("Enlace con el coche establecido\r\n");
    }

    // 4. Magnitud de la aceleración: amag = sqrt(ax^2 + ay^2 + az^2)
    uint16_t amag = (uint16_t)sqrtf((float)t.ax * t.ax + (float)t.ay * t.ay +
                                    (float)t.az * t.az);

    // 5. Detección de incidencias
    alarma_t nueva = ALARMA_NINGUNA;
    bool fuego = !(t.flags & FLAG_TEMP_ERR) &&
                 ((t.temp_cc > UMBRAL_TEMP_CC) || (t.flags & FLAG_TEMP_ALTA));

    if (amag > UMBRAL_IMPACTO_MG || (t.flags & FLAG_IMPACTO)) nueva = ALARMA_ACCIDENTE;
    else if (t.flags & FLAG_SOS)                              nueva = ALARMA_SOS;
    else if (fuego && !fuego_antes)                           nueva = ALARMA_INCENDIO;  // solo en el flanco
    fuego_antes = fuego;

    // 6. Traza por PuTTY de cada trama
    sprintf(l, "#%3u %s ax=%+5d ay=%+5d az=%+5d |a|=%4u mg  T=%d.%02d C  RSSI=%d dBm\r\n",
            (unsigned)t.seq, t.tipo == TIPO_EVENTO ? "EVT" : "per",
            t.ax, t.ay, t.az, amag,
            t.temp_cc / 100, abs(t.temp_cc % 100), t.rssi);
    pc_str(l);

    // 7. Nueva alarma (una más grave sustituye a la que haya sin atender)
    if (nueva > alarma) {
        alarma   = nueva;
        dist_dm  = estimar_distancia_dm(t.rssi);    // la distancia solo se calcula si hay alarma
        t_sirena = T_SIRENA;
        LCD_alarma(&t);

        sprintf(l, "*** %s *** coche %u  |a|=%u mg  T=%d.%02d C  distancia ~%u.%u m\r\n",
                alarma == ALARMA_ACCIDENTE ? "ACCIDENTE" :
                alarma == ALARMA_SOS       ? "SOS MANUAL" : "POSIBLE INCENDIO",
                (unsigned)t.id, amag, t.temp_cc / 100, abs(t.temp_cc % 100),
                dist_dm / 10, dist_dm % 10);
        pc_str(l);
    }

    LCD_datos(&t, amag);
}


// ===================== Tarea de 10 ms ==========
static void tarea_tick(void)
{
    static bool     s1_antes   = false;
    static bool     buzz_on    = false;
    static uint16_t t_parpadeo = 0;

    // 1. S1: el operador atiende la alarma (flanco de pulsación)
    bool s1 = SWITCH_get_edumkii_S1();
    if (s1 && !s1_antes && alarma != ALARMA_NINGUNA) {
        alarma   = ALARMA_NINGUNA;
        t_sirena = 0;
        LCD_alarma(NULL);
        pc_str("Alarma atendida por el operador\r\n");
    }
    s1_antes = s1;

    // 2. Enlace: si pasan T_SIN_SENAL sin tramas válidas, se da por perdido
    if (t_sin_senal < T_SIN_SENAL) {
        t_sin_senal++;
        if (t_sin_senal == T_SIN_SENAL && enlace_ok) {
            enlace_ok = false;
            LCD_enlace(false);
            pc_str("*** SIN SENAL del coche ***\r\n");
        }
    }

    // 3. Sirena: pitidos de 250 ms mientras dure t_sirena
    if (t_sirena) t_sirena--;
    bool sonar = (t_sirena != 0) && ((t_sirena / T_SIRENA_SEMI) & 1);
    if (sonar != buzz_on) {                 // solo se llama al driver cuando cambia
        buzz_on = sonar;
        if (sonar) buzzer_on();
        else       buzzer_off();
    }

    // 4. LED
    if (t_destello) t_destello--;
    t_parpadeo = (t_parpadeo + 1) % T_PARPADEO;

    if (alarma != ALARMA_NINGUNA) LED_set_rgb(t_parpadeo < T_PARPADEO / 2, 0, 0); // rojo parpadeando
    else if (!enlace_ok)          LED_set_rgb(1, 1, 0);   // amarillo: sin señal
    else if (t_destello)          LED_set_rgb(0, 0, 1);   // azul: trama recibida
    else                          LED_set_rgb(0, 1, 0);   // verde: enlace OK
}


// ============== ACTUADORES ================
static void buzzer_on(void)  { BUZ_sound_on(); }
static void buzzer_off(void) { BUZ_sound_off(); }


// ============== SALIDA A PuTTY (eUSCI_A0) ================
static void pc_str(const char *s)
{
    while (*s) HAL_UART_PC_tx_byte((uint8_t)*s++);
}

static void pc_hex(uint8_t b)
{
    const char *h = "0123456789ABCDEF";
    HAL_UART_PC_tx_byte(h[b >> 4]);
    HAL_UART_PC_tx_byte(h[b & 0x0F]);
}


// =========== LCD =============
static void lcd_linea(uint8_t fila, const char *txt, uint32_t color)
{
    char buf[22];
    uint8_t i = 0;
    while (txt[i] && i < 21) {
        buf[i] = txt[i];
        i++;
    }
    while (i < 21) {
        buf[i++] = ' ';                     // sobreescribe sin borrar (sin parpadeo)
    }
    buf[21] = '\0';
    Graphics_setForegroundColor(&g_sContext, color);
    Graphics_drawString(&g_sContext, (int8_t *)buf, 21, 1,
                        20 + 11 * fila, OPAQUE_TEXT);
}

static void LCD_init(void)
{
    CRYSTALFONTZ_init();
    CRYSTALFONTZ_set_orientation(LCD_ORIENTATION_UP);
    Graphics_initContext(&g_sContext, &g_sCRYSTALFONTZ);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    LCD_dibujar_pantalla();
    LCD_enlace(false);
    LCD_alarma(NULL);
}

static void LCD_dibujar_pantalla(void)
{
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_NAVY);
    Graphics_clearDisplay(&g_sContext);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)"D.A.V.E. - CENTRAL",
                                AUTO_STRING_LENGTH, 64, 8, OPAQUE_TEXT);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    Graphics_drawLineH(&g_sContext, 4, 123, 16);    // separador
}

static void LCD_enlace(bool ok)
{
    if (ok) lcd_linea(0, "Enlace: OK",        GRAPHICS_COLOR_GREEN);
    else    lcd_linea(0, "Enlace: SIN SENAL", GRAPHICS_COLOR_YELLOW);
}

static void LCD_datos(const trama_t *t, uint16_t amag)
{
    char l[32];

    sprintf(l, "Coche %u  Trama #%u", (unsigned)t->id, (unsigned)t->seq);
    lcd_linea(1, l, GRAPHICS_COLOR_WHITE);

    sprintf(l, "|a|  %5u mg", amag);
    lcd_linea(2, l, amag > UMBRAL_IMPACTO_MG ? GRAPHICS_COLOR_RED : GRAPHICS_COLOR_CYAN);

    if (t->flags & FLAG_TEMP_ERR) {
        lcd_linea(3, "T    ERROR SENSOR", GRAPHICS_COLOR_RED);
    } else {
        sprintf(l, "T    %d.%02d C", t->temp_cc / 100, abs(t->temp_cc % 100));
        lcd_linea(3, l, GRAPHICS_COLOR_WHITE);
    }

    sprintf(l, "RSSI %4d dBm", t->rssi);
    lcd_linea(4, l, GRAPHICS_COLOR_WHITE);

    sprintf(l, "Perdidas %u  Err %u", perdidas, errores);
    lcd_linea(5, l, GRAPHICS_COLOR_LIGHT_GRAY);
}

// t solo se usa para mostrar la temperatura en caso de incendio (puede ser NULL)
static void LCD_alarma(const trama_t *t)
{
    char l[32];

    switch (alarma) {
    case ALARMA_ACCIDENTE:
        lcd_linea(6, "HA HABIDO UN", GRAPHICS_COLOR_RED);
        sprintf(l, "ACCIDENTE! d~%u.%u m", dist_dm / 10, dist_dm % 10);
        lcd_linea(7, l, GRAPHICS_COLOR_RED);
        lcd_linea(8, "S1 para atender", GRAPHICS_COLOR_WHITE);
        break;

    case ALARMA_SOS:
        lcd_linea(6, "SOS MANUAL", GRAPHICS_COLOR_RED);
        sprintf(l, "Distancia ~%u.%u m", dist_dm / 10, dist_dm % 10);
        lcd_linea(7, l, GRAPHICS_COLOR_RED);
        lcd_linea(8, "S1 para atender", GRAPHICS_COLOR_WHITE);
        break;

    case ALARMA_INCENDIO:
        lcd_linea(6, "POSIBLE INCENDIO", GRAPHICS_COLOR_ORANGE);
        if (t) sprintf(l, "T=%d.%02d C d~%u.%u m", t->temp_cc / 100, abs(t->temp_cc % 100),
                       dist_dm / 10, dist_dm % 10);
        else   sprintf(l, "d~%u.%u m", dist_dm / 10, dist_dm % 10);
        lcd_linea(7, l, GRAPHICS_COLOR_ORANGE);
        lcd_linea(8, "S1 para atender", GRAPHICS_COLOR_WHITE);
        break;

    default:
        lcd_linea(6, "Sin incidencias", GRAPHICS_COLOR_GREEN);
        lcd_linea(7, "", GRAPHICS_COLOR_WHITE);
        lcd_linea(8, "", GRAPHICS_COLOR_WHITE);
        break;
    }
}


// ==================== MAIN ====================
void main(void)
{
    WDT_A_hold(WDT_A_BASE);

    HAL_UART_init_gpio();
    HAL_LCD_init_gpio();
    HAL_DIGOUT_init_gpio();
    HAL_DIGIN_init_gpio();
    HAL_PWM_init_gpio();
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 | GPIO_PIN5,
                                               GPIO_PRIMARY_MODULE_FUNCTION);  // pines del cristal

    PMM_unlockLPM5();
    Clocks_init();              // después del unlock: el cristal necesita sus pines activos

    // Periféricos (después del unlock y de fijar los relojes)
    HAL_UART_config();
    HAL_LCD_config();
    HAL_DIGOUT_config();
    HAL_DIGIN_config();
    HAL_PWM_config();

    LCD_init();
    LED_set_rgb(1, 1, 0);       // amarillo: esperando al coche
    TimerA_init();
    __bis_SR_register(GIE);

    pc_str("\r\n== D.A.V.E. Central: esperando tramas del coche ==\r\n");

    while (1) {
        // Vaciar todo lo que haya llegado del CC1310
        while (HAL_UART_CC_data_available())
            rx_byte(HAL_UART_CC_rx_byte());

        if (flagTick) {
            flagTick = false;
            tarea_tick();
        }
    }
}


// ========== TIMER: tick de 10 ms ================
void TimerA_init(void)
{
    Timer_A_initUpModeParam TimerConfig =
    {
         TIMER_A_CLOCKSOURCE_ACLK,          // ACLK = 32768 Hz (cristal)
         TIMER_A_CLOCKSOURCE_DIVIDER_1,
         (aclk_hz / TICK_HZ) - 1,           // 32768/100 -> 327 cuentas = 10 ms
         TIMER_A_TAIE_INTERRUPT_ENABLE,
         TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
         TIMER_A_DO_CLEAR
    };
    Timer_A_initUpMode(TIMER_A1_BASE, &TimerConfig);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}


#pragma vector = TIMER1_A1_VECTOR
__interrupt void timer1_a1_isr_handler(void)
{
    if (Timer_A_getInterruptStatus(TIMER_A1_BASE) == TIMER_A_INTERRUPT_PENDING) {
        Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
        flagTick = true;
    }
}

