// MAIN Nodo_Tx_MSP430
#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <grlib.h>

#include <drivers/crystalfontz.h>
#include <drivers/kxtc9_2050.h>
#include <drivers/led.h>
#include <drivers/switch.h>
#include <drivers/buzzer.h>
#include <drivers/tmp006.h>
#include <hal/hal_uart.h>
#include <hal/hal_lcd.h>
#include <hal/hal_adc.h>
#include <hal/hal_digital_output.h>
#include <hal/hal_digital_input.h>
#include <hal/hal_pwm.h>
#include <hal/hal_i2c.h>

// ================== Configuración ===================
#define TICK_HZ         100u           // muestreo del acelerómetro (10ms)
#define TICKS_TRAMA     (1u * TICK_HZ) //trama perioódica cada 1s
#define HOLDOFF_EVENTO  (2u * TICK_HZ) //2s sin repetir aviso de impacto
#define T_ALARMA        (3u * TICK_HZ) //LED rojo 3s
#define T_PITIDO        (TICK_HZ / 4)  // pitido de 250ms
#define T_DESTELLO      (TICK_HZ / 10) // destello azul 100ms

#define UMBRAL_IMPACTO_MG   1800    //hay q calibrarlo y justificarlo en la memoria
#define UMBRAL_M2           ((int32_t)UMBRAL_IMPACTO_MG * UMBRAL_IMPACTO_MG)
#define UMBRAL_TEMP_CC      4500    //45,00 ºC
#define HISTERESIS_CC 200 // 2,00 ºC .> se desactiva por debajo de 43,00 ºC

/* =============== Estructura de la TRAMA (igual q en nodoRx) ========================0
    [0] cabecera 0xA5    [1] ID coche    [2] tipo (1 perioódica, 2 evento)
    [3] secuencia        [4-5] ax mg    [6-7] ay mg     [8-9] az mg  (int16, little-endian)
    [10-11] temp en centesimas de ºC (int16, little endian)
    [12] flags: bit0 impacto, bit1 temp alta, bit2 SOS, bit3 error sensor T
    [13] CRC-8 (polinomiio 0x07) de los bytes 0...12 (13 bytes total)
*/
#define TRAMA_SYNC      0xA5
#define TRAMA_LEN       14
#define ID_COCHE        0x01
#define TIPO_PERIODICA  0x01
#define TIPO_EVENTO     0x02
#define FLAG_IMPACTO    0x01
#define FLAG_TEMP_ALTA  0x02
#define FLAG_SOS        0x04
#define FLAG_TEMP_ERR   0x08

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
} muestra_t;


// ========== DECLARACIÓN FUNCIONES ==================
static void Clocks_init(void);
void TimerA_init(void);
static void tarea_tick(void);
static uint8_t enviar_trama(const muestra_t *p, uint8_t flags, bool evento);
static int16_t accel_a_mg(float g);
static uint8_t crc8(const uint8_t *d, uint8_t n);
static void put_i16(uint8_t *p, int16_t v);
static void led_actualizar(uint16_t t_alarma, uint16_t t_destello, bool temp_alta);
static void buzzer_on(void);
static void buzzer_off(void);
static void LCD_init(void);
void LCD_dibujar_pantalla(void);
void LCD_actualizar_status(bool status);
static void LCD_actualizar_datos(const muestra_t *p, int16_t temp_cc, uint8_t flags, uint8_t tipo);
static bool temp_leer_cc(int16_t *t_cc);


// ==================== FIJAR VALORES ==============

volatile bool flagTick = false;
static uint16_t aclk_hz = 32768;
static bool sistemaOn = false;
static uint8_t seq = 0;

Graphics_Context g_sContext;


// =================== RELOJES =============
static void Clocks_init(void)
{
    FRAMCtl_A_configureWaitStateControl(FRAMCTL_A_ACCESS_TIME_CYCLES_1); //16MHz -> espera 1 ciclo
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4); // DCO tbm a 16MHz
    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 16MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 16MHz

    // ACLK desde el cristal de 32768Hz  (ya q el VLO varía mucho entre las dos placas Tx y Rx)
    CS_setExternalClockSource(32768, 0);
    if (CS_turnOnLFXTWithTimeout(CS_LFXT_DRIVE_3, 1000000)) {
        CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
        aclk_hz = 32768;
    }
    else { // si no va el cristal, seguimos con el VLO
        CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);
        aclk_hz = 9400;
    }
}


// ===================== FUNCIONES PARA LA TRAMA ====================
// Función de cambio de tipo de dato (para el output del acelerometro)
static int16_t accel_a_mg(float g) {
    float mg = g * 1000.0f;
    if (mg > 32767.0f) mg = 32767.0f;
    if (mg < -32768.0f) mg = -32768.0f;
    return (mg >= 0.0f) ? (int16_t)(mg + 0.5f) : (int16_t)(mg - 0.5f);
}


// Función de correción de errores
static uint8_t crc8(const uint8_t *d, uint8_t n) {
    uint8_t crc = 0x00;
    uint8_t i;
    while (n--) {
        crc = crc ^ *d++;
        for (i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}


// Función para pasar a little endian
static void put_i16(uint8_t *p, int16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)((uint16_t)v >> 8);
}


// Temperatura de la pastilla en centésimas de ºC (2450 = 24,50 ºC).
// Devuelve false si el sensor no responde (así no se manda un valor falso).
static bool temp_leer_cc(int16_t *t_cc)
{
    *t_cc = 2450;
    return true;
}


// FUnción para enviar la trama
static uint8_t enviar_trama(const muestra_t *p, uint8_t flags, bool evento) {
    static bool temp_alta_antes = false;
    int16_t temp_cc = 0;
    uint8_t t[TRAMA_LEN];

    if (!temp_leer_cc(&temp_cc)) {
        flags = flags | FLAG_TEMP_ERR;
        temp_cc = 0;
    }
    else {
        bool alta = temp_alta_antes ? (temp_cc > UMBRAL_TEMP_CC - HISTERESIS_CC) : (temp_cc > UMBRAL_TEMP_CC);
        if (alta) flags = flags | FLAG_TEMP_ALTA;
        if (alta && !temp_alta_antes) evento = true;
        temp_alta_antes = alta;
    }

    t[0] = TRAMA_SYNC;
    t[1] = ID_COCHE;
    t[2] = evento ? TIPO_EVENTO : TIPO_PERIODICA;
    t[3] = seq;
    put_i16(&t[4], p->ax);
    put_i16(&t[6], p->ay);
    put_i16(&t[8], p->az);
    put_i16(&t[10], temp_cc);
    t[12] = flags;
    t[13] = crc8(t, 13);

    HAL_UART_tx_array(t, TRAMA_LEN);   // envia al CC1310 (del transmisor)
    LCD_actualizar_datos(p, temp_cc, flags, t[2]);
    seq++;
    return flags;
}


// ===================== Tarea de 10 ms ==========
static void tarea_tick(void) {
    static muestra_t pico;
    static int32_t pico_m2 = -1;
    static uint16_t n = 0, holdoff = 0, t_alarma = 0, t_destello = 0, t_pitido = 0;
    static bool temp_alta = false;
    static bool s1_antes = false, s2_antes = false;

    // 1. Botones: flanco de pulsación y los lee cada 10ms (es el antirrebotre)
    bool s1 = !SWITCH_get_edumkii_S1();
    bool s2 = !SWITCH_get_edumkii_S2();
    bool sos = s2 && !s2_antes && sistemaOn;


    if (s1 && !s1_antes) {
        sistemaOn = !sistemaOn;
        pico_m2 = -1;
        n = 0;
        LCD_actualizar_status(sistemaOn);
    }
    s1_antes = s1;
    s2_antes = s2;

    // 2. Temporizadores de los acutadores
    if (t_alarma) t_alarma--;
    if (t_destello) t_destello--;
    if (t_pitido && --t_pitido == 0) buzzer_off();

    if (sistemaOn) {
        // 3. Acelerómetro lo lee
        muestra_t m;
        float gx = KXTC9_get_accel_x();
        float gy = KXTC9_get_accel_y();
        float gz = KXTC9_get_accel_z();
        HAL_ADC_trigger_single_conversion();

        m.ax = accel_a_mg(gx);
        m.ay = accel_a_mg(gy);
        m.az = accel_a_mg(gz);

        int32_t m2 = (int32_t)m.ax * m.ax + (int32_t)m.ay * m.ay + (int32_t)m.az * m.az;
        if (m2 > pico_m2) { // pico de |a| de la ventana
            pico_m2 = m2;
            pico = m;
        }

        bool impacto = false;
        if (holdoff) holdoff--;
        else if (m2 > UMBRAL_M2) {
            impacto = true;
            holdoff = HOLDOFF_EVENTO;
        }

        // 4. ENVIAR: inmediatamente si hay evento; sino, una vez por segundo
        n++;
        if (impacto || sos || n >= TICKS_TRAMA) {
            uint8_t flags = (impacto ? FLAG_IMPACTO : 0) | (sos ? FLAG_SOS : 0);
            flags = enviar_trama(&pico, flags, impacto || sos);
            temp_alta = (flags & FLAG_TEMP_ALTA) != 0;
            pico_m2 = -1;
            n = 0;

            if (flags & (FLAG_IMPACTO | FLAG_SOS)) {
                t_alarma = T_ALARMA;
                t_pitido = T_PITIDO;
                buzzer_on();
            }
            else t_destello = T_DESTELLO;
        }
    }
    led_actualizar(t_alarma, t_destello, temp_alta);
}


// ============== ACTUADORES ================
static void led_actualizar(uint16_t t_alarma, uint16_t t_destello, bool temp_alta) {
    if (!sistemaOn) LED_set_rgb(0, 0, 0); // apagado
    else if (t_alarma) LED_set_rgb(1, 0, 0); //rojo: impacto / SOS
    else if (t_destello) LED_set_rgb(0, 0, 1); //azul: trama enviada
    else if (temp_alta) LED_set_rgb(1, 1, 0); //amarillo: temp alta
    else LED_set_rgb(0, 1, 0); // verde: funcionando
}


static void buzzer_on(void) { //MODIFICARLA PARA Q QUEDE MÁS GUAY
    BUZ_sound_on();
}


static void buzzer_off(void) {
    BUZ_sound_off();
}


// =========== LCD =============
static void lcd_linea(uint8_t fila, const char *txt, uint32_t color) {
    char buf[22];
    uint8_t i = 0;
    while (txt[i] && i < 21) {
        buf[i] = txt[i];
        i++;
    }
    while (i < 21) {
        buf[i++] = ' '; // sobreescribe sin borrar (sin parpadeo)
    }
    buf[21] = '\0';
    Graphics_setForegroundColor(&g_sContext, color);
    Graphics_drawString(&g_sContext, (int8_t *)buf, 21, 1,
                        20 + 11 * fila, OPAQUE_TEXT);
}


static void LCD_init(void) {
    CRYSTALFONTZ_init();
    CRYSTALFONTZ_set_orientation(LCD_ORIENTATION_UP);
    Graphics_initContext(&g_sContext, &g_sCRYSTALFONTZ);
    Graphics_setFont(&g_sContext, &g_sFontFixed6x8);
    LCD_dibujar_pantalla();
    LCD_actualizar_status(false);
}


void LCD_dibujar_pantalla(void) {
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_NAVY);
    Graphics_clearDisplay(&g_sContext);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_drawStringCentered(&g_sContext, (int8_t *)"D.A.V.E. - COCHE",
                                    AUTO_STRING_LENGTH, 64, 8, OPAQUE_TEXT);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    Graphics_drawLineH(&g_sContext, 4, 123, 16);  // separador
}


void LCD_actualizar_status(bool status) {
    if (status) lcd_linea(0, "Sistema: ON", GRAPHICS_COLOR_GREEN);
    else lcd_linea(0, "Sistema: OFF", GRAPHICS_COLOR_GRAY);
}


static void LCD_actualizar_datos(const muestra_t *p, int16_t temp_cc, uint8_t flags, uint8_t tipo) {
    char l[32];
    uint16_t amag = (uint16_t)sqrtf((float)p->ax * p->ax + (float)p->ay * p->ay +
                                    (float)p->az * p->az);

    sprintf(l, "Trama #%-3u %s", (unsigned)seq, tipo == TIPO_EVENTO ? "EVENTO" : "(1 s)");
    lcd_linea(1, l, GRAPHICS_COLOR_WHITE);
    sprintf(l, "ax  %+6d mg", p->ax);
    lcd_linea(2, l, GRAPHICS_COLOR_WHITE);
    sprintf(l, "ay  %+6d mg", p->ay);
    lcd_linea(3, l, GRAPHICS_COLOR_WHITE);
    sprintf(l, "az  %+6d mg", p->az);
    lcd_linea(4, l, GRAPHICS_COLOR_WHITE);
    sprintf(l, "|a| %6u mg", amag);
    lcd_linea(5, l, GRAPHICS_COLOR_CYAN);

    if (flags & FLAG_TEMP_ERR) {
        lcd_linea(6, "T   ERROR I2C", GRAPHICS_COLOR_RED);
    } else {
        sprintf(l, "T   %d.%02d C", temp_cc / 100, abs(temp_cc % 100));
        lcd_linea(6, l, GRAPHICS_COLOR_WHITE);
    }

    if      (flags & FLAG_IMPACTO)   lcd_linea(8, ">> IMPACTO <<",    GRAPHICS_COLOR_RED);
    else if (flags & FLAG_SOS)       lcd_linea(8, ">> SOS MANUAL <<", GRAPHICS_COLOR_RED);
    else if (flags & FLAG_TEMP_ALTA) lcd_linea(8, "TEMPERATURA ALTA", GRAPHICS_COLOR_YELLOW);
    else                             lcd_linea(8, "Estado: normal",   GRAPHICS_COLOR_GREEN);
}


void main(void)
{

    WDT_A_hold(WDT_A_BASE);

    HAL_UART_init_gpio();
    HAL_LCD_init_gpio();
    HAL_ADC_init_gpio();
    HAL_DIGOUT_init_gpio();
    HAL_DIGIN_init_gpio();
    HAL_PWM_init_gpio();
    HAL_I2C_init_gpio();
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 | GPIO_PIN5,
                                               GPIO_PRIMARY_MODULE_FUNCTION);  // pines del cristal

    PMM_unlockLPM5();
    Clocks_init();              // después del unlock: el cristal necesita sus pines activos

    // Periféricos (después del unlock y de fijar los relojes)
    HAL_UART_config();
    HAL_LCD_config();
    HAL_ADC_config();
    HAL_DIGOUT_config();
    HAL_DIGIN_config();
    HAL_PWM_config();
    HAL_I2C_config();

    LCD_init();

    LED_set_rgb(0, 0, 0);
    __bis_SR_register(GIE);
    TimerA_init();
    TMP006_config();

    while (1) {
        __disable_interrupt();             // comprobar el flag y dormir sin carrera
        if (!flagTick)
            __bis_SR_register(LPM0_bits | GIE);   // el Timer nos despierta cada 10 ms
        __enable_interrupt();

        flagTick = false;
        tarea_tick();
    }
}

// ========== Timer: tick the 10ms ================0
void TimerA_init(void)
{
    Timer_A_initUpModeParam TimerConfig =
    {
         TIMER_A_CLOCKSOURCE_ACLK, //9.4kHz
         TIMER_A_CLOCKSOURCE_DIVIDER_1,
         // cuentas: ahora 32768/100 --> 10 ms
         (aclk_hz / TICK_HZ) - 1,
         TIMER_A_TAIE_INTERRUPT_ENABLE,
         TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
         TIMER_A_DO_CLEAR
    };
    Timer_A_initUpMode(TIMER_A1_BASE, &TimerConfig);
    Timer_A_enableInterrupt(TIMER_A1_BASE);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}


#pragma vector = TIMER1_A1_VECTOR
__interrupt void timer1_a1_isr_handler(void) {
    uint16_t status = Timer_A_getInterruptStatus(TIMER_A1_BASE);
    if (status == TIMER_A_INTERRUPT_PENDING) {
        Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
        flagTick = true;
        __bic_SR_register_on_exit(LPM0_bits); // para despertar al main al salir
    }
}


