# D.A.V.E. — Detección Automática de Vehículos y Emergencias

Proyecto final de **Microprocesadores** (Grupo 1). Sistema de alerta de accidentes con dos nodos comunicados por radio Sub-1 GHz.

Después de un accidente, el conductor no siempre puede pedir ayuda. En D.A.V.E. el coche avisa solo: el nodo transmisor mide aceleraciones y temperatura y las envía por radio a una central (por ejemplo, la de una aseguradora). La central detecta accidentes, posibles incendios o peticiones de SOS, da la alarma y estima a qué distancia está el coche a partir del RSSI de la señal.

**Autores:** Jesús Bandrés, Ana Carolina Casares, Ainhoa Gibaja, Kinga Kerekes y Nicolás Ros.

---

## 1. Arquitectura

Cada nodo tiene las mismas placas: un **MSP-EXP430FR5994** con la **BOOSTXL-EDUMKII** (sensores, LCD, LED RGB, zumbador, pulsadores) y un **LAUNCHXL-CC1310** que hace de módem de radio. MSP430 y CC1310 se comunican por UART.

| Carpeta | Proyecto CCS | Placa | Función |
|---|---|---|---|
| [`msp_tx/`](msp_tx) | `Nodo_TX_MSP430_v2` | MSP430FR5994 + EDUMKII (coche) | Lee el acelerómetro cada 10 ms, detecta impactos, gestiona S1 (on/off) y S2 (SOS), monta la trama de 14 bytes y la envía al CC1310. |
| [`cc1310_tx/`](cc1310_tx) | `Radio_Tx_CC1310_v2` | LAUNCHXL-CC1310 (coche) | Puente UART - radio: recibe la trama, comprueba el CRC y la retransmite sin modificarla. |
| [`cc1310_rx/`](cc1310_rx) | `Radio_Rx_CC1310` | LAUNCHXL-CC1310 (central) | Puente radio - UART: valida la trama, le añade el RSSI y un segundo CRC y la envía al MSP430 (16 bytes). |
| [`msp_rx/`](msp_rx) | `Nodo_Rx_MSP430` | MSP430FR5994 + EDUMKII (central) | Recibe, verifica y desempaqueta. Detecta alarmas, estima la distancia, controla LCD, LED, sirena y la traza por PuTTY. |

---

## 2. Funcionamiento

### Nodo TX (coche) — `msp_tx/main.c`

- Tick de 10 ms con Timer_A1 alimentado por ACLK (cristal de 32 768 Hz, si no arranca, VLO). Entre ticks la CPU duerme en LPM0.
- En cada tick, si el sistema está encendido:
  1. Lee las tres aceleraciones y las convierte a mg (`int16`).
  2. Calcula `ax² + ay² + az²` y lo compara con el umbral al cuadrado (sin raíz cuadrada).
  3. Guarda la muestra pico de la ventana para que un golpe breve no se pierda entre dos tramas.
- Envío de tramas
  - *Periódica* (tipo 1): una por segundo.
  - *Evento* (tipo 2): inmediata si hay impacto (|a| > 1800 mg, con 2 s de guarda), SOS (S2) o la temperatura pasa a alta (> 45 °C).
- Botones: S1 enciende/apaga el sistema, S2 envía un SOS manual.
- Aviso local: LED rojo + pitidosi hay evento, destello azul de 100 ms en cada trama periódica. El LCD muestra el estado, la última trama, las aceleraciones, |a| y la temperatura.

### Radio TX — `cc1310_tx/rfEasyLinkTx_nortos.c`

- UART a 115 200 baudios (binaria). Monta la trama byte a byte buscando la cabecera `0xA5`, comprueba el CRC-8 y la envía con EasyLink a 863 MHz, dirección destino `0xAA`, potencia fija de 10 dBm (necesaria para que el RSSI tenga sentido como medida de distancia).
- Si el CRC falla, busca la siguiente cabecera dentro del buffer para resincronizarse sin perder la trama siguiente.
- `MODO_DIAG` permite depurar por partes: `0` solo radio (trama de prueba cada segundo), `1` solo UART, `2` puente completo (el que se usa).

### Radio RX — `cc1310_rx/rfEasyLinkRx_nortos.c`

- Recepción síncrona con filtro de dirección `0xAA` y timeout de 2 s.
- Valida longitud (14), cabecera y CRC. Si es correcta, envía al MSP430: trama (14) + RSSI (1) + CRC-8 del enlace UART (1) = 16 bytes.
- `ENVIAR_PRUEBA_UART = 1` envía una trama de prueba al arrancar para comprobar el tramo CC1310 → MSP430 sin radio.

### Nodo RX (central) — `msp_rx/main.c`

- La UART A3 recibe por interrupción en un buffer circular de 64 bytes (`hal/hal_uart.c`). El bucle principal vacía ese buffer y reconstruye las tramas de 16 bytes con la misma lógica de resincronización por cabecera.
- Comprueba los dos CRC (el del coche y el del enlace UART). Las tramas corruptas se cuentan y se vuelcan en hexadecimal por PuTTY.
- Cuenta tramas perdidas con los saltos del número de secuencia y declara enlace perdido si pasan 3 s sin tramas válidas.
- Alarmas por prioridad (una más grave sustituye a otra sin atender):
  1. **Accidente:** |a| > 1800 mg o flag de impacto.
  2. **SOS manual:** flag SOS.
  3. **Posible incendio:** temperatura > 45 °C (solo en el flanco).
- Con una alarma nueva estima la distancia con el modelo log-distancia
  `d = d0 · 10^((RSSI(d0) − RSSI) / (10·n))`, activa la sirena (pitidos de 250 ms durante 10 s como máximo) y muestra el aviso en el LCD. **S1** da la alarma por atendida.

---

## 3. Protocolo de trama

Todos los campos de 16 bits van en little-endian. CRC-8 con polinomio `0x07` y valor inicial `0x00`, implementado igual en los cuatro programas.

Trama del coche (14 bytes) — MSP430 Tx → CC1310 Tx → radio → CC1310 Rx

| Byte | Campo | Tipo | Notas |
|---|---|---|---|
| 0 | Cabecera | `uint8` | `0xA5` |
| 1 | ID coche | `uint8` | `0x01` |
| 2 | Tipo | `uint8` | 1 = periódica, 2 = evento |
| 3 | Secuencia | `uint8` | +1 por trama |
| 4–5 | ax | `int16` | mg |
| 6–7 | ay | `int16` | mg |
| 8–9 | az | `int16` | mg |
| 10–11 | Temperatura | `int16` | centésimas de °C |
| 12 | Flags | `uint8` | bit0 impacto · bit1 temperatura alta · bit2 SOS · bit3 error del sensor de temperatura |
| 13 | CRC-8 | `uint8` | bytes 0–12 |

**Ampliación en el tramo CC1310 Rx → MSP430 Rx (16 bytes)**

| Byte | Campo | Tipo | Notas |
|---|---|---|---|
| 14 | RSSI | `int8` | dBm, medido por el CC1310 receptor |
| 15 | CRC-8 UART | `uint8` | bytes 0–14 (cubre también el RSSI) |

> Cualquier cambio en el formato de la trama hay que hacerlo a la vez en los cuatro proyectos.

---

## 4. Hardware y cableado

Por nodo: 1× MSP-EXP430FR5994, 1× BOOSTXL-EDUMKII, 1× LAUNCHXL-CC1310 con antena y 3 cables hembra-hembra. Cada placa se alimenta por su propio USB.

En el LAUNCHXL-CC1310 hay que quitar los jumpers RXD y TXD y conectar los cables en el lado del microcontrolador.

| MSP430FR5994 | | CC1310 |
|---|---|---|
| P6.0 / UCA3TXD (J1 pin 4) | → | DIO2 (RXD) |
| P6.1 / UCA3RXD (J1 pin 3) | ← | DIO3 (TXD) |
| GND | ↔ | GND |

En el coche solo se usa TX del MSP430 (P6.0 → DIO2) y en la central solo RX (DIO3 → P6.1); el GND siempre es obligatorio. UART a 115 200 baudios, 8N1.

### Periféricos utilizados

| Periférico | `msp_tx` | `msp_rx` |
|---|---|---|
| CS | DCO 16 MHz (MCLK, SMCLK); ACLK = LFXT 32 768 Hz (VLO de reserva) | Igual |
| Timer_A1 | Tick de 10 ms (despierta de LPM0) | Tick de 10 ms |
| Timer_B0 (PWM) | Zumbador | Zumbador (sirena) |
| ADC12_B | Acelerómetro KXTC9-2050 (A12–A14), por interrupción | — |
| eUSCI_B2 (I²C) | TMP006 | — |
| eUSCI_B (SPI) | LCD 128×128 con GRLIB | LCD 128×128 con GRLIB |
| eUSCI_A3 (UART) | TX hacia el CC1310 | RX desde el CC1310 (interrupción + buffer circular) |
| eUSCI_A0 (UART) | — | Traza a PuTTY por el USB de la LaunchPad |
| GPIO | S1/S2 (interrupción de puerto), LED RGB | S1, LED RGB |

---

## 5. Compilar y cargar

Software: Code Composer Studio 10.3 (compilador TI 20.2.4.LTS), SDK del curso `msp-exp430FR5994-edumkii` (DriverLib 2.91.13.01 y GRLIB 3.21, incluidas en cada proyecto MSP430) y SimpleLink CC13x0 SDK 4.10.01.01 para los CC1310.

1. Clonar el repositorio.
2. En CCS: Project → Import CCS Projects…, seleccionar la carpeta del repositorio e importar los cuatro proyectos.
3. Comprobar que `FREQ_HZ` es la misma en `cc1310_tx` y `cc1310_rx` (863 MHz) y que `MODO_DIAG = 2` en `cc1310_tx`.
4. Compilar cada proyecto y cargarlo en su placa con Debug; después, Terminate. El programa queda grabado y arranca solo al alimentar la placa.

Orden recomendado de arranque: CC1310 Rx → MSP430 Rx → CC1310 Tx → MSP430 Tx. Si no se respeta, el sistema se resincroniza solo con la cabecera de la siguiente trama.

### Calibración

| Constante | Fichero | Valor actual | Significado |
|---|---|---|---|
| `UMBRAL_IMPACTO_MG` | `msp_tx/main.c`, `msp_rx/main.c` | 1800 | Umbral de impacto en mg (igual en ambos) |
| `UMBRAL_TEMP_CC` | `msp_tx/main.c`, `msp_rx/main.c` | 4500 | 45,00 °C |
| `RSSI_REF_DBM` | `msp_rx/main.c` | −40 | RSSI medido a `D_REF_M` |
| `D_REF_M` | `msp_rx/main.c` | 1,0 | Distancia de referencia (m) |
| `N_PROPAG` | `msp_rx/main.c` | 2,5 | Exponente de pérdidas (2 en espacio libre; 2,5–4 en interiores) |

---

## 6. Señalización y depuración

| Placa | Indicación | Significado |
|---|---|---|
| MSP430 Tx | Apagado | Sistema OFF |
| | Verde | Funcionando |
| | Azul (100 ms) | Trama periódica enviada |
| | Rojo (3 s) + pitido | Impacto o SOS enviado |
| | Amarillo | Temperatura alta |
| MSP430 Rx | Amarillo | Sin señal del coche |
| | Verde | Enlace OK |
| | Azul (100 ms) | Trama válida recibida |
| | Rojo parpadeando + sirena | Alarma activa (S1 para atender) |
| CC1310 Tx | Verde ×1/×2/×3 al arrancar | LEDs / UART / radio OK |
| | Rojo ×N repetido | Error de arranque (2 UART, 3 EasyLink, 4 frecuencia, 5 potencia) |
| | Verde breve | Trama reenviada por radio |
| | Rojo breve | Fallo de transmisión |
| | Rojo + verde | CRC incorrecto en la UART |
| CC1310 Rx | Verde ×1/×2/×3 al arrancar | LEDs / UART / radio OK |
| | Verde (cambia) | Trama válida reenviada al MSP430 |
| | Rojo fijo | Timeout: sin señal del coche |
| | Rojo (cambia) | Paquete inválido o error de radio |

**PuTTY:** puerto *MSP Application UART* de la LaunchPad del nodo Rx, Serial, 115 200 baudios. Muestra cada trama recibida, las alarmas, los errores de CRC (en hexadecimal) y los cambios de estado del enlace.

| Síntoma | Causa probable |
|---|---|
| No llega nada por UART | TX/RX sin cruzar, GND sin conectar o jumpers RXD/TXD puestos |
| Llega basura | Baudrate distinto de 115 200 o GND flojo |
| CC1310 Rx en rojo fijo | Frecuencias distintas, antena mal roscada o coche apagado (S1) |
| Errores de CRC continuos | Formato de trama distinto entre proyectos |

---

## 7. Limitaciones conocidas

- La lectura de temperatura (`temp_leer_cc()` en `msp_tx/main.c`) devuelve un valor fijo de 24,50 °C; el TMP006 se configura pero su lectura no está conectada a la trama.
- El nodo Tx arranca ya encendido: como `tarea_tick()` usa `!SWITCH_get_edumkii_S1()`, el primer tick se interpreta como pulsación. Después, cada pulsación de S1 alterna ON/OFF con normalidad.
- La distancia por RSSI es orientativa: depende mucho del entorno y de la calibración de `RSSI_REF_DBM` y `N_PROPAG`.
- Un solo coche (ID fijo `0x01`); el protocolo admite más, pero la central no los distingue.
