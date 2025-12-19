# STM32 (Nucleo-L432KC) – Control de nivel con HC-SR04 + PID / Gain Scheduling + bombas (L298N) + UART

Proyecto para medir nivel (ultrasónico) y accionar dos bombas (inyección / extracción) usando PWM, con control **PID** o **Gain Scheduling**, y telemetría/comandos por **UART** hacia una interfaz (PC/ESP32).

## Hardware

- **MCU:** STM32L432KC (Nucleo-32)
- **Sensor:** HC-SR04 (ultrasónico)
- **Driver:** L298N (2 canales)
- **Actuadores:** 2 bombas DC (o 2 motores)
- **Interfaz:** UART (a PC vía ST-LINK VCP o a un ESP32 como puente)

### Conexiones (según este `main.c`)

#### HC-SR04
- `TRIG` → **PA3** (GPIO salida)
- `ECHO` → **PA8** (GPIO entrada)
- `VCC` → 5V
- `GND` → GND

> Importante: el pin ECHO del HC-SR04 entrega 5V. Usa divisor resistivo o level shifting a 3.3V si tu módulo no lo trae.

#### L298N – Direcciones
**Bomba Inyección (Motor A)**
- IN1 → **PA6**
- IN2 → **PA7**
- ENA (PWM) → **TIM2_CH1**

**Bomba Extracción (Motor B)**
- IN3 → **PB0**
- IN4 → **PB1**
- ENB (PWM) → **TIM2_CH2**

#### UART
- USART1 configurado a **115200, 8N1**
- Usado para:
  - recibir comandos tipo `SP,2.0` / `MODE,1`
  - enviar telemetría `T,...` y mensajes `HB`, `BOOT`, `ACK...`

> En esta versión no se reasignan pines en el README porque dependen de tu CubeMX/placa. Revisa en CubeMX el mapeo de **USART1 TX/RX** que quedó en tu proyecto.

---

## Firmware: qué hace

### 1) Medición por ultrasonido
Cada ciclo de control:
- Genera pulso TRIG de ~10 µs usando **TIM1** como contador de µs.
- Mide el ancho del pulso ECHO con lecturas del contador TIM1.
- Distancia (cm) ≈ `((Value2 - Value1) * 0.034) / 2`
- Convierte a “nivel” con:
  - `y = 29.4 - Distance`
- Aplica filtro IIR simple:
  - `dist_f = a * dist_f + (1-a) * y`
  - `a_filt = 0.7`

### 2) Control PID / Gain Scheduling
Periodo de control: `CTRL_TS_MS = 100` ms (10 Hz)

Ganancias base:
- `Kp_base = 400`
- `Ki_base = 3.2444`
- `Kd_base = 0.1013`

Modo:
- `MODE,0` → PID base
- `MODE,1` → Gain Scheduling (ajusta ganancias según |error| y cruces por cero)

Saturación:
- `u ∈ [-255, 255]`

Anti-windup suave:
- `Kw = 0.05`

### 3) Actuación de bombas (PWM)
- Si `u_sat > 0` → **inyección** (CH1)
- Si `u_sat < 0` → **extracción** (CH2)
- Escala PWM a partir del ARR del timer:
  - `duty = (u/255) * ARR`

Direcciones fijas (ajusta si gira al revés):
- Inyección: IN1=1, IN2=0
- Extracción: IN3=1, IN4=0

---

## Protocolo UART

### Comandos hacia STM32
- Cambiar setpoint (cm):
  - `SP,2.0`
- Cambiar modo:
  - `MODE,0` (PID)
  - `MODE,1` (Gain Scheduling)

Fin de comando: `\n` o `\r\n`

### Respuestas / telemetría desde STM32
- En el arranque:
  - `STM32 OK: manda SP,16 o MODE,1 / MODE,0`
  - `BOOT,SP=...,MODE=...`
- Heartbeat cada 1 s:
  - `HB,SP=...,MODE=...`
- Eco de recepción:
  - `RX,<tu_comando>`
- ACK:
  - `ACK_SP,<sp_x100>`
  - `ACK_MODE,<0|1>`
- Trama de datos cada ciclo de control:
  - `T,<dist_x100>,<sp_x100>,<u_x100>,<mode>`
  - Todo va escalado por 100 (enteros), para evitar floats en la UART.

Ejemplo:
- `T,215,200,10350,1`
  - dist=2.15 cm
  - sp=2.00 cm
  - u=103.50 (en escala -255..255)
  - modo=1 (GS)

---

## Configuración en STM32CubeMX (lo que debe quedar igual)

### Clock (SystemClock_Config)
- MSI + PLL habilitado (tal como está en el código generado)
- FLASH_LATENCY_4
- APB1 = APB2 = HCLK/1

### TIM1 (base timer para microsegundos)
- Prescaler = **71**
- Period = **65535**
- CounterMode = Up
- Uso: medir tiempos para HC-SR04 (~1 µs por tick si el timer corre a 72 MHz)

### TIM2 (PWM para bombas)
- Prescaler = **71**
- Period (ARR) = **99**
- PWM mode: PWM1 en:
  - Channel 1 (ENA)
  - Channel 2 (ENB)

> Con PSC=71 y ARR=99, el PWM queda ~10 kHz si TIM2 corre a 72 MHz.

### USART1
- BaudRate = **115200**
- 8 bits, 1 stop, sin paridad
- TX/RX habilitados
- Recepción por interrupción:
  - `HAL_UART_Receive_IT(&huart1, &rx_byte, 1);`

### GPIO
- PA3 (TRIG) salida push-pull
- PA8 (ECHO) entrada
- PA6/PA7 salidas (IN1/IN2)
- PB0/PB1 salidas (IN3/IN4)

---

## Cómo compilar y flashear

1. Abrir el proyecto en **STM32CubeIDE**.
2. Build (Debug o Release).
3. Conectar la Nucleo por USB (ST-LINK).
4. Flash/Run.

---

## Cómo probar rápido

1. Conecta UART a tu interfaz (PC o ESP32).
2. Abre un monitor serial a **115200**.
3. Deberías ver `BOOT...` y luego `HB...` cada 1 s.
4. Envía:
   - `MODE,1` para Gain Scheduling
   - `SP,2.0` para fijar setpoint a 2 cm
5. Observa tramas `T,...` y verifica que PWM actúe en la bomba correcta.

---

## Notas importantes (errores típicos)

- **HC-SR04 ECHO a 5V:** protege el pin del STM32 (divisor o level shifter).
- Si no ves nada por UART:
  - revisa que estás en el puerto correcto (ST-LINK VCP)
  - baudrate 115200
  - la red eléctrica y GND común si usas ESP32 como puente
- Si las bombas giran al revés:
  - invierte IN1/IN2 o IN3/IN4 en `main.c` (bloque “Sentidos fijos”)

---

## Estructura de control

- `process_cmd_main()` maneja `SP` y `MODE`
- `control_step()` calcula `u_sat`
- `pumps_write()` aplica PWM a TIM2 CH1/CH2
- Loop principal:
  1) recibe comandos (flag `cmd_ready`)
  2) heartbeat
  3) lectura ultrasonido
  4) filtro
  5) control
  6) PWM
  7) telemetría

---

## Licencia
Define la licencia que usará tu repo (MIT/BSD/GPL/etc.).
