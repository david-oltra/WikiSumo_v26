# 🤖 WIKISUMO v26

Firmware para robot de competición **MiniSumo** basado en **ESP32-S3**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

---

## 📋 Descripción

Robot de sumo autónomo con control de motores paso a paso por **STEP/DIR (RMT)**, sensores de distancia **ToF**, sensores de línea **QRE1113** con calibración persistente en **NVS**, display OLED de estado, receptor **RC5** y comunicación **ESP-NOW** con el mando.

---

## ✨ Características

- **2 motores paso a paso** con drivers **TMC2209** (control STEP/DIR vía RMT).
- **5 sensores de distancia VL6180X** (ToF, I2C con XSHUT).
- **6 sensores de línea QRE1113** (ADC) con calibración guiada por botón BOOT.
- **Persistencia en NVS** de los umbrales de los QRE.
- **Leds WS2812B** (3 LEDs) para debug visual con colores mixtos.
- **Display SSD1306** 128×32 (dohyo, strategy, estado).
- **Receptor RC5 (TSOP4838)** para start/stop.
- **ESP-NOW** para recibir la strategy desde el mando.
- **Sistema de zonas** con histéresis (IDLE → SLOW → APPROACH → PUSH).
- **Cambio dinámico de corriente** según distancia al rival (800 mA ↔ 2000 mA).
- **Escape de borde** con debounce configurable.
- **Modo ciego** en la strategy de emergencia (ignora sensores).
- **Rampa software** para cambios suaves de velocidad.
- **Frenada rápida** para inversiones de dirección.
- Arquitectura **FreeRTOS** basada en tareas.

---

## 🔧 Hardware

### Componentes principales

| Componente         | Cantidad | Descripción                                    |
|--------------------|:--------:|------------------------------------------------|
| ESP32-S3           |    1     | MCU principal                                  |
| TMC2209            |    2     | Driver de motor paso a paso (UART + STEP/DIR)  |
| VL6180X            |   2-5    | Sensor de distancia ToF (I2C)                  |
| QRE1113            |   2-6    | Sensor reflectivo de línea (ADC)               |
| WS2812B            |    3     | LEDs RGB direccionables                        |
| SSD1306 (128×32)   |    1     | Display OLED I2C                               |
| TSOP4838           |    1     | Receptor IR RC5                                |

### Notas importantes de hardware

- **Bus UART single-wire de los TMC2209**: los pines TX (GPIO43) y RX (GPIO44) del ESP32 deben ir unidos al PDN_UART de los drivers. Es imprescindible:
  - Una **resistencia pull-up** en la línea (interna del ESP32 o externa de 1 kΩ a VIO).
  - Una **resistencia de 1 kΩ en serie** entre el TX y el PDN_UART.
  - El RX debe ir **directo** al PDN_UART (sin resistencia en serie).
- **Pin CLK del TMC2209**: debe estar a **GND** para que use el oscilador interno a 12 MHz y el baud rate de 115200 bps.
- **Direcciones UART**: los pines MS1/MS2 configuran la dirección de cada driver (0x00, 0x01...). Deben ser distintas.
- **LED WS2812B**: si está cerca de un VL6180X, puede afectar ligeramente a las lecturas. Separar físicamente si es posible.

---

## 🔌 Pinout

### Motores (TMC2209)

| Señal        | GPIO |
|--------------|:----:|
| MOTOR_A_STEP |  10  |
| MOTOR_A_DIR  |  11  |
| MOTOR_B_STEP |   8  |
| MOTOR_B_DIR  |   9  |
| UART_TX      |  43  |
| UART_RX      |  44  |

### I2C (VL6180X + SSD1306)

| Señal | GPIO |
|-------|:----:|
| SDA   |   3  |
| SCL   |   2  |

### VL6180X (XSHUT)

| Sensor | GPIO |
|:------:|:----:|
| 1 |  7 |
| 2 |  4 |
| 3 |  5 |
| 4 |  6 |
| 5 | 12 |

### QRE1113 (ADC)

| Sensor | GPIO |
|:------:|:----:|
| 1 | 34 |
| 2 | 39 |
| 3 | 32 |
| 4 | 33 |
| 5 | 36 |
| 6 | 25 |

### Otros

| Función         | GPIO |
|-----------------|:----:|
| WS2812B data    |   1  |
| RC5 (TSOP4838)  |  13  |
| LED de estado   |  48  |
| Botón BOOT      |   0  |

---

## 🎮 Uso

### Mando IR (RC5)

- **START** → arranca el robot.
- **STOP** → detiene el robot.
- **START** en STOPPED → reinicia el dispositivo.
- **PROGRAM** → cambia el número base de dohyo.

### Calibración de los QRE

El robot carga los umbrales desde **NVS** al arrancar. En el primer arranque (NVS vacío), lanza una **calibración guiada**:

1. **Paso 1**: colocar el robot sobre **negro**, pulsar BOOT.
2. **Paso 2**: colocar el **QRE1** sobre **blanco**, pulsar BOOT.
3. **Paso 3**: colocar el **QRE6** sobre **blanco**, pulsar BOOT.

El umbral se calcula como **punto medio** entre negro y blanco, y se guarda en NVS.

**Recalibrar** (al cambiar de dohyo):

1. Encender el robot y esperar a que esté en **POWER_ON**.
2. Pulsar **BOOT**.
3. Seguir los 3 pasos guiados por el display.
4. El robot guarda los nuevos valores y **se reinicia** automáticamente.

### LEDs WS2812B (debug)

Cada LED combina un sensor VL y un QRE:

| LED | Color | Significado |
|:---:|:---:|---|
| 0 | 🔵 Azul | VL2 detectando algo |
| 0 | 🔴 Rojo | QRE1 detectando blanco |
| 0 | 🟣 Magenta | Ambos a la vez |
| 1 | 🔵 Azul | VL3 detectando algo |
| 2 | 🔵 Azul | VL4 detectando algo |
| 2 | 🔴 Rojo | QRE6 detectando blanco |
| 2 | 🟣 Magenta | Ambos a la vez |

---

## 🧠 Estrategias

Recibidas desde el mando remoto vía ESP-NOW:

| ID | Nombre | Descripción |
|:--:|---|---|
| 0  | **Default** | Avance recto continuo |
| 1  | **Ataque lateral** | Gira a la izquierda 300 ms, luego avance recto |
| 2  | **Emergencia (modo ciego)** | Avanza 300 ms → gira 300 ms → repite en bucle. **Ignora todos los sensores.** |

### Zonas de aproximación al rival

El sistema de **zonas** ajusta velocidad y corriente según la distancia frontal (VL6180X #3):

| Zona | Nombre | Distancia | Velocidad | Corriente |
|:---:|---|---|:---:|:---:|
| 0 | IDLE | > 120 mm | 600 Hz | 800 mA |
| 1 | SLOW | 90-120 mm | 2000 Hz | 800 mA |
| 2 | APPROACH | 60-90 mm | 2000 Hz | 800 mA |
| 3 | PUSH | < 60 mm | 500 Hz | **2000 mA** |

La **histéresis** (umbrales de subida vs bajada) evita oscilaciones cuando la distancia está en el límite:
- **Subir** de zona: `d < 120 / 90 / 60 mm`.
- **Bajar** de zona: `d >= 135 / 105 / 75 mm`.

### Jerarquía de prioridades de sensores

```
1. QRE (borde)              → máxima prioridad (sobrescribe todo)
   ↓
2. VL3 (frontal, zona ≥ 2)  → bloquea laterales desde APPROACH
   ↓
3. VL2 / VL4 (laterales)    → solo actúan en zonas 0 y 1 (IDLE, SLOW)
   ↓
4. Strategy (ESP-NOW)       → dirección por defecto
```

### Escape por borde

Los QRE1113 controlan la detección de borde del dohyo:

- **QRE1 en blanco** → giro cerrado a la derecha.
- **QRE6 en blanco** → giro cerrado a la izquierda.
- **Ambos en blanco** → marcha atrás recta.

El escape dura **300 ms** mínimo aunque el sensor deje de ver la línea. Se aplica **debounce** de 2 lecturas para evitar falsos positivos.

---

## 🔋 WiFi y ADC2

Para evitar el **ruido del WiFi en el ADC2** (donde está el QRE6), el firmware:

1. Inicializa ESP-NOW **después** de la calibración (con WiFi OFF).
2. Mantiene ESP-NOW activo durante POWER_ON (para recibir la strategy).
3. **Apaga ESP-NOW automáticamente** al pasar a STARTED para liberar el ADC2.

Es irreversible sin reiniciar, pero aceptable porque la strategy se fija antes de la ronda.

---

## 🗺️ Roadmap

- [x] Driver TMC2209 (STEP/DIR con RMT)
- [x] Sensores VL6180X (I2C)
- [x] Sensores QRE1113 (ADC + NVS)
- [x] Tira WS2812B
- [x] Receptor RC5
- [x] Display SSD1306
- [x] ESP-NOW
- [x] Calibración guiada por botón BOOT
- [x] Persistencia NVS de umbrales
- [x] Sistema de zonas con histéresis
- [x] Cambio dinámico de corriente
- [x] Estrategias 0, 1 y 2 (con modo ciego)
- [x] Jerarquía de prioridades entre sensores
- [ ] Ajuste del ángulo de montaje de los VL6180X (ven el suelo)
- [ ] Integración IMU BMI160 (detección de vuelco)
- [ ] Estrategias adicionales
- [ ] Optimización del consumo

---

## 📄 Licencia

Este proyecto está bajo la **GNU General Public License v3.0**.

Copyright (C) 2026 **david_wiki**

Consulta el archivo [LICENSE](LICENSE) para más detalles.

---

## 👤 Autor

**david_wiki**

- GitHub: [@david-oltra](https://github.com/david-oltra)
