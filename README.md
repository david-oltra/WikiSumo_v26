# 🤖 WIKISUMO

Firmware para robot de competición **MiniSumo** basado en **ESP32-S3**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

---

## 📋 Descripción

Robot minisumo autónomo con control de motores paso a paso, sensores de distancia ToF, sensores de línea, receptor IR, display OLED y comunicación inalámbrica ESP-NOW.

---

## ✨ Características

- 2 motores paso a paso con drivers **TMC2209** (UART + STEP/DIR)
- 5 sensores de distancia **VL6180X** (ToF, I2C)
- 6 sensores de línea **QRE1113** (ADC)
- Leds **WS2812B** (3 LEDs)
- Display **SSD1306** 128×32 (I2C)
- Receptor **RC5 (TSOP4838)** para start/stop
- **ESP-NOW** para recibir estrategia desde mando remoto
- Persistencia de configuración en **NVS**
- Arquitectura **FreeRTOS** basada en tareas

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
| Servo PWM       |   6  |

---

## 🎮 Uso

### Mando IR (RC5)

- **START** → arranca el robot
- **STOP** → detiene el robot
- **START** en STOPPED → reinicia el dispositivo
- **PROGRAM** → cambia el número base de dohyo

### LEDs WS2812B

| Color      | Significado              |
|------------|--------------------------|
| 🟢 Verde   | Inicialización correcta  |
| 🔴 Rojo    | Error en inicialización  |
| 🟣 Púrpura | Efecto de bienvenida     |
| ⚫ Apagado | Inactivo                 |

---

## 🧠 Estrategias

Recibidas desde el mando remoto vía ESP-NOW:

| ID | Descripción                             |
|:--:|-----------------------------------------|
| 0  | Modo por defecto (avance frontal)       |
| 1  | Giro a la izquierda durante 2 s         |
| 2  | Emergencia (alterna sentido cada 1 s)   |

Ajustes dinámicos según distancia frontal (VL6180X #3) y sensores de línea (QRE1113 #1 y #6).

---

## 🗺️ Roadmap

- [x] Driver TMC2209
- [x] Sensores VL6180X
- [x] Sensores QRE1113
- [x] Leds WS2812B
- [x] Receptor RC5
- [x] Display SSD1306
- [x] ESP-NOW
- [x] Persistencia NVS
- [ ] Integración IMU BMI160
- [ ] Control PID

---

## 📄 Licencia

Este proyecto está bajo la **GNU General Public License v3.0**.

Copyright (C) 2026 **david_wiki**

Consulta el archivo [LICENSE](LICENSE) para más detalles.

---

## 👤 Autor

**david_wiki**

- GitHub: [@david-oltra](https://github.com/david-oltra)
