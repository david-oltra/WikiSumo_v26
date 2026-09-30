/**
 * @file    main.c
 * @brief   Firmware principal del robot WIKISUMO.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Orquesta la inicialización y ejecución de todos los subsistemas:
 *
 *   - 2x TMC2209 (drivers de motor paso a paso) por STEP/DIR (RMT) + UART para config
 *   - 5x VL6180X (sensores ToF) por I2C con XSHUT
 *   - 6x QRE1113 (sensores de línea reflectivos) por ADC
 *   - Tira WS2812B (3 LEDs) por RMT
 *   - Display SSD1306 128x32 por I2C
 *   - Receptor RC5 (TSOP4838) para start/stop
 *   - ESP-NOW para recibir la "strategy" desde el mando
 *   - BMI160 (IMU) para detección de vuelco (DESHABILITADO temporalmente)
 *
 * Flujo de arranque:
 *   1. Se inicializa el display y el bus I2C maestro.
 *   2. Se lanzan tareas de inicialización (TMC2209, RC5, WS2812B,
 *      VL6180X, QRE1113) de forma secuencial; cada una notifica
 *      al hilo principal al terminar (mecanismo del "boot_handle").
 *   3. Se lanzan las tareas de ejecución continua (tmc2209_task,
 *      ssd1306_task, espnow_rx_task).
 *
 * ⚠️  ESTADO ACTUAL DE PRUEBAS:
 *   - BMI160 deshabilitado (comentado).
 *   - VL6180X: lógica de zonas deshabilitada (los sensores ven el
 *     suelo por problema de ángulo de montaje).
 *   - QRE1113: umbrales persistentes en NVS. Calibración guiada con
 *     el botón BOOT solo la primera vez o bajo demanda (pulsar BOOT
 *     en POWER_ON).
 *   - ESP-NOW: se apaga automáticamente al pasar a STARTED para
 *     liberar el ADC2 del ruido de WiFi (QRE6 usa ADC2).
 *   - Display: muestra VL6180X #2 y zona actual (modo debug).
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

/* Librería estándar de C (newlib) */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

/* FreeRTOS */
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/* ESP-IDF */
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <driver/rmt_tx.h>
#include "driver/i2c.h"

/* Módulos propios */
#include "tmc2209.h"
#include "vl6180x.h"
#include "qre1113.h"
#include "ws2812b.h"
#include "servo.h"
#include "pid.h"
#include "espnow.h"
#include "rc5_module.h"
#include "ssd1306.h"
// #include "bmi160.h"   /* Deshabilitado temporalmente */

/* =====================================================================
 *  CONFIGURACIÓN DE HARDWARE
 * ===================================================================== */

/* --- Bus I2C (SSD1306 + VL6180X) --- */
#define I2C_PORT        I2C_NUM_0
#define SCL             GPIO_NUM_2
#define SDA             GPIO_NUM_3

/* --- Pines de propósito general --- */
#define MODE_PIN        GPIO_NUM_13     /* Entrada analógica selector de modo */
#define SERVO_PIN       GPIO_NUM_7      /* ⚠️ Colisión con VL6180X_1           */
#define START_PIN       GPIO_NUM_13     /* ⚠️ Colisión con MODE_PIN            */

/* --- VL6180X (XSHUT individual por sensor) --- */
#define VL6180X_1       GPIO_NUM_7      /* ⚠️ Colisión con SERVO_PIN           */
#define VL6180X_2       GPIO_NUM_4
#define VL6180X_3       GPIO_NUM_5
#define VL6180X_4       GPIO_NUM_6
#define VL6180X_5       GPIO_NUM_12

/* --- QRE1113 (canales ADC) --- */
#define QRE1113_1       ADC_CHANNEL_6   /* GPIO34 */
#define QRE1113_2       ADC_CHANNEL_3   /* GPIO39 */
#define QRE1113_3       ADC_CHANNEL_4   /* GPIO32 */
#define QRE1113_4       ADC_CHANNEL_5   /* GPIO33 */
#define QRE1113_5       ADC_CHANNEL_0   /* GPIO36 */
#define QRE1113_6       ADC_CHANNEL_1   /* GPIO25 (ADC2) */

/* --- UART compartida por los dos TMC2209 (solo para config) --- */
#define UART_NUM        UART_NUM_0
#define UART_TX_PIN     GPIO_NUM_43
#define UART_RX_PIN     GPIO_NUM_44

/* --- Direcciones UART de cada TMC2209 --- */
#define MOTOR_A_ADDR    0x00
#define MOTOR_B_ADDR    0x01

/* --- Pines STEP/DIR de cada motor --- */
#define MOTOR_A_DIR     GPIO_NUM_11
#define MOTOR_A_STEP    GPIO_NUM_10
#define MOTOR_B_DIR     GPIO_NUM_9
#define MOTOR_B_STEP    GPIO_NUM_8

/* --- Tira WS2812B --- */
#define LED_PIN             1           /**< GPIO de datos               */
#define LED_NUM_LEDS        3           /**< Número de LEDs              */
#define LED_RESOLUTION      40000000    /**< Resolución RMT (40 MHz)     */
#define LED_BRIGHTNESS      255         /**< Brillo global 0-255         */

/* --- RC5 / LED de estado --- */
#define TSOP4838_OUT_PIN    GPIO_NUM_13 /* ⚠️ Colisión con MODE_PIN/START_PIN */
#define STATUS_LED_PIN      GPIO_NUM_48

/* --- Botón BOOT del ESP32-S3 (GPIO 0) ---
 * ⚠️  GPIO 0 es un strapping pin. Durante el arranque el chip lo lee
 *     para decidir si entra en modo descarga. Como ya tiene pull-up
 *     interno (+ externo en la DevKit), es seguro usarlo en runtime.
 *
 *     NO pulsar durante un reset: entraría en modo descarga. */
#define BOOT_BUTTON_PIN     GPIO_NUM_0

/* --- BMI160 (DESHABILITADO TEMPORALMENTE) ---
 * Se mantienen los defines comentados para reactivarlos cuando
 * se solucione el montaje del sensor. */
// #define LIFT_ENTER_DEG   20.0f   /* umbral para considerar morro arriba */
// #define LIFT_EXIT_DEG    10.0f   /* umbral para considerar morro abajo  */
// #define LIFT_CONFIRM_MS  80      /* tiempo mínimo por encima del umbral */
// static volatile bool s_robot_lifted = false;

/* =====================================================================
 *  INSTANCIAS DE PERIFÉRICOS
 * ===================================================================== */

/* --- Drivers de motor --- */
static tmc2209_t motor1 = {
    .step_pin  = MOTOR_A_STEP,
    .dir_pin   = MOTOR_A_DIR,
    .uart_addr = MOTOR_A_ADDR,
    .uart_num  = UART_NUM,
    .tx_pin    = UART_TX_PIN,
    .rx_pin    = UART_RX_PIN
};

static tmc2209_t motor2 = {
    .step_pin  = MOTOR_B_STEP,
    .dir_pin   = MOTOR_B_DIR,
    .uart_addr = MOTOR_B_ADDR,
    .uart_num  = UART_NUM,
    .tx_pin    = UART_TX_PIN,
    .rx_pin    = UART_RX_PIN
};

/* --- Sensores de distancia VL6180X ---
 * Cada uno con su propio XSHUT para poder reasignar dirección I2C.
 * Modo por defecto: MIN_DISTANCE (para detección de rival / pared). */
vl6180x_t vl_sensor_1 = { .id=1, .xshut_pin=VL6180X_1, .i2c_addr=0x30, .config_mode=VL6180X_CONFIG_MODE_MIN_DISTANCE };
vl6180x_t vl_sensor_2 = { .id=2, .xshut_pin=VL6180X_2, .i2c_addr=0x31, .config_mode=VL6180X_CONFIG_MODE_MIN_DISTANCE };
vl6180x_t vl_sensor_3 = { .id=3, .xshut_pin=VL6180X_3, .i2c_addr=0x32, .config_mode=VL6180X_CONFIG_MODE_MIN_DISTANCE };
vl6180x_t vl_sensor_4 = { .id=4, .xshut_pin=VL6180X_4, .i2c_addr=0x33, .config_mode=VL6180X_CONFIG_MODE_MIN_DISTANCE };
vl6180x_t vl_sensor_5 = { .id=5, .xshut_pin=VL6180X_5, .i2c_addr=0x34, .config_mode=VL6180X_CONFIG_MODE_MIN_DISTANCE };

/* --- Sensores de línea QRE1113 --- */
qre1113_sensor_t qre_sensor1 = { .id=1, .adc_channel=QRE1113_1, .is_adc2=false };
qre1113_sensor_t qre_sensor2 = { .id=2, .adc_channel=QRE1113_2, .is_adc2=false };
qre1113_sensor_t qre_sensor3 = { .id=3, .adc_channel=QRE1113_3, .is_adc2=false };
qre1113_sensor_t qre_sensor4 = { .id=4, .adc_channel=QRE1113_4, .is_adc2=false };
qre1113_sensor_t qre_sensor5 = { .id=5, .adc_channel=QRE1113_5, .is_adc2=false };
qre1113_sensor_t qre_sensor6 = { .id=6, .adc_channel=QRE1113_6, .is_adc2=true  };

/* --- LEDs WS2812B --- */
ws2812b_config_t led = {
    .gpio_num          = LED_PIN,
    .num_leds          = LED_NUM_LEDS,
    .rmt_resolution_hz = LED_RESOLUTION,
};

/* --- Display OLED SSD1306 --- */
static SSD1306_t oled_dev;
static char      display_buffer[16] = {0};


/* =====================================================================
 *  ESTADO GLOBAL DE ARRANQUE
 * ===================================================================== */

static TaskHandle_t boot_handle;        /**< Handle del hilo principal para sincronizar el boot */
static TaskHandle_t h_ws2812b_task   = NULL;
static TaskHandle_t h_vl6180x_task   = NULL;
static TaskHandle_t h_qre1113_task   = NULL;
// static TaskHandle_t h_bmi160_task    = NULL;    /* BMI160: sin usar por ahora */
static TaskHandle_t h_ssd1306_task   = NULL;
static TaskHandle_t h_espnow_rx_task = NULL;
static TaskHandle_t h_tmc2209_task   = NULL;

/** Banderas que indican si un subsistema terminó su init correctamente. */
typedef struct {
    uint8_t rc5;
    uint8_t ws2812b;
    uint8_t vl6180x;
    uint8_t qre1113;
    uint8_t tmc2209;
    uint8_t ssd1306;
    uint8_t bmi160;
} boot_t;
boot_t boot;

/**
 * @brief  Zona actual del robot respecto al rival (para histéresis).
 *
 * Valores:
 *   0 = IDLE      (lejos, >120 mm)
 *   1 = SLOW      (~90-120 mm)
 *   2 = APPROACH  (~60-90 mm)
 *   3 = PUSH      (<60 mm)
 *
 * Es global (no local a tmc2209_task) para que ssd1306_task pueda
 * mostrarla en el display.
 */
static volatile int g_last_zone = 0;

/* =====================================================================
 *  ESP-NOW
 * ===================================================================== */

/** MAC del mando remoto (¡cámbiala por la tuya!). */
static uint8_t remote_mac[6] = {0x8C, 0xD0, 0xB2, 0xA8, 0x5A, 0x04};

/** Última strategy vista, para loguear sólo cuando cambia. */
static uint8_t last_strategy = 0xFF;

/** Tag de logs del módulo. */
static char TAG[] = "MAIN";

/* =====================================================================
 *  UTILIDADES
 * ===================================================================== */

/**
 * @brief Inicializa el bus I2C a 100 kHz.
 *
 * NOTA: esta función no se usa actualmente (el SSD1306 y el VL6180X
 * inicializan el bus por su cuenta). Se mantiene como referencia.
 */
void init_i2c(void)
{
    i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SDA,
        .scl_io_num       = SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", SDA, SCL);
}

/**
 * @brief Devuelve los milisegundos transcurridos desde un instante dado.
 *
 * @param time  Marca de tiempo obtenida con esp_timer_get_time().
 * @return      Tiempo transcurrido en ms.
 */
int64_t get_elapsed_time_ms(int64_t time)
{
    return (esp_timer_get_time() - time) / 1000;
}

/**
 * @brief Comprueba si un dispositivo I2C responde en una dirección.
 *
 * @param port  Puerto I2C.
 * @param addr  Dirección de 7 bits a probar.
 * @return      ESP_OK si responde, error en caso contrario.
 */
esp_err_t ssd1306_is_connected(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Escanea el bus I2C y muestra por consola los dispositivos presentes.
 *
 * Utilidad de depuración. NO se llama desde app_main.
 */
void i2c_scan(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SDA,
        .scl_io_num       = SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    while (1) {
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (uint8_t i = 3; i < 0x78; i++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);

            esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd,
                                                 10 / portTICK_PERIOD_MS);
            if (i % 16 == 0) printf("\n%.2x:", i);
            if (res == 0)    printf(" %.2x", i);
            else             printf(" --");
            i2c_cmd_link_delete(cmd);
        }
        printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief  Apaga ESP-NOW y el WiFi para liberar el ADC2.
 *
 * En el ESP32-S3, el ADC2 comparte hardware con el WiFi. Aunque
 * pueden coexistir, el WiFi introduce ruido en las lecturas del ADC2.
 * Como el QRE1113 #6 está en ADC2, apagamos el WiFi cuando el robot
 * pasa a STARTED para tener lecturas limpias.
 *
 * Una vez apagado, ESP-NOW deja de funcionar hasta reiniciar el
 * ESP32. Esto es aceptable porque la strategy se fija antes de la
 * ronda (durante POWER_ON).
 *
 * @note Esta función es irreversible sin reiniciar el ESP32.
 */
void stop_espnow(void)
{
    /* 1. Desinicializar ESP-NOW (libera sus recursos internos) */
    esp_err_t err = esp_now_deinit();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_INIT) {
        ESP_LOGW(TAG, "esp_now_deinit: %s", esp_err_to_name(err));
    }

    /* 2. Parar el WiFi (para el radio; mantiene la inicialización) */
    err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "✅ ESP-NOW y WiFi apagados. ADC2 libre de ruido WiFi.");
}

/**
 * @brief  Detiene todas las tareas del robot excepto rc5_task.
 *
 * Se invoca cuando el robot debe pasar a un estado de parada definitiva,
 * normalmente por:
 *   - Detección de vuelco desde bmi160_task (deshabilitado actualmente).
 *   - Transición a ROBOT_STOPPED por el mando RC5.
 *
 * IMPORTANTE: rc5_task NO se borra. Sigue viva para que el receptor
 * del mando continúe operativo (aunque el robot solo pueda volver a
 * arrancar reiniciando el ESP32, según las reglas del WRO).
 */
void stop_robot(void)
{
    ESP_LOGE(TAG, "🛑 STOP ROBOT");

    /* --- Subsistema de LEDs --- */
    // ws2812b_set_led(1, WS2812B_RED);
    // ws2812b_refresh();
    if (h_ws2812b_task) vTaskDelete(h_ws2812b_task);

    /* --- Sensores de distancia VL6180X ---
     * Apagamos los tres sensores vía XSHUT antes de borrar la tarea
     * para que no queden emitiendo luz innecesariamente. */
    gpio_set_level(vl_sensor_2.xshut_pin, 0);
    gpio_set_level(vl_sensor_3.xshut_pin, 0);
    gpio_set_level(vl_sensor_4.xshut_pin, 0);
    if (h_vl6180x_task) vTaskDelete(h_vl6180x_task);

    /* --- Sensores de línea QRE1113 --- */
    if (h_qre1113_task) vTaskDelete(h_qre1113_task);

    /* --- IMU BMI160 (deshabilitado) --- */
    // if (h_bmi160_task) vTaskDelete(h_bmi160_task);

    /* --- Display SSD1306 --- */
    if (h_ssd1306_task) vTaskDelete(h_ssd1306_task);

    /* --- ESP-NOW --- */
    if (h_espnow_rx_task) vTaskDelete(h_espnow_rx_task);

    /* --- Motores TMC2209 (AL FINAL) ---
     * Se borra la última porque stop_robot() suele llamarse desde
     * dentro de tmc2209_task. Borrarla antes detendría la ejecución
     * del resto de esta función. */
    // if (h_tmc2209_task) vTaskDelete(h_tmc2209_task);

    /* rc5_task NO se borra: sigue vivo escuchando el mando. */
}

/* =====================================================================
 *  CALIBRACIÓN GUIADA DE LOS QRE CON EL BOTÓN BOOT
 * ===================================================================== */

/**
 * @brief  Inicializa el botón BOOT como entrada con pull-up.
 *
 * GPIO 0 tiene pull-up interno del chip + externo en la DevKit. Al
 * pulsar el botón, el pin se conecta a GND (nivel bajo).
 */
static void boot_button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/**
 * @brief  Espera a que se pulse el botón BOOT.
 *
 * Ignora pulsaciones largas previas (si ya estaba pulsado, espera a
 * que se suelte). Detecta el flanco de bajada con debounce de 50 ms.
 *
 * @param timeout_ms  Timeout máximo de espera en ms (0 = infinito).
 *
 * @return true si se pulsó, false si se agotó el timeout.
 */
static bool boot_button_wait(uint32_t timeout_ms)
{
    int64_t start = esp_timer_get_time();

    /* Esperar a que suelte el botón si estaba pulsado */
    while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Esperar al flanco de bajada */
    for (;;) {
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            /* Debounce: confirmar que sigue pulsado 50 ms después */
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                /* Esperar a que suelte para no contar doble */
                while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                return true;
            }
        }

        if (timeout_ms > 0) {
            uint32_t elapsed = (esp_timer_get_time() - start) / 1000;
            if (elapsed >= timeout_ms) return false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  Mide el valor promedio y el ruido de un sensor.
 *
 * Toma `samples` lecturas consecutivas y devuelve el promedio y el
 * ruido (max - min). El ruido es un buen indicador de si la lectura
 * es fiable: valores < 50 indican estabilidad; > 150 indican
 * problemas de hardware o luz ambiente.
 *
 * @param[in]  sensor       Puntero al sensor.
 * @param[out] out_average  Puntero donde devolver el promedio.
 * @param[out] out_noise    Puntero donde devolver el ruido (max - min).
 * @param[in]  samples      Número de muestras a tomar.
 *
 * @return true si se tomaron suficientes muestras válidas.
 */
static bool measure_sensor(qre1113_sensor_t *sensor,
                           uint16_t *out_average,
                           uint16_t *out_noise,
                           int samples)
{
    uint32_t sum   = 0;
    uint16_t min_v = 4095;
    uint16_t max_v = 0;
    int      valid = 0;

    for (int i = 0; i < samples; i++) {
        if (qre1113_read_raw(sensor) == ESP_OK) {
            uint16_t v = sensor->raw_value;
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid < samples / 2) {
        return false;
    }

    *out_average = (uint16_t)(sum / valid);
    *out_noise   = max_v - min_v;
    return true;
}

/* =====================================================================
 *  NVS — PERSISTENCIA DE LOS THRESHOLDS DE LOS QRE
 * ===================================================================== */

#define QRE_NVS_NAMESPACE   "qre_cfg"
#define QRE_NVS_KEY_THR1    "thr1"
#define QRE_NVS_KEY_THR6    "thr6"

/**
 * @brief  Inicializa NVS si no lo está (idempotente).
 *
 * Si otra parte del firmware (por ejemplo rc5_module) ya la inicializó,
 * esta llamada devuelve ESP_OK sin hacer nada.
 */
static void qre_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    /* Otros errores (ya inicializado) se ignoran */
}

/**
 * @brief  Guarda los thresholds de los QRE en NVS.
 *
 * @param[in] thr1  Threshold del QRE1.
 * @param[in] thr6  Threshold del QRE6.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
static esp_err_t qre_nvs_save(uint16_t thr1, uint16_t thr6)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(QRE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open RW: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_u16(handle, QRE_NVS_KEY_THR1, thr1);
    nvs_set_u16(handle, QRE_NVS_KEY_THR6, thr6);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "💾 NVS: thr1=%u, thr6=%u", thr1, thr6);
    return ESP_OK;
}

/**
 * @brief  Carga los thresholds de los QRE desde NVS.
 *
 * @param[out] thr1  Puntero donde devolver el threshold del QRE1.
 * @param[out] thr6  Puntero donde devolver el threshold del QRE6.
 *
 * @return ESP_OK si ambos valores existen, error en caso contrario.
 */
static esp_err_t qre_nvs_load(uint16_t *thr1, uint16_t *thr6)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(QRE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_u16(handle, QRE_NVS_KEY_THR1, thr1);
    if (err != ESP_OK) { nvs_close(handle); return err; }

    err = nvs_get_u16(handle, QRE_NVS_KEY_THR6, thr6);
    nvs_close(handle);
    return err;
}

/* =====================================================================
 *  CALIBRACIÓN GUIADA (función reutilizable)
 * ===================================================================== */

/**
 * @brief  Ejecuta la calibración guiada de los QRE (3 pasos + cálculo).
 *
 * Pasos:
 *   1. Robot sobre NEGRO     → pulsar BOOT → mide QRE1 y QRE6.
 *   2. QRE1 sobre BLANCO     → pulsar BOOT → mide QRE1.
 *   3. QRE6 sobre BLANCO     → pulsar BOOT → mide QRE6.
 *   4. Calcula el umbral como punto medio y lo guarda en las
 *      estructuras qre_sensor1 y qre_sensor6.
 *
 * @return true si se completó con valores válidos (sin inversión).
 */
static bool run_guided_calibration(void)
{
    const int SAMPLES = 200;

    uint16_t black1 = 0, white1 = 0;
    uint16_t black6 = 0, white6 = 0;
    uint16_t noise_b1 = 0, noise_w1 = 0;
    uint16_t noise_b6 = 0, noise_w6 = 0;

    /* --- PASO 1: NEGRO --- */
    ssd1306_clear_screen(&oled_dev, false);
    snprintf(display_buffer, sizeof(display_buffer), " QRE CAL       ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " Pon NEGRO     ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " Pulsa BOOT    ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    ESP_LOGW(TAG, "🎯 PASO 1: NEGRO, pulsa BOOT");
    boot_button_wait(0);

    snprintf(display_buffer, sizeof(display_buffer), " Midiendo negro");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    measure_sensor(&qre_sensor1, &black1, &noise_b1, SAMPLES);
    measure_sensor(&qre_sensor6, &black6, &noise_b6, SAMPLES);
    ESP_LOGI(TAG, "NEGRO: Q1=%u (n=%u), Q6=%u (n=%u)",
             black1, noise_b1, black6, noise_b6);

    /* --- PASO 2: BLANCO Q1 --- */
    snprintf(display_buffer, sizeof(display_buffer), " Pon QRE1      ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " en BLANCO     ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " Pulsa BOOT    ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    ESP_LOGW(TAG, "🎯 PASO 2: QRE1 BLANCO, pulsa BOOT");
    boot_button_wait(0);

    snprintf(display_buffer, sizeof(display_buffer), " Midiendo QRE1 ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    measure_sensor(&qre_sensor1, &white1, &noise_w1, SAMPLES);
    ESP_LOGI(TAG, "BLANCO Q1=%u (n=%u)", white1, noise_w1);

    /* --- PASO 3: BLANCO Q6 --- */
    snprintf(display_buffer, sizeof(display_buffer), " Pon QRE6      ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " en BLANCO     ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
    snprintf(display_buffer, sizeof(display_buffer), " Pulsa BOOT    ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    ESP_LOGW(TAG, "🎯 PASO 3: QRE6 BLANCO, pulsa BOOT");
    boot_button_wait(0);

    snprintf(display_buffer, sizeof(display_buffer), " Midiendo QRE6 ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    measure_sensor(&qre_sensor6, &white6, &noise_w6, SAMPLES);
    ESP_LOGI(TAG, "BLANCO Q6=%u (n=%u)", white6, noise_w6);

    /* --- Validar y calcular --- */
    bool error = false;
    if (white1 >= black1 || white6 >= black6) {
        ESP_LOGE(TAG, "⚠️  VALORES INVERTIDOS");
        ESP_LOGE(TAG, "    Q1: negro=%u blanco=%u", black1, white1);
        ESP_LOGE(TAG, "    Q6: negro=%u blanco=%u", black6, white6);
        error = true;
    }

    int delta1 = (int)black1 - (int)white1;
    int delta6 = (int)black6 - (int)white6;
    if (delta1 < 100 || delta6 < 100) {
        ESP_LOGW(TAG, "⚠️  DELTA PEQUEÑO: Q1=%d, Q6=%d", delta1, delta6);
    }

    qre_sensor1.threshold = (black1 + white1) / 2;
    qre_sensor6.threshold = (black6 + white6) / 2;

    ESP_LOGW(TAG, "📏 THRESHOLDS: Q1=%u (d=%d), Q6=%u (d=%d)",
             qre_sensor1.threshold, delta1,
             qre_sensor6.threshold, delta6);

    /* --- Mostrar resultado --- */
    ssd1306_clear_screen(&oled_dev, false);
    if (error) {
        snprintf(display_buffer, sizeof(display_buffer), " ERROR QRE     ");
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " Revisa sensor ");
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    } else {
        snprintf(display_buffer, sizeof(display_buffer), " CALIB OK      ");
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " Q1 %5u-%5u",
                 black1, white1);
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " Q6 %5u-%5u",
                 black6, white6);
        ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " T  %5u-%5u",
                 qre_sensor1.threshold, qre_sensor6.threshold);
        ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);
    }

    vTaskDelay(pdMS_TO_TICKS(2500));
    ssd1306_clear_screen(&oled_dev, false);

    return !error;
}

/**
 * @brief  Recalibración bajo demanda: apaga WiFi, calibra y reinicia.
 *
 * Se llama desde qre1113_task cuando el usuario pulsa BOOT estando en
 * POWER_ON. Como el WiFi está activo en ese momento, hay que apagarlo
 * antes de calibrar para no contaminar las lecturas del ADC2 (QRE6).
 *
 * Apagar ESP-NOW es irreversible sin reiniciar, así que terminamos
 * con esp_restart(). Al volver a arrancar, se cargarán los nuevos
 * thresholds desde NVS.
 */
static void recalibrate_and_reboot(void)
{
    ESP_LOGW(TAG, "🔧 RECALIBRACIÓN SOLICITADA — apagando WiFi");

    /* 1. Parar la tarea del display para que no interfiera */
    if (h_ssd1306_task) {
        vTaskDelete(h_ssd1306_task);
        h_ssd1306_task = NULL;
    }

    /* 2. Apagar ESP-NOW y WiFi para tener el ADC2 limpio */
    stop_espnow();

    /* 3. Esperar a que el radio se apague de verdad */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 4. Calibración guiada */
    bool ok = run_guided_calibration();

    /* 5. Guardar en NVS (aunque haya error, guardamos lo medido) */
    qre_nvs_save(qre_sensor1.threshold, qre_sensor6.threshold);

    /* 6. Mensaje final y reinicio */
    ssd1306_clear_screen(&oled_dev, false);
    snprintf(display_buffer, sizeof(display_buffer),
             ok ? " Reiniciando... " : " ERROR - reinicia");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
    vTaskDelay(pdMS_TO_TICKS(1500));

    esp_restart();
}

/* =====================================================================
 *  TAREAS DE INICIALIZACIÓN
 * ===================================================================== */

/**
 * @brief Inicializa los dos drivers TMC2209.
 *
 * Al terminar, marca boot.tmc2209 y notifica al hilo principal.
 * La UART se usa SOLO para configurar los registros del chip
 * (corriente, microsteps, etc.). El movimiento se hace por STEP/DIR.
 *
 * Además verifica que ambos motores hayan quedado configurados con los
 * mismos microsteps. Si no coinciden, muestra un aviso en el display
 * durante 3 segundos para que el usuario lo vea.
 */
void tmc2209_init(void)
{
    ESP_LOGW(TAG, "TMC2209 INIT");

    /* Una sola UART compartida por ambos drivers (solo config) */
    tmc2209_init_uart(UART_NUM, UART_TX_PIN, UART_RX_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Pines DIR/STEP y RMT para cada motor */
    tmc2209_init_motor(&motor1);
    vTaskDelay(pdMS_TO_TICKS(100));
    tmc2209_init_motor(&motor2);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Configuración completa (corriente, microsteps, etc.) */
    ESP_LOGI(TAG, "Configurando motores...");
    tmc2209_config_init(&motor1);
    vTaskDelay(pdMS_TO_TICKS(200));
    tmc2209_config_init(&motor2);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Forzar 1 microstep explícitamente en ambos */
    tmc2209_set_microsteps(&motor1, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    tmc2209_set_microsteps(&motor2, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ============================================================
     *  VERIFICACIÓN: leer microsteps reales de cada motor
     * ============================================================ */
    uint16_t ms1 = tmc2209_get_microsteps(&motor1);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint16_t ms2 = tmc2209_get_microsteps(&motor2);

    ESP_LOGW(TAG, "Microsteps leídos: motor1=%u, motor2=%u", ms1, ms2);

    /* --- Aviso en display si no coinciden --- */
    if (ms1 != 1 || ms2 != 1) {
        ESP_LOGE(TAG, "⚠️  MICROSTEPS DISTINTOS: m1=%u, m2=%u", ms1, ms2);
        ESP_LOGE(TAG, "    → el motor2 no responde correctamente a UART");

        /* Mostrar aviso en el display */
        ssd1306_clear_screen(&oled_dev, false);

        snprintf(display_buffer, sizeof(display_buffer), "⚠ TMC ERROR ");
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);

        snprintf(display_buffer, sizeof(display_buffer), " Micropasos    ");
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);

        snprintf(display_buffer, sizeof(display_buffer), "1=%u 2=%u",
                 ms1, ms2);
        ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);

        snprintf(display_buffer, sizeof(display_buffer), " Revisa motor2 ");
        ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

        vTaskDelay(pdMS_TO_TICKS(3000));   /* mantener aviso 3 s */

        ssd1306_clear_screen(&oled_dev, false);
    }

    ESP_LOGW(TAG, "TMC2209 INIT OK");

    boot.tmc2209 = true;
    xTaskNotifyGive(boot_handle);
}

/**
 * @brief Tarea de los QRE1113: init + carga de NVS + calibración si hace falta.
 *
 * Flujo:
 *   1. Init del ADC y de los dos sensores.
 *   2. Init del botón BOOT.
 *   3. Intenta cargar thresholds de NVS.
 *      - Si existen → los aplica y no calibra.
 *      - Si no existen → calibración guiada + guardar en NVS.
 *   4. Bucle continuo: lectura + detección de BOOT en POWER_ON para
 *      recalibrar bajo demanda.
 */
void qre1113_task(void *pvParameters)
{
    ESP_LOGW(TAG, "QRE1113 INIT");

    /* 1. Driver ADC */
    if (qre1113_init_adc() != ESP_OK) {
        ESP_LOGE(TAG, "init_adc falló");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    /* 2. Inicialización de los sensores */
    if (qre1113_init_sensor(&qre_sensor1) != ESP_OK ||
        qre1113_init_sensor(&qre_sensor6) != ESP_OK) {
        ESP_LOGE(TAG, "init_sensor falló");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    /* 3. Botón BOOT + NVS */
    boot_button_init();
    qre_nvs_init();

    /* 4. Intentar cargar thresholds de NVS */
    uint16_t thr1 = 0, thr6 = 0;
    esp_err_t nvs_err = qre_nvs_load(&thr1, &thr6);

    if (nvs_err == ESP_OK && thr1 > 0 && thr6 > 0) {
        /* --- Cargar de NVS --- */
        qre_sensor1.threshold = thr1;
        qre_sensor6.threshold = thr6;
        ESP_LOGW(TAG, "✅ Thresholds de NVS: Q1=%u, Q6=%u", thr1, thr6);

        ssd1306_clear_screen(&oled_dev, false);
        snprintf(display_buffer, sizeof(display_buffer), " QRE desde NVS ");
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " Q1=%5u    ", thr1);
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " Q6=%5u    ", thr6);
        ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " BOOT=p/calib  ");
        ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ssd1306_clear_screen(&oled_dev, false);
    } else {
        /* --- Sin NVS → calibración guiada --- */
        ESP_LOGW(TAG, "⚠️  NVS vacío, calibrando...");
        bool ok = run_guided_calibration();
        if (ok) {
            qre_nvs_save(qre_sensor1.threshold, qre_sensor6.threshold);
        }
    }

    boot.qre1113 = true;
    xTaskNotifyGive(boot_handle);

    /* ============================================================
     *  Bucle continuo: lectura + monitor de recalibración
     * ============================================================ */
    for (;;) {
        qre1113_read_filtered(&qre_sensor1);
        qre1113_read_filtered(&qre_sensor6);

        /* --- Recalibración bajo demanda ---
         * Si estamos en POWER_ON y el usuario pulsa BOOT,
         * apagamos WiFi, calibramos y reiniciamos. */
        if (rc5_module_get_state() == ROBOT_POWER_ON &&
            gpio_get_level(BOOT_BUTTON_PIN) == 0) {

            vTaskDelay(pdMS_TO_TICKS(50));   /* debounce */
            if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {

                /* Esperar a que suelte el botón */
                while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                recalibrate_and_reboot();
                /* No retorna */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Tarea de inicialización y lectura continua de los VL6180X.
 *
 * Como todos comparten la misma dirección I2C por defecto (0x29), es
 * necesario encenderlos uno a uno vía XSHUT y reasignarles dirección.
 *
 * NOTA: los sensores leen su distancia correctamente, pero en el
 * montaje actual están viendo el suelo por un problema de ángulo.
 * Su lógica de control está deshabilitada en tmc2209_task.
 */
void vl6180x_task(void *pvParameters)
{
    ESP_LOGW(TAG, "VL6180X INIT");

    /* 1. Bus I2C: intentamos crearlo; si ya existe (por el SSD1306),
     *    nos adjuntamos al existente. */
    ESP_LOGI(TAG, "Intentando inicializar bus I2C...");
    esp_err_t ret = vl6180x_init_i2c(SDA, SCL, 400000, I2C_PORT);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Bus I2C inicializado desde cero");
    } else {
        ESP_LOGW(TAG, "vl6180x_init_i2c falló (%s), reutilizando bus existente",
                 esp_err_to_name(ret));
        ret = vl6180x_attach_i2c(I2C_PORT);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error adjuntando VL6180X al bus existente: %s",
                     esp_err_to_name(ret));
            xTaskNotifyGive(boot_handle);
            vTaskDelete(NULL);
        }
        ESP_LOGI(TAG, "VL6180X adjuntado al bus I2C existente");
    }

    /* 2. Apagamos todos los sensores vía XSHUT */
    ws2812b_set_led(0, WS2812B_GREEN);
    gpio_set_level(vl_sensor_2.xshut_pin, 0);
    gpio_set_level(vl_sensor_3.xshut_pin, 0);
    gpio_set_level(vl_sensor_4.xshut_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 3. Encendemos y configuramos uno a uno */
    if (vl6180x_init(&vl_sensor_2) != ESP_OK) {
        ESP_LOGE(TAG, "Error en inicialización sensor 2");
        ws2812b_set_led(0, WS2812B_RED);
    } else {
        ws2812b_set_led(0, WS2812B_BLACK);
    }

    ws2812b_set_led(1, WS2812B_GREEN);
    gpio_set_level(vl_sensor_3.xshut_pin, 0);
    gpio_set_level(vl_sensor_4.xshut_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (vl6180x_init(&vl_sensor_3) != ESP_OK) {
        ESP_LOGE(TAG, "Error en inicialización sensor 3");
        ws2812b_set_led(1, WS2812B_RED);
    } else {
        ws2812b_set_led(1, WS2812B_BLACK);
    }

    ws2812b_set_led(2, WS2812B_GREEN);
    gpio_set_level(vl_sensor_4.xshut_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (vl6180x_init(&vl_sensor_4) != ESP_OK) {
        ESP_LOGE(TAG, "Error en inicialización sensor 4");
        ws2812b_set_led(2, WS2812B_RED);
    } else {
        ws2812b_set_led(2, WS2812B_BLACK);
    }

    ESP_LOGW(TAG, "VL6180X INIT OK");

    boot.vl6180x = true;
    xTaskNotifyGive(boot_handle);

    /* 4. Lectura continua. */
    uint8_t distance = 0;
    for (;;) {
        if (vl_sensor_2.is_initialized) vl6180x_read_distance(&vl_sensor_2, &distance);
        if (vl_sensor_3.is_initialized) vl6180x_read_distance(&vl_sensor_3, &distance);
        if (vl_sensor_4.is_initialized) vl6180x_read_distance(&vl_sensor_4, &distance);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Inicializa los leds WS2812B y refresca periódicamente.
 *
 * Durante el init hace un "chase" morado de bienvenida.
 */
void ws2812b_task(void *pvParameters)
{
    ESP_LOGW(TAG, "WS2812B INIT");

    if (ws2812b_init(&led) == ESP_OK) {
        /* Efecto de bienvenida: dos pasadas de chase morado */
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t i = 0; i < 3; i++) {
                ws2812b_set_led(i, WS2812B_PURPLE);
                ws2812b_refresh();
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            for (uint8_t i = 0; i < 3; i++) {
                ws2812b_set_led(i, WS2812B_BLACK);
                ws2812b_refresh();
                vTaskDelay(pdMS_TO_TICKS(150));
            }
        }
    } else {
        ESP_LOGE(TAG, "ws2812b_init falló, eliminando task");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    boot.ws2812b = true;
    xTaskNotifyGive(boot_handle);

    /* Refresh periódico (los cambios se aplican al llamar a refresh) */
    for (;;) {
        ws2812b_refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Inicializa el receptor RC5 (TSOP4838) y procesa eventos.
 *
 * El propio módulo RC5 gestiona el LED de estado.
 *
 * IMPORTANTE: esta tarea detecta la transición a ROBOT_STARTED y en
 * ese momento apaga ESP-NOW y el WiFi para liberar el ADC2 del ruido
 * de radio que afecta al QRE1113 #6.
 */
void rc5_task(void *pvParameters)
{
    rc5_module_init(TSOP4838_OUT_PIN, STATUS_LED_PIN);

    boot.rc5 = true;
    xTaskNotifyGive(boot_handle);

    /* Flag local para no llamar a stop_espnow() más de una vez */
    bool espnow_stopped = false;

    for (;;) {

        if (rc5_module_process()) {
            switch (rc5_module_get_state()) {
                case ROBOT_STARTED:
                    ESP_LOGI(TAG, "Robot STARTED");

                    /* ⚠️  Apagar ESP-NOW al pasar a STARTED para liberar
                     * el ADC2 del ruido del WiFi. Se hace una sola vez. */
                    if (!espnow_stopped) {
                        stop_espnow();
                        espnow_stopped = true;
                    }
                    break;

                case ROBOT_POWER_ON:
                    ESP_LOGI(TAG, "Robot POWER_ON");
                    break;

                case ROBOT_STOPPED:
                    ESP_LOGI(TAG, "Robot STOPPED");
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =====================================================================
 *  TAREAS DE EJECUCIÓN CONTINUA
 * ===================================================================== */

/* ---------------------------------------------------------------------
 *  Parámetros del control por STEP/DIR
 * ------------------------------------------------------------------- */

/**
 * Pasos por ráfaga SOLO para el giro de escape (zafado).
 * En la rampa normal se calcula dinámicamente según la velocidad
 * para que la duración sea siempre ~MOTOR_BURST_DURATION_MS.
 */
#define MOTOR_STEPS_PER_BURST       20

/* Velocidades de referencia (pasos/s) */
#define MOTOR_SPEED_IDLE            1000    /* Marcha normal                        */
#define MOTOR_SPEED_PUSH            500    /* Empuje contra rival (dist < 60 mm)   */
#define MOTOR_SPEED_APPROACH        2000   /* Aproximación (60-90 mm)              */
#define MOTOR_SPEED_SLOW_APPROACH   2000   /* Aproximación lenta (90-120 mm)       */
#define MOTOR_SPEED_ESCAPE          1000   /* Giro de zafado                       */
#define MOTOR_SPEED_EMERGENCY       500    /* Estrategia de emergencia             */

/* Corrientes (mA) */
#define MOTOR_CURRENT_IDLE          800
#define MOTOR_CURRENT_PUSH          2000

/* --- Microsteps fijos ---
 * Se aplican UNA SOLA VEZ en tmc2209_init() y no se cambian en
 * caliente. Valores bajos = más par (menos resonancia).
 *   1 = máximo par (puede resonar a baja velocidad)
 *   2 = buen equilibrio
 *   4 = punto dulce habitual
 */
#define MOTOR_MICROSTEPS_FIXED      1

/* --- Parámetros de la rampa NORMAL (cambios de velocidad) --- */
#define MOTOR_ACCEL_HZ_PER_BURST    200   /* +200 Hz por ráfaga */
#define MOTOR_DECEL_HZ_PER_BURST    200   /* -200 Hz por ráfaga */
#define MOTOR_MIN_SPEED             300   /* Velocidad mínima para no calarse */

/* --- Parámetros de la rampa RÁPIDA (cambios de dirección) --- */
#define MOTOR_DECEL_FAST_HZ_PER_BURST   2500  /* -2500 Hz por ráfaga */
#define MOTOR_BURST_FAST_STEPS          15    /* Pasos por ráfaga durante la frenada */

/* --- Duración objetivo de cada ráfaga (ms) --- */
#define MOTOR_BURST_DURATION_MS     20

/* --- Tamaño máximo de una ráfaga (para el buffer estático) --- */
#define MOTOR_MAX_BURST_STEPS       600

/* --- Límites del cálculo automático de pasos por ráfaga --- */
#define MOTOR_BURST_MIN_STEPS       20
#define MOTOR_BURST_MAX_STEPS       600

/* --- Escape por borde de dohyo ---
 * Tiempo mínimo (ms) que dura la maniobra de escape desde que el QRE
 * detecta la línea. */
#define BORDER_ESCAPE_MIN_MS        300

/* --- Debounce de los QRE ---
 * Número de lecturas consecutivas por debajo del umbral necesarias
 * para declarar "borde detectado". Las lecturas llegan cada ~10 ms,
 * así que 3 lecturas ≈ 30 ms de confirmación.
 * Sirve para filtrar falsos positivos por ruido del ADC o reflejos
 * puntuales. */
#define QRE_DEBOUNCE_COUNT          1

/* =====================================================================
 *  HELPER: RÁFAGA DE PASOS SIMULTÁNEA EN AMBOS MOTORES
 * ===================================================================== */

/* --- Buffers estáticos compartidos para las ráfagas --- */
static rmt_symbol_word_t s_burst_buf1[MOTOR_MAX_BURST_STEPS];
static rmt_symbol_word_t s_burst_buf2[MOTOR_MAX_BURST_STEPS];

/**
 * @brief  Genera una ráfaga de pulsos STEP en ambos motores a la vez.
 *
 * @param[in] steps     Número de pasos por motor.
 * @param[in] speed_hz  Frecuencia de los pulsos STEP (Hz).
 */
static void run_motors_burst(int steps, int speed_hz)
{
    if (steps <= 0 || speed_hz <= 0) return;

    if (steps > MOTOR_MAX_BURST_STEPS) steps = MOTOR_MAX_BURST_STEPS;

    int period_us = 1000000 / speed_hz;
    int half      = period_us / 2;
    if (half < 1) half = 1;

    for (int i = 0; i < steps; i++) {
        s_burst_buf1[i].level0    = 1;
        s_burst_buf1[i].duration0 = half;
        s_burst_buf1[i].level1    = 0;
        s_burst_buf1[i].duration1 = half;
        s_burst_buf2[i]           = s_burst_buf1[i];
    }

    rmt_transmit_config_t cfg = {
        .loop_count      = 0,
        .flags.eot_level = 0,
    };
    size_t size = steps * sizeof(rmt_symbol_word_t);

    rmt_transmit(motor1.rmt_chan, motor1.rmt_encoder, s_burst_buf1, size, &cfg);
    rmt_transmit(motor2.rmt_chan, motor2.rmt_encoder, s_burst_buf2, size, &cfg);

    int timeout_ms = (steps * 1000) / speed_hz + 200;
    rmt_tx_done_event_data_t evt;

    xQueueReceive((QueueHandle_t)motor1.rmt_done_queue,
                  &evt, pdMS_TO_TICKS(timeout_ms));
    xQueueReceive((QueueHandle_t)motor2.rmt_done_queue,
                  &evt, pdMS_TO_TICKS(timeout_ms));
}

/* =====================================================================
 *  HELPER: CAMBIO DE DIRECCIÓN CON FRENADA RÁPIDA
 * ===================================================================== */

static void change_direction_smooth(tmc2209_t *motor,
                                    int8_t    new_dir,
                                    int32_t  *current_speed,
                                    int32_t   min_speed)
{
    /* --- 1. Frenada rápida con ráfagas cortas --- */
    while (*current_speed > min_speed) {
        *current_speed -= MOTOR_DECEL_FAST_HZ_PER_BURST;
        if (*current_speed < min_speed) *current_speed = min_speed;

        run_motors_burst(MOTOR_BURST_FAST_STEPS, *current_speed);
    }

    /* --- 2. Parar del todo --- */
    *current_speed = 0;

    /* --- 3. Dejar que el rotor se asiente --- */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* --- 4. Cambiar DIR con el motor quieto --- */
    gpio_set_level(motor->dir_pin, new_dir < 0 ? 1 : 0);

    /* --- 5. Dejar que el driver asimile el nuevo DIR --- */
    vTaskDelay(pdMS_TO_TICKS(5));
}

/* =====================================================================
 *  TAREA PRINCIPAL DE CONTROL DE MOTORES
 * ===================================================================== */

/**
 * @brief  Tarea principal de control de motores.
 *
 * Modo de operación:
 *   - Control por STEP/DIR vía RMT (sin VACTUAL).
 *   - Escape por borde con QRE1113 y debounce configurable.
 *   - VL6180X activo (zonas + laterales).
 *   - BMI160 deshabilitado temporalmente.
 *   - Microsteps fijos desde la inicialización.
 */
void tmc2209_task(void *pvParameters)
{
    /* --- Estado local --- */
    int8_t   want_dir_m1     = 1;   /* 1 = adelante, -1 = atrás */
    int8_t   want_dir_m2     = 1;
    int8_t   last_dir_m1     = 0;   /* 0 inicial fuerza el primer cambio */
    int8_t   last_dir_m2     = 0;
    uint16_t last_current    = 0;

    int32_t  current_speed   = 0;   /* Velocidad actual (pasos/s), 0 al arrancar */

    /* --- Estado de escape por borde (persistente entre iteraciones) --- */
    bool    border_escape_active = false;
    int64_t border_escape_start  = 0;
    int8_t  escape_dir_m1        = 0;
    int8_t  escape_dir_m2        = 0;

    /* --- Contadores de debounce para los QRE --- */
    uint8_t qre1_debounce = 0;
    uint8_t qre6_debounce = 0;

    /* --- Zona actual (para histéresis) ---
     * Es global (g_last_zone) para que ssd1306_task pueda leerla. */

    /* --- Temporizador de estrategia ---
    * Guardamos el instante en que el robot pasó a STARTED para que
    * la estrategia 1 (giro 300 ms) sepa cuándo empezar a contar.
    * `was_started` sirve para detectar la transición STOPPED→STARTED. */
    int64_t strategy_start_time = 0;
    bool    was_started         = false;

    /* --- Inicializar pines de dirección --- */
    gpio_set_level(motor1.dir_pin, 0);
    gpio_set_level(motor2.dir_pin, 0);

    for (;;) {

        /* ============================================================
         *  DEBUG LEDS — Indicadores visuales con colores mixtos
         *
         *    LED 0 → VL2 (azul) + QRE1 (rojo) → magenta si ambos
         *    LED 1 → VL3 (azul)
         *    LED 2 → VL4 (azul) + QRE6 (rojo) → magenta si ambos
         *
         *  Colores:
         *    ⚫ apagado  → nada detectado
         *    🔵 azul     → solo VL detectando
         *    🔴 rojo     → solo QRE detectando
         *    🟣 magenta  → ambos a la vez
         *
         *  Se refresca cada 200 ms. ELIMINAR EN PRODUCCIÓN.
         * ============================================================ */
        static int64_t last_led_log = 0;
        if (esp_timer_get_time() - last_led_log > 200000) {
            last_led_log = esp_timer_get_time();

            /* --- LED 0: VL2 (azul) + QRE1 (rojo) --- */
            bool vl2_det  = (vl_sensor_2.last_distance < 255);
            bool qre1_det = (qre_sensor1.raw_value < qre_sensor1.threshold);

            if (qre1_det && vl2_det) {
                ws2812b_set_led(0, WS2812B_MAGENTA);   /* ambos */
            } else if (qre1_det) {
                ws2812b_set_led(0, WS2812B_RED);       /* solo QRE */
            } else if (vl2_det) {
                ws2812b_set_led(0, WS2812B_BLUE);      /* solo VL */
            } else {
                ws2812b_set_led(0, WS2812B_BLACK);
            }

            /* --- LED 1: solo VL3 --- */
            if (vl_sensor_3.last_distance < 255) {
                ws2812b_set_led(1, WS2812B_BLUE);
            } else {
                ws2812b_set_led(1, WS2812B_BLACK);
            }

            /* --- LED 2: VL4 (azul) + QRE6 (rojo) --- */
            bool vl4_det  = (vl_sensor_4.last_distance < 255);
            bool qre6_det = (qre_sensor6.raw_value < qre_sensor6.threshold);

            if (qre6_det && vl4_det) {
                ws2812b_set_led(2, WS2812B_MAGENTA);   /* ambos */
            } else if (qre6_det) {
                ws2812b_set_led(2, WS2812B_RED);       /* solo QRE */
            } else if (vl4_det) {
                ws2812b_set_led(2, WS2812B_BLUE);      /* solo VL */
            } else {
                ws2812b_set_led(2, WS2812B_BLACK);
            }
        }


        /* ============================================================
         *  1) ROBOT NO INICIADO → esperar sin consumir CPU
         * ============================================================ */
        if (rc5_module_get_state() != ROBOT_STARTED) {
            current_speed = 0;
            g_last_zone   = 0;

            /* Resetear debounce al parar */
            qre1_debounce = 0;
            qre6_debounce = 0;

            was_started   = false;

            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Detectar transición STOPPED → STARTED y guardar el instante
         * para que la estrategia 1 pueda contar sus 300 ms. */
        if (!was_started) {
            was_started         = true;
            strategy_start_time = esp_timer_get_time();
        }

        /* ============================================================
         *  2) ROBOT LEVANTADO → giro de zafado sobre el eje
         * ⚠️  DESHABILITADO (BMI160 comentado).
         * ============================================================ */
#if 0
        if (s_robot_lifted) {
            ESP_LOGW(TAG, "🔄 Giro de escape (hasta que baje el morro)");

            gpio_set_level(motor1.dir_pin, 0);
            gpio_set_level(motor2.dir_pin, 1);

            while (s_robot_lifted &&
                   rc5_module_get_state() == ROBOT_STARTED) {
                run_motors_burst(MOTOR_STEPS_PER_BURST, MOTOR_SPEED_ESCAPE);
            }

            last_dir_m1   = 0;
            last_dir_m2   = 0;
            current_speed = 0;
            g_last_zone   = 0;

            ESP_LOGI(TAG, "✅ Giro de escape terminado");
            continue;
        }
#endif

        /* ============================================================
         *  3) LEER STRATEGY + SENSORES Y DECIDIR OBJETIVO
         * ============================================================ */

        int32_t  want_speed;
        uint16_t want_current;

        switch (g_last_zone) {
            case 3:  /* PUSH */
                want_speed   = MOTOR_SPEED_PUSH;
                want_current = MOTOR_CURRENT_PUSH;
                break;
            case 2:  /* APPROACH */
                want_speed   = MOTOR_SPEED_APPROACH;
                want_current = MOTOR_CURRENT_IDLE;
                break;
            case 1:  /* SLOW */
                want_speed   = MOTOR_SPEED_SLOW_APPROACH;
                want_current = MOTOR_CURRENT_IDLE;
                break;
            default: /* IDLE */
                want_speed   = MOTOR_SPEED_IDLE;
                want_current = MOTOR_CURRENT_IDLE;
                break;
        }

        /* --- 3.1 Strategy (desde ESP-NOW) --- */
        switch (g_remote_data.strategy) {
            case 1:  /* Girar a la izquierda 300 ms, luego recto */
                if (get_elapsed_time_ms(strategy_start_time) < 300) {
                    want_dir_m1 = -1;
                    want_dir_m2 =  1;
                } else {
                    want_dir_m1 = 1;
                    want_dir_m2 = 1;
                }
                break;

                case 2:  /* Emergencia: avanza 1 s, gira 1 s, en bucle */
                {
                    int64_t t = get_elapsed_time_ms(strategy_start_time);
                    int phase = (t / 1000) % 2;   /* 0 = avance, 1 = giro */

                    if (phase == 0) {
                        /* Fase 0: avance recto */
                        want_dir_m1 = 1;
                        want_dir_m2 = 1;
                    } else {
                        /* Fase 1: giro sobre el eje (izquierda) */
                        want_dir_m1 = -1;
                        want_dir_m2 =  1;
                    }

                    /* Velocidad de emergencia en ambas fases */
                    want_speed = MOTOR_SPEED_EMERGENCY;
                }
                break;

            default: /* Avance recto */
                want_dir_m1 = 1;
                want_dir_m2 = 1;
                break;
        }

        /* --- 3.2 Ajuste por distancia frontal (VL6180X #3) ---
         * ⚠️  DESHABILITADO: los VL6180X ven el suelo por ángulo de
         * montaje. Reactivar cuando se solucione físicamente. */
#if 1
        if (vl_sensor_3.is_initialized) {
            uint16_t d = vl_sensor_3.last_distance;

            if (d > 0) {
                switch (g_last_zone) {

                    case 3:  /* PUSH → salir solo si > 75 */
                        if (d >= 75) {
                            if (d < 105) {
                                g_last_zone  = 2;
                                want_speed   = MOTOR_SPEED_APPROACH;
                                want_current = MOTOR_CURRENT_IDLE;
                            } else if (d < 135) {
                                g_last_zone  = 1;
                                want_speed   = MOTOR_SPEED_SLOW_APPROACH;
                                want_current = MOTOR_CURRENT_IDLE;
                            } else {
                                g_last_zone  = 0;
                                want_speed   = MOTOR_SPEED_IDLE;
                                want_current = MOTOR_CURRENT_IDLE;
                            }
                        }
                        break;

                    case 2:  /* APPROACH → subir a PUSH si < 60, bajar si > 105 */
                        if (d < 60) {
                            g_last_zone  = 3;
                            want_speed   = MOTOR_SPEED_PUSH;
                            want_current = MOTOR_CURRENT_PUSH;
                        } else if (d >= 105) {
                            g_last_zone  = 1;
                            want_speed   = MOTOR_SPEED_SLOW_APPROACH;
                            want_current = MOTOR_CURRENT_IDLE;
                        }
                        break;

                    case 1:  /* SLOW → subir si < 90, bajar si > 135 */
                        if (d < 90) {
                            g_last_zone  = 2;
                            want_speed   = MOTOR_SPEED_APPROACH;
                            want_current = MOTOR_CURRENT_IDLE;
                        } else if (d >= 135) {
                            g_last_zone  = 0;
                            want_speed   = MOTOR_SPEED_IDLE;
                            want_current = MOTOR_CURRENT_IDLE;
                        }
                        break;

                    default: /* IDLE → subir según distancia */
                        if (d < 60) {
                            g_last_zone  = 3;
                            want_speed   = MOTOR_SPEED_PUSH;
                            want_current = MOTOR_CURRENT_PUSH;
                        } else if (d < 90) {
                            g_last_zone  = 2;
                            want_speed   = MOTOR_SPEED_APPROACH;
                            want_current = MOTOR_CURRENT_IDLE;
                        } else if (d < 120) {
                            g_last_zone  = 1;
                            want_speed   = MOTOR_SPEED_SLOW_APPROACH;
                            want_current = MOTOR_CURRENT_IDLE;
                        }
                        break;
                }
            }
        }
#endif

        /* --- 3.3 Sensores laterales VL6180X ---
         * ⚠️  DESHABILITADO: ven el suelo por ángulo de montaje. */
#if 1
        if (vl_sensor_2.is_initialized && vl_sensor_2.last_distance > 0 &&
            vl_sensor_2.last_distance < 255) {
            want_dir_m1 = -1;
        }
        if (vl_sensor_4.is_initialized && vl_sensor_4.last_distance > 0 &&
            vl_sensor_4.last_distance < 255) {
            want_dir_m2 = -1;
        }
#endif

        /* --- 3.4 Sensores de línea (escape por borde) con debounce ---
         *
         * Para evitar falsos positivos por ruido del ADC o reflejos
         * puntuales, exigimos que el sensor esté por debajo del umbral
         * durante QRE_DEBOUNCE_COUNT lecturas consecutivas antes de
         * declarar "borde detectado".
         *
         * Lógica de escape:
         *   - QRE1 (izq) + QRE6 (der) en blanco → MARCHA ATRÁS recta
         *   - Solo QRE1 en blanco → GIRO cerrado hacia la derecha
         *   - Solo QRE6 en blanco → GIRO cerrado hacia la izquierda
         *
         * El escape dura BORDER_ESCAPE_MIN_MS aunque el sensor deje de
         * ver la línea, para no oscilar. */
#if 1
        bool qre1_raw_hit = (qre_sensor1.threshold > 0 &&
                             qre_sensor1.raw_value < qre_sensor1.threshold);
        bool qre6_raw_hit = (qre_sensor6.threshold > 0 &&
                             qre_sensor6.raw_value < qre_sensor6.threshold);

        /* Actualizar contadores de debounce */
        if (qre1_raw_hit) {
            if (qre1_debounce < QRE_DEBOUNCE_COUNT) qre1_debounce++;
        } else {
            qre1_debounce = 0;
        }

        if (qre6_raw_hit) {
            if (qre6_debounce < QRE_DEBOUNCE_COUNT) qre6_debounce++;
        } else {
            qre6_debounce = 0;
        }

        /* Solo declaramos "hit" cuando el contador llega al umbral */
        bool qre1_hit   = (qre1_debounce >= QRE_DEBOUNCE_COUNT);
        bool qre6_hit   = (qre6_debounce >= QRE_DEBOUNCE_COUNT);
        bool border_hit = qre1_hit || qre6_hit;
        bool border_escape_just_started = false;

        if (border_hit) {
            /* --- Recalcular dirección de escape en cada iteración ---
             * Así, si empieza girando con un solo QRE y luego el otro
             * también detecta, el robot pasa a marcha atrás sin tener
             * que esperar a que termine el escape. */
            int8_t new_dir_m1, new_dir_m2;

            if (qre1_hit && qre6_hit) {
                /* Ambos en blanco → marcha atrás recta */
                new_dir_m1 = -1;
                new_dir_m2 = -1;
            } else if (qre1_hit) {
                /* Solo izquierda → giro cerrado a la derecha */
                new_dir_m1 = -1;
                new_dir_m2 =  1;
            } else {
                /* Solo derecha → giro cerrado a la izquierda */
                new_dir_m1 =  1;
                new_dir_m2 = -1;
            }

            if (!border_escape_active) {
                /* --- Iniciar un nuevo escape --- */
                escape_dir_m1        = new_dir_m1;
                escape_dir_m2        = new_dir_m2;
                border_escape_active = true;
                border_escape_start  = esp_timer_get_time();
                border_escape_just_started = true;

            } else if (new_dir_m1 != escape_dir_m1 ||
                       new_dir_m2 != escape_dir_m2) {
                /* --- Cambiar dirección durante el escape ---
                 * Ej: empezamos girando (un solo QRE) y ahora los dos
                 * ven blanco → pasamos a marcha atrás. */
                escape_dir_m1        = new_dir_m1;
                escape_dir_m2        = new_dir_m2;

                /* ⚠️ Reseteamos el timer para que el nuevo escape
                 * dure los 300 ms completos desde este momento.
                 * Si no, podría cambiar de dirección cuando ya casi
                 * había terminado el escape anterior. */
                border_escape_start  = esp_timer_get_time();
                border_escape_just_started = true;
            }
        } else {
            /* Sin borde: comprobar si terminamos el escape */
            if (border_escape_active) {
                int64_t elapsed_ms =
                    (esp_timer_get_time() - border_escape_start) / 1000;
                if (elapsed_ms >= BORDER_ESCAPE_MIN_MS) {
                    border_escape_active = false;
                }
            }
        }

        /* Sobrescribir el objetivo si estamos escapando */
        if (border_escape_active) {
            want_dir_m1  = escape_dir_m1;
            want_dir_m2  = escape_dir_m2;
            want_speed   = MOTOR_SPEED_ESCAPE;
            want_current = MOTOR_CURRENT_PUSH;
        }
#endif

        /* ============================================================
         *  4) APLICAR CAMBIOS (corriente y dirección)
         * ============================================================ */

        /* --- 4.1 Corriente --- */
        if (want_current != last_current) {
            tmc2209_set_current(&motor1, want_current);
            vTaskDelay(pdMS_TO_TICKS(50));
            tmc2209_set_current(&motor2, want_current);
            last_current = want_current;
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        /* --- 4.2 Direcciones (con frenada rápida o escape de borde) ---
         *
         *   - Cambio normal: rampa de frenado rápida.
         *   - Escape de borde: corte de pulsos al instante, cambio de
         *     DIR y arranque directo a velocidad de escape. */
        if (want_dir_m1 != last_dir_m1 || want_dir_m2 != last_dir_m2) {

            if (border_escape_just_started) {
                /* Emergencia: cortar pulsos de golpe y resetear */
                current_speed = 0;
                vTaskDelay(pdMS_TO_TICKS(5));
            } else if (current_speed > 0 &&
                       last_dir_m1 != 0 && last_dir_m2 != 0) {
                /* Frenada rápida normal */
                change_direction_smooth(&motor1, want_dir_m1,
                                        &current_speed, MOTOR_MIN_SPEED);
            }

            /* Aplicar los nuevos DIR */
            if (want_dir_m1 != last_dir_m1) {
                gpio_set_level(motor1.dir_pin, want_dir_m1 < 0 ? 1 : 0);
                last_dir_m1 = want_dir_m1;
            }
            if (want_dir_m2 != last_dir_m2) {
                gpio_set_level(motor2.dir_pin, want_dir_m2 < 0 ? 1 : 0);
                last_dir_m2 = want_dir_m2;
            }

            /* Si acabamos de empezar un escape, arrancar directamente
             * a velocidad de escape (sin rampa de aceleración). */
            if (border_escape_just_started) {
                current_speed = MOTOR_SPEED_ESCAPE;
            }
        }

        /* ============================================================
         *  5) RAMPA NORMAL + RÁFAGA
         * ============================================================ */

        if (current_speed == 0) {
            current_speed = MOTOR_MIN_SPEED;
        }

        if (current_speed < want_speed) {
            current_speed += MOTOR_ACCEL_HZ_PER_BURST;
            if (current_speed > want_speed) current_speed = want_speed;
        } else if (current_speed > want_speed) {
            current_speed -= MOTOR_DECEL_HZ_PER_BURST;
            if (current_speed < want_speed) current_speed = want_speed;
        }

        int burst_steps = (current_speed * MOTOR_BURST_DURATION_MS) / 1000;
        if (burst_steps < MOTOR_BURST_MIN_STEPS) burst_steps = MOTOR_BURST_MIN_STEPS;
        if (burst_steps > MOTOR_BURST_MAX_STEPS) burst_steps = MOTOR_BURST_MAX_STEPS;

        if (border_escape_active && current_speed < MOTOR_SPEED_ESCAPE) {
            current_speed = MOTOR_SPEED_ESCAPE;
        }

        run_motors_burst(burst_steps, current_speed);
    }
}

/**
 * @brief Tarea de refresco del display OLED.
 *
 * ⚠️  MODO DEBUG: muestra el VL6180X #2 (last_distance) y la zona
 *     actual del robot respecto al rival.
 *
 *     Filas:
 *       0 → " VL2 <dist> mm"
 *       1 → " ZONA <n>"
 *       2 → (vacía)
 *       3 → estado RC5 (STOPPED / POWER_ON / STARTED)
 */
void ssd1306_task(void *pvParameters)
{
    /* Si el display no se inicializó, la tarea muere */
    if (!boot.ssd1306) vTaskDelete(NULL);

    ssd1306_clear_screen(&oled_dev, false);

    for (;;) {
        /* Fila 0: modo DOHYO */
        snprintf(display_buffer, sizeof(display_buffer), "               ");
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " DOHYO      %d",
                 rc5_module_get_stop_command() / 2);
        ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);

        /* Fila 1: strategy actual */
        snprintf(display_buffer, sizeof(display_buffer), "               ");
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
        snprintf(display_buffer, sizeof(display_buffer), " STRATEGY   %d",
                 g_remote_data.strategy);
        ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);

        /* Fila 2: vacía */
        snprintf(display_buffer, sizeof(display_buffer), "               ");
        ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);

        /* Fila 3: estado RC5 */
        switch (rc5_module_get_state()) {
            case 0: snprintf(display_buffer, sizeof(display_buffer), "    STOPPED    "); break;
            case 1: snprintf(display_buffer, sizeof(display_buffer), "    POWER_ON   "); break;
            case 2: snprintf(display_buffer, sizeof(display_buffer), "    STARTED    "); break;
            default: break;
        }
        ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* =====================================================================
 *  TAREA BMI160 (DESHABILITADA TEMPORALMENTE)
 * =====================================================================
 *
 * Todo el código del BMI160 está comentado con #if 0. Para reactivarlo
 * cuando se solucione el montaje del sensor:
 *   1. Descomentar el include "bmi160.h" al principio del archivo.
 *   2. Descomentar los defines LIFT_* y la variable s_robot_lifted.
 *   3. Cambiar el #if 0 por #if 1 en la función bmi160_task.
 *   4. Descomentar la creación de la tarea en app_main (sección 4.6).
 *   5. Descomentar la sección 2 de tmc2209_task (s_robot_lifted).
 */
#if 0
void bmi160_task(void *pvParameters)
{
    ESP_LOGW(TAG, "BMI160 INIT");

    bmi160_config_t cfg = BMI160_CONFIG_DEFAULT();
    cfg.i2c_port     = I2C_PORT;
    cfg.sda_io       = SDA;
    cfg.scl_io       = SCL;
    cfg.clk_speed_hz = 400000;
    cfg.i2c_addr     = 0x68;
    cfg.accel_range  = BMI160_ACCEL_RANGE_2G;
    cfg.gyro_range   = BMI160_GYRO_RANGE_2000;
    cfg.accel_odr    = BMI160_ODR_100HZ;
    cfg.gyro_odr     = BMI160_ODR_100HZ;
    cfg.filter_coeff = 0.98f;
    cfg.yaw_integration_threshold = 0.3f;

    if (bmi160_attach_i2c(I2C_PORT) != ESP_OK) {
        ESP_LOGE(TAG, "bmi160_attach_i2c falló");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    if (bmi160_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "bmi160_init falló");
        boot.bmi160 = false;
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    if (bmi160_calibrate_gyro(500) != ESP_OK) {
        ESP_LOGW(TAG, "Calibración BMI160 falló, sigo sin bias");
    }

    ESP_LOGW(TAG, "BMI160 INIT OK");
    boot.bmi160 = true;
    xTaskNotifyGive(boot_handle);

    TickType_t last_wake      = xTaskGetTickCount();
    const TickType_t dt_ticks = pdMS_TO_TICKS(10);
    const float DT            = 0.01f;
    int64_t above_since_us    = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, dt_ticks);

        bmi160_raw_t raw;
        if (bmi160_read_raw(&raw) != ESP_OK) continue;

        bmi160_update(DT);

        float ax_g = raw.accel_x / bmi160_get_accel_scale();
        float ay_g = raw.accel_y / bmi160_get_accel_scale();
        float az_g = raw.accel_z / bmi160_get_accel_scale();

        float pitch = atan2f(ax_g, -az_g) * 180.0f / M_PI;

        if (!s_robot_lifted) {
            if (pitch > LIFT_ENTER_DEG) {
                if (above_since_us == 0) {
                    above_since_us = esp_timer_get_time();
                } else if ((esp_timer_get_time() - above_since_us) / 1000
                           >= LIFT_CONFIRM_MS) {
                    s_robot_lifted = true;
                    ESP_LOGW(TAG, "🆘 Morro levantado (pitch=%.1f°)", pitch);
                }
            } else {
                above_since_us = 0;
            }
        } else {
            if (pitch < LIFT_EXIT_DEG) {
                s_robot_lifted = false;
                above_since_us = 0;
                ESP_LOGI(TAG, "✅ Morro en el suelo (pitch=%.1f°)", pitch);
            }
        }

        if (fabsf(pitch) > 90.0f) {
            ESP_LOGE(TAG, "🚨 STOP manual por giro (pitch=%.1f°) → STOP", pitch);
            rc5_module_force_state(ROBOT_STOPPED, rc5_module_get_stop_command());
            s_robot_lifted = false;
            above_since_us = 0;
            vTaskDelete(NULL);
        }
    }
}
#endif  /* BMI160 deshabilitado */

/**
 * @brief Tarea de recepción ESP-NOW.
 *
 * ⚠️  Esta tarea deja de funcionar cuando rc5_task llama a stop_espnow()
 * al pasar a STARTED. A partir de ese momento, g_remote_data queda
 * congelado con la última strategy recibida. Es el comportamiento
 * deseado para que el ADC2 no tenga ruido WiFi durante el combate.
 */
static void espnow_rx_task(void *arg)
{
    ESP_LOGI(TAG, "ESPNOW RX task arrancada");

    for (;;) {
        if (espnow_has_new_data()) {
            espnow_get_remote_data(&g_remote_data);

            if (g_remote_data.strategy != last_strategy) {
                ESP_LOGI(TAG, "Nueva strategy: %u", g_remote_data.strategy);
                last_strategy = g_remote_data.strategy;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =====================================================================
 *  PUNTO DE ENTRADA
 * ===================================================================== */

void app_main(void)
{
    /* ---------- 0. Espera de estabilización ---------- */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ---------- 1. Display OLED ---------- */
    i2c_master_init(&oled_dev, SDA, SCL, -1);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (ssd1306_is_connected(I2C_PORT, 0x3C) == ESP_OK) {
        ssd1306_init(&oled_dev, 128, 32);
        vTaskDelay(pdMS_TO_TICKS(100));
        ssd1306_clear_screen(&oled_dev, false);
        ssd1306_contrast(&oled_dev, 0xff);
        ssd1306_display_text(&oled_dev, 0, "    WIKISUMO    ", 16, false);
        ssd1306_display_text(&oled_dev, 3, "   STARTING...   ", 16, false);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ssd1306_clear_screen(&oled_dev, false);
        boot.ssd1306 = true;
    }

    /* ---------- 2. Selector de modo ---------- */
    ESP_LOGW(TAG, "SELECT MODE = MINISUMO");

    /* ---------- 3. Preparar sincronización de boot ---------- */
    boot_handle = xTaskGetCurrentTaskHandle();

    /* ---------- 4. Inicialización secuencial de subsistemas ---------- */

    /* 4.1 TMC2209 */
    snprintf(display_buffer, sizeof(display_buffer), " TMC2209  INIT ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
    tmc2209_init();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.tmc2209
        ? snprintf(display_buffer, sizeof(display_buffer), " TMC2209  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " TMC2209  FAIL ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);

    /* 4.2 RC5 */
    xTaskCreatePinnedToCore(rc5_task, "rc5_task", 4096, NULL, 20, NULL, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    rc5_module_force_state(ROBOT_POWER_ON, rc5_module_get_stop_command());
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 4.3 WS2812B */
    snprintf(display_buffer, sizeof(display_buffer), " WS2812B  INIT ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    xTaskCreatePinnedToCore(ws2812b_task, "ws2812b_task", 4096, NULL, 5,
                            &h_ws2812b_task, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.ws2812b
        ? snprintf(display_buffer, sizeof(display_buffer), " WS2812B  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " WS2812B  FAIL ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);

    /* 4.4 VL6180X (core 1) */
    snprintf(display_buffer, sizeof(display_buffer), " VL6180X  INIT ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
    xTaskCreatePinnedToCore(vl6180x_task, "vl6180x_task", 4096, NULL, 9,
                            &h_vl6180x_task, 1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.vl6180x
        ? snprintf(display_buffer, sizeof(display_buffer), " VL6180X  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " VL6180X  FAIL ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);

    /* 4.5 QRE1113 (con calibración guiada por botón BOOT + NVS) */
    snprintf(display_buffer, sizeof(display_buffer), " QRE1113  INIT ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);
    xTaskCreatePinnedToCore(qre1113_task, "qre1113_task", 4096, NULL, 8,
                            &h_qre1113_task, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.qre1113
        ? snprintf(display_buffer, sizeof(display_buffer), " QRE1113  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " QRE1113  FAIL ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    /* 4.6 BMI160 (DESHABILITADO TEMPORALMENTE)
     *
     * Para reactivar:
     *   1. Descomentar el include "bmi160.h".
     *   2. Descomentar los defines LIFT_* y la variable s_robot_lifted.
     *   3. Cambiar el #if 0 de bmi160_task por #if 1.
     *   4. Descomentar este bloque.
     *   5. Descomentar la sección 2 de tmc2209_task. */
#if 0
    snprintf(display_buffer, sizeof(display_buffer), " BMI160   INIT ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
    xTaskCreatePinnedToCore(bmi160_task, "bmi160_task", 4096, NULL, 7,
                            &h_bmi160_task, 1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.bmi160
        ? snprintf(display_buffer, sizeof(display_buffer), " BMI160   OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " BMI160   FAIL ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
#endif

    /* ---------- 5. Tareas ---------- */
    xTaskCreatePinnedToCore(tmc2209_task, "tmc2209_task", 4096, NULL, 10,
                            &h_tmc2209_task, 0);
    xTaskCreatePinnedToCore(ssd1306_task, "ssd1306_task", 4096, NULL, 6,
                            &h_ssd1306_task, 0);

    /* ---------- 6. ESP-NOW ----------
     * Se inicializa aquí para que esté disponible en POWER_ON (el robot
     * puede cambiar de strategy antes de la ronda). Al pasar a STARTED,
     * rc5_task llama a stop_espnow() para liberar el ADC2 del ruido WiFi. */
    esp_err_t err = espnow_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init falló: %s (el robot sigue con RC5)",
                 esp_err_to_name(err));
    } else {
        espnow_set_peer_mac(remote_mac);

        uint8_t mi_mac[6];
        espnow_get_local_mac(mi_mac);
        ESP_LOGI(TAG, "MAC local: %02X:%02X:%02X:%02X:%02X:%02X",
                 mi_mac[0], mi_mac[1], mi_mac[2],
                 mi_mac[3], mi_mac[4], mi_mac[5]);

        xTaskCreatePinnedToCore(espnow_rx_task, "espnow_rx", 4096, NULL, 5,
                                &h_espnow_rx_task, 0);
    }

    /* ---------- 7. app_main ya ha terminado ---------- */
    vTaskDelete(NULL);
}