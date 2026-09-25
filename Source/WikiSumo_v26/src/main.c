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
 *   - 2x TMC2209 (drivers de motor paso a paso) por UART
 *   - 5x VL6180X (sensores ToF) por I2C con XSHUT
 *   - 6x QRE1113 (sensores de línea reflectivos) por ADC
 *   - Tira WS2812B (3 LEDs) por RMT
 *   - Display SSD1306 128x32 por I2C
 *   - Receptor RC5 (TSOP4838) para start/stop
 *   - ESP-NOW para recibir la "strategy" desde el mando
 *   - BMI160 (pendiente)
 *
 * Flujo de arranque:
 *   1. Se inicializa el display y el bus I2C maestro.
 *   2. Se lanzan tareas de inicialización (TMC2209, RC5, WS2812B,
 *      VL6180X, QRE1113) de forma secuencial; cada una notifica al
 *      hilo principal al terminar (mecanismo del "boot_handle").
 *   3. Se lanzan las tareas de ejecución continua (tmc2209_task,
 *      ssd1306_task, espnow_rx_task).
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

/* FreeRTOS */
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>

/* ESP-IDF */
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/adc.h>
#include <driver/uart.h>
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

/* --- UART compartida por los dos TMC2209 --- */
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

/* --- Tira de LEDs WS2812B --- */
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

/** Banderas que indican si un subsistema terminó su init correctamente. */
typedef struct {
    uint8_t rc5;
    uint8_t ws2812b;
    uint8_t vl6180x;
    uint8_t qre1113;
    uint8_t tmc2209;
    uint8_t ssd1306;
} boot_t;
boot_t boot;

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

/* =====================================================================
 *  TAREAS DE INICIALIZACIÓN
 * ===================================================================== */

/**
 * @brief Inicializa los dos drivers TMC2209 por UART.
 *
 * Al terminar, marca boot.tmc2209 y notifica al hilo principal.
 */
void tmc2209_init(void)
{
    ESP_LOGW(TAG, "TMC2209 INIT");

    /* Una sola UART compartida por ambos drivers */
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
    vTaskDelay(pdMS_TO_TICKS(100));
    tmc2209_config_init(&motor2);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGW(TAG, "TMC2209 INIT OK");

    boot.tmc2209 = true;
    xTaskNotifyGive(boot_handle);
}

/**
 * @brief Tarea de inicialización del ADC y calibración de los QRE1113.
 *
 * Sólo se calibran inicialmente los sensores 1 y 6 (los usados para
 * detección de borde). El resto pueden calibrarse más adelante.
 */
void qre1113_task(void *pvParameters)
{
    ESP_LOGW(TAG, "QRE1113 INIT");

    /* 1. Driver ADC (una sola vez para todos los canales) */
    if (qre1113_init_adc() != ESP_OK) {
        ESP_LOGE(TAG, "init_adc falló, eliminando task");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    /* 2. Inicialización de los sensores concretos que usamos */
    if (qre1113_init_sensor(&qre_sensor1) != ESP_OK) {
        ESP_LOGE(TAG, "qre_sensor1 falló, eliminando task");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }
    if (qre1113_init_sensor(&qre_sensor6) != ESP_OK) {
        ESP_LOGE(TAG, "qre_sensor6 falló, eliminando task");
        xTaskNotifyGive(boot_handle);
        vTaskDelete(NULL);
    }

    /* 3. Calibración (LED azul mientras calibra, luego verde/rojo) */
    ws2812b_set_led(0, WS2812B_BLUE);
    qre1113_calibrate(&qre_sensor1)
        ? ws2812b_set_led(0, WS2812B_BLACK)
        : ws2812b_set_led(0, WS2812B_RED);
    vTaskDelay(pdMS_TO_TICKS(10));

    ws2812b_set_led(2, WS2812B_BLUE);
    qre1113_calibrate(&qre_sensor6)
        ? ws2812b_set_led(2, WS2812B_BLACK)
        : ws2812b_set_led(2, WS2812B_RED);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "📏 Sensor %d: %4d\tSensor %d: %4d",
             qre_sensor1.id, qre_sensor1.threshold,
             qre_sensor6.id, qre_sensor6.threshold);

    boot.qre1113 = true;
    xTaskNotifyGive(boot_handle);

    /* 4. Bucle de lectura continua */
    for (;;) {
        qre1113_read_raw(&qre_sensor1);
        qre1113_read_raw(&qre_sensor6);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Tarea de inicialización y lectura continua de los VL6180X.
 *
 * Como todos comparten la misma dirección I2C por defecto (0x29), es
 * necesario encenderlos uno a uno vía XSHUT y reasignarles dirección.
 *
 * Se inicializan los sensores 2, 3 y 4 (frontal e inferiores laterales).
 * Los sensores 1 y 5 están declarados pero no se inicializan aquí.
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

    /* 4. Lectura continua.
     *    NOTA: usamos la misma variable `distance` para los tres sensores;
     *    funciona porque cada `read_distance` sobreescribe el valor, pero
     *    puede ser confuso. Considera usar variables separadas si quieres
     *    leer los tres "a la vez". */
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
 */
void rc5_task(void *pvParameters)
{
    rc5_module_init(TSOP4838_OUT_PIN, STATUS_LED_PIN);

    boot.rc5 = true;
    xTaskNotifyGive(boot_handle);

    for (;;) {
        if (rc5_module_process()) {
            switch (rc5_module_get_state()) {
                case ROBOT_STARTED:  ESP_LOGI(TAG, "Robot STARTED");   break;
                case ROBOT_POWER_ON: ESP_LOGI(TAG, "Robot POWER_ON");  break;
                case ROBOT_STOPPED:  ESP_LOGI(TAG, "Robot STOPPED");   break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =====================================================================
 *  TAREAS DE EJECUCIÓN CONTINUA
 * ===================================================================== */

/**
 * @brief Tarea principal de control de motores.
 *
 * Mientras el robot está STARTED:
 *   - Aplica la strategy activa (recibida por ESP-NOW).
 *   - Modifica velocidad/microsteps/corriente según los VL6180X.
 *   - Corrige rumbo si los QRE1113 detectan borde.
 *
 * Mientras está STOPPED detiene los motores y espera.
 */
void tmc2209_task(void *pvParameters)
{
    uint8_t  motor1_reverse      = 0;
    uint8_t  motor2_reverse      = 0;
    uint8_t  motor1_last_reverse = 0;
    uint8_t  motor2_last_reverse = 0;
    uint16_t microsteps          = 0;
    uint16_t last_microsteps     = 0;
    uint16_t speed_hz            = 1000;
    uint16_t current             = 800;
    uint16_t last_current        = 800;
    int64_t  last_time;

    for (;;) {
        last_time = esp_timer_get_time();

        /* ---------------- Robot en marcha ---------------- */
        while (rc5_module_get_state() == ROBOT_STARTED) {
            switch (g_remote_data.strategy) {
                case 1:  /* Girar a la izquierda 2 s */
                    if (get_elapsed_time_ms(last_time) < 2000) {
                        motor1_reverse = 1;
                    }
                    break;

                case 2:  /* Emergencia: alterna sentido cada segundo */
                    microsteps = 128;
                    current    = 1200;
                    if (get_elapsed_time_ms(last_time) > 1000) {
                        last_time = esp_timer_get_time();
                        motor1_reverse = !motor1_reverse;
                    }
                    break;

                default:
                    motor1_reverse = 0;
                    motor2_reverse = 0;
                    break;
            }

            /* Ajuste por distancia (sensor frontal) */
            if (vl_sensor_3.is_initialized && vl_sensor_3.last_distance < 60) {
                microsteps     = 256;
                motor1_reverse = 0;
                motor2_reverse = 0;
                speed_hz       = 12000;
                current        = 2000;
            } else if (vl_sensor_3.is_initialized && vl_sensor_3.last_distance < 90) {
                microsteps     = 64;
                motor1_reverse = 0;
                motor2_reverse = 0;
                speed_hz       = 8000;
                current        = 800;
            } else if (vl_sensor_3.is_initialized && vl_sensor_3.last_distance < 120) {
                microsteps     = 16;
                motor1_reverse = 0;
                motor2_reverse = 0;
                speed_hz       = 7000;
                current        = 800;
            } else if (vl_sensor_2.is_initialized && vl_sensor_2.last_distance < 255) {
                motor1_reverse = 1;
            } else if (vl_sensor_4.is_initialized && vl_sensor_4.last_distance < 255) {
                motor2_reverse = 1;
            } else {
                microsteps = 1;
                speed_hz   = 5000;
                current    = 800;
            }

            /* Detección de borde con QRE1113: giro de escape */
            if (qre_sensor1.raw_value < qre_sensor1.threshold) {
                motor2_reverse = 1;
                tmc2209_set_direction(&motor2, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            if (qre_sensor6.raw_value < qre_sensor6.threshold) {
                motor1_reverse = 1;
                tmc2209_set_direction(&motor1, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            /* Aplicar cambios de sentido con rampa */
            if (motor1_reverse != motor1_last_reverse) {
                tmc2209_stop(&motor1);
                vTaskDelay(pdMS_TO_TICKS(10));
                motor1_reverse
                    ? tmc2209_ramp_velocity(&motor1, -speed_hz, 50, 10)
                    : tmc2209_ramp_velocity(&motor1,  speed_hz, 50, 10);
                motor1_last_reverse = motor1_reverse;
            }
            if (motor2_reverse != motor2_last_reverse) {
                tmc2209_set_direction(&motor2, motor2_reverse);
                motor2_last_reverse = motor2_reverse;
            }

            /* Aplicar cambios de microsteps (con parada preventiva) */
            if (microsteps != last_microsteps) {
                tmc2209_set_microsteps(&motor1, microsteps);
                tmc2209_set_microsteps(&motor2, microsteps);
                last_microsteps = microsteps;

                if (motor1.current_velocity + speed_hz > 4000) {
                    tmc2209_stop(&motor1);
                    tmc2209_stop(&motor2);
                }
                tmc2209_ramp_velocity(&motor1, speed_hz, 100, 10);
                tmc2209_ramp_velocity(&motor2, speed_hz, 100, 10);
            }

            /* Aplicar cambios de corriente */
            if (current != last_current) {
                tmc2209_set_current(&motor1, current);
                vTaskDelay(pdMS_TO_TICKS(10));
                tmc2209_set_current(&motor2, current);
                vTaskDelay(pdMS_TO_TICKS(10));
                last_current = current;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* ---------------- Robot detenido ---------------- */
        if (rc5_module_get_state() == ROBOT_STOPPED) {
            tmc2209_stop(&motor1);
            tmc2209_stop(&motor2);
            /* ⚠️ Antes había un for(;;) aquí que nunca salía.
             *    Se sustituye por una espera que sí permite volver
             *    a comprobar el estado RC5. */
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Tarea de refresco del display OLED.
 *
 * Muestra: modo (DOHYO), strategy activa y estado RC5.
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

        /* Fila 3: estado RC5 */
        switch (rc5_module_get_state()) {
            case 0: snprintf(display_buffer, sizeof(display_buffer), "    STOPPED    "); break;
            case 1: snprintf(display_buffer, sizeof(display_buffer), "    POWER_ON   "); break;
            case 2: snprintf(display_buffer, sizeof(display_buffer), "    STARTED    "); break;
            default: break;
        }
        ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Tarea de recepción ESP-NOW.
 *
 * La librería actualiza `g_remote_data` por callback; aquí sólo
 * reaccionamos a cambios (por ejemplo, para loguear).
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

            /* Opcional: responder al mando con el estado del robot
             * uint8_t status = (rc5_module_get_state() == ROBOT_STARTED) ? 1 : 0;
             * espnow_send_robot_status(status);
             */
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =====================================================================
 *  PUNTO DE ENTRADA
 * ===================================================================== */

void app_main(void)
{
    /* Pequeña espera para que se estabilicen las alimentaciones */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ---------- 1. Display OLED (primero para poder ver logs en pantalla) ---------- */
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

    /* ---------- 2. Selector de modo (sólo MINISUMO activo por ahora) ----------
     *
     * El código original leía el modo desde ADC2_CHANNEL_2. De momento
     * forzamos MINISUMO. Cuando termines los otros modos, descomenta
     * el bloque de adc2_get_raw y sus ramas.
     */
    ESP_LOGW(TAG, "SELECT MODE = MINISUMO");

    boot_handle = xTaskGetCurrentTaskHandle();

    /* ---------- 3. Inicialización secuencial de subsistemas ----------
     *  Cada subsistema lanza su task, espera a que termine el init
     *  (xTaskNotifyGive / ulTaskNotifyTake) y actualiza el display. */

    /* 3.1 TMC2209 */
    snprintf(display_buffer, sizeof(display_buffer), " TMC2209  INIT ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);
    tmc2209_init();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.tmc2209
        ? snprintf(display_buffer, sizeof(display_buffer), " TMC2209  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " TMC2209  FAIL ");
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);

    /* 3.2 RC5 */
    xTaskCreatePinnedToCore(rc5_task, "rc5_task", 4096, NULL, 20, NULL, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 3.3 WS2812B */
    snprintf(display_buffer, sizeof(display_buffer), " WS2812B  INIT ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);
    xTaskCreatePinnedToCore(ws2812b_task, "ws2812b_task", 4096, NULL, 7, NULL, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.ws2812b
        ? snprintf(display_buffer, sizeof(display_buffer), " WS2812B  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " WS2812B  FAIL ");
    ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);

    /* 3.4 VL6180X */
    snprintf(display_buffer, sizeof(display_buffer), " VL6180X  INIT ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);
    xTaskCreatePinnedToCore(vl6180x_task, "vl6180x_task", 4096, NULL, 9, NULL, 1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.vl6180x
        ? snprintf(display_buffer, sizeof(display_buffer), " VL6180X  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " VL6180X  FAIL ");
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);

    /* 3.5 QRE1113 */
    snprintf(display_buffer, sizeof(display_buffer), " QRE1113  INIT ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);
    xTaskCreatePinnedToCore(qre1113_task, "qre1113_task", 4096, NULL, 8, NULL, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    boot.qre1113
        ? snprintf(display_buffer, sizeof(display_buffer), " QRE1113  OK   ")
        : snprintf(display_buffer, sizeof(display_buffer), " QRE1113  FAIL ");
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);

    /* ---------- 4. Tareas de ejecución continua ---------- */
    xTaskCreatePinnedToCore(tmc2209_task, "tmc2209_task", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(ssd1306_task, "ssd1306_task", 4096, NULL,  6, NULL, 0);

    /* ---------- 5. ESP-NOW (sólo strategy; RC5 sigue mandando start/stop) ---------- */
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

        xTaskCreatePinnedToCore(espnow_rx_task, "espnow_rx",
                                4096, NULL, 5, NULL, 0);
    }

    /* La tarea principal ya ha hecho su trabajo */
    vTaskDelete(NULL);
}