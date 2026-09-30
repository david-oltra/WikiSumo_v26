#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *              DEFAULTS COMPILE-TIME (override con -D)
 * ============================================================ */

#ifndef BMI160_CFG_ACCEL_RANGE
#define BMI160_CFG_ACCEL_RANGE   BMI160_ACCEL_RANGE_2G
#endif

#ifndef BMI160_CFG_GYRO_RANGE
#define BMI160_CFG_GYRO_RANGE    BMI160_GYRO_RANGE_2000
#endif

#ifndef BMI160_CFG_ACCEL_ODR
#define BMI160_CFG_ACCEL_ODR     BMI160_ODR_100HZ
#endif

#ifndef BMI160_CFG_GYRO_ODR
#define BMI160_CFG_GYRO_ODR      BMI160_ODR_100HZ
#endif

#ifndef BMI160_CFG_FILTER_COEFF
#define BMI160_CFG_FILTER_COEFF  0.98f
#endif

#ifndef BMI160_CFG_YAW_THRESHOLD
#define BMI160_CFG_YAW_THRESHOLD 0.3f
#endif

/* ============================================================
 *              ENUMS DE CONFIGURACIÓN
 * ============================================================ */

/* Los valores coinciden con los bits reales del registro,
 * así se pueden escribir directamente al sensor. */
typedef enum {
    BMI160_ACCEL_RANGE_2G  = 0x03,
    BMI160_ACCEL_RANGE_4G  = 0x05,
    BMI160_ACCEL_RANGE_8G  = 0x08,
    BMI160_ACCEL_RANGE_16G = 0x0C,
} bmi160_accel_range_t;

typedef enum {
    BMI160_GYRO_RANGE_2000 = 0x00,
    BMI160_GYRO_RANGE_1000 = 0x01,
    BMI160_GYRO_RANGE_500  = 0x02,
    BMI160_GYRO_RANGE_250  = 0x03,
    BMI160_GYRO_RANGE_125  = 0x04,
} bmi160_gyro_range_t;

typedef enum {
    BMI160_ODR_25HZ   = 0x06,
    BMI160_ODR_50HZ   = 0x07,
    BMI160_ODR_100HZ  = 0x08,
    BMI160_ODR_200HZ  = 0x09,
    BMI160_ODR_400HZ  = 0x0A,
    BMI160_ODR_800HZ  = 0x0B,
    BMI160_ODR_1600HZ = 0x0C,
} bmi160_odr_t;

/* ============================================================
 *              ESTRUCTURAS
 * ============================================================ */

typedef struct {
    /* --- Bus I2C --- */
    i2c_port_t    i2c_port;
    uint8_t       sda_io;
    uint8_t       scl_io;
    uint32_t      clk_speed_hz;
    uint8_t       i2c_addr;          /* 0x68 (SDO→GND) o 0x69 (SDO→VCC) */
    gpio_pullup_t sda_pullup;
    gpio_pullup_t scl_pullup;

    /* --- Rangos y frecuencias --- */
    bmi160_accel_range_t accel_range;
    bmi160_gyro_range_t  gyro_range;
    bmi160_odr_t         accel_odr;
    bmi160_odr_t         gyro_odr;

    /* --- Filtro complementario --- */
    float filter_coeff;             /* 0..1 (típico 0.98) */
    float yaw_integration_threshold;/* °/s */
} bmi160_config_t;

#define BMI160_CONFIG_DEFAULT() {                     \
    .i2c_port        = I2C_NUM_0,                     \
    .sda_io          = 3,                             \
    .scl_io          = 2,                             \
    .clk_speed_hz    = 400000,                        \
    .i2c_addr        = 0x68,                          \
    .sda_pullup      = GPIO_PULLUP_ENABLE,            \
    .scl_pullup      = GPIO_PULLUP_ENABLE,            \
    .accel_range     = BMI160_CFG_ACCEL_RANGE,        \
    .gyro_range      = BMI160_CFG_GYRO_RANGE,         \
    .accel_odr       = BMI160_CFG_ACCEL_ODR,          \
    .gyro_odr        = BMI160_CFG_GYRO_ODR,           \
    .filter_coeff    = BMI160_CFG_FILTER_COEFF,       \
    .yaw_integration_threshold = BMI160_CFG_YAW_THRESHOLD, \
}

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
} bmi160_raw_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} bmi160_angles_t;

/* ============================================================
 *              API PÚBLICA
 * ============================================================ */

/**
 * @brief Inicializa el BMI160.
 *
 * Si el bus I2C ya estaba instalado por otro componente (SSD1306,
 * VL6180X, etc.), reutiliza el bus existente sin reinstalar el driver.
 *
 * @param config  Puntero a la configuración. Se copia internamente.
 * @return ESP_OK si éxito, código de error en caso contrario.
 */
esp_err_t bmi160_init(const bmi160_config_t *config);

/**
 * @brief Adjunta el BMI160 a un bus I2C ya inicializado por otro driver.
 *
 * Útil cuando se quiere ser explícito sobre el uso compartido del bus.
 * Debe llamarse ANTES de bmi160_init() si el bus ya está en uso.
 * Después de llamar a esta función, bmi160_init() no intentará
 * reinstalar el driver I2C.
 *
 * @param port  Puerto I2C ya inicializado (I2C_NUM_0 o I2C_NUM_1).
 * @return ESP_OK si éxito.
 */
esp_err_t bmi160_attach_i2c(i2c_port_t port);

/**
 * @brief Escanea el bus I2C y muestra por log las direcciones encontradas.
 *        Utilidad de diagnóstico.
 */
void bmi160_i2c_scan(void);

/**
 * @brief Calibra el sesgo del giróscopo. El sensor debe estar QUIETO.
 *
 * @param num_samples  Número de muestras a promediar (recomendado: 500).
 * @return ESP_OK si éxito, código de error en caso contrario.
 */
esp_err_t bmi160_calibrate_gyro(int num_samples);

/**
 * @brief Lee los 6 ejes (acelerómetro + giróscopo) en crudo.
 *
 * @param out  Puntero a la estructura donde se guardan los datos.
 * @return ESP_OK si éxito.
 */
esp_err_t bmi160_read_raw(bmi160_raw_t *out);

/**
 * @brief Realiza una lectura y aplica el filtro complementario.
 *        Debe llamarse periódicamente con el dt real transcurrido.
 *
 * @param dt  Tiempo desde la última llamada, en segundos (ej: 0.01 para 100 Hz).
 * @return ESP_OK si éxito.
 */
esp_err_t bmi160_update(float dt);

/* ------------------------------------------------------------
 * Getters de ángulos (filtrados)
 * ------------------------------------------------------------ */

bmi160_angles_t bmi160_get_angles(void);
float bmi160_get_pitch(void);
float bmi160_get_roll(void);
float bmi160_get_yaw(void);

/* ------------------------------------------------------------
 * Reset manual del Yaw
 * ------------------------------------------------------------ */

/**
 * @brief Resetea el ángulo Yaw a 0°.
 *        Puede llamarse desde cualquier tarea.
 */
void bmi160_reset_yaw(void);

/**
 * @brief Resetea el Yaw a un valor específico (0-360°).
 */
void bmi160_reset_yaw_to(float value);

/* ------------------------------------------------------------
 * Configuración en RUNTIME
 * ------------------------------------------------------------ */

/* Cambia el rango del acelerómetro. Actualiza el factor de escala. */
esp_err_t bmi160_set_accel_range(bmi160_accel_range_t range);

/* Cambia el rango del giróscopo. Actualiza el factor de escala. */
esp_err_t bmi160_set_gyro_range(bmi160_gyro_range_t range);

/* Cambia la ODR del acelerómetro/giróscopo (reenvía a ACC_CONF/GYR_CONF). */
esp_err_t bmi160_set_accel_odr(bmi160_odr_t odr);
esp_err_t bmi160_set_gyro_odr(bmi160_odr_t odr);

/* Ajusta el filtro complementario y el umbral del Yaw en caliente. */
void bmi160_set_filter_coeff(float coeff);
void bmi160_set_yaw_threshold(float deg_per_s);

/* Consultas de configuración actual. */
bmi160_accel_range_t bmi160_get_accel_range(void);
bmi160_gyro_range_t  bmi160_get_gyro_range(void);
bmi160_odr_t         bmi160_get_accel_odr(void);
bmi160_odr_t         bmi160_get_gyro_odr(void);
float bmi160_get_accel_scale(void);   /* LSB/g */
float bmi160_get_gyro_scale(void);    /* LSB/°/s */

#ifdef __cplusplus
}
#endif