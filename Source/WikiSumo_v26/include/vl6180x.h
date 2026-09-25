/**
 * @file    vl6180x.h
 * @brief   Librería para el sensor de distancia VL6180X (ST ToF).
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Proporciona funciones para controlar el sensor de distancia
 *   VL6180X a través de I2C en ESP32-S3.
 *
 *   Cada sensor físico se maneja mediante una instancia de vl6180x_t
 *   que contiene su pin XSHUT, dirección I2C asignada y última
 *   medición. El pin XSHUT permite apagar/encender cada sensor
 *   individualmente para reasignarle una dirección I2C distinta,
 *   evitando conflictos cuando hay varios sensores en el mismo bus.
 *
 *   Modos de configuración disponibles:
 *     - VL6180X_CONFIG_MODE_MAX_RANGE     : prioriza rango (~15-25 cm)
 *     - VL6180X_CONFIG_MODE_MIN_DISTANCE  : prioriza distancias cortas (0-20 mm)
 *
 * @note Compatible con ESP-IDF 5.x.
 * @note El sensor utiliza comunicación I2C a 100 kHz (aunque soporta 400 kHz).
 */

#ifndef VL6180X_H
#define VL6180X_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  CONSTANTES PÚBLICAS
 * ===================================================================== */

/**
 * @def    VL6180X_DEFAULT_ADDR
 * @brief  Dirección I2C por defecto del sensor (fábrica).
 */
#define VL6180X_DEFAULT_ADDR               0x29

/**
 * @def    VL6180X_I2C_TIMEOUT_MS
 * @brief  Timeout por defecto para operaciones I2C (ms).
 */
#define VL6180X_I2C_TIMEOUT_MS             100

/**
 * @def    VL6180X_MAX_CONVERGENCE_TIME_MS
 * @brief  Tiempo máximo de convergencia por defecto (49 ms).
 */
#define VL6180X_MAX_CONVERGENCE_TIME_MS    49

/**
 * @def    VL6180X_MIN_CONVERGENCE_TIME_MS
 * @brief  Tiempo mínimo de convergencia (30 ms para objetos cercanos).
 */
#define VL6180X_MIN_CONVERGENCE_TIME_MS    30

/**
 * @def    VL6180X_CONFIG_MODE_MAX_RANGE
 * @brief  Modo de configuración: máximo rango (objetos lejanos).
 */
#define VL6180X_CONFIG_MODE_MAX_RANGE      0

/**
 * @def    VL6180X_CONFIG_MODE_MIN_DISTANCE
 * @brief  Modo de configuración: mínima distancia (objetos cercanos).
 */
#define VL6180X_CONFIG_MODE_MIN_DISTANCE   1

/* =====================================================================
 *  ESTRUCTURAS PÚBLICAS
 * ===================================================================== */

/**
 * @brief  Estructura que representa un sensor VL6180X.
 *
 * Contiene toda la información necesaria para controlar un sensor
 * VL6180X individual.
 *
 * @note Se debe crear una instancia por cada sensor físico conectado.
 */
typedef struct {
    uint8_t    id;                    /*!< Identificador único del sensor (1-5)             */
    gpio_num_t xshut_pin;             /*!< Pin GPIO conectado a XSHUT                       */
    uint8_t    i2c_addr;              /*!< Dirección I2C asignada (0x30-0x3F)               */
    bool       is_initialized;        /*!< Estado de inicialización                         */
    uint16_t   last_distance;         /*!< Última distancia medida (mm)                     */
    uint32_t   convergence_time_ms;   /*!< Tiempo de convergencia (ms)                      */
    uint8_t    config_mode;           /*!< Modo de configuración (0=lejos, 1=cerca)         */
} vl6180x_t;

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el bus I2C para comunicación con el sensor.
 *
 * Configura los pines GPIO y la frecuencia del bus I2C. Esta función
 * debe llamarse UNA SOLA VEZ antes de usar cualquier sensor VL6180X.
 *
 * @param[in] sda_pin  GPIO para la línea de datos SDA.
 * @param[in] scl_pin  GPIO para la línea de reloj SCL.
 * @param[in] freq_hz  Frecuencia del bus I2C en Hz (recomendado: 100000).
 * @param[in] i2c_num  Número del puerto I2C (0 o 1).
 *
 * @return ESP_OK si éxito, código de error si falla.
 *
 * @example
 *   vl6180x_init_i2c(GPIO_NUM_11, GPIO_NUM_12, 100000, 0);
 */
esp_err_t vl6180x_init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin,
                           uint32_t freq_hz, i2c_port_t i2c_num);

/**
 * @brief  Se adjunta a un bus I2C ya inicializado (no instala el driver).
 *
 * Úsalo cuando otra librería (p. ej. SSD1306) ya instaló el driver.
 *
 * @param[in] i2c_num  Puerto I2C ya en uso (I2C_NUM_0 o I2C_NUM_1).
 *
 * @return ESP_OK en éxito.
 */
esp_err_t vl6180x_attach_i2c(i2c_port_t i2c_num);

/**
 * @brief  Inicializa un sensor VL6180X específico.
 *
 * Realiza el power cycle del sensor, verifica el ID, carga la
 * configuración de registros y cambia la dirección I2C si es necesario.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 *
 * @return
 *   - ESP_OK si éxito.
 *   - ESP_ERR_INVALID_ARG si sensor es NULL.
 *   - ESP_FAIL si el sensor no responde.
 *
 * @note La estructura sensor debe tener configurados id, xshut_pin e
 *       i2c_addr antes de llamar a esta función.
 * @note El power cycle tiene delays de 100 ms y 200 ms para
 *       estabilización.
 *
 * @example
 *   vl6180x_t sensor = {
 *       .id          = 1,
 *       .xshut_pin   = GPIO_NUM_9,
 *       .i2c_addr    = 0x30,
 *       .config_mode = VL6180X_CONFIG_MODE_MAX_RANGE
 *   };
 *   vl6180x_init(&sensor);
 */
esp_err_t vl6180x_init(vl6180x_t *sensor);

/**
 * @brief  Carga la configuración para MÁXIMO RANGO (detección lejana).
 *
 * Prioriza rango sobre velocidad. Utiliza tiempo de convergencia de
 * 63 ms y mayor sensibilidad. Rango efectivo: ~15-25 cm.
 *
 * @param[in] addr  Dirección I2C del sensor.
 *
 * @note Tasa de muestreo aproximada: 15-18 Hz.
 * @note Recomendado para detectar objetos entre 10-25 cm.
 * @warning El rango máximo depende de la reflectividad del objeto.
 */
void vl6180x_load_settings_max_range(uint8_t addr);

/**
 * @brief  Carga la configuración para MÍNIMA DISTANCIA (objetos cercanos).
 *
 * Prioriza precisión en distancias muy cortas. Utiliza tiempo de
 * convergencia de 30 ms y menor sensibilidad para evitar saturación.
 * Rango efectivo: 0-20 mm.
 *
 * @param[in] addr  Dirección I2C del sensor.
 *
 * @note Tasa de muestreo aproximada: 25-30 Hz.
 * @note Recomendado para detectar objetos a menos de 2 cm.
 * @note Ideal para detección de contacto o objetos muy cercanos.
 */
void vl6180x_load_settings_min_distance(uint8_t addr);

/**
 * @brief  Realiza una medición de distancia en modo single-shot.
 *
 * Inicia una medición individual y espera el resultado. Modo más
 * confiable que el continuo para la mayoría de casos.
 *
 * @param[in,out] sensor    Puntero a la estructura del sensor (inicializado).
 * @param[out]    distance  Puntero donde almacenar la distancia medida (mm).
 *
 * @return
 *   - true  si la medición fue exitosa.
 *   - false si hubo timeout o error.
 *
 * @note El timeout máximo es de 200 ms.
 * @note El rango típico es 0-200 mm, dependiendo de la reflectividad
 *       del objeto.
 *
 * @example
 *   uint8_t dist;
 *   if (vl6180x_read_distance(&sensor, &dist)) {
 *       ESP_LOGI(TAG, "Distancia: %d mm", dist);
 *   }
 */
bool vl6180x_read_distance(vl6180x_t *sensor, uint8_t *distance);

/**
 * @brief  Lee el registro de estado del sensor.
 *
 * Útil para depuración y diagnóstico. El bit 2 indica "New Sample Ready".
 *
 * @param[in]  sensor  Puntero a la estructura del sensor.
 * @param[out] status  Puntero donde almacenar el valor del registro (0x04F).
 *
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool vl6180x_get_status(vl6180x_t *sensor, uint8_t *status);

/**
 * @brief  Lee el ID del modelo del sensor.
 *
 * El ID correcto para VL6180X es 0xB4. Útil para verificar la
 * comunicación con el sensor.
 *
 * @param[in]  sensor    Puntero a la estructura del sensor.
 * @param[out] model_id  Puntero donde almacenar el ID.
 *
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool vl6180x_get_model_id(vl6180x_t *sensor, uint8_t *model_id);

/**
 * @brief  Apaga el sensor (pone XSHUT en nivel bajo).
 *
 * El sensor dejará de responder en el bus I2C hasta que se vuelva a
 * encender y reinicializar.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 */
void vl6180x_power_off(vl6180x_t *sensor);

/**
 * @brief  Enciende el sensor (pone XSHUT en nivel alto).
 *
 * Después de encender, se debe llamar nuevamente a vl6180x_init() para
 * reinicializar el sensor.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 *
 * @note No es necesario llamar a power_off antes de power_on.
 */
void vl6180x_power_on(vl6180x_t *sensor);

/**
 * @brief  Configura el tiempo de convergencia del sensor.
 *
 * A mayor tiempo, mayor rango pero menor tasa de muestreo.
 * Valores típicos: 30-63 ms.
 *
 * @param[in,out] sensor   Puntero a la estructura del sensor.
 * @param[in]     time_ms  Tiempo de convergencia en milisegundos (1-63).
 *
 * @return true si éxito, false en caso de error.
 *
 * @note Debe llamarse después de vl6180x_init().
 */
bool vl6180x_set_convergence_time(vl6180x_t *sensor, uint8_t time_ms);

/**
 * @brief  Cambia el modo de configuración del sensor.
 *
 * Permite alternar entre configuración de máximo rango y mínima
 * distancia sin reinicializar todo el sensor.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 * @param[in]     mode    Modo de configuración:
 *                          - VL6180X_CONFIG_MODE_MAX_RANGE (0)
 *                          - VL6180X_CONFIG_MODE_MIN_DISTANCE (1)
 *
 * @return true si éxito, false en caso de error.
 *
 * @example
 *   vl6180x_set_config_mode(&sensor, VL6180X_CONFIG_MODE_MAX_RANGE);
 */
bool vl6180x_set_config_mode(vl6180x_t *sensor, uint8_t mode);

/**
 * @brief  Obtiene la última distancia medida sin hacer nueva lectura.
 *
 * Útil cuando se quiere acceder al valor cacheado sin realizar una
 * nueva medición.
 *
 * @param[in] sensor  Puntero a la estructura del sensor.
 *
 * @return Última distancia medida (mm), o 0xFF si no hay datos.
 */
uint8_t vl6180x_get_last_distance(vl6180x_t *sensor);

/**
 * @brief  Escanea el bus I2C en busca de dispositivos.
 *
 * Útil para depuración y para verificar direcciones de sensores.
 *
 * @param[in]  i2c_num        Puerto I2C a escanear.
 * @param[out] found_devices  Array para almacenar direcciones encontradas.
 * @param[in]  max_devices    Tamaño máximo del array.
 *
 * @return Número de dispositivos encontrados.
 */
int vl6180x_scan_i2c(i2c_port_t i2c_num, uint8_t *found_devices,
                     int max_devices);

#ifdef __cplusplus
}
#endif

#endif /* VL6180X_H */