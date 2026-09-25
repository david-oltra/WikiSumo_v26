/**
 * @file    qre1113.h
 * @brief   Driver para sensores de línea reflectivos QRE1113.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Cada sensor QRE1113 se conecta a un canal del ADC (ADC1 o ADC2) y
 * devuelve un valor crudo de 12 bits (0-4095). El driver permite:
 *
 *   - Inicializar el ADC oneshot (una vez) y cada sensor individualmente.
 *   - Leer el valor crudo o filtrado (media móvil).
 *   - Calibrar el sensor contra una superficie negra para obtener un
 *     umbral de detección de línea.
 *
 * NOTA: los canales de ADC2 no pueden usarse mientras el WiFi está
 * activo. Si se usa ESP-NOW + QRE1113 en ADC2, hay que tenerlo en cuenta.
 */

#ifndef QRE1113_H
#define QRE1113_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  TIPOS
 * ===================================================================== */

/**
 * @brief Estructura que representa un sensor QRE1113.
 */
typedef struct {
    uint8_t                       id;         /*!< Número de identificación (1-6)          */
    adc_channel_t                 adc_channel;/*!< Canal ADC (ADC_CHANNEL_x)               */
    uint16_t                      raw_value;  /*!< Último valor crudo leído (0-4095)       */
    bool                          is_adc2;    /*!< true si es ADC2, false si es ADC1       */
    adc_oneshot_unit_handle_t     adc_handle; /*!< Handle de la unidad ADC utilizada       */
    uint16_t                      threshold;  /*!< Umbral de detección (de la calibración) */
} qre1113_sensor_t;

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el driver ADC oneshot para ADC1 y ADC2.
 *
 * Debe llamarse una sola vez antes de inicializar cualquier sensor.
 *
 * @return
 *   - ESP_OK en éxito.
 *   - Código de error de ESP-IDF en caso contrario.
 */
esp_err_t qre1113_init_adc(void);

/**
 * @brief  Inicializa un sensor QRE1113 concreto.
 *
 * Debe llamarse después de qre1113_init_adc().
 *
 * @param[in,out] sensor  Puntero al sensor a inicializar.
 *
 * @return
 *   - ESP_OK en éxito.
 *   - ESP_ERR_INVALID_ARG si sensor es NULL.
 *   - Código de error de ESP-IDF en caso contrario.
 */
esp_err_t qre1113_init_sensor(qre1113_sensor_t *sensor);

/**
 * @brief  Lee el valor crudo del sensor y lo guarda en sensor->raw_value.
 *
 * @param[in,out] sensor  Puntero al sensor.
 *
 * @return
 *   - ESP_OK en éxito.
 *   - ESP_ERR_INVALID_ARG si sensor es NULL.
 *   - Código de error de ESP-IDF en caso contrario.
 */
esp_err_t qre1113_read_raw(qre1113_sensor_t *sensor);

/**
 * @brief  Lee el valor del sensor aplicando un filtro de media móvil.
 *
 * @param[in,out] sensor  Puntero al sensor.
 *
 * @return
 *   - Valor filtrado en el rango 0-4095.
 *   - 0 si ocurre un error.
 */
uint16_t qre1113_read_filtered(qre1113_sensor_t *sensor);

/**
 * @brief  Calibra el sensor contra una superficie negra.
 *
 * Calcula y almacena el umbral de detección en sensor->threshold.
 *
 * @param[in,out] sensor  Puntero al sensor a calibrar.
 *
 * @return
 *   - 1 si la calibración fue correcta.
 *   - 0 si falló.
 */
uint8_t qre1113_calibrate(qre1113_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* QRE1113_H */