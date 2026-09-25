/**
 * @file    qre1113.c
 * @brief   Implementación del driver para sensores de línea QRE1113.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Detalles de implementación:
 *
 *   - Se crean dos unidades ADC oneshot (ADC1 y ADC2) compartidas por
 *     todos los sensores, una sola vez en qre1113_init_adc().
 *   - Cada sensor guarda un puntero al handle de su unidad ADC.
 *   - Lectura síncrona por sondeo (adc_oneshot_read).
 *   - Filtro de media móvil configurable por QRE1113_FILTER_SAMPLES.
 *   - Calibración contra superficie negra para fijar el umbral de
 *     detección de línea.
 *
 * NOTA: los canales de ADC2 no están disponibles mientras el WiFi está
 * activo. Si se usa ESP-NOW junto con sensores en ADC2, hay que tenerlo
 * en cuenta (ver ESP-IDF: "ADC2 is not available when WiFi is in use").
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "qre1113.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "QRE1113";

/** Atenuación del ADC: rango completo ~0-3.9 V. */
#define QRE1113_ADC_ATTEN       ADC_ATTEN_DB_12

/** Resolución del ADC: 12 bits (0-4095). */
#define QRE1113_ADC_BITWIDTH    ADC_BITWIDTH_12

/** Número de muestras para el filtro de media móvil. */
#define QRE1113_FILTER_SAMPLES  10

/** Margen restado al mínimo detectado durante la calibración. */
#define QRE1113_CALIBRATION_MARGIN  100

/** Número de iteraciones durante la calibración (500 x 10 ms ≈ 5 s). */
#define QRE1113_CALIBRATION_STEPS   500

/** Retardo entre muestras durante la calibración (ms). */
#define QRE1113_CALIBRATION_DELAY_MS  10

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/* Handles estáticos de las unidades ADC (compartidos por todos los sensores) */
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_oneshot_unit_handle_t s_adc2_handle = NULL;

/* =====================================================================
 *  FUNCIONES AUXILIARES
 * ===================================================================== */

/**
 * @brief  Configura un canal del ADC con la atenuación y resolución
 *         definidas para los QRE1113.
 *
 * @param handle   Handle de la unidad ADC.
 * @param channel  Canal a configurar.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
static esp_err_t config_adc_channel(adc_oneshot_unit_handle_t handle,
                                    adc_channel_t channel)
{
    adc_oneshot_chan_cfg_t config = {
        .atten    = QRE1113_ADC_ATTEN,
        .bitwidth = QRE1113_ADC_BITWIDTH,
    };
    return adc_oneshot_config_channel(handle, channel, &config);
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

esp_err_t qre1113_init_adc(void)
{
    esp_err_t ret;

    /* ---------- ADC1 ---------- */
    adc_oneshot_unit_init_cfg_t init_cfg1 = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_cfg1, &s_adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ADC1: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ADC1 inicializado");

    /* ---------- ADC2 ---------- */
    adc_oneshot_unit_init_cfg_t init_cfg2 = {
        .unit_id  = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_cfg2, &s_adc2_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ADC2: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ADC2 inicializado");

    return ESP_OK;
}

esp_err_t qre1113_init_sensor(qre1113_sensor_t *sensor)
{
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Sensor puntero nulo");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    if (!sensor->is_adc2) {
        sensor->adc_handle = s_adc1_handle;
        ret = config_adc_channel(s_adc1_handle, sensor->adc_channel);
    } else {
        sensor->adc_handle = s_adc2_handle;
        ret = config_adc_channel(s_adc2_handle, sensor->adc_channel);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando sensor %u en canal %d: %s",
                 sensor->id, sensor->adc_channel, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Sensor %u inicializado en canal %d (%s)",
             sensor->id, sensor->adc_channel,
             sensor->is_adc2 ? "ADC2" : "ADC1");
    return ESP_OK;
}

esp_err_t qre1113_read_raw(qre1113_sensor_t *sensor)
{
    if (sensor == NULL || sensor->adc_handle == NULL) {
        ESP_LOGE(TAG, "Sensor no inicializado");
        return ESP_ERR_INVALID_STATE;
    }

    int raw;
    esp_err_t ret = adc_oneshot_read(sensor->adc_handle,
                                     sensor->adc_channel, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo sensor %u: %s",
                 sensor->id, esp_err_to_name(ret));
        return ret;
    }

    sensor->raw_value = (uint16_t)raw;
    return ESP_OK;
}

uint16_t qre1113_read_filtered(qre1113_sensor_t *sensor)
{
    if (sensor == NULL) return 0;

    long sum   = 0;
    int  valid = 0;

    for (int i = 0; i < QRE1113_FILTER_SAMPLES; i++) {
        if (qre1113_read_raw(sensor) == ESP_OK) {
            sum += sensor->raw_value;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "No hay muestras válidas para sensor %u", sensor->id);
        return 0;
    }

    return (uint16_t)(sum / valid);
}

uint8_t qre1113_calibrate(qre1113_sensor_t *sensor)
{
    if (sensor == NULL) return false;

    int threshold = 4095;

    ESP_LOGI(TAG, "=== CALIBRACIÓN SENSOR %u ===", sensor->id);
    ESP_LOGI(TAG, "Coloque sobre superficie NEGRA");

    for (int i = 0; i < QRE1113_CALIBRATION_STEPS; i++) {
        if (qre1113_read_raw(sensor) == ESP_OK) {
            if (sensor->raw_value < threshold) threshold = sensor->raw_value;
        }
        vTaskDelay(pdMS_TO_TICKS(QRE1113_CALIBRATION_DELAY_MS));
    }

    /* El umbral se fija un poco por debajo del mínimo detectado para
     * dejar margen frente a la superficie blanca. */
    sensor->threshold = (threshold - QRE1113_CALIBRATION_MARGIN);

    ESP_LOGI(TAG, "UMBRAL: %d", sensor->threshold);

    return true;
}