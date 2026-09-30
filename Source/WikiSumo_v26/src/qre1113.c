/**
 * @file    qre1113.c
 * @brief   Implementación del driver para sensores de línea QRE1113.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Implementa el control de los sensores reflectivos QRE1113 a través
 *   del ADC oneshot del ESP32-S3.
 *
 *   Detalles de implementación:
 *
 *     - Se crean dos unidades ADC oneshot (ADC1 y ADC2) compartidas por
 *       todos los sensores, una sola vez en qre1113_init_adc().
 *     - Se inicializa el esquema de calibración "curve fitting" para
 *       cada unidad ADC. Esto corrige las variaciones del Vref interno
 *       del chip (que puede estar entre 1000 y 1200 mV, en lugar de los
 *       1100 mV nominales) y hace que las lecturas sean consistentes
 *       entre distintas placas.
 *     - Cada sensor guarda un puntero al handle de su unidad ADC.
 *     - Lectura síncrona por sondeo (adc_oneshot_read).
 *     - Filtro de media móvil configurable por QRE1113_FILTER_SAMPLES.
 *     - Calibración contra superficie negra para fijar el umbral de
 *       detección de línea.
 *
 *   IMPORTANTE — ADC2 y WiFi:
 *     Los canales del ADC2 no pueden usarse mientras el WiFi está activo
 *     en algunos ESP32. En el ESP32-S3 técnicamente sí se puede, pero el
 *     WiFi introduce ruido significativo en las lecturas del ADC2.
 *     Como QRE1113 #6 está en ADC2, el firmware principal apaga el
 *     WiFi (ESP-NOW) al pasar a STARTED para tener lecturas limpias.
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "qre1113.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

/**
 * @brief  Porcentaje del promedio que se usa como umbral.
 *
 * Valores bajos (80-85%)  → mucho margen frente al negro, pero puede
 *                            no detectar el blanco si el delta es pequeño.
 * Valores altos (92-95%)  → detecta blanco con deltas pequeños, pero
 *                            poco margen frente al ruido del negro.
 *
 * Recomendado: 94% para QRE1113 con delta ~10%.
 */
#define QRE1113_THRESHOLD_PCT       92

/**
 * @brief  Margen mínimo absoluto por debajo del promedio.
 *
 * Se aplica como mínimo en el cálculo del umbral: el umbral nunca
 * estará a menos de QRE1113_MIN_MARGIN unidades del promedio sobre
 * negro. Protege contra ruido bajo que si no dejaría el umbral
 * demasiado pegado al negro.
 */
#define QRE1113_MIN_MARGIN          150

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/* Handles estáticos de las unidades ADC (compartidos por todos los sensores) */
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_oneshot_unit_handle_t s_adc2_handle = NULL;

/* Handles de calibración del ADC (uno por unidad).
 * Se inicializan en qre1113_init_adc() y se usan para convertir las
 * lecturas crudas a voltaje calibrado en milivoltios. */
static adc_cali_handle_t s_adc1_cali_handle = NULL;
static adc_cali_handle_t s_adc2_cali_handle = NULL;

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

/**
 * @brief  Inicializa el esquema de calibración "curve fitting" para
 *         una unidad ADC.
 *
 * El esquema "curve fitting" usa los valores de calibración grabados
 * en los eFuses del chip durante su fabricación. Corrige las
 * variaciones del Vref interno del ESP32-S3, mejorando la precisión
 * y la consistencia entre distintas placas.
 *
 * @param unit_id   Unidad ADC (ADC_UNIT_1 o ADC_UNIT_2).
 * @param out_handle Puntero donde se almacenará el handle de calibración.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
static esp_err_t init_calibration(adc_unit_t unit_id,
                                  adc_cali_handle_t *out_handle)
{
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = unit_id,
        .atten    = QRE1113_ADC_ATTEN,
        .bitwidth = QRE1113_ADC_BITWIDTH,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config,
                                                          out_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Calibración ADC%d no soportada (%s). "
                      "Se usarán lecturas sin calibrar.",
                 unit_id + 1, esp_err_to_name(ret));
        *out_handle = NULL;
        return ret;
    }

    return ESP_OK;
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

    /* Calibración ADC1 (no es crítico si falla) */
    init_calibration(ADC_UNIT_1, &s_adc1_cali_handle);

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

    /* Calibración ADC2 */
    init_calibration(ADC_UNIT_2, &s_adc2_cali_handle);

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

/**
 * @brief  Lee el valor del sensor y lo convierte a voltaje calibrado.
 *
 * Usa el esquema de calibración del ADC para convertir la lectura
 * cruda a un valor en milivoltios. La calibración corrige las
 * variaciones del Vref interno del chip, así que el valor en mV es
 * consistente entre distintas placas ESP32-S3.
 *
 * @param[in]  sensor      Puntero al sensor.
 * @param[out] voltage_mv  Puntero donde almacenar el voltaje (mV).
 *
 * @return
 *   - ESP_OK en éxito.
 *   - ESP_ERR_INVALID_ARG si sensor o voltage_mv es NULL.
 *   - ESP_ERR_INVALID_STATE si el sensor no está inicializado.
 *   - ESP_ERR_NOT_SUPPORTED si el chip no tiene calibración grabada.
 *   - Código de error de ESP-IDF en caso contrario.
 */
esp_err_t qre1113_read_voltage(qre1113_sensor_t *sensor, int *voltage_mv)
{
    if (sensor == NULL || voltage_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor->adc_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    adc_cali_handle_t cali_handle = sensor->is_adc2
                                    ? s_adc2_cali_handle
                                    : s_adc1_cali_handle;

    if (cali_handle == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Leer valor crudo */
    int raw;
    esp_err_t ret = adc_oneshot_read(sensor->adc_handle,
                                     sensor->adc_channel, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Actualizar también raw_value por consistencia */
    sensor->raw_value = (uint16_t)raw;

    /* Convertir a voltaje calibrado */
    ret = adc_cali_raw_to_voltage(cali_handle, raw, voltage_mv);
    return ret;
}

/**
 * @brief  Calibra el sensor QRE1113 sobre superficie negra.
 *
 * Estrategia:
 *   1. Toma QRE1113_CALIBRATION_STEPS muestras y calcula el PROMEDIO
 *      (más robusto que el mínimo frente a spikes de ruido).
 *   2. También mide el RUIDO (max - min) para saber si la lectura es
 *      estable.
 *   3. Calcula DOS candidatos a umbral:
 *        a) Porcentaje del promedio (QRE1113_THRESHOLD_PCT).
 *        b) Promedio menos un margen basado en el ruido (noise * 3),
 *           con un mínimo absoluto de QRE1113_MIN_MARGIN.
 *   4. Elige el MÁS CONSERVADOR (el más bajo), garantizando que el
 *      umbral esté suficientemente por debajo del negro tanto por
 *      porcentaje como por margen absoluto.
 *
 * @param[in,out] sensor  Puntero al sensor a calibrar.
 *
 * @return
 *   - 1 si la calibración fue correcta.
 *   - 0 si falló (sensor NULL o sin lecturas válidas).
 */
uint8_t qre1113_calibrate(qre1113_sensor_t *sensor)
{
    if (sensor == NULL) return false;

    ESP_LOGI(TAG, "=== CALIBRACIÓN SENSOR %u ===", sensor->id);
    ESP_LOGI(TAG, "Coloque sobre superficie NEGRA");

    /* 1. Recolectar muestras y calcular estadísticas */
    uint32_t sum   = 0;
    uint16_t min_v = 4095;
    uint16_t max_v = 0;
    int      valid = 0;

    for (int i = 0; i < QRE1113_CALIBRATION_STEPS; i++) {
        if (qre1113_read_raw(sensor) == ESP_OK) {
            uint16_t v = sensor->raw_value;
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(QRE1113_CALIBRATION_DELAY_MS));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "Sensor %u: sin lecturas válidas", sensor->id);
        return false;
    }

    uint16_t average = (uint16_t)(sum / valid);
    uint16_t noise   = max_v - min_v;

    /* 2. Calcular los dos candidatos a umbral */

    /* Candidato A: un porcentaje del promedio */
    uint32_t pct_threshold = ((uint32_t)average * QRE1113_THRESHOLD_PCT) / 100;

    /* Candidato B: promedio menos un margen basado en el ruido.
     * Usamos 3x el ruido medido, con un mínimo absoluto para no
     * quedarnos sin margen si el sensor está muy estable. */
    uint16_t margin = noise * 3;
    if (margin < QRE1113_MIN_MARGIN) margin = QRE1113_MIN_MARGIN;

    uint32_t safe_threshold = (average > margin) ? (average - margin) : 0;

    /* 3. Elegir el más CONSERVADOR (el más bajo).
     * Esto garantiza que el umbral esté suficientemente por debajo
     * del negro tanto por porcentaje como por margen absoluto. */
    uint32_t final_threshold = (pct_threshold < safe_threshold)
                               ? pct_threshold
                               : safe_threshold;

    sensor->threshold = (uint16_t)final_threshold;

    ESP_LOGI(TAG, "Sensor %u: promedio=%u, ruido=%u (min=%u, max=%u)",
             sensor->id, average, noise, min_v, max_v);
    ESP_LOGI(TAG, "Sensor %u: umbral=%u (pct=%u%%, margen=%u)",
             sensor->id, sensor->threshold, QRE1113_THRESHOLD_PCT, margin);

    return true;
}