/**
 * @file    vl6180x.c
 * @brief   Implementación de la librería VL6180X.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Implementa el control del sensor de distancia VL6180X (ST ToF)
 *   sobre I2C usando el driver i2c_master de ESP-IDF.
 *
 *   La librería:
 *     - Inicializa (o se adjunta a) el bus I2C.
 *     - Realiza el power-cycle y la secuencia de arranque del sensor.
 *     - Carga los registros privados obligatorios del fabricante.
 *     - Permite cambiar la dirección I2C vía XSHUT.
 *     - Configura modos MÁXIMO RANGO / MÍNIMA DISTANCIA.
 *     - Lee distancias en modo single-shot.
 *     - Ofrece utilidades de diagnóstico (status, model_id, scan).
 *
 *   El sensor arranca siempre en la dirección 0x29 (fábrica). Si hay
 *   varios VL6180X en el mismo bus, cada uno debe encenderse por
 *   separado vía XSHUT y reasignársele una dirección distinta (0x30+).
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "vl6180x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "VL6180X";

/* --- Registros internos del VL6180X --- */

#define VL6180X_REG_I2C_ADDR             0x0212  /*!< Registro para cambiar dirección I2C       */
#define VL6180X_SYSRANGE_START           0x018   /*!< Inicia una medición single-shot           */
#define VL6180X_RESULT_RANGE_VAL         0x062   /*!< Resultado de la última medición           */
#define VL6180X_RESULT_INT_STATUS        0x04f   /*!< Estado de interrupciones                  */
#define VL6180X_SYSTEM_INT_CLEAR         0x015   /*!< Limpia flags de interrupción              */
#define VL6180X_SYSTEM_FRESH_OUT         0x016   /*!< Indica que el sensor se ha reiniciado     */
#define VL6180X_IDENTIFICATION_MODEL_ID  0x000   /*!< ID del modelo (esperado 0xB4)             */
#define VL6180X_SYSTEM_MODE              0x011   /*!< Modo de operación (polling / int)         */
#define VL6180X_SYSRANGE_MAX_CONV        0x01c   /*!< Tiempo máximo de convergencia (ms)        */
#define VL6180X_INTERRUPT_CONFIG         0x014   /*!< Configuración de interrupciones           */
#define VL6180X_ALS_GAIN                 0x03f   /*!< Ganancia del sensor de luz ambiental      */
#define VL6180X_ALS_INTEGRATION_TIME     0x040   /*!< Tiempo de integración del ALS             */

/** ID esperado del modelo VL6180X. */
#define VL6180X_MODEL_ID                 0xB4

/** Número de reintentos de polling al leer una medición. */
#define VL6180X_MEASURE_RETRIES          20

/** Retardo entre reintentos de polling (ms). */
#define VL6180X_MEASURE_RETRY_DELAY_MS   10

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/** Puerto I2C actualmente en uso. */
static i2c_port_t g_i2c_num         = 0;

/** Bandera que indica si el bus I2C está listo para usarse. */
static bool       g_i2c_initialized = false;

/* =====================================================================
 *  FUNCIONES I2C PRIVADAS
 * ===================================================================== */

/**
 * @brief  Escribe un byte en un registro de 16 bits del sensor.
 *
 * @param[in] addr  Dirección I2C del sensor.
 * @param[in] reg   Dirección del registro (16 bits).
 * @param[in] data  Byte a escribir.
 *
 * @return ESP_OK en éxito, error en caso contrario.
 */
static esp_err_t vl6180x_write_byte(uint8_t addr, uint16_t reg, uint8_t data)
{
    if (!g_i2c_initialized) {
        ESP_LOGE(TAG, "I2C no inicializado");
        return ESP_FAIL;
    }

    uint8_t write_buf[3] = { (reg >> 8) & 0xFF, reg & 0xFF, data };
    return i2c_master_write_to_device(g_i2c_num, addr,
                                      write_buf, sizeof(write_buf),
                                      pdMS_TO_TICKS(VL6180X_I2C_TIMEOUT_MS));
}

/**
 * @brief  Lee un byte de un registro de 16 bits del sensor.
 *
 * @param[in]  addr  Dirección I2C del sensor.
 * @param[in]  reg   Dirección del registro (16 bits).
 * @param[out] data  Puntero donde almacenar el byte leído.
 *
 * @return ESP_OK en éxito, error en caso contrario.
 */
static esp_err_t vl6180x_read_byte(uint8_t addr, uint16_t reg, uint8_t *data)
{
    if (!g_i2c_initialized) {
        ESP_LOGE(TAG, "I2C no inicializado");
        return ESP_FAIL;
    }

    uint8_t reg_buf[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    return i2c_master_write_read_device(g_i2c_num, addr,
                                        reg_buf, sizeof(reg_buf),
                                        data, 1,
                                        pdMS_TO_TICKS(VL6180X_I2C_TIMEOUT_MS));
}

/**
 * @brief  Carga los registros privados de configuración (base común).
 *
 * Estos registros no están documentados en el datasheet público y son
 * obligatorios según ST Microelectronics para el correcto funcionamiento
 * del sensor.
 *
 * @param[in] addr  Dirección I2C del sensor.
 */
static void vl6180x_load_private_registers(uint8_t addr)
{
    /* Registros privados obligatorios según ST Microelectronics */
    vl6180x_write_byte(addr, 0x0207, 0x01);
    vl6180x_write_byte(addr, 0x0208, 0x01);
    vl6180x_write_byte(addr, 0x0096, 0x00);
    vl6180x_write_byte(addr, 0x0097, 0xfd);
    vl6180x_write_byte(addr, 0x00e3, 0x00);
    vl6180x_write_byte(addr, 0x00e4, 0x04);
    vl6180x_write_byte(addr, 0x00e5, 0x02);
    vl6180x_write_byte(addr, 0x00e6, 0x01);
    vl6180x_write_byte(addr, 0x00e7, 0x03);
    vl6180x_write_byte(addr, 0x00f5, 0x02);
    vl6180x_write_byte(addr, 0x00d9, 0x05);
    vl6180x_write_byte(addr, 0x00db, 0xce);
    vl6180x_write_byte(addr, 0x00dc, 0x03);
    vl6180x_write_byte(addr, 0x00dd, 0xf8);
    vl6180x_write_byte(addr, 0x009f, 0x00);
    vl6180x_write_byte(addr, 0x00a3, 0x3c);
    vl6180x_write_byte(addr, 0x00b7, 0x00);
    vl6180x_write_byte(addr, 0x00bb, 0x3c);
    vl6180x_write_byte(addr, 0x00b2, 0x09);
    vl6180x_write_byte(addr, 0x00ca, 0x09);
    vl6180x_write_byte(addr, 0x0198, 0x01);
    vl6180x_write_byte(addr, 0x01b0, 0x17);
    vl6180x_write_byte(addr, 0x01ad, 0x00);
    vl6180x_write_byte(addr, 0x00ff, 0x05);
    vl6180x_write_byte(addr, 0x0100, 0x05);
    vl6180x_write_byte(addr, 0x0199, 0x05);
    vl6180x_write_byte(addr, 0x01a6, 0x1b);
    vl6180x_write_byte(addr, 0x01ac, 0x3e);
    vl6180x_write_byte(addr, 0x01a7, 0x1f);
    vl6180x_write_byte(addr, 0x0030, 0x00);
}

/* =====================================================================
 *  FUNCIONES DE CONFIGURACIÓN PÚBLICAS
 * ===================================================================== */

/**
 * @brief  Carga la configuración para MÁXIMO RANGO (detección lejana).
 *
 * Prioriza rango sobre velocidad:
 *   - Tiempo de convergencia: 63 ms.
 *   - Umbral de señal: 0x90 (más sensible).
 *   - Rango efectivo: ~15-25 cm.
 *
 * @param[in] addr  Dirección I2C del sensor.
 */
void vl6180x_load_settings_max_range(uint8_t addr)
{
    ESP_LOGI(TAG, "Cargando configuración MÁXIMO RANGO (lejana)");

    vl6180x_write_byte(addr, 0x001c, 63);   /* Tiempo convergencia: 63 ms        */
    vl6180x_write_byte(addr, 0x00db, 0x90); /* Umbral de señal: 144 (más sensible) */
    vl6180x_write_byte(addr, 0x0011, 0x10); /* Habilita polling                  */
    vl6180x_write_byte(addr, VL6180X_ALS_GAIN, 0x46);             /* Ganancia de luz */
    vl6180x_write_byte(addr, VL6180X_ALS_INTEGRATION_TIME, 0x63); /* 100 ms integración */

    ESP_LOGI(TAG, "✅ Configuración máximo rango aplicada");
}

/**
 * @brief  Carga la configuración para MÍNIMA DISTANCIA (objetos cercanos).
 *
 * Prioriza precisión en distancias cortas:
 *   - Tiempo de convergencia: 30 ms.
 *   - Umbral de señal: 0xCE (menos sensible, evita saturación).
 *   - Rango efectivo: ~0-20 mm.
 *
 * @param[in] addr  Dirección I2C del sensor.
 */
void vl6180x_load_settings_min_distance(uint8_t addr)
{
    ESP_LOGI(TAG, "Cargando configuración MÍNIMA DISTANCIA (cercana)");

    vl6180x_write_byte(addr, 0x001c, 30);   /* Tiempo convergencia: 30 ms          */
    vl6180x_write_byte(addr, 0x00db, 0xCE); /* Umbral de señal: 206 (menos sensible) */
    vl6180x_write_byte(addr, 0x0011, 0x10); /* Habilita polling                     */
    vl6180x_write_byte(addr, VL6180X_ALS_GAIN, 0x46);             /* Ganancia de luz */
    vl6180x_write_byte(addr, VL6180X_ALS_INTEGRATION_TIME, 0x63); /* 100 ms integración */

    ESP_LOGI(TAG, "✅ Configuración mínima distancia aplicada");
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el bus I2C para comunicación con el sensor.
 *
 * Configura los pines GPIO y la frecuencia del bus I2C. Debe llamarse
 * UNA SOLA VEZ antes de usar cualquier sensor VL6180X.
 *
 * Si el driver I2C ya estaba instalado (por ejemplo, por la librería
 * del SSD1306), se reutiliza el bus sin reinstalar el driver.
 *
 * @param[in] sda_pin  GPIO para la línea de datos SDA.
 * @param[in] scl_pin  GPIO para la línea de reloj SCL.
 * @param[in] freq_hz  Frecuencia del bus I2C en Hz (recomendado: 100000).
 * @param[in] i2c_num  Número del puerto I2C (0 o 1).
 *
 * @return ESP_OK si éxito, código de error si falla.
 */
esp_err_t vl6180x_init_i2c(gpio_num_t sda_pin, gpio_num_t scl_pin,
                           uint32_t freq_hz, i2c_port_t i2c_num)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda_pin,
        .scl_io_num       = scl_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t ret = i2c_param_config(i2c_num, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        /* El driver ya está instalado por otra librería (p. ej. SSD1306) */
        ESP_LOGW(TAG, "Driver I2C ya instalado, reutilizando el existente");
        g_i2c_num         = i2c_num;
        g_i2c_initialized = true;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_num         = i2c_num;
    g_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C inicializado: SDA=GPIO%d, SCL=GPIO%d, freq=%lu Hz",
             sda_pin, scl_pin, freq_hz);
    return ESP_OK;
}

/**
 * @brief  Se adjunta a un bus I2C ya inicializado.
 *
 * No instala el driver; simplemente registra el puerto a usar. Útil
 * cuando otra librería (p. ej. SSD1306) ya instaló el driver I2C.
 *
 * @param[in] i2c_num  Puerto I2C ya en uso (I2C_NUM_0 o I2C_NUM_1).
 *
 * @return ESP_OK en éxito.
 */
esp_err_t vl6180x_attach_i2c(i2c_port_t i2c_num)
{
    g_i2c_num         = i2c_num;
    g_i2c_initialized = true;
    ESP_LOGI(TAG, "VL6180X adjuntado al bus I2C existente (puerto %d)", i2c_num);
    return ESP_OK;
}

/**
 * @brief  Inicializa un sensor VL6180X específico.
 *
 * Realiza el power cycle (XSHUT bajo → alto), verifica el model ID,
 * carga los registros privados obligatorios, cambia la dirección I2C
 * si es necesario y aplica la configuración del modo seleccionado.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 *
 * @return
 *   - ESP_OK si éxito.
 *   - ESP_ERR_INVALID_ARG si sensor es NULL.
 *   - ESP_FAIL si el I2C no está inicializado o el sensor no responde.
 *
 * @note La estructura sensor debe tener configurados id, xshut_pin,
 *       i2c_addr y config_mode antes de llamar a esta función.
 */
esp_err_t vl6180x_init(vl6180x_t *sensor)
{
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Puntero a sensor nulo");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_i2c_initialized) {
        ESP_LOGE(TAG, "I2C no inicializado. Llame vl6180x_init_i2c() primero");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Inicializando sensor ID=%d en GPIO %d",
             sensor->id, sensor->xshut_pin);

    /* Configurar pin XSHUT */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sensor->xshut_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    /* Power cycle */
    vl6180x_power_off(sensor);
    vTaskDelay(pdMS_TO_TICKS(100));
    vl6180x_power_on(sensor);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Verificar ID del sensor */
    uint8_t model_id = 0;
    if (vl6180x_read_byte(VL6180X_DEFAULT_ADDR,
                          VL6180X_IDENTIFICATION_MODEL_ID,
                          &model_id) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo leer ID del sensor");
        gpio_set_level(sensor->xshut_pin, 0);
        return ESP_FAIL;
    }

    if (model_id != VL6180X_MODEL_ID) {
        ESP_LOGE(TAG, "ID incorrecto: 0x%02X (esperado 0x%02X)",
                 model_id, VL6180X_MODEL_ID);
        gpio_set_level(sensor->xshut_pin, 0);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor VL6180X detectado (ID: 0x%02X)", model_id);

    /* Cargar registros privados base */
    vl6180x_load_private_registers(VL6180X_DEFAULT_ADDR);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Cambiar dirección I2C si es necesario */
    if (sensor->i2c_addr != VL6180X_DEFAULT_ADDR) {
        ESP_LOGI(TAG, "Cambiando dirección I2C a 0x%02X", sensor->i2c_addr);
        vl6180x_write_byte(VL6180X_DEFAULT_ADDR,
                           VL6180X_REG_I2C_ADDR,
                           sensor->i2c_addr);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Aplicar configuración según el modo seleccionado */
    if (sensor->config_mode == VL6180X_CONFIG_MODE_MIN_DISTANCE) {
        vl6180x_load_settings_min_distance(sensor->i2c_addr);
    } else {
        vl6180x_load_settings_max_range(sensor->i2c_addr);
    }

    /* Limpiar flags de interrupción pendientes */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_SYSTEM_INT_CLEAR, 0x07);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Habilitar interrupción "New Sample Ready" */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_INTERRUPT_CONFIG, 0x04);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Configurar modo polling */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_SYSTEM_MODE, 0x10);
    vTaskDelay(pdMS_TO_TICKS(10));

    sensor->is_initialized = true;
    sensor->last_distance  = 0;

    ESP_LOGI(TAG, "✅ Sensor ID=%d inicializado correctamente en 0x%02X",
             sensor->id, sensor->i2c_addr);

    return ESP_OK;
}

/**
 * @brief  Cambia el modo de configuración del sensor.
 *
 * Permite alternar entre configuración de MÁXIMO RANGO y MÍNIMA
 * DISTANCIA sin reinicializar todo el sensor.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 * @param[in]     mode    Modo de configuración:
 *                          - VL6180X_CONFIG_MODE_MAX_RANGE (0)
 *                          - VL6180X_CONFIG_MODE_MIN_DISTANCE (1)
 *
 * @return
 *   - true  si el cambio fue exitoso.
 *   - false si sensor es NULL o no está inicializado.
 */
bool vl6180x_set_config_mode(vl6180x_t *sensor, uint8_t mode)
{
    if (sensor == NULL || !sensor->is_initialized) {
        return false;
    }

    if (mode == VL6180X_CONFIG_MODE_MIN_DISTANCE) {
        vl6180x_load_settings_min_distance(sensor->i2c_addr);
        sensor->config_mode = mode;
        ESP_LOGI(TAG, "Sensor ID=%d cambiado a modo MÍNIMA DISTANCIA", sensor->id);
    } else {
        vl6180x_load_settings_max_range(sensor->i2c_addr);
        sensor->config_mode = mode;
        ESP_LOGI(TAG, "Sensor ID=%d cambiado a modo MÁXIMO RANGO", sensor->id);
    }

    return true;
}

/**
 * @brief  Realiza una medición de distancia en modo single-shot.
 *
 * Limpia interrupciones, arranca la medición y espera por polling a que
 * el bit "New Sample Ready" se active. Después lee el resultado y
 * actualiza `sensor->last_distance`.
 *
 * @param[in,out] sensor    Puntero a la estructura del sensor (inicializado).
 * @param[out]    distance  Puntero donde almacenar la distancia medida (mm).
 *
 * @return
 *   - true  si la medición fue exitosa.
 *   - false si hubo timeout, error de I2C o argumentos inválidos.
 *
 * @note El timeout máximo es de VL6180X_MEASURE_RETRIES × retardo (200 ms).
 * @note El rango típico es 0-200 mm, dependiendo de la reflectividad
 *       del objeto.
 */
bool vl6180x_read_distance(vl6180x_t *sensor, uint8_t *distance)
{
    if (sensor == NULL || !sensor->is_initialized || distance == NULL) {
        return false;
    }

    /* Limpiar interrupción */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_SYSTEM_INT_CLEAR, 0x07);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Iniciar medición single-shot */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_SYSRANGE_START, 0x01);

    /* Esperar resultado por polling */
    uint8_t status = 0;
    for (int i = 0; i < VL6180X_MEASURE_RETRIES; i++) {
        vTaskDelay(pdMS_TO_TICKS(VL6180X_MEASURE_RETRY_DELAY_MS));
        if (vl6180x_read_byte(sensor->i2c_addr,
                              VL6180X_RESULT_INT_STATUS,
                              &status) == ESP_OK) {
            if (status & 0x04) break;   /* Bit 2 = New Sample Ready */
        }
    }

    if (!(status & 0x04)) {
        ESP_LOGW(TAG, "Timeout en medición sensor ID=%d", sensor->id);
        return false;
    }

    /* Leer distancia */
    uint8_t range = 0;
    if (vl6180x_read_byte(sensor->i2c_addr,
                          VL6180X_RESULT_RANGE_VAL,
                          &range) != ESP_OK) {
        return false;
    }

    /* Limpiar interrupción */
    vl6180x_write_byte(sensor->i2c_addr, VL6180X_SYSTEM_INT_CLEAR, 0x07);

    sensor->last_distance = range;
    *distance             = range;

    return true;
}

/**
 * @brief  Lee el registro de estado del sensor.
 *
 * Devuelve el contenido del registro RESULT_INT_STATUS (0x04F). El bit 2
 * indica "New Sample Ready".
 *
 * @param[in]  sensor  Puntero a la estructura del sensor.
 * @param[out] status  Puntero donde almacenar el valor del registro.
 *
 * @return
 *   - true  si la lectura fue exitosa.
 *   - false si sensor/status es NULL, el sensor no está inicializado o
 *           hubo error de I2C.
 */
bool vl6180x_get_status(vl6180x_t *sensor, uint8_t *status)
{
    if (sensor == NULL || !sensor->is_initialized || status == NULL) {
        return false;
    }

    return vl6180x_read_byte(sensor->i2c_addr,
                             VL6180X_RESULT_INT_STATUS,
                             status) == ESP_OK;
}

/**
 * @brief  Lee el ID del modelo del sensor.
 *
 * El ID correcto para VL6180X es 0xB4. Útil para verificar la
 * comunicación con el sensor.
 *
 * @param[in]  sensor    Puntero a la estructura del sensor.
 * @param[out] model_id  Puntero donde almacenar el ID.
 *
 * @return
 *   - true  si la lectura fue exitosa.
 *   - false si sensor/model_id es NULL o hubo error de I2C.
 *
 * @note Si el sensor no está inicializado, se lee usando la dirección
 *       de fábrica (0x29).
 */
bool vl6180x_get_model_id(vl6180x_t *sensor, uint8_t *model_id)
{
    if (sensor == NULL || model_id == NULL) {
        return false;
    }

    uint8_t addr = sensor->is_initialized ? sensor->i2c_addr
                                          : VL6180X_DEFAULT_ADDR;
    return vl6180x_read_byte(addr,
                             VL6180X_IDENTIFICATION_MODEL_ID,
                             model_id) == ESP_OK;
}

/**
 * @brief  Apaga el sensor (pone XSHUT en nivel bajo).
 *
 * El sensor dejará de responder en el bus I2C hasta que se vuelva a
 * encender con vl6180x_power_on() y reinicializar con vl6180x_init().
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 */
void vl6180x_power_off(vl6180x_t *sensor)
{
    if (sensor == NULL) return;

    gpio_set_level(sensor->xshut_pin, 0);
    sensor->is_initialized = false;
    ESP_LOGI(TAG, "Sensor ID=%d apagado", sensor->id);
}

/**
 * @brief  Enciende el sensor (pone XSHUT en nivel alto).
 *
 * Después de encender, se debe llamar a vl6180x_init() para
 * reinicializar el sensor.
 *
 * @param[in,out] sensor  Puntero a la estructura del sensor.
 *
 * @note No es necesario llamar a vl6180x_power_off() antes.
 */
void vl6180x_power_on(vl6180x_t *sensor)
{
    if (sensor == NULL) return;

    gpio_set_level(sensor->xshut_pin, 1);
    sensor->is_initialized = false;
    ESP_LOGI(TAG, "Sensor ID=%d encendido (requiere reinicialización)", sensor->id);
}

/**
 * @brief  Configura el tiempo de convergencia del sensor.
 *
 * A mayor tiempo, mayor rango pero menor tasa de muestreo.
 * Valores típicos: 30-63 ms.
 *
 * @param[in,out] sensor   Puntero a la estructura del sensor.
 * @param[in]     time_ms  Tiempo de convergencia en ms (1-63).
 *
 * @return
 *   - true  si se aplicó correctamente.
 *   - false si sensor es NULL, no está inicializado, o time_ms está
 *           fuera del rango permitido.
 */
bool vl6180x_set_convergence_time(vl6180x_t *sensor, uint8_t time_ms)
{
    if (sensor == NULL || !sensor->is_initialized) {
        return false;
    }

    if (time_ms < 1 || time_ms > 63) {
        ESP_LOGW(TAG, "Tiempo de convergencia fuera de rango (1-63 ms): %d",
                 time_ms);
        return false;
    }

    esp_err_t ret = vl6180x_write_byte(sensor->i2c_addr,
                                       VL6180X_SYSRANGE_MAX_CONV,
                                       time_ms);
    if (ret == ESP_OK) {
        sensor->convergence_time_ms = time_ms;
        ESP_LOGI(TAG, "Tiempo de convergencia configurado a %d ms", time_ms);
        return true;
    }

    return false;
}

/**
 * @brief  Obtiene la última distancia medida sin hacer nueva lectura.
 *
 * @param[in] sensor  Puntero a la estructura del sensor.
 *
 * @return Última distancia medida (mm), o 0xFF si sensor es NULL.
 */
uint8_t vl6180x_get_last_distance(vl6180x_t *sensor)
{
    if (sensor == NULL) return 0xFF;
    return sensor->last_distance;
}

/**
 * @brief  Escanea el bus I2C en busca de dispositivos.
 *
 * Recorre las direcciones 0x01-0x7E y comprueba cuáles responden a un
 * ACK. Útil para depuración y para verificar las direcciones asignadas
 * a los sensores.
 *
 * @param[in]  i2c_num        Puerto I2C a escanear.
 * @param[out] found_devices  Array donde almacenar las direcciones
 *                            encontradas (puede ser NULL si solo se
 *                            quiere el contador).
 * @param[in]  max_devices    Tamaño máximo del array.
 *
 * @return Número de dispositivos encontrados.
 */
int vl6180x_scan_i2c(i2c_port_t i2c_num, uint8_t *found_devices,
                     int max_devices)
{
    int count = 0;

    ESP_LOGI(TAG, "Escaneando bus I2C...");

    for (uint8_t addr = 1; addr < 127 && count < max_devices; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo encontrado en 0x%02X", addr);
            if (found_devices != NULL) {
                found_devices[count] = addr;
            }
            count++;
        }
    }

    ESP_LOGI(TAG, "Escaneo completado. %d dispositivo(s) encontrado(s)", count);
    return count;
}