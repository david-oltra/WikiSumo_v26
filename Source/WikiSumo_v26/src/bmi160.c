#include "bmi160.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

/* ============================================================
 *                      CONSTANTES INTERNAS
 * ============================================================ */

static const char *TAG = "BMI160";

/* Registros */
#define BMI160_REG_CHIP_ID    0x00
#define BMI160_REG_CMD        0x7E
#define BMI160_REG_ACC_CONF   0x40
#define BMI160_REG_ACC_RANGE  0x41
#define BMI160_REG_GYR_CONF   0x42
#define BMI160_REG_GYR_RANGE  0x43
#define BMI160_REG_DATA       0x12

#define BMI160_CHIP_ID        0xD1
#define I2C_TIMEOUT_MS        1000

/* Bits de bandwidth (bwp) en modo "normal" */
#define BMI160_ACC_BWP_NORMAL 0x01u
#define BMI160_GYR_BWP_NORMAL 0x01u

/* Comandos del registro CMD (0x7E) */
#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15

/* ============================================================
 *                      ESTADO INTERNO
 * ============================================================ */

static bmi160_config_t s_cfg;
static bool            s_initialized = false;

/* Estado del bus I2C (posiblemente compartido con otros drivers) */
static bool            g_i2c_initialized = false;
static i2c_port_t      g_i2c_num         = I2C_NUM_0;

/* Factores de escala en runtime */
static float s_accel_scale = 16384.0f;
static float s_gyro_scale  = 16.4f;

/* Bias del giróscopo */
static float s_bias_x = 0.0f;
static float s_bias_y = 0.0f;
static float s_bias_z = 0.0f;

/* Ángulos filtrados */
static float s_pitch = 0.0f;
static float s_roll  = 0.0f;
static float s_yaw   = 0.0f;

/* ============================================================
 *          HELPERS: rango → factor de escala
 * ============================================================ */

static float accel_scale_from_range(bmi160_accel_range_t r)
{
    switch (r) {
        case BMI160_ACCEL_RANGE_2G:  return 16384.0f;
        case BMI160_ACCEL_RANGE_4G:  return  8192.0f;
        case BMI160_ACCEL_RANGE_8G:  return  4096.0f;
        case BMI160_ACCEL_RANGE_16G: return  2048.0f;
        default:                     return 16384.0f;
    }
}

static float gyro_scale_from_range(bmi160_gyro_range_t r)
{
    switch (r) {
        case BMI160_GYRO_RANGE_2000: return 16.4f;
        case BMI160_GYRO_RANGE_1000: return 32.8f;
        case BMI160_GYRO_RANGE_500:  return 65.6f;
        case BMI160_GYRO_RANGE_250:  return 131.2f;
        case BMI160_GYRO_RANGE_125:  return 262.4f;
        default:                     return 16.4f;
    }
}

/* ============================================================
 *          I2C BAJO NIVEL (patrón "shared bus")
 * ============================================================ */

static esp_err_t i2c_bus_init(void)
{
    if (g_i2c_initialized && g_i2c_num == s_cfg.i2c_port) {
        ESP_LOGD(TAG, "I2C ya inicializado por BMI160, reutilizando");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = s_cfg.sda_io,
        .scl_io_num       = s_cfg.scl_io,
        .sda_pullup_en    = s_cfg.sda_pullup,
        .scl_pullup_en    = s_cfg.scl_pullup,
        .master.clk_speed = s_cfg.clk_speed_hz,
    };

    esp_err_t ret = i2c_param_config(s_cfg.i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(s_cfg.i2c_port, conf.mode, 0, 0, 0);

    /* En ESP-IDF 5.4+ el driver devuelve ESP_FAIL (no ESP_ERR_INVALID_STATE)
     * cuando el bus ya está instalado por otro componente. Aceptamos ambos. */
    if (ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
        ESP_LOGW(TAG, "Driver I2C ya instalado en puerto %d (%s), reutilizando",
                 s_cfg.i2c_port, esp_err_to_name(ret));
        g_i2c_num         = s_cfg.i2c_port;
        g_i2c_initialized = true;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_num         = s_cfg.i2c_port;
    g_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C inicializado: SDA=GPIO%d, SCL=GPIO%d, freq=%lu Hz",
             s_cfg.sda_io, s_cfg.scl_io, (unsigned long)s_cfg.clk_speed_hz);
    return ESP_OK;
}

static esp_err_t bmi160_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(
        s_cfg.i2c_port, s_cfg.i2c_addr,
        buf, sizeof(buf),
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bmi160_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(
        s_cfg.i2c_port, s_cfg.i2c_addr,
        &reg, 1, data, 1,
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bmi160_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        s_cfg.i2c_port, s_cfg.i2c_addr,
        &reg, 1, data, len,
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* ============================================================
 *          ATTACH A UN BUS YA EXISTENTE
 * ============================================================ */

esp_err_t bmi160_attach_i2c(i2c_port_t port)
{
    g_i2c_num         = port;
    g_i2c_initialized = true;
    ESP_LOGI(TAG, "BMI160 adjuntado al bus I2C existente (puerto %d)", port);
    return ESP_OK;
}

/* ============================================================
 *                      INIT
 * ============================================================ */

esp_err_t bmi160_init(const bmi160_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    s_cfg = *config;

    s_accel_scale = accel_scale_from_range(s_cfg.accel_range);
    s_gyro_scale  = gyro_scale_from_range (s_cfg.gyro_range);

    /* Inicialización idempotente del bus */
    if (!g_i2c_initialized) {
        esp_err_t err = i2c_bus_init();
        if (err != ESP_OK) return err;
    } else {
        ESP_LOGI(TAG, "Usando bus I2C ya adjuntado (puerto %d)", g_i2c_num);
    }

    /* Verificar chip */
    uint8_t chip_id = 0;
    esp_err_t err = bmi160_read_reg(BMI160_REG_CHIP_ID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo leer CHIP_ID: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "CHIP_ID: 0x%02X (esperado 0x%02X)", chip_id, BMI160_CHIP_ID);
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "CHIP_ID incorrecto, verifica el bus I2C");
        return ESP_FAIL;
    }

    /* Soft reset */
    err = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));   /* el datasheet recomienda esperar tras reset */

    /* ACC_CONF: bwp[6:4] | odr[3:0] */
    uint8_t acc_conf = (uint8_t)((BMI160_ACC_BWP_NORMAL << 4) |
                                 (s_cfg.accel_odr & 0x0F));
    /* GYR_CONF: bwp[5:4] | odr[3:0] */
    uint8_t gyr_conf = (uint8_t)((BMI160_GYR_BWP_NORMAL << 4) |
                                 (s_cfg.gyro_odr  & 0x0F));

    err = bmi160_write_reg(BMI160_REG_ACC_CONF,  acc_conf);                   if (err) return err;
    err = bmi160_write_reg(BMI160_REG_ACC_RANGE, (uint8_t)s_cfg.accel_range); if (err) return err;
    err = bmi160_write_reg(BMI160_REG_GYR_CONF,  gyr_conf);                   if (err) return err;
    err = bmi160_write_reg(BMI160_REG_GYR_RANGE, (uint8_t)s_cfg.gyro_range);  if (err) return err;

    /* Habilitar acelerómetro en modo normal (CMD 0x11) */
    err = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Habilitar giróscopo en modo normal (CMD 0x15) */
    err = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(80));   /* el gyro tarda ~80 ms en estabilizarse */

    s_initialized = true;
    ESP_LOGI(TAG, "BMI160 OK | accel=0x%02X (%.0f LSB/g), gyro=0x%02X (%.1f LSB/°/s)",
             (unsigned)s_cfg.accel_range, s_accel_scale,
             (unsigned)s_cfg.gyro_range,  s_gyro_scale);
    return ESP_OK;
}

/* ============================================================
 *                      SCAN I2C (diagnóstico)
 * ============================================================ */

void bmi160_i2c_scan(void)
{
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(s_cfg.i2c_port, cmd,
                                             pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo en 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Escaneo completado");
}

/* ============================================================
 *                      LECTURA CRUDA
 * ============================================================ */

esp_err_t bmi160_read_raw(bmi160_raw_t *out)
{
    if (!s_initialized || out == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t buf[12];
    esp_err_t err = bmi160_read_bytes(BMI160_REG_DATA, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    /* Datos little-endian: accel (6B) seguido de gyro (6B) */
    out->accel_x = (int16_t)((buf[1]  << 8) | buf[0]);
    out->accel_y = (int16_t)((buf[3]  << 8) | buf[2]);
    out->accel_z = (int16_t)((buf[5]  << 8) | buf[4]);
    out->gyro_x  = (int16_t)((buf[7]  << 8) | buf[6]);
    out->gyro_y  = (int16_t)((buf[9]  << 8) | buf[8]);
    out->gyro_z  = (int16_t)((buf[11] << 8) | buf[10]);
    return ESP_OK;
}

/* ============================================================
 *                      CALIBRACIÓN GIROSCOPO
 * ============================================================ */

esp_err_t bmi160_calibrate_gyro(int num_samples)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (num_samples <= 0) num_samples = 500;

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Calibrando giróscopo");
    ESP_LOGI(TAG, "Mantén el sensor COMPLETAMENTE QUIETO");
    ESP_LOGI(TAG, "=========================================");
    vTaskDelay(pdMS_TO_TICKS(2000));

    bmi160_raw_t d;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    int valid = 0;

    for (int i = 0; i < num_samples; i++) {
        if (bmi160_read_raw(&d) == ESP_OK) {
            sx += (double)d.gyro_x / s_gyro_scale;
            sy += (double)d.gyro_y / s_gyro_scale;
            sz += (double)d.gyro_z / s_gyro_scale;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (valid == 0) {
        ESP_LOGE(TAG, "No se pudo leer el sensor durante la calibración");
        return ESP_FAIL;
    }

    s_bias_x = (float)(sx / valid);
    s_bias_y = (float)(sy / valid);
    s_bias_z = (float)(sz / valid);

    ESP_LOGI(TAG, "Bias: X=%.2f  Y=%.2f  Z=%.2f (°/s)",
             s_bias_x, s_bias_y, s_bias_z);
    ESP_LOGI(TAG, "=========================================");
    return ESP_OK;
}

/* ============================================================
 *                      FILTRO + UPDATE
 * ============================================================ */

esp_err_t bmi160_update(float dt)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    bmi160_raw_t raw;
    esp_err_t err = bmi160_read_raw(&raw);
    if (err != ESP_OK) return err;

    /* Convertir a unidades físicas */
    float ax = raw.accel_x / s_accel_scale;
    float ay = raw.accel_y / s_accel_scale;
    float az = raw.accel_z / s_accel_scale;

    float gx = (raw.gyro_x / s_gyro_scale) - s_bias_x;
    float gy = (raw.gyro_y / s_gyro_scale) - s_bias_y;
    float gz = (raw.gyro_z / s_gyro_scale) - s_bias_z;

    /* --- Pitch / Roll desde el acelerómetro (solo si magnitud ≈ 1g) --- */
    float pitch_acc = 0.0f;
    float roll_acc  = 0.0f;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag > 0.8f && mag < 1.2f) {
        pitch_acc = atan2f(ay, az)  * 180.0f / (float)M_PI;
        roll_acc  = atan2f(-ax, az) * 180.0f / (float)M_PI;
    }

    /* Integración del giróscopo */
    s_pitch += gx * dt;
    s_roll  += gy * dt;

    /* Filtro complementario */
    s_pitch = s_cfg.filter_coeff * s_pitch +
              (1.0f - s_cfg.filter_coeff) * pitch_acc;
    s_roll  = s_cfg.filter_coeff * s_roll  +
              (1.0f - s_cfg.filter_coeff) * roll_acc;

    /* --- Yaw (solo giróscopo, con umbral anti-ruido) --- */
    if (fabsf(gz) > s_cfg.yaw_integration_threshold) {
        s_yaw += gz * dt;
    }
    if (s_yaw >= 360.0f) s_yaw -= 360.0f;
    if (s_yaw <    0.0f) s_yaw += 360.0f;

    return ESP_OK;
}

/* ============================================================
 *                      GETTERS DE ÁNGULOS
 * ============================================================ */

bmi160_angles_t bmi160_get_angles(void)
{
    bmi160_angles_t a;
    a.pitch = s_pitch;
    a.roll  = s_roll;
    a.yaw   = s_yaw;
    return a;
}

float bmi160_get_pitch(void) { return s_pitch; }
float bmi160_get_roll(void)  { return s_roll;  }
float bmi160_get_yaw(void)   { return s_yaw;   }

/* ============================================================
 *                      RESET DE YAW
 * ============================================================ */

void bmi160_reset_yaw(void)
{
    s_yaw = 0.0f;
    ESP_LOGI(TAG, "Yaw reseteado a 0°");
}

void bmi160_reset_yaw_to(float value)
{
    s_yaw = value;
    if (s_yaw >= 360.0f) s_yaw -= 360.0f;
    if (s_yaw <    0.0f) s_yaw += 360.0f;
    ESP_LOGI(TAG, "Yaw reseteado a %.1f°", s_yaw);
}

/* ============================================================
 *                      SETTERS EN RUNTIME
 * ============================================================ */

esp_err_t bmi160_set_accel_range(bmi160_accel_range_t range)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t err = bmi160_write_reg(BMI160_REG_ACC_RANGE, (uint8_t)range);
    if (err != ESP_OK) return err;

    s_cfg.accel_range = range;
    s_accel_scale = accel_scale_from_range(range);
    ESP_LOGI(TAG, "Accel range → 0x%02X (%.0f LSB/g)",
             (unsigned)range, s_accel_scale);
    return ESP_OK;
}

esp_err_t bmi160_set_gyro_range(bmi160_gyro_range_t range)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t err = bmi160_write_reg(BMI160_REG_GYR_RANGE, (uint8_t)range);
    if (err != ESP_OK) return err;

    s_cfg.gyro_range = range;
    s_gyro_scale = gyro_scale_from_range(range);
    ESP_LOGI(TAG, "Gyro range → 0x%02X (%.1f LSB/°/s)",
             (unsigned)range, s_gyro_scale);
    return ESP_OK;
}

esp_err_t bmi160_set_accel_odr(bmi160_odr_t odr)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    uint8_t v = (uint8_t)((BMI160_ACC_BWP_NORMAL << 4) | (odr & 0x0F));
    esp_err_t err = bmi160_write_reg(BMI160_REG_ACC_CONF, v);
    if (err != ESP_OK) return err;

    s_cfg.accel_odr = odr;
    ESP_LOGI(TAG, "Accel ODR → 0x%02X", (unsigned)odr);
    return ESP_OK;
}

esp_err_t bmi160_set_gyro_odr(bmi160_odr_t odr)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    uint8_t v = (uint8_t)((BMI160_GYR_BWP_NORMAL << 4) | (odr & 0x0F));
    esp_err_t err = bmi160_write_reg(BMI160_REG_GYR_CONF, v);
    if (err != ESP_OK) return err;

    s_cfg.gyro_odr = odr;
    ESP_LOGI(TAG, "Gyro ODR → 0x%02X", (unsigned)odr);
    return ESP_OK;
}

void bmi160_set_filter_coeff(float c)
{
    if (c < 0.0f) c = 0.0f;
    if (c > 1.0f) c = 1.0f;
    s_cfg.filter_coeff = c;
}

void bmi160_set_yaw_threshold(float t)
{
    if (t < 0.0f) t = 0.0f;
    s_cfg.yaw_integration_threshold = t;
}

/* ============================================================
 *                      GETTERS DE CONFIGURACIÓN
 * ============================================================ */

bmi160_accel_range_t bmi160_get_accel_range(void) { return s_cfg.accel_range; }
bmi160_gyro_range_t  bmi160_get_gyro_range(void)  { return s_cfg.gyro_range;  }
bmi160_odr_t         bmi160_get_accel_odr(void)   { return s_cfg.accel_odr;   }
bmi160_odr_t         bmi160_get_gyro_odr(void)    { return s_cfg.gyro_odr;    }

float bmi160_get_accel_scale(void) { return s_accel_scale; }
float bmi160_get_gyro_scale(void)  { return s_gyro_scale;  }