/**
 * @file    servo.c
 * @brief   Implementación del driver para servo motor usando LEDC PWM.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Implementa el control de un servo estándar (0-180°) mediante el
 *   periférico LEDC del ESP32, que genera la señal PWM de 50 Hz.
 *
 *   El servo se controla internamente mediante un ángulo (float, 0-180).
 *   Este ángulo se traduce a un ancho de pulso en microsegundos según
 *   los valores de calibración (s_pulse_min_us, s_pulse_max_us) y,
 *   finalmente, a un valor de duty cycle para el LEDC.
 *
 *   Además, ofrece movimiento suave mediante velocidad controlada
 *   (grados por segundo). Este modo requiere llamar periódicamente a
 *   servo_update() desde una tarea.
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "servo.h"

#include "driver/ledc.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include <math.h>

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "SERVO";

/** Timer LEDC utilizado por el servo. */
#define LEDC_TIMER      LEDC_TIMER_0

/** Canal LEDC utilizado por el servo. */
#define LEDC_CHANNEL    LEDC_CHANNEL_0

/** Modo de velocidad del LEDC. */
#define LEDC_MODE       LEDC_LOW_SPEED_MODE

/** Delta máximo de tiempo (s) entre actualizaciones para evitar saltos. */
#define SERVO_MAX_DT    0.1f

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/** GPIO configurado para el servo. */
static uint8_t  s_gpio_num        = SERVO_DEFAULT_GPIO;

/** Frecuencia PWM configurada (Hz). */
static uint32_t s_freq_hz         = SERVO_DEFAULT_FREQ;

/** Resolución del LEDC en bits. */
static uint8_t  s_resolution_bits = SERVO_DEFAULT_RES;

/** Pulso mínimo (0°) en microsegundos. */
static uint16_t s_pulse_min_us    = SERVO_PULSE_MIN_US;

/** Pulso máximo (180°) en microsegundos. */
static uint16_t s_pulse_max_us    = SERVO_PULSE_MAX_US;

/** Bandera que indica si el servo está inicializado. */
static bool     s_initialized     = false;

/** Ángulo actual del servo (grados). */
static float    s_current_angle   = SERVO_ANGLE_CENTER;

/** Ángulo objetivo del servo (grados). */
static float    s_target_angle    = SERVO_ANGLE_CENTER;

/** Velocidad de movimiento (°/s). 0 = sin movimiento suave. */
static float    s_speed           = 0.0f;

/** Timestamp de la última actualización (ms). */
static uint32_t s_last_update_time = 0;

/* =====================================================================
 *  FUNCIONES AUXILIARES
 * ===================================================================== */

/**
 * @brief  Convierte un ancho de pulso en microsegundos a duty cycle.
 *
 * @param[in] pulse_us  Ancho de pulso en microsegundos.
 *
 * @return Duty cycle correspondiente (0 a 2^resolución - 1).
 */
static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    uint32_t period_us = 1000000 / s_freq_hz;
    return (uint32_t)(((uint64_t)pulse_us * ((1 << s_resolution_bits) - 1))
                      / period_us);
}

/**
 * @brief  Convierte un ángulo (0-180°) a ancho de pulso en microsegundos.
 *
 * El ángulo se satura al rango [SERVO_ANGLE_MIN, SERVO_ANGLE_MAX].
 *
 * @param[in] angle  Ángulo en grados (0-180).
 *
 * @return Ancho de pulso en microsegundos.
 */
static uint16_t angle_to_pulse_us(float angle)
{
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    return s_pulse_min_us +
           (uint16_t)((angle - SERVO_ANGLE_MIN) *
                      (s_pulse_max_us - s_pulse_min_us) /
                      (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN));
}

/**
 * @brief  Convierte un ancho de pulso en microsegundos a ángulo.
 *
 * @param[in] pulse_us  Ancho de pulso en microsegundos.
 *
 * @return Ángulo en grados, saturado al rango [0, 180].
 */
static float pulse_us_to_angle(uint16_t pulse_us)
{
    if (pulse_us <= s_pulse_min_us) return SERVO_ANGLE_MIN;
    if (pulse_us >= s_pulse_max_us) return SERVO_ANGLE_MAX;

    return SERVO_ANGLE_MIN +
           (float)(pulse_us - s_pulse_min_us) *
           (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN) /
           (s_pulse_max_us - s_pulse_min_us);
}

/**
 * @brief  Aplica el ángulo actual del servo al hardware (LEDC).
 *
 * Traduce s_current_angle a un duty cycle y lo escribe en el canal
 * LEDC del servo.
 */
static void servo_apply_angle(void)
{
    if (!s_initialized) return;

    uint16_t pulse_us = angle_to_pulse_us(s_current_angle);
    uint32_t duty     = pulse_us_to_duty(pulse_us);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    ESP_LOGD(TAG, "Ángulo: %.1f°, Pulso: %d us, Duty: %d",
             s_current_angle, pulse_us, duty);
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el servo en el GPIO por defecto.
 *
 * Usa la configuración por defecto: 50 Hz, 12 bits y SERVO_DEFAULT_GPIO.
 *
 * @return
 *   - true  si la inicialización fue exitosa.
 *   - false en caso contrario.
 */
bool servo_init(void)
{
    return servo_init_gpio(SERVO_DEFAULT_GPIO);
}

/**
 * @brief  Inicializa el servo en un GPIO específico.
 *
 * Usa la frecuencia y resolución por defecto (50 Hz, 12 bits).
 *
 * @param[in] gpio_num  Número del GPIO donde está conectado el servo.
 *
 * @return
 *   - true  si la inicialización fue exitosa.
 *   - false en caso contrario.
 */
bool servo_init_gpio(uint8_t gpio_num)
{
    return servo_init_custom(gpio_num, SERVO_DEFAULT_FREQ, SERVO_DEFAULT_RES);
}

/**
 * @brief  Inicializa el servo con configuración personalizada.
 *
 * Configura el timer LEDC con la frecuencia y resolución indicadas y
 * el canal LEDC sobre el GPIO del servo. Al terminar, posiciona el
 * servo en el ángulo central (90°).
 *
 * @param[in] gpio_num         Número del GPIO donde está conectado el servo.
 * @param[in] freq_hz          Frecuencia en Hz (típicamente 50 Hz para servos).
 * @param[in] resolution_bits  Resolución en bits (típicamente 12).
 *
 * @return
 *   - true  si la inicialización fue exitosa.
 *   - false en caso contrario.
 */
bool servo_init_custom(uint8_t gpio_num, uint32_t freq_hz,
                       uint8_t resolution_bits)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Servo ya inicializado");
        return true;
    }

    s_gpio_num        = gpio_num;
    s_freq_hz         = freq_hz;
    s_resolution_bits = resolution_bits;

    ESP_LOGI(TAG, "Inicializando servo en GPIO %d, %lu Hz, %d bits",
             s_gpio_num, s_freq_hz, s_resolution_bits);

    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = s_resolution_bits,
        .freq_hz         = s_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando temporizador LEDC: %s",
                 esp_err_to_name(ret));
        return false;
    }

    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .gpio_num   = s_gpio_num,
        .duty       = 0,
        .hpoint     = 0,
    };

    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal LEDC: %s",
                 esp_err_to_name(ret));
        return false;
    }

    s_initialized   = true;
    s_current_angle = SERVO_ANGLE_CENTER;
    s_target_angle  = SERVO_ANGLE_CENTER;

    servo_set_angle(SERVO_ANGLE_CENTER);

    ESP_LOGI(TAG, "Servo inicializado correctamente");
    return true;
}

/**
 * @brief  Mueve el servo a un ángulo específico.
 *
 * El ángulo se satura al rango [SERVO_ANGLE_MIN, SERVO_ANGLE_MAX]. El
 * movimiento es inmediato (no usa velocidad controlada).
 *
 * @param[in] angle  Ángulo en grados (0 a 180).
 *
 * @note Los valores fuera de rango se saturan automáticamente.
 */
void servo_set_angle(float angle)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Servo no inicializado");
        return;
    }

    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    s_current_angle = angle;
    s_target_angle  = angle;
    s_speed         = 0;

    servo_apply_angle();
    ESP_LOGI(TAG, "Servo movido a %.1f°", angle);
}

/**
 * @brief  Mueve el servo a un ángulo objetivo con velocidad controlada.
 *
 * Configura el estado interno para que servo_update() vaya acercando
 * el ángulo actual al objetivo a la velocidad indicada.
 *
 * @param[in] target_angle  Ángulo objetivo (0 a 180 grados).
 * @param[in] speed         Velocidad de movimiento (grados por segundo).
 *
 * @note Esta función es no bloqueante; debe llamarse periódicamente a
 *       servo_update() para que el movimiento progrese.
 */
void servo_set_angle_with_speed(float target_angle, float speed)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Servo no inicializado");
        return;
    }

    if (target_angle < SERVO_ANGLE_MIN) target_angle = SERVO_ANGLE_MIN;
    if (target_angle > SERVO_ANGLE_MAX) target_angle = SERVO_ANGLE_MAX;

    s_target_angle     = target_angle;
    s_speed            = speed;
    s_last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "Movimiento iniciado: %.1f° -> %.1f° a %.1f°/s",
             s_current_angle, s_target_angle, s_speed);
}

/**
 * @brief  Obtiene el ángulo actual del servo.
 *
 * @return Ángulo actual en grados (0 a 180).
 */
float servo_get_angle(void)
{
    return s_current_angle;
}

/**
 * @brief  Actualiza el servo.
 *
 * Calcula el incremento de ángulo en función del tiempo transcurrido
 * y de la velocidad configurada, y aplica el nuevo ángulo al hardware.
 *
 * Debe llamarse periódicamente para que el movimiento con velocidad
 * controlada progrese.
 *
 * @note Sólo es necesario si se usa servo_set_angle_with_speed().
 */
void servo_update(void)
{
    if (!s_initialized) return;
    if (s_speed <= 0)   return;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float    dt  = (now - s_last_update_time) / 1000.0f;
    s_last_update_time = now;

    /* Limitar dt para evitar saltos tras pausas largas */
    if (dt > SERVO_MAX_DT) dt = SERVO_MAX_DT;

    float diff     = s_target_angle - s_current_angle;
    float max_step = s_speed * dt;

    if (fabsf(diff) <= max_step) {
        /* Hemos llegado (o superado) el objetivo */
        s_current_angle = s_target_angle;
        s_speed         = 0;
    } else {
        s_current_angle += (diff > 0) ? max_step : -max_step;
    }

    servo_apply_angle();
}

/**
 * @brief  Establece el pulso directamente en microsegundos.
 *
 * Traduce el pulso a un ángulo equivalente y lo aplica al servo.
 * Útil para calibración o control fino.
 *
 * @param[in] pulse_us  Ancho de pulso en microsegundos (típicamente 500-2500).
 */
void servo_set_pulse_us(uint16_t pulse_us)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Servo no inicializado");
        return;
    }

    s_current_angle = pulse_us_to_angle(pulse_us);
    s_target_angle  = s_current_angle;
    s_speed         = 0;

    uint32_t duty = pulse_us_to_duty(pulse_us);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    ESP_LOGI(TAG, "Pulso establecido: %d us (ángulo: %.1f°)",
             pulse_us, s_current_angle);
}

/**
 * @brief  Calibra los pulsos mínimo y máximo del servo.
 *
 * Actualiza los valores de calibración y reaplica el ángulo actual
 * para que el cambio tenga efecto inmediato.
 *
 * @param[in] min_us  Pulso correspondiente a 0 grados (microsegundos).
 * @param[in] max_us  Pulso correspondiente a 180 grados (microsegundos).
 */
void servo_calibrate(uint16_t min_us, uint16_t max_us)
{
    s_pulse_min_us = min_us;
    s_pulse_max_us = max_us;
    ESP_LOGI(TAG, "Servo calibrado: min=%d us, max=%d us", min_us, max_us);

    servo_apply_angle();
}

/**
 * @brief  Desinicializa el servo y libera los recursos asociados.
 *
 * Detiene el canal LEDC y reinicia el timer. Después de llamar a esta
 * función es necesario volver a llamar a alguna de las funciones
 * servo_init*() antes de usar el servo.
 */
void servo_deinit(void)
{
    if (!s_initialized) return;

    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_timer_rst(LEDC_MODE, LEDC_TIMER);

    s_initialized = false;
    ESP_LOGI(TAG, "Servo desinicializado");
}