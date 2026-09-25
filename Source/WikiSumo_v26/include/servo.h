/**
 * @file    servo.h
 * @brief   Driver para controlar un servo motor usando LEDC PWM.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Proporciona una API sencilla para controlar servos estándar
 *   (0-180°) mediante el periférico LEDC del ESP32, que genera la
 *   señal PWM de 50 Hz necesaria.
 *
 *   Características:
 *     - Inicialización con GPIO, frecuencia y resolución configurables.
 *     - Control directo por ángulo o por pulso (µs).
 *     - Movimiento suave mediante velocidad controlada (no bloqueante).
 *     - Calibración de los pulsos mínimo y máximo.
 *
 *   El rango de pulsos por defecto es 500-2500 µs para 0-180°.
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  CONFIGURACIÓN POR DEFECTO
 * ===================================================================== */

/** GPIO por defecto para el servo. */
#define SERVO_DEFAULT_GPIO     6

/** Frecuencia base para servos (50 Hz). */
#define SERVO_DEFAULT_FREQ     50

/** Resolución por defecto: 12 bits (0-4095). */
#define SERVO_DEFAULT_RES      12

/** Pulso mínimo (0 grados) en microsegundos. */
#define SERVO_PULSE_MIN_US     500

/** Pulso máximo (180 grados) en microsegundos. */
#define SERVO_PULSE_MAX_US     2500

/** Ángulo mínimo (grados). */
#define SERVO_ANGLE_MIN        0

/** Ángulo máximo (grados). */
#define SERVO_ANGLE_MAX        180

/** Ángulo central (grados). */
#define SERVO_ANGLE_CENTER     90

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
bool servo_init(void);

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
bool servo_init_gpio(uint8_t gpio_num);

/**
 * @brief  Inicializa el servo con configuración personalizada.
 *
 * @param[in] gpio_num        Número del GPIO donde está conectado el servo.
 * @param[in] freq_hz         Frecuencia en Hz (típicamente 50 Hz para servos).
 * @param[in] resolution_bits Resolución en bits (típicamente 12).
 *
 * @return
 *   - true  si la inicialización fue exitosa.
 *   - false en caso contrario.
 */
bool servo_init_custom(uint8_t gpio_num, uint32_t freq_hz,
                       uint8_t resolution_bits);

/**
 * @brief  Mueve el servo a un ángulo específico.
 *
 * @param[in] angle  Ángulo en grados (0 a 180).
 *
 * @note Los valores fuera de rango se saturan automáticamente.
 */
void servo_set_angle(float angle);

/**
 * @brief  Mueve el servo a un ángulo objetivo con velocidad controlada.
 *
 * @param[in] target_angle  Ángulo objetivo (0 a 180 grados).
 * @param[in] speed         Velocidad de movimiento (grados por segundo).
 *
 * @note Esta función es no bloqueante; debe llamarse periódicamente a
 *       servo_update() para que el movimiento progrese.
 */
void servo_set_angle_with_speed(float target_angle, float speed);

/**
 * @brief  Obtiene el ángulo actual del servo.
 *
 * @return Ángulo actual en grados (0 a 180).
 */
float servo_get_angle(void);

/**
 * @brief  Actualiza el servo.
 *
 * Debe llamarse periódicamente para que el movimiento con velocidad
 * controlada progrese.
 *
 * @note Sólo es necesario si se usa servo_set_angle_with_speed().
 */
void servo_update(void);

/**
 * @brief  Establece el pulso directamente en microsegundos.
 *
 * @param[in] pulse_us  Ancho de pulso en microsegundos (típicamente 500-2500).
 */
void servo_set_pulse_us(uint16_t pulse_us);

/**
 * @brief  Calibra los pulsos mínimo y máximo del servo.
 *
 * @param[in] min_us  Pulso correspondiente a 0 grados (microsegundos).
 * @param[in] max_us  Pulso correspondiente a 180 grados (microsegundos).
 */
void servo_calibrate(uint16_t min_us, uint16_t max_us);

/**
 * @brief  Desinicializa el servo y libera los recursos asociados.
 */
void servo_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */