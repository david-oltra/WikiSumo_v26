/**
 * @file    tmc2209.h
 * @brief   Driver para el controlador de motor paso a paso TMC2209.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   El TMC2209 puede controlarse de dos formas complementarias:
 *
 *     1) Modo STEP/DIR clásico, usando RMT para generar los pulsos
 *        STEP de forma precisa y no bloqueante (tmc2209_move_motor).
 *
 *     2) Modo VACTUAL por UART, usando el generador interno del chip.
 *        Permite movimiento continuo a velocidad constante sin ocupar
 *        la CPU (tmc2209_set_velocity).
 *
 *   Además, el módulo expone utilidades para:
 *     - Configurar corriente RMS, microsteps y otros registros.
 *     - Leer registros internos (DRV_STATUS, CHOPCONF, etc.).
 *     - Realizar diagnósticos rápidos de comunicación y estado.
 *
 *   La UART es compartida entre todos los drivers (direccionables por
 *   uart_addr), por lo que debe inicializarse una sola vez con
 *   tmc2209_init_uart().
 */

#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  TIPOS
 * ===================================================================== */

/**
 * @brief Estructura que representa un motor controlado por TMC2209.
 *
 * Contiene toda la configuración de pines, comunicación UART y recursos
 * RMT asociados al motor.
 */
typedef struct {
    uint8_t step_pin;                   /*!< Pin GPIO para la señal STEP                          */
    uint8_t dir_pin;                    /*!< Pin GPIO para la señal DIR                           */
    uint8_t uart_addr;                  /*!< Dirección UART del driver (0x00-0x03)                */
    uint8_t uart_num;                   /*!< Número de periférico UART (UART_NUM_0, UART_NUM_1..) */
    uint8_t tx_pin;                     /*!< Pin TX del ESP32 (conectado a PDN_UART)              */
    uint8_t rx_pin;                     /*!< Pin RX del ESP32 (conectado a PDN_UART)              */

    rmt_channel_handle_t rmt_chan;      /*!< Canal RMT para generar pulsos STEP                   */
    rmt_encoder_handle_t rmt_encoder;   /*!< Encoder RMT para copiar símbolos                     */
    void                *rmt_done_queue;/*!< Cola FreeRTOS para notificar fin de TX RMT           */
    bool                 initialized;   /*!< true si el motor ha sido inicializado correctamente  */

    /* --- Estado para control por VACTUAL (UART) --- */
    bool    vactual_active;             /*!< true si el motor se mueve por VACTUAL                */
    int32_t current_velocity;           /*!< Velocidad actual (µsteps/s, signo = sentido)         */
} tmc2209_t;

/* =====================================================================
 *  INICIALIZACIÓN DEL SISTEMA
 * ===================================================================== */

/**
 * @brief  Inicializa el periférico UART compartido por todos los drivers.
 *
 * Debe llamarse una sola vez antes de inicializar cualquier motor.
 *
 * @param[in] uart_num  Número de UART (p. ej. UART_NUM_0).
 * @param[in] tx_pin    Pin TX del ESP32.
 * @param[in] rx_pin    Pin RX del ESP32.
 */
void tmc2209_init_uart(uint8_t uart_num, uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief  Inicializa un motor específico.
 *
 * Configura pines DIR, crea la cola RMT y registra el callback de fin
 * de transmisión.
 *
 * @param[in,out] motor  Puntero a la estructura tmc2209_t del motor.
 */
void tmc2209_init_motor(tmc2209_t *motor);

/* =====================================================================
 *  MOVIMIENTO Y DIRECCIÓN (STEP/DIR con RMT)
 * ===================================================================== */

/**
 * @brief  Genera una ráfaga de pulsos STEP usando RMT.
 *
 * No bloqueante a nivel de CPU, pero la función espera a que termine
 * la transmisión antes de retornar.
 *
 * @param[in,out] motor     Puntero al motor.
 * @param[in]     steps     Número de pasos a generar (positivo).
 * @param[in]     speed_hz  Velocidad en pasos por segundo (Hz).
 */
void tmc2209_move_motor(tmc2209_t *motor, int steps, int speed_hz);

/**
 * @brief  Cambia la dirección de giro del motor en modo STEP/DIR.
 *
 * NOTA: este pin se ignora cuando el motor se controla por VACTUAL.
 *
 * @param[in,out] motor    Puntero al motor.
 * @param[in]     reverse  true = reversa, false = adelante.
 */
void tmc2209_set_direction(tmc2209_t *motor, bool reverse);

/* =====================================================================
 *  MOVIMIENTO CONTINUO POR VACTUAL (UART)
 * ===================================================================== */

/**
 * @brief  Mueve el motor de forma continua usando el generador interno
 *         del TMC2209 (registro VACTUAL).
 *
 * La velocidad se mantiene indefinidamente hasta enviar otro valor,
 * enviar 0, o deshabilitar el driver.
 *
 * @param[in,out] motor     Puntero al motor.
 * @param[in]     speed_hz  Velocidad en microsteps/segundo:
 *                            - > 0 : un sentido.
 *                            - < 0 : sentido inverso.
 *                            - = 0 : parada inmediata.
 *
 * @note
 *   - Al usar VACTUAL, el pin DIR se ignora: el sentido lo marca el signo.
 *   - La parada es inmediata, sin rampa. Para suavidad usar
 *     tmc2209_ramp_velocity().
 */
void tmc2209_set_velocity(tmc2209_t *motor, int32_t speed_hz);

/**
 * @brief  Detiene el motor cuando se está controlando por VACTUAL.
 *
 * Escribe 0 en el registro 0x22 (VACTUAL). La parada es inmediata.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_stop(tmc2209_t *motor);

/**
 * @brief  Realiza una rampa lineal de velocidad hasta el valor objetivo.
 *
 * Útil para arrancar y detener suavemente sin perder pasos.
 *
 * @param[in,out] motor      Puntero al motor.
 * @param[in]     target_hz  Velocidad final en µsteps/s (puede ser negativa).
 * @param[in]     step_hz    Incremento por iteración (siempre positivo).
 * @param[in]     delay_ms   Tiempo entre incrementos en milisegundos.
 */
void tmc2209_ramp_velocity(tmc2209_t *motor, int32_t target_hz,
                           int32_t step_hz, uint32_t delay_ms);

/**
 * @brief  Detiene el motor con una rampa de desaceleración hasta 0.
 *
 * @param[in,out] motor      Puntero al motor.
 * @param[in]     step_hz    Decremento por iteración (positivo).
 * @param[in]     delay_ms   Tiempo entre decrementos en milisegundos.
 */
void tmc2209_stop_ramp(tmc2209_t *motor, int32_t step_hz, uint32_t delay_ms);

/* =====================================================================
 *  CONFIGURACIÓN UART Y REGISTROS
 * ===================================================================== */

/**
 * @brief  Habilita el modo UART en el driver (activa bits 6 y 7 de GCONF).
 *
 * Debe llamarse después de que la comunicación UART esté establecida.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_configure_uart(tmc2209_t *motor);

/**
 * @brief  Establece la corriente RMS del motor (en miliamperios).
 *
 * Actualiza el registro IHOLD_IRUN (0x10).
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     mA     Corriente deseada en mA (0-2000 aprox.).
 */
void tmc2209_set_current(tmc2209_t *motor, uint16_t mA);

/**
 * @brief  Obtiene la corriente real que el driver está entregando al motor.
 *
 * Lee DRV_STATUS (0x6F) y extrae el campo CS_ACTUAL.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return Corriente en mA (aproximada).
 */
uint16_t tmc2209_get_current(tmc2209_t *motor);

/**
 * @brief  Configura la resolución de microsteps.
 *
 * Actualiza los bits MRES del registro CHOPCONF (0x6C).
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     steps  Microsteps: 1, 2, 4, 8, 16, 32, 64, 128, 256.
 */
void tmc2209_set_microsteps(tmc2209_t *motor, uint16_t steps);

/**
 * @brief  Lee la configuración actual de microsteps.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return Número de microsteps (1, 2, 4, ... 256).
 */
uint16_t tmc2209_get_microsteps(tmc2209_t *motor);

/**
 * @brief  Envía una secuencia predefinida de comandos de configuración.
 *
 * Secuencia extraída originalmente de un analizador lógico. Útil para
 * una inicialización rápida del driver.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_config_init(tmc2209_t *motor);

/* =====================================================================
 *  ESCRITURA / LECTURA DIRECTA DE REGISTROS
 * ===================================================================== */

/**
 * @brief  Escribe un valor de 32 bits en un registro del TMC2209.
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     reg    Dirección del registro (0x00 - 0x7F).
 * @param[in]     value  Valor a escribir.
 */
void tmc2209_write(tmc2209_t *motor, uint8_t reg, uint32_t value);

/**
 * @brief  Lee un registro del TMC2209 y devuelve su valor de 32 bits.
 *
 * @param[in] motor  Puntero al motor.
 * @param[in] reg    Dirección del registro.
 *
 * @return Valor leído, o 0 si falla la comunicación.
 */
uint32_t tmc2209_read(tmc2209_t *motor, uint8_t reg);

/* =====================================================================
 *  DIAGNÓSTICO Y ESTADO
 * ===================================================================== */

/**
 * @brief  Prueba la comunicación con el driver leyendo DRV_STATUS.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return 1 si hay respuesta, 0 si no.
 */
uint8_t tmc2209_test_communication(tmc2209_t *motor);

/**
 * @brief  Imprime por consola el estado completo del driver (DRV_STATUS).
 *
 * @param[in] motor  Puntero al motor.
 */
void tmc2209_status(tmc2209_t *motor);

/**
 * @brief  Ejecuta un diagnóstico completo del motor.
 *
 * Comprueba comunicación, estado, corriente en reposo y en movimiento.
 * Mueve el motor 5000 pasos a 1000 Hz para medir IRUN.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_run_full_diagnostic(tmc2209_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_H */