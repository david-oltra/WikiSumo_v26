/**
 * @file pid.h
 * @brief Controlador PID para el robot autobalanceado
 * @author david_wiki
 * @date 2026
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================
// ESTRUCTURAS
// ============================================================

/**
 * @brief Estructura para el controlador PID
 */
typedef struct {
    float kp;               //!< Ganancia proporcional
    float ki;               //!< Ganancia integral
    float kd;               //!< Ganancia derivativa
    float setpoint;         //!< Valor objetivo (referencia)
    
    float output;           //!< Salida del PID (-100 a 100)
    float integral_error;   //!< Error acumulado (término integral)
    float last_error;       //!< Error anterior (término derivativo)
    
    float output_min;       //!< Límite mínimo de salida
    float output_max;       //!< Límite máximo de salida
    float integral_limit;   //!< Límite para anti-windup
    
    uint32_t last_time;     //!< Último tiempo de actualización (ms)
} pid_controller_t;

// ============================================================
// PROTOTIPOS DE FUNCIONES
// ============================================================

/**
 * @brief Inicializa el controlador PID
 * @param pid Puntero a la estructura PID
 * @param kp Ganancia proporcional
 * @param ki Ganancia integral
 * @param kd Ganancia derivativa
 * @param setpoint Valor objetivo
 * @param output_min Límite mínimo de salida
 * @param output_max Límite máximo de salida
 * @param integral_limit Límite para anti-windup
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, 
              float setpoint, float output_min, float output_max, 
              float integral_limit);

/**
 * @brief Resetea el controlador PID
 * @param pid Puntero a la estructura PID
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief Actualiza el controlador PID con cálculo automático de dt
 * @param pid Puntero a la estructura PID
 * @param current_value Valor actual medido (feedback)
 * @param current_time Tiempo actual en milisegundos
 * @return Salida del controlador PID
 */
float pid_update(pid_controller_t *pid, float current_value, uint32_t current_time);

/**
 * @brief Cambia el setpoint (valor objetivo) del PID
 * @param pid Puntero a la estructura PID
 * @param setpoint Nuevo valor objetivo
 */
void pid_set_setpoint(pid_controller_t *pid, float setpoint);

/**
 * @brief Cambia las ganancias del PID en tiempo real
 * @param pid Puntero a la estructura PID
 * @param kp Nueva ganancia proporcional
 * @param ki Nueva ganancia integral
 * @param kd Nueva ganancia derivativa
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Obtiene el error actual
 * @param pid Puntero a la estructura PID
 * @param current_value Valor actual medido
 * @return Error actual (setpoint - current_value)
 */
float pid_get_error(pid_controller_t *pid, float current_value);

/**
 * @brief Verifica si la salida está saturada
 * @param pid Puntero a la estructura PID
 * @return true si la salida está en límite, false en caso contrario
 */
bool pid_is_saturated(pid_controller_t *pid);

#endif // PID_H