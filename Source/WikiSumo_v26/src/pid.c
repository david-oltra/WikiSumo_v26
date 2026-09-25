/**
 * @file pid.c
 * @brief Implementación del controlador PID
 * @author david_wiki
 * @date 2026
 */

#include "pid.h"
#include <math.h>

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, 
              float setpoint, float output_min, float output_max, 
              float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_limit = integral_limit;
    
    pid->output = 0.0f;
    pid->integral_error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_time = 0;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral_error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->last_time = 0;
}

float pid_update(pid_controller_t *pid, float current_value, uint32_t current_time)
{
    // Calcular delta time
    float dt = 0.01f;  // Valor por defecto (10ms)
    
    if (pid->last_time > 0) {
        dt = (current_time - pid->last_time) / 1000.0f;
        
        // Limitar dt para evitar valores extremos
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.001f) dt = 0.01f;
    }
    
    pid->last_time = current_time;
    
    // Calcular error
    float error = pid->setpoint - current_value;
    
    // Término integral con anti-windup
    pid->integral_error += error * dt;
    if (pid->integral_error > pid->integral_limit) {
        pid->integral_error = pid->integral_limit;
    }
    if (pid->integral_error < -pid->integral_limit) {
        pid->integral_error = -pid->integral_limit;
    }
    
    // Término derivativo
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    
    // Calcular salida PID
    float output = pid->kp * error + pid->ki * pid->integral_error + pid->kd * derivative;
    
    // Limitar salida
    if (output > pid->output_max) {
        output = pid->output_max;
    }
    if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    pid->output = output;
    return output;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_get_error(pid_controller_t *pid, float current_value)
{
    return pid->setpoint - current_value;
}

bool pid_is_saturated(pid_controller_t *pid)
{
    return (pid->output >= pid->output_max || pid->output <= pid->output_min);
}