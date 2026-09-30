#ifndef SWITCH_H
#define SWITCH_H

#include <driver/gpio.h>

// Tipo de callback (recibe un argumento opcional)
typedef void (*switch_callback_t)(void *arg);

// Inicializa los pulsadores con los pines que se indiquen
void switch_init(gpio_num_t up_pin, gpio_num_t push_pin, gpio_num_t down_pin);

// Registra múltiples callbacks para cada botón
void switch_register_up(switch_callback_t cb, void *arg);
void switch_register_down(switch_callback_t cb, void *arg);
void switch_register_push(switch_callback_t cb, void *arg);

#endif