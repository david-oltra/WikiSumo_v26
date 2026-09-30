#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <driver/i2c.h>

typedef struct {
    uint8_t screen;
    uint8_t level;
    uint8_t sublevel;
    uint8_t dohyo;
    uint8_t strategy;
    uint8_t disable_toff;
    uint8_t disable_qre;
    uint8_t calibrate_qre;
} display_t;

// Inicializa la pantalla: necesita puerto I2C, pines SDA y SCL
void display_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

// Actualiza la pantalla según el estado del menú
void display_update(display_t *menu);

// Manejadores de pulsadores (sin cambios)
void display_handle_up(void *arg);
void display_handle_down(void *arg);
void display_handle_push(void *arg);

// Obtener valores seleccionados
uint8_t display_get_dohyo(display_t *menu);
uint8_t display_get_strategy(display_t *menu);

#endif