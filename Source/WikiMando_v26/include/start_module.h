#ifndef START_MODULE_H_
#define START_MODULE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Direcciones fijas del protocolo Start Module
#define START_ADDR_START_STOP   0x07
#define START_ADDR_PROGRAM      0x0B

// Configuración del módulo (incluye LED de estado)
typedef struct {
    uint8_t ir_led_gpio;            // GPIO del LED infrarrojo (se pasa a rc5)
    uint8_t status_led_gpio;        // GPIO del LED de estado (-1 = no usar)
    bool status_led_active_high;    // true = 1 enciende, false = 0 enciende
    // Parámetros RC5 (se pasan a la librería rc5)
    uint32_t rc5_resolution_hz;      // 1e6
    uint32_t rc5_carrier_freq_hz;   // 38000
    float rc5_carrier_duty_cycle;   // 0.5
} start_module_config_t;

// Inicialización general (RC5 + LED de estado)
bool start_module_init(const start_module_config_t *config);
void start_module_deinit(void);

// Funciones específicas del Start Module
bool start_module_program(uint8_t dohyo);          // envía programación (addr 0x0B)
bool start_module_send_start(uint8_t dohyo);       // envía start (addr 0x07)
bool start_module_send_stop(uint8_t dohyo);        // envía stop (addr 0x07)

// Control directo del LED de estado (si se necesita)
void start_module_led_on(void);
void start_module_led_off(void);
void start_module_led_toggle(void);

// Utilidades de conversión (opcionales)
uint8_t start_module_stop_cmd_from_dohyo(uint8_t dohyo);
uint8_t start_module_start_cmd_from_dohyo(uint8_t dohyo);

#ifdef __cplusplus
}
#endif

#endif