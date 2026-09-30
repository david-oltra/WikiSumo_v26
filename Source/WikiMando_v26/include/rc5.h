#ifndef RC5_H_
#define RC5_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constantes del protocolo RC5
#define RC5_BIT_TIME_US          889
#define RC5_NUM_BITS             14
#define RC5_REPEAT_DELAY_MS      114

// Configuración del transmisor
typedef struct {
    uint8_t tx_gpio;                // GPIO del LED infrarrojo
    uint32_t resolution_hz;         // Resolución RMT (1e6 = 1µs)
    uint32_t carrier_freq_hz;       // 38000 Hz
    float carrier_duty_cycle;       // 0.5
} rc5_tx_config_t;

// Inicialización y desinicialización del transmisor RC5
bool rc5_tx_init(const rc5_tx_config_t *config);
void rc5_tx_deinit(void);

// Envío de tramas
bool rc5_send_raw(uint8_t address, uint8_t command, int repeat, bool flip_toggle);

// Control del toggle bit (público para casos especiales)
uint8_t rc5_get_toggle(void);
void rc5_reset_toggle(void);

#ifdef __cplusplus
}
#endif

#endif