#include "start_module.h"
#include "rc5.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "START_MODULE";
static uint8_t status_led_pin = -1;
static bool led_active_high = true;
static bool initialized = false;

bool start_module_init(const start_module_config_t *config) {
    if (!config) return false;

    // Inicializar LED de estado si se especificó
    status_led_pin = config->status_led_gpio;
    led_active_high = config->status_led_active_high;
    if (status_led_pin != (uint8_t)-1 && status_led_pin < GPIO_NUM_MAX) {
        gpio_config_t led_conf = {
            .pin_bit_mask = (1ULL << status_led_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&led_conf);
        start_module_led_off();  // apagado inicial
        ESP_LOGI(TAG, "LED de estado configurado en GPIO %d", status_led_pin);
    } else {
        status_led_pin = -1;
        ESP_LOGI(TAG, "LED de estado no utilizado");
    }

    // Inicializar librería RC5
    rc5_tx_config_t rc5_cfg = {
        .tx_gpio = config->ir_led_gpio,
        .resolution_hz = config->rc5_resolution_hz,
        .carrier_freq_hz = config->rc5_carrier_freq_hz,
        .carrier_duty_cycle = config->rc5_carrier_duty_cycle,
    };
    if (!rc5_tx_init(&rc5_cfg)) {
        ESP_LOGE(TAG, "Fallo al inicializar RC5");
        return false;
    }

    initialized = true;
    ESP_LOGI(TAG, "Start Module inicializado correctamente");
    return true;
}

void start_module_deinit(void) {
    if (!initialized) return;
    rc5_tx_deinit();
    if (status_led_pin != (uint8_t)-1) {
        // opcional: dejar el LED apagado
        start_module_led_off();
        gpio_reset_pin(status_led_pin);
        status_led_pin = -1;
    }
    initialized = false;
    ESP_LOGI(TAG, "Start Module desinicializado");
}

// Comandos de alto nivel
bool start_module_program(uint8_t dohyo) {
    uint8_t stop_cmd = start_module_stop_cmd_from_dohyo(dohyo);
    return rc5_send_raw(START_ADDR_PROGRAM, stop_cmd, 3, true);
}

bool start_module_send_start(uint8_t dohyo) {
    uint8_t start_cmd = start_module_start_cmd_from_dohyo(dohyo);
    return rc5_send_raw(START_ADDR_START_STOP, start_cmd, 3, true);
}

bool start_module_send_stop(uint8_t dohyo) {
    uint8_t stop_cmd = start_module_stop_cmd_from_dohyo(dohyo);
    return rc5_send_raw(START_ADDR_START_STOP, stop_cmd, 3, true);
}

// Control LED
void start_module_led_on(void) {
    if (status_led_pin == (uint8_t)-1) return;
    gpio_set_level(status_led_pin, led_active_high ? 1 : 0);
}

void start_module_led_off(void) {
    if (status_led_pin == (uint8_t)-1) return;
    gpio_set_level(status_led_pin, led_active_high ? 0 : 1);
}

void start_module_led_toggle(void) {
    if (status_led_pin == (uint8_t)-1) return;
    bool current = gpio_get_level(status_led_pin);
    gpio_set_level(status_led_pin, !current);
}

// Utilidades
uint8_t start_module_stop_cmd_from_dohyo(uint8_t dohyo) {
    return dohyo * 2;
}

uint8_t start_module_start_cmd_from_dohyo(uint8_t dohyo) {
    return start_module_stop_cmd_from_dohyo(dohyo) + 1;
}