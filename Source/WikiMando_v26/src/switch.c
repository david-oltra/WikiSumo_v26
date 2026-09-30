#include "switch.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define MAX_CALLBACKS   5

static const char *TAG = "SWITCH";

static gpio_num_t pin_up, pin_push, pin_down;   // guardar pines configurados

typedef struct {
    switch_callback_t cb;
    void *arg;
} callback_entry_t;

static callback_entry_t up_callbacks[MAX_CALLBACKS];
static int up_count = 0;
static callback_entry_t down_callbacks[MAX_CALLBACKS];
static int down_count = 0;
static callback_entry_t push_callbacks[MAX_CALLBACKS];
static int push_count = 0;

static void execute_callbacks(callback_entry_t *callbacks, int count) {
    for (int i = 0; i < count; i++) {
        if (callbacks[i].cb) callbacks[i].cb(callbacks[i].arg);
    }
}

static void switch_task(void *pvParameter) {
    int last_up = 0, last_down = 0, last_push = 0;
    for (;;) {
        int up = gpio_get_level(pin_up);
        int down = gpio_get_level(pin_down);
        int push = gpio_get_level(pin_push);

        if (up && !last_up) {
            execute_callbacks(up_callbacks, up_count);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (down && !last_down) {
            execute_callbacks(down_callbacks, down_count);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (push && !last_push) {
            execute_callbacks(push_callbacks, push_count);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        last_up = up; last_down = down; last_push = push;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void switch_init(gpio_num_t up_pin, gpio_num_t push_pin, gpio_num_t down_pin) {
    pin_up = up_pin;
    pin_push = push_pin;
    pin_down = down_pin;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_up) | (1ULL << pin_push) | (1ULL << pin_down);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Switches inicializados en pines UP:%d, PUSH:%d, DOWN:%d", pin_up, pin_push, pin_down);

    xTaskCreate(switch_task, "switch_task", 2048, NULL, 23, NULL);
}

void switch_register_up(switch_callback_t cb, void *arg) {
    if (up_count < MAX_CALLBACKS) {
        up_callbacks[up_count].cb = cb;
        up_callbacks[up_count].arg = arg;
        up_count++;
    } else ESP_LOGE(TAG, "Too many UP callbacks");
}

void switch_register_down(switch_callback_t cb, void *arg) {
    if (down_count < MAX_CALLBACKS) {
        down_callbacks[down_count].cb = cb;
        down_callbacks[down_count].arg = arg;
        down_count++;
    } else ESP_LOGE(TAG, "Too many DOWN callbacks");
}

void switch_register_push(switch_callback_t cb, void *arg) {
    if (push_count < MAX_CALLBACKS) {
        push_callbacks[push_count].cb = cb;
        push_callbacks[push_count].arg = arg;
        push_count++;
    } else ESP_LOGE(TAG, "Too many PUSH callbacks");
}