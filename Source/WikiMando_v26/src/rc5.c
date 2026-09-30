#include "rc5.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include <string.h>

static const char *TAG = "RC5";
static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static uint8_t toggle_bit = 0;
static bool initialized = false;

static uint16_t build_frame(uint8_t address, uint8_t command, uint8_t toggle) {
    uint16_t frame = 0;
    frame |= (0x03 << 12);              // start bits 1,1
    frame |= ((toggle & 0x01) << 11);   // toggle bit
    frame |= ((address & 0x1F) << 6);   // address
    frame |= (command & 0x3F);          // command
    return frame;
}

static void frame_to_items(uint16_t frame, rmt_symbol_word_t *items, size_t *num) {
    *num = 0;
    const uint32_t three_quarter = (uint32_t)(RC5_BIT_TIME_US * 0.75);
    const uint32_t quarter = (uint32_t)(RC5_BIT_TIME_US * 0.25);
    for (int i = RC5_NUM_BITS - 1; i >= 0; i--) {
        int bit = (frame >> i) & 1;
        items[*num].duration0 = bit ? three_quarter : quarter;
        items[*num].level0 = 1;
        items[*num].duration1 = bit ? quarter : three_quarter;
        items[*num].level1 = 0;
        (*num)++;
    }
}

static bool send_raw_frame(uint16_t frame, int repeat) {
    if (!initialized) return false;
    rmt_symbol_word_t items[RC5_NUM_BITS];
    size_t num_items;
    frame_to_items(frame, items, &num_items);
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    for (int i = 0; i < repeat; i++) {
        esp_err_t ret = rmt_transmit(tx_channel, copy_encoder, items,
                                     num_items * sizeof(rmt_symbol_word_t), &tx_config);
        if (ret != ESP_OK) return false;
        rmt_tx_wait_all_done(tx_channel, pdMS_TO_TICKS(1000));
        if (i < repeat - 1) vTaskDelay(pdMS_TO_TICKS(RC5_REPEAT_DELAY_MS));
    }
    return true;
}

bool rc5_tx_init(const rc5_tx_config_t *config) {
    if (!config) return false;
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->tx_gpio,
        .mem_block_symbols = 64,
        .resolution_hz = config->resolution_hz,
        .trans_queue_depth = 4,
    };
    if (rmt_new_tx_channel(&tx_cfg, &tx_channel) != ESP_OK) return false;
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = config->carrier_freq_hz,
        .duty_cycle = config->carrier_duty_cycle,
    };
    if (rmt_apply_carrier(tx_channel, &carrier_cfg) != ESP_OK) return false;
    rmt_copy_encoder_config_t enc_cfg = {};
    if (rmt_new_copy_encoder(&enc_cfg, &copy_encoder) != ESP_OK) return false;
    if (rmt_enable(tx_channel) != ESP_OK) return false;
    initialized = true;
    toggle_bit = 0;
    ESP_LOGI(TAG, "RC5 TX init OK on GPIO %d", config->tx_gpio);
    return true;
}

void rc5_tx_deinit(void) {
    if (!initialized) return;
    if (tx_channel) rmt_disable(tx_channel);
    if (copy_encoder) rmt_del_encoder(copy_encoder);
    if (tx_channel) rmt_del_channel(tx_channel);
    tx_channel = NULL;
    copy_encoder = NULL;
    initialized = false;
    ESP_LOGI(TAG, "RC5 TX deinit");
}

bool rc5_send_raw(uint8_t address, uint8_t command, int repeat, bool flip_toggle) {
    if (!initialized) return false;
    if (repeat < 1) repeat = 1;
    uint16_t frame = build_frame(address, command, toggle_bit);
    ESP_LOGI(TAG, "Send addr=0x%02X cmd=0x%02X toggle=%d rep=%d", address, command, toggle_bit, repeat);
    bool ok = send_raw_frame(frame, repeat);
    if (ok && flip_toggle) toggle_bit ^= 1;
    return ok;
}

uint8_t rc5_get_toggle(void) { return toggle_bit; }
void rc5_reset_toggle(void) { toggle_bit = 0; }