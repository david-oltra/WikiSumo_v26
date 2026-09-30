#include "display.h"
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static SSD1306_t oled_dev;
static char display_buffer[16] = {0};

// Función privada para inicializar el hardware I2C y SSD1306
static void ssd1306_hw_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    i2c_master_init(&oled_dev, sda, scl, -1); // asumiendo que i2c_master_init usa esos pines
    ssd1306_init(&oled_dev, 128, 64);
    ssd1306_clear_screen(&oled_dev, false);
    ssd1306_contrast(&oled_dev, 0xff);
    ssd1306_display_text(&oled_dev, 0, "    WIKISUMO    ", 16, false);
    ssd1306_display_text(&oled_dev, 4, "   STARTING...   ", 16, false);
    // display_update();
}

// Implementación pública
void display_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    ssd1306_hw_init(i2c_port, sda_pin, scl_pin);
}

void display_update(display_t *menu) {
    ssd1306_clear_line(&oled_dev, 4, false);
    ssd1306_clear_line(&oled_dev, 6, false);

    switch (menu->screen) {
        case 0:
            snprintf(display_buffer, sizeof(display_buffer), "     START    ");
            ssd1306_display_text(&oled_dev, 4, display_buffer, 16, false);
            break;
        case 1:
            snprintf(display_buffer, sizeof(display_buffer), "      STOP    ");
            ssd1306_display_text(&oled_dev, 4, display_buffer, 16, false);
            break;
        case 2:
            snprintf(display_buffer, sizeof(display_buffer), "     PROGRAM  ");
            ssd1306_display_text(&oled_dev, 4, display_buffer, 16, false);
            break;
        case 3: // CONFIG
            snprintf(display_buffer, sizeof(display_buffer), "     CONFIG   ");
            ssd1306_display_text(&oled_dev, 4, display_buffer, 16, false);
            switch (menu->level) {
                case 1:
                    ssd1306_display_text(&oled_dev, 4, "      DOHYO   ", 16, false);
                    snprintf(display_buffer, sizeof(display_buffer), "       %2u    ", menu->dohyo);
                    ssd1306_display_text(&oled_dev, 6, display_buffer, 16, menu->sublevel ? true : false);
                    break;
                case 2:
                    ssd1306_display_text(&oled_dev, 4, "    STRATEGY  ", 16, false);
                    snprintf(display_buffer, sizeof(display_buffer), "       %2u    ", menu->strategy);
                    ssd1306_display_text(&oled_dev, 6, display_buffer, 16, menu->sublevel ? true : false);
                    break;
                case 3:
                    ssd1306_display_text(&oled_dev, 4, "  DISABLE TOF ", 16, false);
                    (menu->disable_toff) ? ssd1306_display_text(&oled_dev, 6, "      YES     ", 16, true) : ssd1306_display_text(&oled_dev, 6, "       NO     ", 16, false) ;
                    break;
                case 4:
                    ssd1306_display_text(&oled_dev, 4, "  DISABLE QRE ", 16, false);
                    (menu->disable_qre) ? ssd1306_display_text(&oled_dev, 6, "      YES     ", 16, true) : ssd1306_display_text(&oled_dev, 6, "       NO     ", 16, false) ;
                    break;
                case 5:
                    ssd1306_display_text(&oled_dev, 4, " CALIBRATE QRE", 16, false);
                    break;
                case 6:
                    ssd1306_display_text(&oled_dev, 4, "     RETURN   ", 16, false);
                    break;
                default: break;
            }
            break;
        default: break;
    }
}


uint8_t display_get_dohyo(display_t *menu) {
    return menu->dohyo;
}

uint8_t display_get_strategy(display_t *menu) {
    return menu->strategy;
}