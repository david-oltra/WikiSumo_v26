#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "display.h"
#include "switch.h"
#include "rc5.h"
#include "start_module.h"
#include "espnow.h"

#define I2C_PORT I2C_NUM_0
#define SDA GPIO_NUM_8
#define SCL GPIO_NUM_9
#define JOYSTICK_X ADC1_CHANNEL_0
#define JOYSTICK_Y ADC1_CHANNEL_1
#define JOYSTICK_SWITCH GPIO_NUM_3
#define SWITCH_UP GPIO_NUM_7
#define SWITCH_PUSH GPIO_NUM_6
#define SWITCH_DOWN GPIO_NUM_5
#define IR_LED_GPIO 4       /**< GPIO para el LED infrarrojo */
#define STATUS_LED_GPIO -1       /**< GPIO para LED de estado */

#define TAG "MAIN"

display_t menu ={0};

start_module_config_t sm_cfg = {
    .ir_led_gpio = IR_LED_GPIO,
    .status_led_gpio = STATUS_LED_GPIO,
    .status_led_active_high = true,
    .rc5_resolution_hz = 1000000,
    .rc5_carrier_freq_hz = 38000,
    .rc5_carrier_duty_cycle = 0.5f,
};

remote_data_t remote = {0};
static uint8_t peer_mac[6] = {0x3c, 0x0f, 0x02, 0xe0, 0xa4, 0x74};
//remote_mac 8c:d0:b2:a8:5a:04

// void handle_recv(uint8_t *mac_origen, uint8_t *datos, int len) {
//     if (len == sizeof(remote_data_t)) {
//         remote_data_t *msg = (remote_data_t*)datos;
//         ESP_LOGI(TAG, "Recibido de %02x:%02x:%02x:%02x:%02x:%02x",
//                  mac_origen[0], mac_origen[1], mac_origen[2],
//                  mac_origen[3], mac_origen[4], mac_origen[5]);
//     }
// }

static void espnow_rx_task(void *arg)
{
    remote_data_t rx;
    while (1) {
        if (espnow_has_new_data()) {
            espnow_get_remote_data(&rx);
            // haz algo con rx (actualizar display, mover motores, etc.)
            ESP_LOGI(TAG, "RX: start=%u strat=%u dohyo=%u toff=%u qre=%u",
                     rx.start, rx.strategy, rx.dohyo,
                     rx.disable_toff, rx.disable_qre);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void handle_up(void *arg) {
    if (menu.level == 0) { // pantalla principal
        menu.screen = (menu.screen + 1) % 4;
    }
    else if (menu.level == 1 && menu.sublevel == 1) {
        menu.dohyo++;
    }
    else if (menu.level == 2 && menu.sublevel == 1) {
        menu.strategy++;
        remote.strategy = menu.strategy;
    }
    else {
        if (menu.level < 6) menu.level++;
        else menu.level = 1;
    }
    display_update(&menu);
}

void handle_down(void *arg) {
    if (menu.level == 0) {
        menu.screen = (menu.screen == 0) ? 3 : menu.screen - 1;
    }
    else if (menu.level == 1 && menu.sublevel == 1) {
        if (menu.dohyo > 0) menu.dohyo--;
    }
    else if (menu.level == 2 && menu.sublevel == 1) {
        if (menu.strategy > 0) menu.strategy--;
        remote.strategy = menu.strategy;
    }
    else {
        if (menu.level > 1) menu.level--;
        else menu.level = 6;
    }
    display_update(&menu);
}

void handle_push(void *arg) {
    if (menu.level == 0) {
        switch (menu.screen) {
            case 0: // START
                start_module_send_start(menu.dohyo);
                remote.start = 1;
                espnow_send_to_default_peer((uint8_t*)&remote, sizeof(remote));
                break;
            case 1: // STOP
                start_module_send_stop(menu.dohyo);
                remote.start = 0;
                espnow_send_to_default_peer((uint8_t*)&remote, sizeof(remote));
                break;
            case 2: // PROGRAM
                start_module_program(menu.dohyo);
                break;
            case 3: // CONFIG
                menu.level = 1;
                break;
        }
    }
    else {
        switch (menu.level) {
            case 1: // DOHYO
                menu.sublevel = !menu.sublevel;
                break;
            case 2: // STRATEGY
                menu.sublevel = !menu.sublevel;
                espnow_send_to_default_peer((uint8_t*)&remote, sizeof(remote));
                break;
            case 3: // DISABLE TOFF
                menu.disable_toff = !menu.disable_toff;
                remote.disable_toff = menu.disable_toff;
                espnow_send_to_default_peer((uint8_t*)&remote, sizeof(remote));
                break;
            case 4: // DISABLE QRE
                menu.disable_qre = !menu.disable_qre;
                remote.disable_qre = menu.disable_qre;
                espnow_send_to_default_peer((uint8_t*)&remote, sizeof(remote));
                break;
            case 5: // CALIBRATE QRE

                break;
            case 6: // RETURN
                menu.level = 0;
                menu.sublevel = 0;
                break;
            // case 3,4,5: acciones futuras
            default: break;
        }
    }
    display_update(&menu);
}

void app_main(void) {

    // Inicializar display con sus pines I2C
    display_init(I2C_PORT, SDA, SCL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_update(&menu);

    // Inicializar switches con sus pines
    switch_init(SWITCH_UP, SWITCH_PUSH, SWITCH_DOWN);  // UP, PUSH, DOWN

    // Registrar los manejadores del display (y otros que quieras)
    switch_register_up(handle_up, NULL);
    switch_register_down(handle_down, NULL);
    switch_register_push(handle_push, NULL);

    // Ejemplo: agregar otra acción al botón PUSH (por ejemplo, un pitido)
    // switch_register_push(mi_funcion_beep, NULL);

    start_module_init(&sm_cfg);

    espnow_init();
    espnow_set_peer_mac(peer_mac);    // usa la MAC de main.c y re-registra 

    uint8_t mi_mac[6];
    espnow_get_local_mac(mi_mac);
    ESP_LOGI(TAG, "Mi MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mi_mac[0], mi_mac[1], mi_mac[2], mi_mac[3], mi_mac[4], mi_mac[5]);


    xTaskCreate(espnow_rx_task, "espnow_rx", 3072, NULL, 5, NULL);

    vTaskDelete(NULL);
    // for (;;) {
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
}
