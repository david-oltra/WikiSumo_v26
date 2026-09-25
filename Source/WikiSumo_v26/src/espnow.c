/**
 * @file espnow.c
 * @brief Implementación del driver para comunicación ESP-NOW
 * @author david_wiki
 * @date 2026
 *
 * NOTAS DE USO:
 *  - espnow_get_local_mac() SOLO es válido tras espnow_init() (WiFi ya arrancado).
 *  - El canal WiFi se fija a ESP_CHANNEL en espnow_init_wifi(). No cambies
 *    de canal después o los peers dejarán de coincidir.
 *  - espnow_set_peer_mac() re-registra el peer automáticamente.
 */

#include "espnow.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include <string.h>        // memcpy
#include "esp_log.h"       // ESP_LOGI/W/D/E
#include "nvs_flash.h"     // nvs_flash_init, nvs_flash_erase, ESP_ERR_NVS_*
// ============================================================
// CONFIGURACIÓN
// ============================================================
#ifndef ESP_CHANNEL
#define ESP_CHANNEL 1
#endif

#define ESPNOW_CONN_TIMEOUT_US  (3 * 1000 * 1000)  // 3 segundos

// ============================================================
// VARIABLES GLOBALES
// ============================================================
remote_data_t g_remote_data = {0};
robot_data_t  g_robot_data  = {0};

// ============================================================
// VARIABLES PRIVADAS
// ============================================================
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN]     = {0x8C, 0xBF, 0xEA, 0xB9, 0xD0, 0x64};
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static bool     s_has_new_data  = false;
static bool     s_peer_registered = false;
static int64_t  s_last_rx_us    = 0;
static int64_t  s_last_tx_ok_us = 0;

static const char *TAG = "ESPNOW";

// ============================================================
// CALLBACKS
// ============================================================

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info,
                           const uint8_t *data, int data_len)
{
    if (data_len == sizeof(remote_data_t)) {
        memcpy(&g_remote_data, data, sizeof(remote_data_t));
        s_has_new_data = true;
        s_last_rx_us   = esp_timer_get_time();
        ESP_LOGD(TAG, "Datos recibidos de " MACSTR, MAC2STR(esp_now_info->src_addr));
    } else {
        ESP_LOGW(TAG, "Recibidos %d bytes, esperado %d",
                 data_len, (int)sizeof(remote_data_t));
    }
}

static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_last_tx_ok_us     = esp_timer_get_time();
        g_robot_data.status = 1;
        ESP_LOGD(TAG, "Envío OK a " MACSTR, MAC2STR(tx_info->des_addr));
    } else {
        g_robot_data.status = 0;
        ESP_LOGW(TAG, "Envío FALLÓ a " MACSTR, MAC2STR(tx_info->des_addr));
    }
}

// ============================================================
// HELPERS INTERNOS
// ============================================================

/**
 * @brief Añade o reemplaza un peer de forma segura.
 */
static esp_err_t add_or_replace_peer(const uint8_t *peer_addr)
{
    esp_now_peer_info_t peer_info = {
        .channel = ESP_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);

    esp_err_t ret = esp_now_add_peer(&peer_info);

    if (ret == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Peer ya existe, reemplazando: " MACSTR, MAC2STR(peer_addr));
        ESP_ERROR_CHECK(esp_now_del_peer(peer_addr));
        ret = esp_now_add_peer(&peer_info);
    }

    return ret;
}

// ============================================================
// FUNCIONES PÚBLICAS
// ============================================================

esp_err_t espnow_init_wifi(void)
{
    ESP_LOGI(TAG, "Inicializando WiFi...");

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());

    // --- FIX 1: fijar el canal explícitamente ---
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t primary = 0;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &secondary);
    ESP_LOGI(TAG, "WiFi inicializado en canal %u", primary);

    return ESP_OK;
}

esp_err_t espnow_init(void)
{
    ESP_LOGI(TAG, "Inicializando ESP-NOW...");

    esp_err_t ret = espnow_init_wifi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando WiFi");
        return ret;
    }

    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // --- FIX 6: registrar peer broadcast ---
    ret = add_or_replace_peer(s_broadcast_mac);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo registrar broadcast: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Peer broadcast registrado");
    }

    ESP_LOGI(TAG, "ESP-NOW inicializado correctamente");
    return ESP_OK;
}

esp_err_t espnow_register_peer(uint8_t *peer_addr)
{
    if (peer_addr == NULL) {
        ESP_LOGE(TAG, "Dirección MAC del peer es NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Registrando peer: " MACSTR, MAC2STR(peer_addr));

    // --- FIX 3: add_or_replace_peer maneja ESP_ERR_ESPNOW_EXIST ---
    esp_err_t ret = add_or_replace_peer(peer_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando peer: %s", esp_err_to_name(ret));
        return ret;
    }

    s_peer_registered = true;
    ESP_LOGI(TAG, "Peer registrado correctamente");
    return ESP_OK;
}

esp_err_t espnow_register_default_peer(void)
{
    return espnow_register_peer(s_peer_mac);
}

esp_err_t espnow_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    if (peer_addr == NULL || data == NULL || len == 0) {
        ESP_LOGE(TAG, "Parámetros inválidos para envío");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_now_send(peer_addr, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error enviando datos: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t espnow_send_to_default_peer(const uint8_t *data, size_t len)
{
    return espnow_send_data(s_peer_mac, data, len);
}

esp_err_t espnow_send_robot_status(uint8_t status)
{
    robot_data_t robot_data = {
        .status = status
    };

    return espnow_send_to_default_peer((const uint8_t *)&robot_data,
                                       sizeof(robot_data_t));
}

void espnow_get_remote_data(remote_data_t *data)
{
    if (data != NULL) {
        memcpy(data, &g_remote_data, sizeof(remote_data_t));
        s_has_new_data = false;
    }
}

bool espnow_has_new_data(void)
{
    return s_has_new_data;
}

/**
 * @brief Devuelve true si ha habido actividad (RX o TX OK) en los últimos
 *        ESPNOW_CONN_TIMEOUT_US microsegundos.
 *
 * --- FIX 4: conexión basada en timeout real ---
 */
bool espnow_is_connected(void)
{
    int64_t now = esp_timer_get_time();
    int64_t last_activity = (s_last_rx_us > s_last_tx_ok_us)
                          ? s_last_rx_us
                          : s_last_tx_ok_us;

    if (last_activity == 0) {
        return false;   // nunca ha habido actividad
    }
    return (now - last_activity) < ESPNOW_CONN_TIMEOUT_US;
}

/**
 * @brief Lee la MAC local (STA).
 *
 * --- FIX 7: solo válido tras espnow_init() ---
 */
esp_err_t espnow_get_local_mac(uint8_t *mac)
{
    if (mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAC local: " MACSTR, MAC2STR(mac));
    } else {
        ESP_LOGE(TAG, "Error leyendo MAC local: %s", esp_err_to_name(ret));
    }

    return ret;
}

void espnow_get_peer_mac(uint8_t *mac)
{
    if (mac != NULL) {
        memcpy(mac, s_peer_mac, ESP_NOW_ETH_ALEN);
    }
}

/**
 * @brief Cambia la MAC peer en runtime y RE-REGISTRA el peer.
 *
 * --- FIX 3 (cont.): antes solo copiaba la MAC pero no re-registraba ---
 */
void espnow_set_peer_mac(uint8_t *mac)
{
    if (mac == NULL) {
        return;
    }

    // Eliminar el peer anterior si estaba registrado
    if (s_peer_registered) {
        esp_err_t del_ret = esp_now_del_peer(s_peer_mac);
        if (del_ret != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo eliminar peer anterior: %s",
                     esp_err_to_name(del_ret));
        }
        s_peer_registered = false;
    }

    // Actualizar MAC y re-registrar
    memcpy(s_peer_mac, mac, ESP_NOW_ETH_ALEN);
    ESP_LOGI(TAG, "MAC peer actualizada: " MACSTR, MAC2STR(s_peer_mac));

    esp_err_t ret = add_or_replace_peer(s_peer_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error re-registrando peer: %s", esp_err_to_name(ret));
    } else {
        s_peer_registered = true;
    }
}

void espnow_deinit(void)
{
    ESP_LOGI(TAG, "Desinicializando ESP-NOW...");

    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();

    s_peer_registered = false;
    s_has_new_data    = false;
    s_last_rx_us      = 0;
    s_last_tx_ok_us   = 0;

    ESP_LOGI(TAG, "ESP-NOW desinicializado");
}