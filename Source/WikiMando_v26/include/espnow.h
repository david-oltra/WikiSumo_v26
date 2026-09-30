/**
 * @file espnow.h
 * @brief Driver para comunicación ESP-NOW
 * @author david_wiki
 * @date 2026
 *
 * USO TÍPICO:
 *   1. espnow_init();                    // arranca WiFi + ESP-NOW + broadcast
 *   2. espnow_register_default_peer();   // registra la MAC por defecto
 *      (o espnow_set_peer_mac(mac) si quieres cambiarla)
 *   3. espnow_get_local_mac(mi_mac);     // SOLO válido tras espnow_init()
 *   4. Enviar: espnow_send_to_default_peer(&data, sizeof(data));
 *   5. Recibir: if (espnow_has_new_data()) espnow_get_remote_data(&rx);
 */

#ifndef ESPNOW_H
#define ESPNOW_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_now.h"      // ESP_NOW_ETH_ALEN
#include "esp_mac.h"      // MACSTR / MAC2STR

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// CONFIGURACIÓN
// ============================================================

/**
 * @brief Canal WiFi/ESP-NOW usado por todos los peers.
 *        Ambos dispositivos DEBEN estar en el mismo canal.
 *        Modifícalo aquí si quieres otro canal (1..13).
 */
#define ESP_CHANNEL 1

// ============================================================
// TIPOS DE DATOS
// ============================================================

/**
 * @brief Datos que el mando envía al robot.
 */
typedef struct {
    uint8_t  start;        /**< 1 = arrancar, 0 = parar */
    uint8_t  strategy;     /**< índice de estrategia seleccionada */
    uint8_t  dohyo;        /**< tamaño/config del dohyo */
    uint8_t  disable_toff; /**< 1 = desactivar time-off */
    uint8_t  disable_qre;  /**< 1 = desactivar sensores QRE */
} remote_data_t;

/**
 * @brief Datos que el robot envía de vuelta al mando.
 */
typedef struct {
    uint8_t  status;       /**< 1 = OK, 0 = error (actualizado por el send_cb) */
} robot_data_t;

// ============================================================
// VARIABLES GLOBALES (definidas en espnow.c)
// ============================================================

/** Último paquete remote_data_t recibido. */
extern remote_data_t g_remote_data;

/** Último robot_data_t (estado del último envío). */
extern robot_data_t  g_robot_data;

// ============================================================
// API PÚBLICA
// ============================================================

/**
 * @brief Inicializa NVS, netif, WiFi en modo STA y fija el canal ESP_CHANNEL.
 *        Normalmente no la llamas directamente: espnow_init() ya lo hace.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
esp_err_t espnow_init_wifi(void);

/**
 * @brief Inicializa WiFi + ESP-NOW, registra callbacks y añade el peer
 *        broadcast (FF:FF:FF:FF:FF:FF).
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
esp_err_t espnow_init(void);

/**
 * @brief Registra (o reemplaza si ya existe) un peer con la MAC dada.
 *
 * @param peer_addr Puntero a 6 bytes con la MAC del peer.
 * @return ESP_OK en éxito, ESP_ERR_INVALID_ARG si peer_addr es NULL,
 *         o código de error de esp_now_add_peer.
 */
esp_err_t espnow_register_peer(uint8_t *peer_addr);

/**
 * @brief Registra el peer por defecto (s_peer_mac interno).
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
esp_err_t espnow_register_default_peer(void);

/**
 * @brief Envía datos a un peer concreto.
 *
 * @param peer_addr MAC destino (6 bytes).
 * @param data      Puntero a los datos.
 * @param len       Tamaño en bytes.
 * @return ESP_OK si el paquete se encoló (el resultado real llega al send_cb).
 */
esp_err_t espnow_send_data(const uint8_t *peer_addr,
                           const uint8_t *data,
                           size_t len);

/**
 * @brief Envía datos al peer por defecto.
 */
esp_err_t espnow_send_to_default_peer(const uint8_t *data, size_t len);

/**
 * @brief Envía un robot_data_t con el status indicado al peer por defecto.
 */
esp_err_t espnow_send_robot_status(uint8_t status);

/**
 * @brief Copia el último remote_data_t recibido y limpia el flag de "nuevo".
 *
 * @param data Destino. Si es NULL no hace nada.
 */
void espnow_get_remote_data(remote_data_t *data);

/**
 * @brief Devuelve true si ha llegado un nuevo paquete desde la última
 *        llamada a espnow_get_remote_data().
 */
bool espnow_has_new_data(void);

/**
 * @brief Devuelve true si ha habido actividad (RX o TX OK) en los últimos
 *        3 segundos. NO es una conexión real, solo un indicador de actividad.
 */
bool espnow_is_connected(void);

/**
 * @brief Lee la MAC STA local.
 *
 * @warning Solo es válido DESPUÉS de espnow_init() (WiFi ya arrancado).
 *
 * @param mac Buffer de 6 bytes donde se escribirá la MAC.
 * @return ESP_OK en éxito, ESP_ERR_INVALID_ARG si mac es NULL.
 */
esp_err_t espnow_get_local_mac(uint8_t *mac);

/**
 * @brief Copia la MAC peer actual en el buffer dado.
 */
void espnow_get_peer_mac(uint8_t *mac);

/**
 * @brief Cambia la MAC peer en runtime y RE-REGISTRA el peer automáticamente.
 *
 * @param mac Nueva MAC (6 bytes). Si es NULL no hace nada.
 */
void espnow_set_peer_mac(uint8_t *mac);

/**
 * @brief Detiene WiFi y desinicializa ESP-NOW.
 */
void espnow_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // ESPNOW_H