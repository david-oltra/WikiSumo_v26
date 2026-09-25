/**
 * @file    rc5_module.c
 * @brief   Implementación del módulo de control por mando RC5.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Envuelve el decodificador RC5 (rc5_decode) y añade:
 *
 *     - Una máquina de estados del robot (STOPPED / POWER_ON / STARTED).
 *     - Gestión del "número base" de dohyo (comando de stop).
 *     - Control no bloqueante de un LED de estado (OFF / ON / BLINK).
 *     - Persistencia en NVS del estado y del comando de dohyo.
 *     - Comando especial de PROGRAM para cambiar el dohyo desde el mando.
 *     - Reinicio del dispositivo si se pulsa START estando en STOPPED.
 *
 *   Estados:
 *     - ROBOT_POWER_ON : estado inicial tras arranque o programación.
 *     - ROBOT_STARTED  : motores activos (combate en curso).
 *     - ROBOT_STOPPED  : robot detenido (parpadeo lento del LED).
 *
 *   Direcciones RC5 usadas:
 *     - 0x07 (ADDR_START_STOP) : comandos de arranque / parada.
 *     - 0x0B (ADDR_PROGRAM)    : comandos de programación del dohyo.
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "rc5_module.h"
#include "rc5_decode.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "nvs.h"

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "RC5_MOD";

/* ---------- NVS ---------- */
#define NVS_NAMESPACE        "robot_cfg"    /*!< Namespace de NVS usado por el módulo */
#define NVS_KEY_STOP         "stop_cmd"     /*!< Clave NVS del comando base de dohyo  */
#define NVS_KEY_STATE        "robot_state"  /*!< Clave NVS del último estado guardado */

/* ---------- Direcciones RC5 ---------- */
#define ADDR_START_STOP      0x07           /*!< Dirección de los comandos start/stop */
#define ADDR_PROGRAM         0x0B           /*!< Dirección de los comandos de program */

/* ---------- Parpadeo del LED ---------- */
#define BLINK_SLOW_MS        1000           /*!< Periodo de parpadeo lento (ms)       */
#define BLINK_PROGRAM_MS     200            /*!< Periodo de parpadeo al programar (ms)*/
#define PROGRAM_BLINK_CYCLES 2              /*!< Ciclos de parpadeo al programar      */

/* =====================================================================
 *  TIPOS INTERNOS
 * ===================================================================== */

/**
 * @brief Modos de funcionamiento del LED de estado.
 */
typedef enum {
    LED_MODE_OFF,           /*!< LED apagado                                    */
    LED_MODE_ON,            /*!< LED encendido fijo                             */
    LED_MODE_BLINK_SLOW,    /*!< Parpadeo lento (estado STOPPED)                */
    LED_MODE_BLINK_PROGRAM  /*!< Parpadeo rápido tras un evento de programación */
} led_mode_t;

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

static robot_state_t current_state   = ROBOT_POWER_ON;  /*!< Estado actual del robot   */
static uint8_t       stop_command    = 0;               /*!< Comando base de dohyo     */

static gpio_num_t    led_pin             = GPIO_NUM_NC; /*!< Pin del LED de estado     */
static led_mode_t    led_mode            = LED_MODE_OFF;/*!< Modo actual del LED       */
static bool          led_current_state   = false;       /*!< Estado actual del LED     */
static int64_t       last_led_toggle_us  = 0;           /*!< Timestamp del último toggle */
static int           program_blink_counter = 0;         /*!< Contador de parpadeos     */

/* =====================================================================
 *  PERSISTENCIA (NVS)
 * ===================================================================== */

/**
 * @brief  Carga el estado y el comando de dohyo desde NVS.
 *
 * Según la especificación del módulo, el robot SIEMPRE arranca en
 * ROBOT_POWER_ON, independientemente del estado guardado. Se conserva
 * el comando de stop para no perder la configuración del dohyo.
 */
static void nvs_load_state(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS no inicializada o namespace no encontrado, "
                      "usando valores por defecto");
        current_state = ROBOT_POWER_ON;
        stop_command  = 0;
        return;
    }

    uint8_t saved_stop  = 0;
    uint8_t saved_state = 0;

    err = nvs_get_u8(handle, NVS_KEY_STOP, &saved_stop);
    if (err == ESP_OK) {
        stop_command = saved_stop;
    } else {
        stop_command = 0;
    }

    err = nvs_get_u8(handle, NVS_KEY_STATE, &saved_state);
    if (err != ESP_OK) {
        saved_state = ROBOT_POWER_ON;  /* valor por defecto para el log */
    }

    /* --- CONSULTA AL INICIO: mostrar valores crudos guardados en NVS --- */
    ESP_LOGI(TAG, "CONSULTA NVS -> Dohyo (stop_cmd) = %d, Estado guardado = %d",
             saved_stop, saved_state);

    /* Según la especificación, el módulo SIEMPRE arranca en POWER_ON */
    current_state = ROBOT_POWER_ON;

    if (saved_state == ROBOT_STOPPED) {
        ESP_LOGI(TAG, "Estado previo STOPPED → restaurado a POWER_ON");
    } else if (saved_state == ROBOT_STARTED) {
        ESP_LOGI(TAG, "Estado previo STARTED → restaurado a POWER_ON");
    }

    nvs_close(handle);

    /* Log del estado efectivo tras la restauración */
    ESP_LOGI(TAG, "NVS cargada y aplicada: stopCmd=%d, state=%d",
             stop_command, current_state);
}

/**
 * @brief  Guarda el estado actual y el comando de dohyo en NVS.
 */
static void nvs_save_state(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS para escritura: %s",
                 esp_err_to_name(err));
        return;
    }

    nvs_set_u8(handle, NVS_KEY_STOP,  stop_command);
    nvs_set_u8(handle, NVS_KEY_STATE, (uint8_t)current_state);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "NVS guardada: stopCmd=%d, state=%d",
             stop_command, current_state);
}

/* =====================================================================
 *  CONTROL DEL LED (no bloqueante)
 * ===================================================================== */

/**
 * @brief  Enciende o apaga el LED de estado.
 *
 * @param[in] on  true para encender, false para apagar.
 */
static void led_set(bool on)
{
    if (led_pin != GPIO_NUM_NC) {
        gpio_set_level(led_pin, on ? 1 : 0);
        led_current_state = on;
    }
}

/**
 * @brief  Ajusta el modo del LED en función del estado del robot.
 */
static void update_led_mode_based_on_state(void)
{
    switch (current_state) {
        case ROBOT_POWER_ON: led_mode = LED_MODE_OFF;         break;
        case ROBOT_STARTED:  led_mode = LED_MODE_ON;          break;
        case ROBOT_STOPPED:  led_mode = LED_MODE_BLINK_SLOW;  break;
    }
    if (led_mode != LED_MODE_BLINK_PROGRAM) {
        program_blink_counter = 0;
    }
}

/**
 * @brief  Inicia el parpadeo rápido de "programación".
 */
static void start_program_blink(void)
{
    led_mode              = LED_MODE_BLINK_PROGRAM;
    program_blink_counter = PROGRAM_BLINK_CYCLES * 2;
    last_led_toggle_us    = esp_timer_get_time();
    led_set(false);
}

/**
 * @brief  Actualiza el estado del LED según el modo actual.
 *
 * Función no bloqueante: se apoya en esp_timer_get_time() para saber
 * cuándo toca el siguiente cambio de estado del LED.
 */
static void update_led(void)
{
    if (led_pin == GPIO_NUM_NC) return;

    int64_t now        = esp_timer_get_time();
    int64_t elapsed_ms = (now - last_led_toggle_us) / 1000;

    switch (led_mode) {
        case LED_MODE_OFF:
            led_set(false);
            break;

        case LED_MODE_ON:
            led_set(true);
            break;

        case LED_MODE_BLINK_SLOW:
            if (elapsed_ms >= BLINK_SLOW_MS) {
                led_set(!led_current_state);
                last_led_toggle_us = now;
            }
            break;

        case LED_MODE_BLINK_PROGRAM:
            if (elapsed_ms >= BLINK_PROGRAM_MS && program_blink_counter > 0) {
                led_set(!led_current_state);
                program_blink_counter--;
                last_led_toggle_us = now;

                if (program_blink_counter == 0) {
                    /* Al terminar, volvemos al modo del estado actual */
                    update_led_mode_based_on_state();
                    if (led_mode == LED_MODE_OFF) {
                        led_set(false);
                    } else if (led_mode == LED_MODE_ON) {
                        led_set(true);
                    } else if (led_mode == LED_MODE_BLINK_SLOW) {
                        led_set(false);
                        last_led_toggle_us = now;
                    }
                }
            }
            break;
    }
}

/* =====================================================================
 *  LÓGICA DE LA MÁQUINA DE ESTADOS
 * ===================================================================== */

/**
 * @brief  Procesa un comando RC5 recibido y actualiza el estado interno.
 *
 * @param[in] addr  Dirección RC5 del comando.
 * @param[in] cmd   Comando RC5 recibido.
 *
 * @return
 *   - true  si el comando ha provocado un cambio de estado o de dohyo.
 *   - false en caso contrario.
 */
static bool process_command(uint8_t addr, uint8_t cmd)
{
    bool           changed       = false;
    robot_state_t  new_state     = current_state;
    bool           program_event = false;

    if (addr == ADDR_PROGRAM) {
        /* ---------- Comando de programación de dohyo ---------- */
        if (current_state == ROBOT_POWER_ON || current_state == ROBOT_STARTED) {
            uint8_t new_stop = cmd & 0xFE;
            if (new_stop != stop_command) {
                stop_command = new_stop;
                new_state    = ROBOT_POWER_ON;
                changed      = true;
                program_event = true;
                ESP_LOGI(TAG, "PROGRAMADO: nuevo dohyo %d (stop=%d, start=%d)",
                         stop_command, stop_command, stop_command + 1);
            }
        } else {
            ESP_LOGW(TAG, "Comando de programación ignorado (robot STOPPED)");
        }
    }
    else if (addr == ADDR_START_STOP) {
        /* ---------- Comando de arranque / parada ---------- */
        uint8_t base     = cmd & 0xFE;
        bool    is_start = (cmd & 0x01) == 0x01;

        if (base == stop_command) {
            if (is_start) {
                /* --- Si está STOPPED y recibe START → reiniciar --- */
                if (current_state == ROBOT_STOPPED) {
                    ESP_LOGW(TAG, "START recibido en estado STOPPED. "
                                  "Reiniciando dispositivo...");
                    nvs_save_state();
                    /* Pequeña espera para que el log se envíe por UART */
                    vTaskDelay(pdMS_TO_TICKS(50));
                    esp_restart();
                    /* esp_restart() no retorna; el código de abajo
                     * no se ejecutará. */
                }
                /* --- Comportamiento normal --- */
                else if (current_state == ROBOT_POWER_ON) {
                    new_state = ROBOT_STARTED;
                    changed   = true;
                    ESP_LOGI(TAG, "COMANDO START recibido. Robot INICIADO.");
                } else {
                    ESP_LOGW(TAG, "START ignorado (estado actual=%d)",
                             current_state);
                }
            } else {
                /* --- Comando STOP --- */
                if (current_state != ROBOT_STOPPED) {
                    new_state = ROBOT_STOPPED;
                    changed   = true;
                    ESP_LOGI(TAG, "COMANDO STOP recibido. Robot DETENIDO.");
                }
            }
        } else {
            ESP_LOGD(TAG, "Comando para otro dohyo (base=%d)", base);
        }
    }
    else {
        ESP_LOGD(TAG, "Dirección desconocida 0x%02X", addr);
    }

    /* ---------- Aplicar cambios ---------- */
    if (changed) {
        current_state = new_state;

        if (!program_event) {
            /* Actualización del LED tras un cambio de estado normal */
            update_led_mode_based_on_state();
            if (led_mode == LED_MODE_OFF) {
                led_set(false);
            } else if (led_mode == LED_MODE_ON) {
                led_set(true);
            } else if (led_mode == LED_MODE_BLINK_SLOW) {
                led_set(false);
                last_led_toggle_us = esp_timer_get_time();
            }
        } else {
            /* Parpadeo especial tras un evento de programación */
            start_program_blink();
        }

        /* Guardar en NVS cada vez que cambia el estado o el comando */
        nvs_save_state();
    }

    return changed;
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el módulo RC5.
 *
 * Configura el LED de estado (si procede), inicializa NVS, carga el
 * estado y el comando de dohyo desde NVS y arranca el decodificador
 * RC5 sobre el pin del TSOP4838.
 *
 * NOTA: el robot SIEMPRE arranca en ROBOT_POWER_ON, independientemente
 * del estado guardado previamente en NVS.
 *
 * @param[in] out_pin        Pin GPIO conectado a la salida del TSOP4838.
 * @param[in] led_pin_param  Pin GPIO para el LED de estado. Pasar
 *                           GPIO_NUM_NC si no se desea usar LED.
 */
void rc5_module_init(gpio_num_t out_pin, gpio_num_t led_pin_param)
{
    led_pin = led_pin_param;

    /* ---------- Configurar LED de estado (si procede) ---------- */
    if (led_pin != GPIO_NUM_NC) {
        gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_pin, 0);
    }

    /* ---------- Inicializar NVS (solo una vez en el sistema) ---------- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ---------- Cargar estado y comando desde NVS ---------- */
    nvs_load_state();

    /* ---------- Configurar el LED según el estado restaurado ---------- */
    update_led_mode_based_on_state();
    if (led_mode == LED_MODE_OFF) {
        led_set(false);
    } else if (led_mode == LED_MODE_ON) {
        led_set(true);
    } else if (led_mode == LED_MODE_BLINK_SLOW) {
        led_set(false);
        last_led_toggle_us = esp_timer_get_time();
    }

    /* ---------- Inicializar el decodificador RC5 ---------- */
    rc5_decode_init(out_pin);

    ESP_LOGI(TAG, "Módulo RC5 listo (LED controlado, NVS cargada)");
}

/**
 * @brief  Procesa un ciclo del módulo RC5.
 *
 * Actualiza el estado del LED (no bloqueante) y comprueba si hay una
 * nueva trama RC5 decodificada. Debe llamarse periódicamente desde el
 * bucle principal (por ejemplo cada 10 ms).
 *
 * @return
 *   - true  si se ha recibido y procesado un comando RC5 que ha
 *           provocado un cambio de estado o de dohyo.
 *   - false en caso contrario.
 */
bool rc5_module_process(void)
{
    update_led();

    uint8_t addr, cmd;
    if (rc5_decode_get_frame(&addr, &cmd)) {
        return process_command(addr, cmd);
    }
    return false;
}

/**
 * @brief  Devuelve el estado actual del robot.
 *
 * @return Estado actual (ROBOT_STOPPED, ROBOT_POWER_ON o ROBOT_STARTED).
 */
robot_state_t rc5_module_get_state(void)
{
    return current_state;
}

/**
 * @brief  Devuelve el comando base de dohyo (comando de stop).
 *
 * Este valor identifica el modo de combate seleccionado. El comando
 * de start es siempre `stop_command + 1`.
 *
 * @return Comando base de dohyo (0-254, siempre par).
 */
uint8_t rc5_module_get_stop_command(void)
{
    return stop_command;
}

/**
 * @brief  Fuerza manualmente el estado del robot y el comando de dohyo.
 *
 * Pensado para pruebas o para uso interno desde otras partes del
 * firmware. Actualiza el LED, guarda el nuevo estado en NVS y lo
 * registra por log.
 *
 * @param[in] state     Nuevo estado a imponer.
 * @param[in] stop_cmd  Nuevo comando base de dohyo.
 */
void rc5_module_force_state(robot_state_t state, uint8_t stop_cmd)
{
    current_state = state;
    stop_command  = stop_cmd;

    update_led_mode_based_on_state();
    if (led_mode == LED_MODE_OFF) {
        led_set(false);
    } else if (led_mode == LED_MODE_ON) {
        led_set(true);
    } else if (led_mode == LED_MODE_BLINK_SLOW) {
        led_set(false);
        last_led_toggle_us = esp_timer_get_time();
    }

    /* Guardar también al forzar estado */
    nvs_save_state();

    ESP_LOGI(TAG, "Estado forzado: state=%d, stopCmd=%d", state, stop_cmd);
}