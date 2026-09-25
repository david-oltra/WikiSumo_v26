/**
 * @file    rc5_module.h
 * @brief   Módulo de alto nivel para el control por mando RC5.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Este módulo envuelve el decodificador RC5 (rc5_decode) y añade la
 * lógica de estados del robot (STOPPED / POWER_ON / STARTED) y la
 * gestión de los comandos del mando, incluyendo el "número base" de
 * dohyo (comando de stop) que determina el modo de combate.
 *
 * Uso típico:
 * @code
 *   rc5_module_init(GPIO_NUM_13, GPIO_NUM_48);
 *   for (;;) {
 *       if (rc5_module_process()) {
 *           // ha llegado un comando, consultar rc5_module_get_state()
 *       }
 *       vTaskDelay(pdMS_TO_TICKS(10));
 *   }
 * @endcode
 */

#ifndef RC5_MODULE_H
#define RC5_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  TIPOS
 * ===================================================================== */

/**
 * @brief Estados posibles del robot controlados por el mando.
 */
typedef enum {
    ROBOT_STOPPED  = 0,  /*!< Robot parado                                        */
    ROBOT_POWER_ON = 1,  /*!< Robot encendido pero sin arrancar (standby)         */
    ROBOT_STARTED  = 2   /*!< Robot en combate (motores activos)                  */
} robot_state_t;

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el módulo RC5.
 *
 * Configura el decodificador sobre el pin del TSOP4838 y, opcionalmente,
 * un LED de estado.
 *
 * @param[in] out_pin  Pin GPIO conectado a la salida del TSOP4838.
 * @param[in] led_pin  Pin GPIO para el LED de estado. Pasar
 *                     GPIO_NUM_NC si no se desea usar LED.
 */
void rc5_module_init(gpio_num_t out_pin, gpio_num_t led_pin);

/**
 * @brief  Procesa la lógica del módulo. Debe llamarse periódicamente
 *         desde el bucle principal.
 *
 * @return
 *   - true  si se ha recibido y procesado un comando RC5.
 *   - false en caso contrario.
 */
bool rc5_module_process(void);

/**
 * @brief  Devuelve el estado actual del robot.
 *
 * @return Estado actual (ROBOT_STOPPED, ROBOT_POWER_ON o ROBOT_STARTED).
 */
robot_state_t rc5_module_get_state(void);

/**
 * @brief  Devuelve el comando de stop configurado (número base del dohyo).
 *
 * Este valor se usa para identificar el modo de combate seleccionado
 * desde el mando.
 *
 * @return Comando de stop (número base del dohyo).
 */
uint8_t rc5_module_get_stop_command(void);

/**
 * @brief  Fuerza manualmente el estado del robot (para pruebas).
 *
 * @param[in] state     Nuevo estado a imponer.
 * @param[in] stop_cmd  Comando de stop asociado (número base del dohyo).
 */
void rc5_module_force_state(robot_state_t state, uint8_t stop_cmd);

#ifdef __cplusplus
}
#endif

#endif /* RC5_MODULE_H */