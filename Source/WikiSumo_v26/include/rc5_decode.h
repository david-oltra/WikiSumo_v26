/**
 * @file    rc5_decode.h
 * @brief   Decodificador de tramas RC5 (protocolo Philips) vía GPIO.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Este módulo implementa un decodificador por interrupciones para
 * el protocolo RC5 utilizado por mandos infrarrojos estándar
 * (TSOP4838 y compatibles). Extrae la dirección y el comando de
 * cada trama recibida.
 *
 * Uso típico:
 * @code
 *   rc5_decode_init(GPIO_NUM_13);
 *   uint8_t addr, cmd;
 *   if (rc5_decode_get_frame(&addr, &cmd)) {
 *       // procesar comando
 *       rc5_decode_reset();
 *   }
 * @endcode
 */

#ifndef RC5_DECODE_H
#define RC5_DECODE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa el decodificador RC5.
 *
 * Configura el pin de entrada como GPIO y registra la interrupción
 * correspondiente para decodificar los flancos de la señal RC5.
 *
 * @param[in] out_pin  Pin GPIO conectado a la salida del receptor IR.
 */
void rc5_decode_init(gpio_num_t out_pin);

/**
 * @brief  Comprueba si hay una trama RC5 completa disponible.
 *
 * Si hay una trama disponible, copia la dirección y el comando en los
 * punteros indicados.
 *
 * @param[out] address  Puntero donde se almacenará la dirección (5 bits).
 * @param[out] command  Puntero donde se almacenará el comando (6 bits).
 *
 * @return
 *   - true  si se ha recibido una trama completa y los datos son válidos.
 *   - false si todavía no hay trama disponible.
 */
bool rc5_decode_get_frame(uint8_t *address, uint8_t *command);

/**
 * @brief  Reinicia el decodificador para prepararlo para la siguiente trama.
 *
 * Debe llamarse después de procesar una trama recibida.
 */
void rc5_decode_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* RC5_DECODE_H */