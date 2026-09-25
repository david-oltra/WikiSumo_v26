/**
 * @file    ws2812b.h
 * @brief   Librería para controlar tiras de LEDs WS2812B usando RMT
 *          en ESP32-S3.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Proporciona una API sencilla para manejar tiras de LEDs WS2812B
 *   (NeoPixel) usando el periférico RMT del ESP32-S3. La librería
 *   mantiene un buffer interno con el color de cada LED y sólo se
 *   envía a la tira física al llamar a ws2812b_refresh().
 *
 *   Además de las operaciones básicas (encender, apagar, asignar
 *   colores), incluye utilidades de manipulación de color:
 *     - Conversión HSV → RGB.
 *     - Aplicación de brillo (escalado proporcional).
 *     - Mezcla de colores.
 *     - Cálculo del complementario.
 *     - Ajuste fino de brillo (delta).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  ESTRUCTURAS
 * ===================================================================== */

/**
 * @brief  Estructura que representa un color RGB.
 */
typedef struct {
    uint8_t r;   /*!< Componente rojo  (0-255) */
    uint8_t g;   /*!< Componente verde (0-255) */
    uint8_t b;   /*!< Componente azul  (0-255) */
} ws2812b_color_t;

/**
 * @brief  Estructura de configuración para la tira de LEDs WS2812B.
 */
typedef struct {
    int gpio_num;           /*!< Número del pin GPIO para la línea de datos                       */
    int num_leds;           /*!< Cantidad de LEDs en la tira                                      */
    int rmt_resolution_hz;  /*!< Resolución del periférico RMT en Hz (recomendado: 40 000 000)    */
} ws2812b_config_t;

/* =====================================================================
 *  COLORES PREDEFINIDOS
 * ===================================================================== */

/** @brief Colores básicos predefinidos. */
extern const ws2812b_color_t WS2812B_RED;      /*!< Rojo puro             */
extern const ws2812b_color_t WS2812B_GREEN;    /*!< Verde puro            */
extern const ws2812b_color_t WS2812B_BLUE;     /*!< Azul puro             */
extern const ws2812b_color_t WS2812B_YELLOW;   /*!< Amarillo              */
extern const ws2812b_color_t WS2812B_CYAN;     /*!< Cian                  */
extern const ws2812b_color_t WS2812B_MAGENTA;  /*!< Magenta               */
extern const ws2812b_color_t WS2812B_WHITE;    /*!< Blanco                */
extern const ws2812b_color_t WS2812B_BLACK;    /*!< Negro (apagado)       */

/** @brief Colores adicionales. */
extern const ws2812b_color_t WS2812B_ORANGE;   /*!< Naranja               */
extern const ws2812b_color_t WS2812B_PURPLE;   /*!< Púrpura               */
extern const ws2812b_color_t WS2812B_PINK;     /*!< Rosa                  */
extern const ws2812b_color_t WS2812B_LIME;     /*!< Verde lima            */
extern const ws2812b_color_t WS2812B_TEAL;     /*!< Verde azulado         */
extern const ws2812b_color_t WS2812B_INDIGO;   /*!< Índigo                */
extern const ws2812b_color_t WS2812B_GOLD;     /*!< Dorado                */
extern const ws2812b_color_t WS2812B_SILVER;   /*!< Plateado              */

/* =====================================================================
 *  FUNCIONES DE UTILIDAD DE COLOR
 * ===================================================================== */

/**
 * @brief  Crea un color RGB a partir de componentes individuales.
 *
 * @param[in] r  Componente rojo  (0-255).
 * @param[in] g  Componente verde (0-255).
 * @param[in] b  Componente azul  (0-255).
 *
 * @return Estructura ws2812b_color_t con el color resultante.
 */
static inline ws2812b_color_t ws2812b_rgb(uint8_t r, uint8_t g, uint8_t b) {
    ws2812b_color_t color = {r, g, b};
    return color;
}

/**
 * @brief  Crea un color a partir de valores HSV (Tono/Saturación/Valor).
 *
 * @param[in] hue         Tono (0-360 grados).
 * @param[in] saturation  Saturación (0-255).
 * @param[in] value       Valor/Brillo (0-255).
 *
 * @return Color RGB resultante.
 */
ws2812b_color_t ws2812b_hsv(uint16_t hue, uint8_t saturation, uint8_t value);

/**
 * @brief  Aplica un factor de brillo a un color.
 *
 * Escala cada componente RGB proporcionalmente al valor de brillo
 * indicado.
 *
 * @param[in] color       Color original.
 * @param[in] brightness  Brillo a aplicar (0-255, donde 255 es brillo máximo).
 *
 * @return Color con brillo ajustado.
 *
 * @example
 *   ws2812b_color_t dimmed = ws2812b_apply_brightness(WS2812B_RED, 128);
 */
ws2812b_color_t ws2812b_apply_brightness(ws2812b_color_t color,
                                         uint8_t brightness);

/**
 * @brief  Mezcla dos colores.
 *
 * @param[in] color1  Primer color.
 * @param[in] color2  Segundo color.
 * @param[in] ratio   Proporción del segundo color (0-255, donde 128 es 50/50).
 *
 * @return Color resultante de la mezcla.
 */
ws2812b_color_t ws2812b_mix_colors(ws2812b_color_t color1,
                                   ws2812b_color_t color2,
                                   uint8_t ratio);

/**
 * @brief  Calcula el color complementario (inverso).
 *
 * @param[in] color  Color original.
 *
 * @return Color complementario.
 */
ws2812b_color_t ws2812b_complementary(ws2812b_color_t color);

/**
 * @brief  Aumenta o disminuye el brillo de un color.
 *
 * @param[in] color  Color original.
 * @param[in] delta  Cambio de brillo (-255 a 255):
 *                     - Negativo: oscurece.
 *                     - Positivo: aclara.
 *
 * @return Color con brillo ajustado.
 */
ws2812b_color_t ws2812b_adjust_brightness(ws2812b_color_t color,
                                          int16_t delta);

/* =====================================================================
 *  FUNCIONES PRINCIPALES
 * ===================================================================== */

/**
 * @brief  Inicializa la tira de LEDs WS2812B.
 *
 * Configura el canal RMT, el encoder y reserva el buffer interno de
 * LEDs. Debe llamarse una sola vez antes de cualquier otra función
 * de la librería.
 *
 * @param[in] config  Puntero a la estructura de configuración.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
esp_err_t ws2812b_init(const ws2812b_config_t *config);

/**
 * @brief  Desinicializa la tira de LEDs y libera todos los recursos.
 */
void ws2812b_deinit(void);

/**
 * @brief  Asigna un color a un LED específico usando componentes RGB.
 *
 * @param[in] index  Índice del LED (0 = primer LED).
 * @param[in] r      Componente rojo  (0-255).
 * @param[in] g      Componente verde (0-255).
 * @param[in] b      Componente azul  (0-255).
 */
void ws2812b_set_led_rgb(int index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  Asigna un color a un LED específico usando estructura de color.
 *
 * @param[in] index  Índice del LED (0 = primer LED).
 * @param[in] color  Color a asignar.
 */
static inline void ws2812b_set_led(int index, ws2812b_color_t color) {
    ws2812b_set_led_rgb(index, color.r, color.g, color.b);
}

/**
 * @brief  Obtiene el color actual de un LED del buffer interno.
 *
 * @param[in]  index  Índice del LED.
 * @param[out] color  Puntero donde se almacenará el color.
 */
void ws2812b_get_led(int index, ws2812b_color_t *color);

/**
 * @brief  Envía los datos del buffer interno a los LEDs físicos.
 *
 * Es necesario llamar a esta función para que los cambios realizados
 * con ws2812b_set_led*() se reflejen en la tira.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
esp_err_t ws2812b_refresh(void);

/**
 * @brief  Apaga todos los LEDs (pone el buffer a cero).
 *
 * @note No envía los datos a la tira; hay que llamar a
 *       ws2812b_refresh() para hacerlo efectivo.
 */
void ws2812b_clear_all(void);

/**
 * @brief  Establece todos los LEDs con el mismo color usando
 *         componentes RGB.
 *
 * @param[in] r  Componente rojo  (0-255).
 * @param[in] g  Componente verde (0-255).
 * @param[in] b  Componente azul  (0-255).
 */
void ws2812b_set_all_rgb(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  Establece todos los LEDs con el mismo color usando estructura.
 *
 * @param[in] color  Color a aplicar.
 */
static inline void ws2812b_set_all(ws2812b_color_t color) {
    ws2812b_set_all_rgb(color.r, color.g, color.b);
}

/**
 * @brief  Establece todos los LEDs con el mismo color aplicando un
 *         brillo global.
 *
 * @param[in] color       Color base.
 * @param[in] brightness  Brillo global (0-255).
 */
static inline void ws2812b_set_all_brightness(ws2812b_color_t color,
                                              uint8_t brightness) {
    ws2812b_color_t dimmed = ws2812b_apply_brightness(color, brightness);
    ws2812b_set_all(dimmed);
}

/**
 * @brief  Obtiene el número de LEDs configurados.
 *
 * @return Número de LEDs en la tira, o 0 si no está inicializada.
 */
int ws2812b_get_num_leds(void);

/**
 * @brief  Verifica si la librería está inicializada.
 *
 * @return true si está inicializada, false en caso contrario.
 */
bool ws2812b_is_initialized(void);

#ifdef __cplusplus
}
#endif