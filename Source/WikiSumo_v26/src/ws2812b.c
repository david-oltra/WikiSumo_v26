/**
 * @file    ws2812b.c
 * @brief   Implementación de la librería para controlar tiras de LEDs
 *          WS2812B usando RMT en ESP32-C6 / ESP32-S3.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   Implementa el control de tiras WS2812B mediante un encoder RMT
 *   personalizado que traduce bytes a símbolos con los tiempos del
 *   protocolo (T0H, T0L, T1H, T1L) y añade un reset al final de cada
 *   transmisión.
 *
 *   Se mantiene un buffer interno (GRB) con el color de cada LED. Los
 *   cambios en el buffer sólo se envían a la tira al llamar a
 *   ws2812b_refresh().
 *
 *   Además, incluye funciones de utilidad para manipular colores:
 *     - Conversión HSV → RGB.
 *     - Aplicación de brillo (escalado proporcional).
 *     - Mezcla de dos colores.
 *     - Cálculo del complementario.
 *     - Ajuste fino de brillo (delta).
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "ws2812b.h"

#include "driver/rmt_tx.h"

#include "esp_log.h"

#include "soc/rmt_reg.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "ws2812b";

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/** Canal RMT TX usado para transmitir a la tira. */
static rmt_channel_handle_t s_tx_channel  = NULL;

/** Encoder RMT específico para WS2812B. */
static rmt_encoder_handle_t s_led_encoder = NULL;

/** Buffer interno con los bytes a enviar (GRB por LED). */
static uint8_t             *s_led_buffer  = NULL;

/** Número de LEDs configurados. */
static int                  s_num_leds    = 0;

/** Bandera que indica si la librería está inicializada. */
static bool                 s_initialized = false;

/* =====================================================================
 *  COLORES PREDEFINIDOS
 * ===================================================================== */

const ws2812b_color_t WS2812B_RED     = {255,   0,   0};
const ws2812b_color_t WS2812B_GREEN   = {  0, 255,   0};
const ws2812b_color_t WS2812B_BLUE    = {  0,   0, 255};
const ws2812b_color_t WS2812B_YELLOW  = {255, 255,   0};
const ws2812b_color_t WS2812B_CYAN    = {  0, 255, 255};
const ws2812b_color_t WS2812B_MAGENTA = {255,   0, 255};
const ws2812b_color_t WS2812B_WHITE   = {255, 255, 255};
const ws2812b_color_t WS2812B_BLACK   = {  0,   0,   0};

const ws2812b_color_t WS2812B_ORANGE  = {255, 165,   0};
const ws2812b_color_t WS2812B_PURPLE  = {128,   0, 128};
const ws2812b_color_t WS2812B_PINK    = {255, 192, 203};
const ws2812b_color_t WS2812B_LIME    = { 50, 205,  50};
const ws2812b_color_t WS2812B_TEAL    = {  0, 128, 128};
const ws2812b_color_t WS2812B_INDIGO  = { 75,   0, 130};
const ws2812b_color_t WS2812B_GOLD    = {255, 215,   0};
const ws2812b_color_t WS2812B_SILVER  = {192, 192, 192};

/* =====================================================================
 *  FUNCIONES DE UTILIDAD DE COLOR
 * ===================================================================== */

/**
 * @brief  Convierte un color HSV a RGB.
 *
 * Implementa la conversión estándar HSV → RGB con H en grados (0-360),
 * S y V en rango 0-255.
 *
 * @param[in] hue         Tono (0-360 grados).
 * @param[in] saturation  Saturación (0-255).
 * @param[in] value       Valor/Brillo (0-255).
 *
 * @return Color RGB resultante.
 */
ws2812b_color_t ws2812b_hsv(uint16_t hue, uint8_t saturation, uint8_t value)
{
    ws2812b_color_t color;
    uint8_t  sat   = saturation;
    uint8_t  val   = value;
    uint16_t hue6  = (hue % 360) / 60;
    uint16_t hue_f = (hue % 360) - (hue6 * 60);

    uint8_t p = val * (255 - sat) / 255;
    uint8_t q = val * (255 - (sat * hue_f / 60)) / 255;
    uint8_t t = val * (255 - (sat * (60 - hue_f) / 60)) / 255;

    switch (hue6) {
        case 0:
            color.r = val; color.g = t;   color.b = p;
            break;
        case 1:
            color.r = q;   color.g = val; color.b = p;
            break;
        case 2:
            color.r = p;   color.g = val; color.b = t;
            break;
        case 3:
            color.r = p;   color.g = q;   color.b = val;
            break;
        case 4:
            color.r = t;   color.g = p;   color.b = val;
            break;
        default:
            color.r = val; color.g = p;   color.b = q;
            break;
    }

    return color;
}

/**
 * @brief  Aplica un factor de brillo a un color.
 *
 * Escala cada componente RGB proporcionalmente al brillo indicado.
 *
 * @param[in] color       Color original.
 * @param[in] brightness  Brillo a aplicar (0-255).
 *
 * @return Color con brillo ajustado.
 */
ws2812b_color_t ws2812b_apply_brightness(ws2812b_color_t color,
                                         uint8_t brightness)
{
    ws2812b_color_t result;

    if (brightness == 255) {
        return color;
    } else if (brightness == 0) {
        result.r = 0;
        result.g = 0;
        result.b = 0;
        return result;
    }

    result.r = (color.r * brightness) / 255;
    result.g = (color.g * brightness) / 255;
    result.b = (color.b * brightness) / 255;

    return result;
}

/**
 * @brief  Mezcla dos colores según un ratio.
 *
 * @param[in] color1  Primer color.
 * @param[in] color2  Segundo color.
 * @param[in] ratio   Proporción del segundo color (0-255).
 *
 * @return Color resultante de la mezcla.
 */
ws2812b_color_t ws2812b_mix_colors(ws2812b_color_t color1,
                                   ws2812b_color_t color2,
                                   uint8_t ratio)
{
    ws2812b_color_t result;
    uint16_t ratio1 = 255 - ratio;
    uint16_t ratio2 = ratio;

    result.r = (color1.r * ratio1 + color2.r * ratio2) / 255;
    result.g = (color1.g * ratio1 + color2.g * ratio2) / 255;
    result.b = (color1.b * ratio1 + color2.b * ratio2) / 255;

    return result;
}

/**
 * @brief  Calcula el color complementario (inverso).
 *
 * @param[in] color  Color original.
 *
 * @return Color complementario.
 */
ws2812b_color_t ws2812b_complementary(ws2812b_color_t color)
{
    ws2812b_color_t result;
    result.r = 255 - color.r;
    result.g = 255 - color.g;
    result.b = 255 - color.b;
    return result;
}

/**
 * @brief  Ajusta el brillo de un color sumando/restando un delta.
 *
 * @param[in] color  Color original.
 * @param[in] delta  Cambio de brillo (-255 a 255).
 *
 * @return Color con brillo ajustado y saturado a 0-255.
 */
ws2812b_color_t ws2812b_adjust_brightness(ws2812b_color_t color, int16_t delta)
{
    ws2812b_color_t result;
    int16_t new_r, new_g, new_b;

    new_r = color.r + delta;
    new_g = color.g + delta;
    new_b = color.b + delta;

    result.r = (new_r < 0) ? 0 : (new_r > 255) ? 255 : (uint8_t)new_r;
    result.g = (new_g < 0) ? 0 : (new_g > 255) ? 255 : (uint8_t)new_g;
    result.b = (new_b < 0) ? 0 : (new_b > 255) ? 255 : (uint8_t)new_b;

    return result;
}

/* =====================================================================
 *  ENCODER PARA WS2812B (RMT)
 * ===================================================================== */

/**
 * @brief  Encoder compuesto para WS2812B.
 *
 * Combina un bytes_encoder (para traducir cada byte a símbolos T0/T1)
 * con un copy_encoder (para emitir el pulso de reset al final).
 */
typedef struct {
    rmt_encoder_t        base;           /*!< Encoder base (debe ser el primer campo) */
    rmt_encoder_handle_t bytes_encoder;  /*!< Encoder de bytes → símbolos             */
    rmt_encoder_handle_t copy_encoder;   /*!< Encoder para emitir el reset            */
} rmt_led_strip_encoder_t;

/**
 * @brief  Función de encode del encoder WS2812B.
 *
 * Traduce los bytes del buffer a símbolos RMT y añade el pulso de
 * reset (nivel bajo) al final de la transmisión.
 *
 * @param[in]  encoder       Puntero al encoder base.
 * @param[in]  channel       Canal RMT asociado.
 * @param[in]  primary_data  Puntero a los datos (buffer GRB).
 * @param[in]  data_size     Tamaño de los datos en bytes.
 * @param[out] ret_state     Estado de la codificación.
 *
 * @return Número de símbolos codificados.
 */
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder,
                                   rmt_channel_handle_t channel,
                                   const void *primary_data,
                                   size_t data_size,
                                   rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder =
        __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder  = led_encoder->copy_encoder;

    rmt_encode_state_t session_state   = RMT_ENCODING_RESET;
    size_t             encoded_symbols = 0;

    /* Codificar los bytes a símbolos RMT */
    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel,
                                             primary_data, data_size,
                                             &session_state);

    if (session_state & RMT_ENCODING_COMPLETE) {
        /* Resetear el encoder de bytes para la siguiente ronda */
        rmt_encoder_reset(bytes_encoder);

        /* Añadir el pulso de reset (nivel bajo) al final */
        rmt_symbol_word_t reset = {
            .duration0 = 100, .level0 = 0,
            .duration1 = 0,   .level1 = 0,
        };
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &reset,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
    }

    *ret_state = session_state;
    return encoded_symbols;
}

/**
 * @brief  Libera los recursos del encoder WS2812B.
 *
 * @param[in] encoder  Puntero al encoder base.
 *
 * @return ESP_OK siempre.
 */
static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder =
        __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

/**
 * @brief  Reinicia el estado interno del encoder WS2812B.
 *
 * @param[in] encoder  Puntero al encoder base.
 *
 * @return ESP_OK siempre.
 */
static esp_err_t rmt_reset_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder =
        __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    return ESP_OK;
}

/**
 * @brief  Crea el encoder WS2812B.
 *
 * Configura los tiempos de bit para el protocolo WS2812B:
 *   - bit 0: T0H = 0.4 µs, T0L = 0.85 µs  (a 40 MHz)
 *   - bit 1: T1H = 0.8 µs, T1L = 0.45 µs  (a 40 MHz)
 *
 * MSB primero, y añade un pulso de reset (>50 µs) al final.
 *
 * @param[out] ret_encoder  Handle del encoder creado.
 *
 * @return ESP_OK en éxito, código de error en caso contrario.
 */
static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;

    led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    if (!led_encoder) {
        return ESP_ERR_NO_MEM;
    }

    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del    = rmt_del_led_strip_encoder;
    led_encoder->base.reset  = rmt_reset_led_strip_encoder;

    /* Encoder de bytes → símbolos RMT con los tiempos del WS2812B */
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .duration0 = 12,   /* 0.4 µs @ 40 MHz = 16 ticks (ajustado para C6) */
            .level0    = 1,
            .duration1 = 28,   /* 0.85 µs @ 40 MHz = 34 ticks                   */
            .level1    = 0,
        },
        .bit1 = {
            .duration0 = 28,   /* 0.8 µs @ 40 MHz = 32 ticks                    */
            .level0    = 1,
            .duration1 = 12,   /* 0.45 µs @ 40 MHz = 18 ticks                   */
            .level1    = 0,
        },
        .flags.msb_first = 1,  /* MSB primero                                   */
    };

    ret = rmt_new_bytes_encoder(&bytes_encoder_config,
                                &led_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(led_encoder);
        return ret;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config,
                               &led_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return ret;
    }

    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

/**
 * @brief  Inicializa la tira de LEDs WS2812B.
 *
 * Configura el canal RMT, el encoder WS2812B, habilita el canal y
 * reserva el buffer interno (GRB) para todos los LEDs.
 *
 * Si la librería ya estaba inicializada, devuelve ESP_OK sin hacer
 * nada.
 *
 * @param[in] config  Puntero a la estructura de configuración.
 *
 * @return
 *   - ESP_OK en éxito.
 *   - ESP_ERR_INVALID_ARG si config es NULL o tiene valores inválidos.
 *   - ESP_ERR_NO_MEM si falla la reserva del buffer.
 *   - Código de error de ESP-IDF en caso contrario.
 */
esp_err_t ws2812b_init(const ws2812b_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "WS2812B ya inicializado");
        return ESP_OK;
    }

    if (!config || config->num_leds <= 0 || config->gpio_num < 0) {
        ESP_LOGE(TAG, "Configuración inválida");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Inicializando WS2812B: %d LEDs en GPIO %d",
             config->num_leds, config->gpio_num);

    /* Configuración del canal RMT TX */
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = config->gpio_num,
        .mem_block_symbols = 48,
        .resolution_hz     = config->rmt_resolution_hz
                              ? config->rmt_resolution_hz
                              : 40000000,
        .trans_queue_depth = 4,
        .flags.with_dma    = false,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &s_tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al crear canal RMT: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rmt_new_led_strip_encoder(&s_led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al crear encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        return ret;
    }

    ret = rmt_enable(s_tx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar canal RMT: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_led_encoder);
        s_led_encoder = NULL;
        rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        return ret;
    }

    s_num_leds   = config->num_leds;
    s_led_buffer = calloc(s_num_leds, 3);
    if (!s_led_buffer) {
        ESP_LOGE(TAG, "Error al asignar memoria");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "WS2812B inicializado correctamente");
    return ESP_OK;
}

/**
 * @brief  Desinicializa la tira de LEDs y libera todos los recursos.
 *
 * Libera el buffer interno, el encoder y el canal RMT. Después de
 * llamar a esta función es necesario volver a llamar a ws2812b_init()
 * antes de usar cualquier otra función de la librería.
 */
void ws2812b_deinit(void)
{
    if (!s_initialized) return;

    if (s_led_buffer) {
        free(s_led_buffer);
        s_led_buffer = NULL;
    }

    if (s_led_encoder) {
        rmt_del_encoder(s_led_encoder);
        s_led_encoder = NULL;
    }

    if (s_tx_channel) {
        rmt_disable(s_tx_channel);
        rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
    }

    s_initialized = false;
    s_num_leds    = 0;
    ESP_LOGI(TAG, "WS2812B desinicializado");
}

/**
 * @brief  Asigna un color a un LED específico usando componentes RGB.
 *
 * Internamente se almacena en orden GRB, que es el que espera el
 * WS2812B. Si el índice está fuera de rango o la librería no está
 * inicializada, la función no hace nada.
 *
 * @param[in] index  Índice del LED (0 = primer LED).
 * @param[in] r      Componente rojo  (0-255).
 * @param[in] g      Componente verde (0-255).
 * @param[in] b      Componente azul  (0-255).
 *
 * @note Los cambios no se reflejan en la tira hasta llamar a
 *       ws2812b_refresh().
 */
void ws2812b_set_led_rgb(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized || !s_led_buffer) return;
    if (index < 0 || index >= s_num_leds) return;

    /* Orden GRB para WS2812B */
    s_led_buffer[index * 3 + 0] = g;  /* Green */
    s_led_buffer[index * 3 + 1] = r;  /* Red   */
    s_led_buffer[index * 3 + 2] = b;  /* Blue  */
}

/**
 * @brief  Obtiene el color actual de un LED del buffer interno.
 *
 * @param[in]  index  Índice del LED.
 * @param[out] color  Puntero donde se almacenará el color.
 *
 * @note Si el índice está fuera de rango o color es NULL, la función
 *       no modifica nada.
 */
void ws2812b_get_led(int index, ws2812b_color_t *color)
{
    if (!s_initialized || !s_led_buffer || !color) return;
    if (index < 0 || index >= s_num_leds) return;

    color->g = s_led_buffer[index * 3 + 0];
    color->r = s_led_buffer[index * 3 + 1];
    color->b = s_led_buffer[index * 3 + 2];
}

/**
 * @brief  Envía los datos del buffer interno a los LEDs físicos.
 *
 * Realiza la transmisión RMT del buffer completo (num_leds × 3 bytes)
 * y espera hasta 100 ms a que termine.
 *
 * @return
 *   - ESP_OK en éxito.
 *   - ESP_ERR_INVALID_STATE si la librería no está inicializada.
 *   - Código de error de ESP-IDF si falla la transmisión.
 */
esp_err_t ws2812b_refresh(void)
{
    if (!s_initialized || !s_tx_channel || !s_led_encoder || !s_led_buffer) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count      = 0,
        .flags.eot_level = 0,
    };

    esp_err_t ret = rmt_transmit(s_tx_channel, s_led_encoder, s_led_buffer,
                                 s_num_leds * 3, &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al transmitir: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rmt_tx_wait_all_done(s_tx_channel, 100);
    return ret;
}

/**
 * @brief  Apaga todos los LEDs (pone el buffer a cero).
 *
 * @note Los cambios no se reflejan en la tira hasta llamar a
 *       ws2812b_refresh().
 */
void ws2812b_clear_all(void)
{
    if (!s_initialized || !s_led_buffer) return;
    memset(s_led_buffer, 0, s_num_leds * 3);
}

/**
 * @brief  Establece todos los LEDs con el mismo color usando
 *         componentes RGB.
 *
 * @param[in] r  Componente rojo  (0-255).
 * @param[in] g  Componente verde (0-255).
 * @param[in] b  Componente azul  (0-255).
 *
 * @note Los cambios no se reflejan en la tira hasta llamar a
 *       ws2812b_refresh().
 */
void ws2812b_set_all_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized || !s_led_buffer) return;

    for (int i = 0; i < s_num_leds; i++) {
        s_led_buffer[i * 3 + 0] = g;
        s_led_buffer[i * 3 + 1] = r;
        s_led_buffer[i * 3 + 2] = b;
    }
}

/**
 * @brief  Obtiene el número de LEDs configurados.
 *
 * @return Número de LEDs en la tira, o 0 si no está inicializada.
 */
int ws2812b_get_num_leds(void)
{
    return s_num_leds;
}

/**
 * @brief  Verifica si la librería está inicializada.
 *
 * @return true si está inicializada, false en caso contrario.
 */
bool ws2812b_is_initialized(void)
{
    return s_initialized;
}