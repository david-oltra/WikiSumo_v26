/**
 * @file    rc5_decode.c
 * @brief   Implementación del decodificador de tramas RC5 vía GPIO.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   El decodificador se basa en una ISR que registra cada flanco
 *   (subida/bajada) con su timestamp. Cuando transcurre un tiempo
 *   superior a TIMEOUT_US desde el primer flanco, se considera que
 *   la trama ha terminado y se decodifica la secuencia de duraciones
 *   en bits (1 = pulso, 0 = espacio), reconstruyendo después la
 *   dirección y el comando según el formato RC5 (14 bits).
 *
 * Formato de trama RC5:
 *   [S1 S2 | T | A4 A3 A2 A1 A0 | C5 C4 C3 C2 C1 C0]
 *   - S1, S2 : bits de start (siempre 1, 1)
 *   - T      : bit de toggle (cambia en cada pulsación)
 *   - A4..A0 : dirección (5 bits)
 *   - C5..C0 : comando (6 bits)
 *
 * ⚠️  NOTAS DE SEGURIDAD (ISR):
 *   decode_edges() se ejecuta en contexto de interrupción (llamada
 *   desde gpio_isr). Por eso se evita llamar a ESP_EARLY_LOGI desde
 *   ella (ver bloque comentado al final de decode_edges). Si se
 *   necesita depurar, hacerlo desde la tarea que llama a
 *   rc5_decode_get_frame().
 *
 *   Del mismo modo, strcat() y strlen() en decode_edges() no son
 *   IRAM-safe. Funciona mientras la flash cache esté activa, pero
 *   conviene tenerlo presente si se endurece el ISR.
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "rc5_decode.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "RC5_DEC";

/** Duración nominal de un bit RC5 (µs). */
#define RC5_BIT_TIME            889

/**
 * Timeout para considerar terminada la trama.
 * La trama RC5 son 14 bits ≈ 24.9 ms. El timeout debe superar eso,
 * más margen.
 */
#define TIMEOUT_US              30000

/** Duración mínima para considerar una arista válida (filtro anti-ruido). */
#define MIN_IGNORE_THRESHOLD    300

/* Umbrales para decodificar la duración de un pulso o espacio.
 * Cada umbral marca el límite superior para 1, 2, 3... bits
 * consecutivos del mismo nivel. */
#define THRESHOLD_1             (RC5_BIT_TIME * 3 / 2)
#define THRESHOLD_2             (RC5_BIT_TIME * 5 / 2)
#define THRESHOLD_3             (RC5_BIT_TIME * 7 / 2)
#define THRESHOLD_4             (RC5_BIT_TIME * 9 / 2)
#define THRESHOLD_5             (RC5_BIT_TIME * 11 / 2)
#define THRESHOLD_6             (RC5_BIT_TIME * 13 / 2)

/** Número máximo de aristas que se almacenan por trama. */
#define MAX_EDGES               100

/* =====================================================================
 *  ESTADO INTERNO
 * ===================================================================== */

/** Estado interno del decodificador. */
static struct {
    int64_t  edges_time[MAX_EDGES];  /*!< Timestamps relativos de cada arista (µs) */
    int      edges_type[MAX_EDGES];  /*!< Tipo de arista: 0 = bajada, 1 = subida   */
    uint32_t total_edges;            /*!< Número de aristas registradas            */
    bool     active;                 /*!< (reservado, no usado actualmente)        */
    char     bits[256];              /*!< Cadena de bits decodificada              */
    uint8_t  address;                /*!< Dirección RC5 (5 bits)                   */
    uint8_t  command;                /*!< Comando RC5 (6 bits)                     */
    bool     frame_ready;            /*!< true si hay una trama lista para leer    */
    int64_t  start_time_us;          /*!< Timestamp del primer flanco de la trama  */
    bool     first_edge_detected;    /*!< true si ya se detectó el primer flanco   */
} rc5;

/** Pin de entrada configurado. */
static gpio_num_t input_pin = GPIO_NUM_NC;

/** Flag que habilita/deshabilita la captura desde la ISR. */
static bool measuring = false;

/* =====================================================================
 *  FUNCIONES AUXILIARES
 * ===================================================================== */

/**
 * @brief  Traduce la duración de un pulso (nivel alto) al número de
 *         bits "1" correspondientes.
 *
 * @param  d  Duración en µs.
 * @return Cadena con los bits "1" (hasta 6), o "" si es demasiado larga.
 */
static const char *decode_pulse(int64_t d)
{
    if      (d < THRESHOLD_1) return "1";
    else if (d < THRESHOLD_2) return "11";
    else if (d < THRESHOLD_3) return "111";
    else if (d < THRESHOLD_4) return "1111";
    else if (d < THRESHOLD_5) return "11111";
    else if (d < THRESHOLD_6) return "111111";
    else                      return "";
}

/**
 * @brief  Traduce la duración de un espacio (nivel bajo) al número de
 *         bits "0" correspondientes.
 *
 * @param  d  Duración en µs.
 * @return Cadena con los bits "0" (hasta 6), o "" si es demasiado larga.
 */
static const char *decode_space(int64_t d)
{
    if      (d < THRESHOLD_1) return "0";
    else if (d < THRESHOLD_2) return "00";
    else if (d < THRESHOLD_3) return "000";
    else if (d < THRESHOLD_4) return "0000";
    else if (d < THRESHOLD_5) return "00000";
    else if (d < THRESHOLD_6) return "000000";
    else                      return "";
}

/**
 * @brief  Decodifica la secuencia de aristas registradas y rellena
 *         rc5.address, rc5.command y rc5.frame_ready.
 *
 * ⚠️  Se ejecuta en contexto de ISR (llamada desde gpio_isr).
 *     Evita usar funciones no IRAM-safe (ESP_EARLY_LOGI, strcat,
 *     strlen, etc.).
 */
static void decode_edges(void)
{
    rc5.bits[0] = '\0';

    /* Recorremos pares consecutivos de aristas: la diferencia de
     * tiempo entre ellas es la duración del nivel anterior. */
    for (uint32_t i = 0; i < rc5.total_edges - 1; i++) {
        int64_t dur = rc5.edges_time[i + 1] - rc5.edges_time[i];
        if (dur >= MIN_IGNORE_THRESHOLD) {
            const char *dec = (rc5.edges_type[i] == 0)
                              ? decode_pulse(dur)
                              : decode_space(dur);
            strcat(rc5.bits, dec);
        }
    }

    /* La última arista no tiene pareja; se asume 1 bit del nivel
     * contrario (fin de trama). */
    if (rc5.total_edges > 0) {
        if (rc5.edges_type[rc5.total_edges - 1] == 1) strcat(rc5.bits, "0");
        else                                          strcat(rc5.bits, "1");
    }

    int len = strlen(rc5.bits);
    rc5.address     = 0;
    rc5.command     = 0;
    rc5.frame_ready = false;

    if (len >= 8) {
        /* Dirección: bits 3..7 (5 bits) */
        for (int i = 0; i < 5; i++) {
            if (rc5.bits[3 + i] == '1')
                rc5.address |= (1 << (4 - i));
        }
        /* Comando: bits 8..13 (6 bits), sólo si la trama es completa */
        if (len >= 14) {
            for (int i = 0; i < 6; i++) {
                if (rc5.bits[8 + i] == '1')
                    rc5.command |= (1 << (5 - i));
            }
        }
        rc5.frame_ready = true;
    }

    /* ------------------------------------------------------------------
     * ⚠️  LOG DESHABILITADO A PROPÓSITO
     *
     *   Esta función se ejecuta en contexto de ISR (llamada desde
     *   gpio_isr). ESP_EARLY_LOGI usa la UART y NO es seguro desde
     *   una interrupción: puede provocar crashes, corromper logs o
     *   disparar el watchdog.
     *
     *   Si necesitas depurar la trama recibida, loguéala desde la
     *   tarea que llama a rc5_decode_get_frame(), NO desde aquí.
     *
     *   Alternativa segura (si CONFIG_LOG_MODE_IRAM_SUPPORT está
     *   activado): usar ESP_DRAM_LOGI en lugar de ESP_EARLY_LOGI.
     *
     *   ----- CÓDIGO ORIGINAL (comentado, no se ejecuta) -----
     *
     *   ESP_EARLY_LOGI(TAG, "edges=%u bits=%s len=%d addr=0x%02X cmd=0x%02X",
     *                  rc5.total_edges, rc5.bits, len, rc5.address, rc5.command);
     *
     *   ------------------------------------------------------
     * ------------------------------------------------------------------ */
}

/* =====================================================================
 *  ISR
 * ===================================================================== */

/**
 * @brief  ISR que se ejecuta en cada flanco del pin del receptor IR.
 *
 * Máquina de estados:
 *   1) Si aún no se ha detectado el primer flanco de bajada, se espera
 *      a uno y se inicializa el buffer.
 *   2) Si ya estamos dentro de una trama:
 *      - Si el tiempo desde el inicio supera TIMEOUT_US, se decodifica
 *        y se rearma el estado para la siguiente trama.
 *      - Si no, se registra la nueva arista.
 */
static void IRAM_ATTR gpio_isr(void *arg)
{
    if (!measuring) return;

    int64_t now   = esp_timer_get_time();
    int     level = gpio_get_level(input_pin);

    /* ---------- 1) Esperando el primer flanco de bajada ---------- */
    if (!rc5.first_edge_detected) {
        if (level == 0) {
            rc5.first_edge_detected = true;
            rc5.start_time_us       = now;
            rc5.total_edges         = 1;
            rc5.edges_time[0]       = 0;
            rc5.edges_type[0]       = 0;
        }
        return;
    }

    /* ---------- 2) Dentro de una trama ---------- */
    int64_t elapsed = now - rc5.start_time_us;

    if (elapsed > TIMEOUT_US) {
        /* La trama anterior ha terminado. Decodificar. */
        decode_edges();

        /* Rearmamos el estado aquí mismo para poder capturar la
         * SIGUIENTE trama sin perder aristas. */
        rc5.total_edges = 0;

        if (level == 0) {
            /* Esta arista es un flanco de bajada: sirve como primer
             * flanco de la nueva trama. */
            rc5.start_time_us       = now;
            rc5.total_edges         = 1;
            rc5.edges_time[0]       = 0;
            rc5.edges_type[0]       = 0;
            rc5.first_edge_detected = true;
        } else {
            /* Flanco de subida suelto: esperamos al siguiente flanco
             * de bajada para empezar de nuevo. */
            rc5.first_edge_detected = false;
        }
        return;
    }

    /* ---------- 3) Registrar arista dentro de la trama ---------- */
    if (rc5.total_edges < MAX_EDGES) {
        rc5.edges_time[rc5.total_edges] = elapsed;
        rc5.edges_type[rc5.total_edges] = level;
        rc5.total_edges++;
    }
}

/* =====================================================================
 *  API PÚBLICA
 * ===================================================================== */

void rc5_decode_init(gpio_num_t out_pin)
{
    input_pin = out_pin;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << input_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);

    /* Instalamos el servicio ISR una sola vez (compartido con otros
     * módulos que puedan usar GPIO ISR). */
    static bool isr_installed = false;
    if (!isr_installed) {
        gpio_install_isr_service(0);
        isr_installed = true;
    }

    gpio_isr_handler_add(input_pin, gpio_isr, NULL);

    rc5_decode_reset();

    ESP_LOGI(TAG, "Decodificador RC5 iniciado en GPIO %d", input_pin);
}

bool rc5_decode_get_frame(uint8_t *address, uint8_t *command)
{
    if (!rc5.frame_ready) return false;

    if (address) *address = rc5.address;
    if (command) *command = rc5.command;

    rc5.frame_ready = false;

    /* Ya NO hace falta resetear measuring/first_edge_detected aquí:
     * la ISR se autorearma en cada timeout. */
    return true;
}

void rc5_decode_reset(void)
{
    memset(&rc5, 0, sizeof(rc5));
    measuring = true;
    rc5.first_edge_detected = false;
}