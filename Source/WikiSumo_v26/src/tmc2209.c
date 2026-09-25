/**
 * @file    tmc2209.c
 * @brief   Implementación del driver para el controlador de motor
 *          paso a paso TMC2209.
 *
 * @author  david_wiki
 * @date    2026
 * @licencia GPL-3.0 license
 *
 * Descripción:
 *   El módulo ofrece dos vías de control complementarias:
 *
 *     1) STEP/DIR con RMT (tmc2209_move_motor).
 *        Genera ráfagas de pulsos STEP de forma precisa usando el
 *        periférico RMT del ESP32. Es la vía usada para movimientos
 *        cortos y posicionados.
 *
 *     2) VACTUAL por UART (tmc2209_set_velocity).
 *        Utiliza el generador interno del TMC2209. Permite movimiento
 *        continuo a velocidad constante sin intervención de la CPU.
 *
 *   Además incluye:
 *     - Protocolo UART (CRC-8, lectura y escritura de registros).
 *     - Configuración de corriente, microsteps y otros parámetros.
 *     - Diagnóstico (comunicación, DRV_STATUS y corrientes medidas).
 *
 *   La UART es compartida por todos los drivers del bus y se
 *   inicializa una sola vez con tmc2209_init_uart().
 */

/* =====================================================================
 *  INCLUDES
 * ===================================================================== */

#include "tmc2209.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

/* =====================================================================
 *  DEFINES Y CONSTANTES
 * ===================================================================== */

/** Tag de logs del módulo. */
static const char *TAG = "TMC2209";

/**
 * Factor de conversión velocidad → VACTUAL.
 *
 * Con el reloj interno por defecto del TMC2209 (12 MHz):
 *   f_step = VACTUAL * (f_clk / 2^24) = VACTUAL * 0.715 [Hz]
 * Por tanto:  VACTUAL = velocidad_hz / 0.715
 */
#define TMC2209_VACTUAL_FACTOR  0.715f

/* =====================================================================
 *  SECCIÓN 1: STEP/DIR con RMT
 * ===================================================================== */

/**
 * @brief  Callback invocado por el periférico RMT al terminar una
 *         transmisión.
 *
 * Se ejecuta en contexto de interrupción. Envía un evento a la cola
 * para desbloquear la tarea que espera el fin de la transmisión.
 *
 * @param[in] channel    Canal RMT que ha terminado.
 * @param[in] edata      Datos del evento de fin de transmisión.
 * @param[in] user_data  Cola FreeRTOS a la que notificar.
 *
 * @return true si se ha despertado una tarea de mayor prioridad.
 */
static bool rmt_done_callback(rmt_channel_handle_t channel,
                              const rmt_tx_done_event_data_t *edata,
                              void *user_data)
{
    BaseType_t    high_task_wakeup = pdFALSE;
    QueueHandle_t queue            = (QueueHandle_t)user_data;
    xQueueSendFromISR(queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/* =====================================================================
 *  INICIALIZACIÓN
 * ===================================================================== */

/**
 * @brief  Inicializa el periférico UART compartido por todos los drivers.
 *
 * Configura la UART a 115200 bps, 8N1, sin flow control, e instala el
 * driver con buffers de 512 bytes en RX y TX.
 *
 * @param[in] uart_num  Número de UART (p. ej. UART_NUM_0).
 * @param[in] tx_pin    Pin TX del ESP32.
 * @param[in] rx_pin    Pin RX del ESP32.
 */
void tmc2209_init_uart(uint8_t uart_num, uint8_t tx_pin, uint8_t rx_pin)
{
    uart_config_t uart_config = {
        .baud_rate           = 115200,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 512, 512, 20, NULL, 0));

    ESP_LOGI(TAG, "UART inicializado: TX=GPIO%d, RX=GPIO%d", tx_pin, rx_pin);
}

/**
 * @brief  Inicializa un motor específico.
 *
 * Configura el pin DIR, crea la cola RMT, el canal RMT, el encoder,
 * registra el callback de fin de transmisión y deja el motor
 * habilitado con VACTUAL a 0.
 *
 * @param[in,out] motor  Puntero a la estructura tmc2209_t del motor.
 */
void tmc2209_init_motor(tmc2209_t *motor)
{
    /* 1. Configurar pin DIR como salida */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << motor->dir_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(motor->dir_pin, 0);

    /* 2. Crear cola para notificación de fin de transmisión RMT */
    QueueHandle_t queue   = xQueueCreate(1, sizeof(rmt_tx_done_event_data_t));
    motor->rmt_done_queue = (void *)queue;

    /* 3. Configurar canal RMT */
    rmt_tx_channel_config_t tx_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = motor->step_pin,
        .mem_block_symbols = 48,
        .resolution_hz     = 1000000,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &motor->rmt_chan));

    /* 4. Crear encoder (copiar símbolos) */
    rmt_copy_encoder_config_t encoder_config = {0};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &motor->rmt_encoder));

    /* 5. Registrar el callback de fin de transmisión */
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = rmt_done_callback,
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(motor->rmt_chan,
                                                    &cbs,
                                                    motor->rmt_done_queue));

    /* 6. Habilitar el canal RMT */
    ESP_ERROR_CHECK(rmt_enable(motor->rmt_chan));

    /* 7. Estado de VACTUAL a cero (motor parado por defecto) */
    motor->vactual_active   = false;
    motor->current_velocity = 0;

    motor->initialized = true;
    ESP_LOGI(TAG, "Motor inicializado: STEP=%d DIR=%d addr=0x%02X",
             motor->step_pin, motor->dir_pin, motor->uart_addr);
}

/* =====================================================================
 *  GENERACIÓN DE PULSOS RMT (STEP/DIR)
 * ===================================================================== */

/**
 * @brief  Genera una ráfaga de pulsos STEP usando RMT.
 *
 * Bloqueante desde el punto de vista del llamante (espera a que el
 * RMT termine), pero sin consumo activo de CPU durante la transmisión.
 *
 * @param[in,out] motor     Puntero al motor.
 * @param[in]     steps     Número de pasos a generar (positivo).
 * @param[in]     speed_hz  Velocidad en pasos por segundo (Hz).
 *
 * @note
 *   - Si el motor estaba siendo controlado por VACTUAL, se detiene
 *     primero para no mezclar generadores.
 *   - El buffer de símbolos se reserva dinámicamente con malloc.
 */
void tmc2209_move_motor(tmc2209_t *motor, int steps, int speed_hz)
{
    if (!motor->initialized || steps <= 0 || speed_hz <= 0) return;

    /* Si VACTUAL estaba activo, lo paramos para no mezclar generadores */
    if (motor->vactual_active) {
        tmc2209_stop(motor);
    }

    int period_us = 1000000 / speed_hz;
    int half      = period_us / 2;
    if (half < 1) half = 1;

    /* Reservamos un símbolo RMT por cada paso */
    rmt_symbol_word_t *symbols =
        (rmt_symbol_word_t *)malloc(steps * sizeof(rmt_symbol_word_t));
    if (!symbols) {
        ESP_LOGE(TAG, "No memory for %d steps", steps);
        return;
    }
    for (int i = 0; i < steps; i++) {
        symbols[i].level0    = 1;
        symbols[i].duration0 = half;
        symbols[i].level1    = 0;
        symbols[i].duration1 = half;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags      = {0},
    };
    esp_err_t err = rmt_transmit(motor->rmt_chan,
                                 motor->rmt_encoder,
                                 symbols,
                                 steps * sizeof(rmt_symbol_word_t),
                                 &tx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_transmit failed: %d", err);
        free(symbols);
        return;
    }

    /* Cálculo del timeout: duración estimada + margen */
    int duration_ms = (steps * 1000) / speed_hz;
    int timeout_ms  = duration_ms + 500;
    if (timeout_ms < 100) timeout_ms = 100;

    /* Esperamos a que el callback de RMT nos notifique el fin */
    QueueHandle_t            queue    = (QueueHandle_t)motor->rmt_done_queue;
    rmt_tx_done_event_data_t evt_data;
    if (xQueueReceive(queue, &evt_data, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for RMT done "
                      "(steps=%d, speed=%d, timeout=%d ms)",
                 steps, speed_hz, timeout_ms);
    }

    free(symbols);
}

/**
 * @brief  Cambia la dirección de giro del motor en modo STEP/DIR.
 *
 * @param[in,out] motor    Puntero al motor.
 * @param[in]     reverse  true = reversa, false = adelante.
 *
 * @note Este pin se ignora cuando el motor se controla por VACTUAL.
 */
void tmc2209_set_direction(tmc2209_t *motor, bool reverse)
{
    gpio_set_level(motor->dir_pin, reverse ? 1 : 0);
}

/* =====================================================================
 *  SECCIÓN 2: PROTOCOLO UART (CRC, escritura, lectura)
 * ===================================================================== */

/**
 * @brief  Calcula el CRC-8 de un buffer según el polinomio 0x07.
 *
 * @param[in] data  Puntero a los datos.
 * @param[in] len   Número de bytes.
 *
 * @return CRC-8 calculado.
 */
static uint8_t tmc2209_crc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            byte >>= 1;
        }
    }
    return crc;
}

/**
 * @brief  Escribe un valor de 32 bits en un registro del TMC2209.
 *
 * Construye el paquete de 8 bytes (sync + addr + reg|W + 4 bytes de
 * datos + CRC) y lo envía por UART.
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     reg    Dirección del registro (0x00 - 0x7F).
 * @param[in]     value  Valor a escribir.
 */
void tmc2209_write(tmc2209_t *motor, uint8_t reg, uint32_t value)
{
    uint8_t packet[8];
    packet[0] = 0x05;                     /* sync              */
    packet[1] = motor->uart_addr;         /* dirección         */
    packet[2] = 0x80 | (reg & 0x7F);      /* write + registro  */
    packet[3] = (value >> 24) & 0xFF;
    packet[4] = (value >> 16) & 0xFF;
    packet[5] = (value >>  8) & 0xFF;
    packet[6] =  value        & 0xFF;
    packet[7] = tmc2209_crc(packet, 7);

    uart_write_bytes(motor->uart_num, (const char *)packet, 8);
}

/**
 * @brief  Lee un registro del TMC2209 y devuelve su valor de 32 bits.
 *
 * Envía una petición de 4 bytes y espera una respuesta de 8 bytes
 * (con preámbulo). Valida el CRC y devuelve el valor leído.
 *
 * @param[in] motor  Puntero al motor.
 * @param[in] reg    Dirección del registro.
 *
 * @return Valor leído, o 0 si falla la comunicación o el CRC.
 */
uint32_t tmc2209_read(tmc2209_t *motor, uint8_t reg)
{
    uint8_t request[4];
    uint8_t buffer[20];

    request[0] = 0x05;
    request[1] = motor->uart_addr;
    request[2] = reg & 0x7F;
    request[3] = tmc2209_crc(request, 3);

    uart_flush_input(motor->uart_num);
    uart_write_bytes(motor->uart_num, (const char *)request, 4);

    vTaskDelay(pdMS_TO_TICKS(10));

    int len = uart_read_bytes(motor->uart_num, buffer, sizeof(buffer),
                              pdMS_TO_TICKS(100));

    if (len >= 12) {
        uint8_t *resp = &buffer[4];

        if (resp[0] == 0x05 && resp[1] == 0xFF) {
            uint8_t calc_crc = tmc2209_crc(resp, 7);
            if (calc_crc == resp[7]) {
                uint32_t value = (resp[3] << 24) | (resp[4] << 16) |
                                 (resp[5] <<  8) |  resp[6];
                return value;
            } else {
                ESP_LOGW(TAG, "CRC mismatch: calc 0x%02X, got 0x%02X",
                         calc_crc, resp[7]);
            }
        } else {
            ESP_LOGW(TAG, "Invalid response header: %02X %02X",
                     resp[0], resp[1]);
        }
    } else {
        ESP_LOGW(TAG, "Short read: only %d bytes", len);
    }
    return 0;
}

/* =====================================================================
 *  SECCIÓN 3: CONFIGURACIÓN DE ALTO NIVEL
 * ===================================================================== */

/**
 * @brief  Habilita el modo UART en el driver.
 *
 * Activa los bits 6 y 7 de GCONF (pdn_disable y mstep_reg_select).
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_configure_uart(tmc2209_t *motor)
{
    uint32_t value = tmc2209_read(motor, 0x00);
    value |= (1 << 6) | (1 << 7);
    tmc2209_write(motor, 0x00, value);
}

/**
 * @brief  Establece la corriente RMS del motor (en miliamperios).
 *
 * Calcula el valor de IRUN según la fórmula del datasheet y actualiza
 * el registro IHOLD_IRUN (0x10). También ajusta IHOLD al mismo valor
 * si IRUN es menor que el IHOLD actual.
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     mA     Corriente deseada en mA (0-2000 aprox.).
 */
void tmc2209_set_current(tmc2209_t *motor, uint16_t mA)
{
    uint32_t value = 0x0001000A;   /* IHOLD=0, IRUN=0, IHOLDDELAY=1 por defecto */
    uint16_t irun  = 0;

    if (mA != 0) {
        float Rsense = 0.1f;
        float I_rms  = mA / 1000.0f;
        /* Fórmula del datasheet para IRUN en función de la corriente RMS */
        float irun_float = 32.0f * 1.41421f * I_rms * (Rsense + 0.02f) / 0.325f - 1.0f;
        irun = (uint8_t)irun_float;
        if (irun > 31) irun = 31;
    }
    value |= (irun << 8);

    if (irun < (value & 0xF)) {
        value &= ~0xF;
        value |= irun;
    }
    tmc2209_write(motor, 0x10, value);
}

/**
 * @brief  Convierte el campo CS_ACTUAL (0-31) a corriente RMS en mA.
 *
 * @param[in] cs_actual  Valor del campo CS_ACTUAL.
 *
 * @return Corriente aproximada en mA.
 */
static uint16_t cs_to_ma(uint8_t cs_actual)
{
    float current_rms = (cs_actual + 1) * 0.325f /
                        (32.0f * 1.41421f * (0.1f + 0.02f)) * 1000.0f;
    return (uint16_t)(current_rms);
}

/**
 * @brief  Obtiene la corriente real que el driver está entregando.
 *
 * Lee DRV_STATUS (0x6F) y extrae el campo CS_ACTUAL.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return Corriente en mA (aproximada).
 */
uint16_t tmc2209_get_current(tmc2209_t *motor)
{
    uint32_t drv_status = tmc2209_read(motor, 0x6F);
    uint8_t  cs_actual  = (drv_status >> 16) & 0x1F;
    return cs_to_ma(cs_actual);
}

/**
 * @brief  Configura la resolución de microsteps.
 *
 * Actualiza los bits MRES del registro CHOPCONF (0x6C).
 *
 * @param[in,out] motor  Puntero al motor.
 * @param[in]     steps  Microsteps: 1, 2, 4, 8, 16, 32, 64, 128, 256.
 */
void tmc2209_set_microsteps(tmc2209_t *motor, uint16_t steps)
{
    uint8_t chopconf = 0;
    switch (steps) {
        case 256: chopconf = 0x0; break;
        case 128: chopconf = 0x1; break;
        case  64: chopconf = 0x2; break;
        case  32: chopconf = 0x3; break;
        case  16: chopconf = 0x4; break;
        case   8: chopconf = 0x5; break;
        case   4: chopconf = 0x6; break;
        case   2: chopconf = 0x7; break;
        case   1: chopconf = 0x8; break;
        default:
            ESP_LOGW(TAG, "Microsteps %d no soportado, usando 16", steps);
            chopconf = 0x4;
            steps    = 16;
    }
    uint32_t value = tmc2209_read(motor, 0x6C);
    value &= ~((1 << 24) | (1 << 25) | (1 << 26) | (1 << 27));
    value |= (chopconf << 24);
    tmc2209_write(motor, 0x6C, value);
}

/**
 * @brief  Lee la configuración actual de microsteps.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return Número de microsteps (1, 2, 4, ... 256).
 */
uint16_t tmc2209_get_microsteps(tmc2209_t *motor)
{
    uint16_t mres     = 0;
    uint32_t chopconf = tmc2209_read(motor, 0x6C);
    switch ((chopconf >> 24) & 0x0F) {
        case 0x0: mres = 256; break;
        case 0x1: mres = 128; break;
        case 0x2: mres =  64; break;
        case 0x3: mres =  32; break;
        case 0x4: mres =  16; break;
        case 0x5: mres =   8; break;
        case 0x6: mres =   4; break;
        case 0x7: mres =   2; break;
        case 0x8: mres =   1; break;
    }
    return mres;
}

/* =====================================================================
 *  SECCIÓN 4: SECUENCIA DE CONFIGURACIÓN PREDEFINIDA
 * ===================================================================== */

/**
 * @brief  Tabla de comandos de configuración predefinidos.
 *
 * Cada entrada es {registro, byte3, byte2, byte1, byte0}, extraída de
 * una captura del analizador lógico.
 */
static const uint8_t config_commands[][5] = {
    {0x00, 0x00, 0x00, 0x01, 0xC4}, /* GCONF          */
    {0x10, 0x00, 0x01, 0x14, 0x0A}, /* IHOLD_IRUN     */
    {0x6C, 0x08, 0x03, 0x00, 0x45}, /* CHOPCONF       */
    {0x11, 0x00, 0x00, 0x00, 0x14}, /* TPOWERDOWN     */
    {0x13, 0x00, 0x00, 0x00, 0x00}, /* TPWMTHRS       */
    {0x22, 0x00, 0x00, 0x00, 0x00}, /* VACTUAL        */
    {0x42, 0x00, 0x00, 0x04, 0xD1}, /* COOLCONF       */
    {0x14, 0x00, 0x00, 0x00, 0x00}, /* TCOOLTHRS      */
    {0x40, 0x00, 0x00, 0x00, 0x00}  /* SGTHRS         */
};

/**
 * @brief  Envía una secuencia predefinida de comandos de configuración.
 *
 * Para cada comando: escribe el registro, lo vuelve a leer y comprueba
 * que el valor leído coincide con el escrito. Al terminar, deja
 * VACTUAL a 0 por seguridad.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_config_init(tmc2209_t *motor)
{
    int32_t value      = 0;
    int32_t read_value = 0;

    for (int i = 0; i < sizeof(config_commands) / sizeof(config_commands[0]); i++) {
        value = (config_commands[i][1] << 24) | (config_commands[i][2] << 16) |
                (config_commands[i][3] <<  8) |  config_commands[i][4];

        tmc2209_write(motor, config_commands[i][0], value);
        vTaskDelay(pdMS_TO_TICKS(10));

        read_value = tmc2209_read(motor, config_commands[i][0]);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (value == read_value) {
            ESP_LOGI(TAG, "CONF OK  REG 0x%02X = 0x%08X",
                     config_commands[i][0], value);
        } else {
            ESP_LOGW(TAG, "CONF ERR REG 0x%02X  WR:0x%08X  RD:0x%08X",
                     config_commands[i][0], value, read_value);
        }
    }

    /* Al terminar la config, dejamos VACTUAL a 0 por seguridad */
    motor->vactual_active   = false;
    motor->current_velocity = 0;
}

/* =====================================================================
 *  SECCIÓN 5: DIAGNÓSTICO
 * ===================================================================== */

/**
 * @brief  Prueba la comunicación con el driver leyendo DRV_STATUS.
 *
 * Realiza hasta 3 intentos de lectura de DRV_STATUS (0x6F) con 50 ms
 * entre ellos. Si obtiene una respuesta no nula, se considera OK.
 *
 * @param[in] motor  Puntero al motor.
 *
 * @return 1 si hay respuesta, 0 si no.
 */
uint8_t tmc2209_test_communication(tmc2209_t *motor)
{
    ESP_LOGI(TAG, "Probando (dir 0x%02X)...", motor->uart_addr);
    for (int i = 0; i < 3; i++) {
        uint32_t drv_status = tmc2209_read(motor, 0x6F);
        if (drv_status != 0) {
            ESP_LOGI(TAG, "✅ respondió: DRV_STATUS=0x%08X", drv_status);
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGE(TAG, "❌ sin respuesta");
    return 0;
}

/**
 * @brief  Imprime por consola el estado completo del driver.
 *
 * Lee DRV_STATUS (0x6F) y decodifica los campos más relevantes:
 * estado del motor, modo de chopper, corriente actual, flags de
 * temperatura, carga abierta, cortocircuitos y sobretemperatura.
 *
 * @param[in] motor  Puntero al motor.
 */
void tmc2209_status(tmc2209_t *motor)
{
    uint32_t drv_status = tmc2209_read(motor, 0x6F);

    bool    stst      = (drv_status >> 31) & 0x01;
    bool    stealth   = (drv_status >> 30) & 0x01;
    uint8_t cs_actual = (drv_status >> 16) & 0x1F;
    bool    t120      = (drv_status >>  8) & 0x01;
    bool    t143      = (drv_status >>  9) & 0x01;
    bool    t150      = (drv_status >> 10) & 0x01;
    bool    t157      = (drv_status >> 11) & 0x01;
    bool    olb       = (drv_status >>  7) & 0x01;
    bool    ola       = (drv_status >>  6) & 0x01;
    bool    s2vsb     = (drv_status >>  5) & 0x01;
    bool    s2vsa     = (drv_status >>  4) & 0x01;
    bool    s2gb      = (drv_status >>  3) & 0x01;
    bool    s2ga      = (drv_status >>  2) & 0x01;
    bool    ot        = (drv_status >>  1) & 0x01;
    bool    otpw      =  drv_status        & 0x01;

    ESP_LOGI(TAG, "=== DRV_STATUS ===");
    ESP_LOGI(TAG, "Motor %s", (stst) ? "parado" : "en marcha");
    ESP_LOGI(TAG, "Driver en modo %s",
             (stealth) ? "StealthChop" : "SpreadCycle");
    ESP_LOGI(TAG, "Actual motor current : %dmA", cs_to_ma(cs_actual));

    if      (t157) ESP_LOGW(TAG, "Temperatura superior a 157º");
    else if (t150) ESP_LOGW(TAG, "Temperatura superior a 150º");
    else if (t143) ESP_LOGW(TAG, "Temperatura superior a 143º");
    else if (t120) ESP_LOGW(TAG, "Temperatura superior a 120º");

    ESP_LOGI(TAG, "Motor : %s", (ola || olb) ? "NO detectado" : "conectado");
    if (ola)   ESP_LOGI(TAG, "Carga abierta detectada en la fase A");
    if (olb)   ESP_LOGI(TAG, "Carga abierta detectada en la fase B");
    if (s2vsa) ESP_LOGW(TAG, "Cortocircuito detectado en el MOSFET de lado bajo de la fase A");
    if (s2vsb) ESP_LOGW(TAG, "Cortocircuito detectado en el MOSFET de lado bajo de la fase B");
    if (s2ga)  ESP_LOGW(TAG, "Cortocircuito a tierra detectado en la fase A");
    if (s2gb)  ESP_LOGW(TAG, "Cortocircuito a tierra detectado en la fase B");
    if (ot)    ESP_LOGW(TAG, "Se ha alcanzado el límite de sobretemperatura");
    if (otpw)  ESP_LOGW(TAG, "Se ha superado el umbral de preaviso de sobretemperatura");

    ESP_LOGI(TAG, "==================");
}

/**
 * @brief  Ejecuta un diagnóstico completo del motor.
 *
 * Comprueba comunicación, imprime DRV_STATUS, mide corriente en reposo,
 * mueve el motor 5000 pasos a 1000 Hz y vuelve a medir la corriente
 * para comparar IHOLD vs IRUN.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_run_full_diagnostic(tmc2209_t *motor)
{
    ESP_LOGI(TAG, "\n========== DIAGNÓSTICO TMC2209 ==========");
    ESP_LOGI(TAG, "");

    if (tmc2209_test_communication(motor)) {
        tmc2209_status(motor);
        vTaskDelay(pdMS_TO_TICKS(10));

        uint16_t ihold = tmc2209_get_current(motor);
        vTaskDelay(pdMS_TO_TICKS(100));

        tmc2209_move_motor(motor, 5000, 1000);
        uint16_t irun = tmc2209_get_current(motor);

        ESP_LOGI(TAG, "=========================================");
        ESP_LOGI(TAG, "Corriente motor parado (IHOLD) %umA", ihold);
        ESP_LOGI(TAG, "Corriente motor marcha (IRUN) %umA", irun);
        ESP_LOGI(TAG, "=========================================");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "======= Test motor %02X finalizado =======", motor->uart_addr);
    ESP_LOGI(TAG, "=========================================\n");
}

/* =====================================================================
 *  SECCIÓN 6: CONTROL POR VACTUAL (UART)
 * ===================================================================== */

/**
 * @brief  Mueve el motor de forma continua usando el generador interno
 *         del TMC2209 (registro VACTUAL).
 *
 * La velocidad se mantiene indefinidamente hasta enviar otro valor,
 * enviar 0, o deshabilitar el driver.
 *
 * @param[in,out] motor     Puntero al motor.
 * @param[in]     speed_hz  Velocidad en microsteps/segundo:
 *                            - > 0 : un sentido.
 *                            - < 0 : sentido inverso.
 *                            - = 0 : parada inmediata.
 *
 * @note
 *   - Al usar VACTUAL, el pin DIR se ignora: el sentido lo marca el signo.
 *   - La parada es inmediata, sin rampa. Para suavidad usar
 *     tmc2209_ramp_velocity().
 */
void tmc2209_set_velocity(tmc2209_t *motor, int32_t speed_hz)
{
    if (!motor->initialized) return;

    /* Conversión a valor VACTUAL (con truncado hacia cero) */
    int32_t vactual = (int32_t)(speed_hz / TMC2209_VACTUAL_FACTOR);

    /* Limitar al rango de 24 bits con signo: ±(2^23 - 1) */
    if (vactual >  8388607) vactual =  8388607;
    if (vactual < -8388607) vactual = -8388607;

    /* Enmascarar a 24 bits (complemento a dos para negativos) */
    uint32_t value = (uint32_t)vactual & 0x00FFFFFF;

    tmc2209_write(motor, 0x22, value);

    /* Actualizar estado interno */
    motor->current_velocity = speed_hz;
    motor->vactual_active   = (speed_hz != 0);
}

/**
 * @brief  Detiene el motor cuando se está controlando por VACTUAL.
 *
 * Escribe 0 en el registro 0x22 (VACTUAL). La parada es inmediata.
 *
 * @param[in,out] motor  Puntero al motor.
 */
void tmc2209_stop(tmc2209_t *motor)
{
    if (!motor->initialized) return;
    tmc2209_write(motor, 0x22, 0);
    motor->current_velocity = 0;
    motor->vactual_active   = false;
}

/**
 * @brief  Realiza una rampa lineal de velocidad hasta el valor objetivo.
 *
 * Útil para arrancar y detener suavemente sin perder pasos.
 *
 * @param[in,out] motor      Puntero al motor.
 * @param[in]     target_hz  Velocidad final en µsteps/s (puede ser negativa).
 * @param[in]     step_hz    Incremento por iteración (siempre positivo).
 * @param[in]     delay_ms   Tiempo entre incrementos en milisegundos.
 */
void tmc2209_ramp_velocity(tmc2209_t *motor, int32_t target_hz,
                           int32_t step_hz, uint32_t delay_ms)
{
    if (!motor->initialized) return;
    if (step_hz <= 0) step_hz = 1;

    int32_t current = motor->current_velocity;

    while (current != target_hz) {
        if (current < target_hz) {
            current += step_hz;
            if (current > target_hz) current = target_hz;
        } else {
            current -= step_hz;
            if (current < target_hz) current = target_hz;
        }
        tmc2209_set_velocity(motor, current);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief  Detiene el motor con una rampa de desaceleración hasta 0.
 *
 * @param[in,out] motor      Puntero al motor.
 * @param[in]     step_hz    Decremento por iteración (positivo).
 * @param[in]     delay_ms   Tiempo entre decrementos en milisegundos.
 */
void tmc2209_stop_ramp(tmc2209_t *motor, int32_t step_hz, uint32_t delay_ms)
{
    tmc2209_ramp_velocity(motor, 0, step_hz, delay_ms);
}