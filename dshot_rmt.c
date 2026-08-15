
#include "dshot_rmt.h"
#include "dshot_esc_encoder.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define MAX_DSHOT_CHANNELS 4
static const char *TAG = "DSHOT";

/* ============================
   ESTADO GLOBAL
   ============================ */
static portMUX_TYPE dshot_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool frame_done[MAX_DSHOT_CHANNELS] = {
    true, true, true, true
};

/* ============================
   CALLBACK DE FIN DE ENVÍO
   ============================ */
static bool dshot_tx_end_callback(rmt_channel_handle_t channel,
                                  const rmt_tx_done_event_data_t *edata,
                                  void *user_ctx)
{
    int id = 0;
    rmt_get_channel_id(channel, &id);

    if (id >= 0 && id < MAX_DSHOT_CHANNELS)
    {
        portENTER_CRITICAL_ISR(&dshot_mux);
        frame_done[id] = true;
        portEXIT_CRITICAL_ISR(&dshot_mux);
    }

    return false; // no volver a llamar al scheduler
}

/* ============================
   CONFIGURACIÓN DE ENCODER
   ============================ */
static rmt_transmit_config_t tx_cfg = {
    .loop_count = 0,
    .flags.eot_level = 0
};

/* ============================
   INICIALIZACIÓN DE CANAL
   ============================ */
esp_err_t dshot_init_channel(dshot_channel_t *dshot,
                             gpio_num_t gpio,
                             uint32_t resolution_hz,
                             dshot_speed_t speed)
{
    rmt_tx_channel_config_t tx_ch_cfg = {
        .gpio_num = gpio,
        .clk_src = SOC_MOD_CLK_PLL_F80M,
        .resolution_hz = resolution_hz,
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
        .intr_priority = 1,
        .flags.with_dma = false,
        .flags.invert_out = false
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_ch_cfg, &dshot->channel));

    dshot_esc_encoder_config_t enc_cfg = {
        .resolution = resolution_hz,
        .baud_rate = speed,
        .post_delay_us = 35
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&enc_cfg, &dshot->encoder));
    ESP_ERROR_CHECK(rmt_enable(dshot->channel));

    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = dshot_tx_end_callback
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(dshot->channel, &cbs, NULL));

    int id = 0;
    rmt_get_channel_id(dshot->channel, &id);

    ESP_LOGI(TAG, "Canal DSHOT en GPIO %d -> RMT ID %d", gpio, id);

    return ESP_OK;
}

/* ============================
   ENVÍO ÚNICO NO BLOQUEANTE
   ============================ */
esp_err_t dshot_send(dshot_channel_t *dshot,
                     uint16_t throttle_or_cmd,
                     bool telemetry)
{
    int id = 0;
    rmt_get_channel_id(dshot->channel, &id);

    dshot_esc_throttle_t thr = {
        .throttle = throttle_or_cmd,
        .telemetry_req = telemetry ? 1 : 0
    };

    /* Verificar si el canal está libre */
    portENTER_CRITICAL(&dshot_mux);
    if (!frame_done[id]) {
        portEXIT_CRITICAL(&dshot_mux);
        return ESP_ERR_TIMEOUT;  // el frame anterior no ha terminado
    }
    frame_done[id] = false;
    portEXIT_CRITICAL(&dshot_mux);

    /* Enviar frame */
    return rmt_transmit(dshot->channel, dshot->encoder, &thr, sizeof(thr), &tx_cfg);
}

/* ============================
   ENVÍO CON REINTENTOS
   ============================ */
esp_err_t dshot_send_with_retry(dshot_channel_t *dshot,
                                uint16_t throttle_or_cmd,
                                bool telemetry,
                                int max_retries)
{
    esp_err_t ret;

    for (int i = 0; i < max_retries; i++)
    {
        ret = dshot_send(dshot, throttle_or_cmd, telemetry);
        if (ret != ESP_ERR_TIMEOUT)    // éxito o error real
            return ret;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_ERR_TIMEOUT;
}

#include "soc/soc_caps.h"

/* ============================
   INSPECCIÓN DE CAPACIDADES RMT
   ============================ */
void dshot_print(void)
{
    ESP_LOGI(TAG, "====== RMT Hardware Capabilities ======");

#ifdef SOC_RMT_GROUPS
    ESP_LOGI(TAG, "RMT Groups                : %d", SOC_RMT_GROUPS);
#else
    ESP_LOGI(TAG, "RMT Groups                : not defined");
#endif

#ifdef SOC_RMT_CHANNELS_PER_GROUP
    ESP_LOGI(TAG, "Channels per group        : %d", SOC_RMT_CHANNELS_PER_GROUP);
#endif

#ifdef SOC_RMT_TX_CANDIDATES_PER_GROUP
    ESP_LOGI(TAG, "TX candidates per group   : %d", SOC_RMT_TX_CANDIDATES_PER_GROUP);
#endif

#ifdef SOC_RMT_RX_CANDIDATES_PER_GROUP
    ESP_LOGI(TAG, "RX candidates per group   : %d", SOC_RMT_RX_CANDIDATES_PER_GROUP);
#endif

#ifdef SOC_RMT_MEM_WORDS_PER_CHANNEL
    ESP_LOGI(TAG, "Memory words per channel  : %d", SOC_RMT_MEM_WORDS_PER_CHANNEL);
#endif

#ifdef SOC_RMT_SUPPORT_DMA
    ESP_LOGI(TAG, "DMA support               : %s",
             SOC_RMT_SUPPORT_DMA ? "YES" : "NO");
#endif

#ifdef SOC_RMT_SUPPORT_TX_SYNCHRO
    ESP_LOGI(TAG, "TX synchronization        : %s",
             SOC_RMT_SUPPORT_TX_SYNCHRO ? "YES" : "NO");
#endif

#ifdef SOC_RMT_SUPPORT_TX_LOOP_COUNT
    ESP_LOGI(TAG, "TX loop count             : %s",
             SOC_RMT_SUPPORT_TX_LOOP_COUNT ? "YES" : "NO");
#endif

    ESP_LOGI(TAG, "=======================================");
}
