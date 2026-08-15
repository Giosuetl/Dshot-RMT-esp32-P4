#pragma once
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

typedef enum {
    DSHOT150 = 150000,
    DSHOT300 = 300000,
    DSHOT600 = 600000
} dshot_speed_t;

typedef struct {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    gpio_num_t pin;
    dshot_speed_t speed;
} dshot_channel_t;

esp_err_t dshot_init_channel(dshot_channel_t *dshot, gpio_num_t gpio, uint32_t resolution_hz, dshot_speed_t speed);
esp_err_t dshot_send(dshot_channel_t *dshot, uint16_t throttle_or_cmd, bool telemetry);
void dshot_init_tx_config(void);
void dshot_print(void);