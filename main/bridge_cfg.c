#include "bridge_cfg.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#define NVS_NAMESPACE "bridge"
#define NVS_KEY_CFG   "cfg"

static const char *TAG = "bridge_cfg";

static const uint8_t k_factory_cfg[BRIDGE_CFG_LEN] = BRIDGE_CFG_FACTORY_DEFAULT;

/* GATT 的读/写回调都在 NimBLE host task 上串行执行，uart_task 不读这份配置，
 * 所以保持与原单文件实现一致：不加锁。 */
static uint8_t s_cfg[BRIDGE_CFG_LEN] = BRIDGE_CFG_FACTORY_DEFAULT;

const uint8_t *bridge_cfg_bytes(void)
{
    return s_cfg;
}

uint32_t bridge_cfg_baud(void)
{
    return (uint32_t)s_cfg[2] | ((uint32_t)s_cfg[3] << 8) |
           ((uint32_t)s_cfg[4] << 16) | ((uint32_t)s_cfg[5] << 24);
}

uart_parity_t bridge_cfg_parity(void)
{
    switch (s_cfg[0]) {
    case 0: return UART_PARITY_DISABLE;
    case 1: return UART_PARITY_EVEN;
    default: return UART_PARITY_ODD;
    }
}

uart_stop_bits_t bridge_cfg_stop_bits(void)
{
    return s_cfg[1] == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
}

bool bridge_cfg_valid(const uint8_t raw[BRIDGE_CFG_LEN])
{
    if (raw[0] > 2 || (raw[1] != 1 && raw[1] != 2))
        return false;
    uint32_t baud = (uint32_t)raw[2] | ((uint32_t)raw[3] << 8) |
                    ((uint32_t)raw[4] << 16) | ((uint32_t)raw[5] << 24);
    return baud >= 1200 && baud <= 1000000;
}

esp_err_t bridge_cfg_set(const uint8_t raw[BRIDGE_CFG_LEN])
{
    if (!bridge_cfg_valid(raw))
        return ESP_ERR_INVALID_ARG;
    memcpy(s_cfg, raw, sizeof(s_cfg));
    return ESP_OK;
}

void bridge_cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return;
    size_t len = sizeof(s_cfg);
    if (nvs_get_blob(h, NVS_KEY_CFG, s_cfg, &len) != ESP_OK ||
        len != sizeof(s_cfg)) {
        memcpy(s_cfg, k_factory_cfg, sizeof(s_cfg));
    }
    nvs_close(h);
}

void bridge_cfg_store(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    if (nvs_set_blob(h, NVS_KEY_CFG, s_cfg, sizeof(s_cfg)) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
}
