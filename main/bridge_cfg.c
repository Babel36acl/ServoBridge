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
    uint8_t blob[BRIDGE_CFG_LEN];

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "无已保存配置，使用出厂默认 19200 8E1");
        return; /* s_cfg 初值即出厂默认 */
    }
    size_t len = sizeof(blob);
    esp_err_t err = nvs_get_blob(h, NVS_KEY_CFG, blob, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(blob) || !bridge_cfg_valid(blob)) {
        /* 长度正好 6 但内容非法（旧固件残留/写坏）也必须回落：
         * 否则会拿非法波特率去 uart_param_config，ESP_ERROR_CHECK 直接 abort，设备起不来 */
        ESP_LOGW(TAG, "NVS 配置不可用（%s, len=%u），回落出厂默认 19200 8E1",
                 esp_err_to_name(err), (unsigned)len);
        memcpy(s_cfg, k_factory_cfg, sizeof(s_cfg));
        return;
    }
    memcpy(s_cfg, blob, sizeof(s_cfg));
}

void bridge_cfg_store(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS 打开失败（%s），配置只在本次上电有效", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(h, NVS_KEY_CFG, s_cfg, sizeof(s_cfg));
    if (err == ESP_OK)
        err = nvs_commit(h);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "配置写入 NVS 失败（%s），掉电后不保持", esp_err_to_name(err));
    nvs_close(h);
}
