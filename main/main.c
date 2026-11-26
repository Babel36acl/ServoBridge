/* ServoBridge —— BLE(NUS) ↔ UART0(RS485) 透传固件（入口）
 *
 * 目标板：Waveshare ESP32-C6-Touch-LCD-1.47（只用到 UART0 与 BLE；屏、触摸、IMU 均不参与）
 * 外设：自动收发（自动流向）TTL→RS485 隔离模块，仅接 TX/RX/GND 三根线
 *
 * 数据通路：
 *   BLE write  (NUS RX, 6E400002-…) ──整帧──▶ UART0 TX (GPIO17 / U0TXD)
 *   UART0 RX   (GPIO16 / U0RXD)     ──空闲切帧──▶ BLE notify (NUS TX, 6E400003-…)
 *
 * 控制通路（带外，与 Modbus 字节流隔离）：
 *   CTRL 特性 9F1D0002-…，读写同一 6 字节结构；写入即生效并落 NVS，掉电保持。
 *   出厂默认 19200 8E1。格式与校验见 bridge_cfg.h。
 *
 * 帧保真约定（Modbus RTU 以 3.5 字符静默定帧界，BLE 包间隔远大于此）：
 *   - 中央端（平板 / 桌面应用）必须把一个完整 Modbus 帧放进单个 BLE write；
 *   - 固件对下行整帧一次 uart_write_bytes，帧内零间隙；
 *   - 上行以 uart_set_rx_timeout(4) 的空闲事件为帧界，整帧一次 notify 发出，
 *     超过 ATT MTU-3 时连续分段，中央端按字节流重组。
 *
 * 换手动方向的 485 模块（DE/RE 引出）时：把 GPIO7 接 DE/RE，
 * 在 uart_bridge.c 的 uart_bridge_init() 里加 uart_set_pin(UART_PORT, -1, -1, 7, -1)
 * 把 RTS 映射为方向脚，并用 uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX)
 * 让驱动自动切向、滤掉自己发出的回环字节，其余逻辑不动。
 */

#include "esp_err.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nvs_flash.h"

#include "ble_gap.h"
#include "ble_gatt.h"
#include "bridge_cfg.h"
#include "uart_bridge.h"

static const char *TAG = "servobridge";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    bridge_cfg_load();
    uart_bridge_init();
    uart_bridge_apply_cfg();

    ESP_ERROR_CHECK(nimble_port_init());
    ble_gatt_init();
    ble_gap_start();

    uart_bridge_start();

    ESP_LOGI(TAG, "ServoBridge 就绪：%u baud，等待 BLE 连接", (unsigned)bridge_cfg_baud());
}
