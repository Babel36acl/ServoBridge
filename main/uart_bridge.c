#include "uart_bridge.h"

#include <assert.h>
#include <stdlib.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_hs.h"

#include "ble_gap.h"
#include "ble_gatt.h"
#include "bridge_cfg.h"

#define UART_PORT          UART_NUM_0
#define UART_RX_RING_SIZE  2048
#define UART_TX_RING_SIZE  1024
#define UART_EVT_QUEUE_LEN 24
#define UART_CHUNK_MAX     2048
#define RX_IDLE_SYMBOLS    4 /* 空闲阈值（符号时间），> 3.5 字符，用于切帧 */

static const char *TAG = "uart_bridge";

static QueueHandle_t s_uart_evt_queue;

void uart_bridge_init(void)
{
    uart_config_t cfg = {
        .baud_rate = (int)bridge_cfg_baud(),
        .data_bits = UART_DATA_8_BITS,
        .parity = bridge_cfg_parity(),
        .stop_bits = bridge_cfg_stop_bits(),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    /* 引脚用芯片默认映射：TX=GPIO17(U0TXD)、RX=GPIO16(U0RXD)，显式固定，便于换芯片时对表 */
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_RING_SIZE, UART_TX_RING_SIZE,
                                        UART_EVT_QUEUE_LEN, &s_uart_evt_queue, 0));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT, RX_IDLE_SYMBOLS));
}

void uart_bridge_wait_tx_done(void)
{
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
}

void uart_bridge_apply_cfg(void)
{
    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT, bridge_cfg_baud()));
    ESP_ERROR_CHECK(uart_set_parity(UART_PORT, bridge_cfg_parity()));
    ESP_ERROR_CHECK(uart_set_stop_bits(UART_PORT, bridge_cfg_stop_bits()));
    const uint8_t *raw = bridge_cfg_bytes();
    ESP_LOGI(TAG, "UART0 %u baud，校验=%u，停止位=%u",
             (unsigned)bridge_cfg_baud(), raw[0], raw[1]);
}

static void uart_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(UART_CHUNK_MAX);
    assert(buf != NULL);
    uart_event_t evt;
    for (;;) {
        if (xQueueReceive(s_uart_evt_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;
        if (evt.type == UART_FIFO_OVF || evt.type == UART_BUFFER_FULL) {
            /* 溢出后驱动不会自行恢复：必须清空输入并重置事件队列，
             * 否则后续字节持续被丢弃、错误事件刷屏 */
            ESP_LOGW(TAG, "UART0 接收溢出（%s），清空缓冲重新切帧",
                     evt.type == UART_FIFO_OVF ? "FIFO_OVF" : "BUFFER_FULL");
            uart_flush_input(UART_PORT);
            xQueueReset(s_uart_evt_queue);
            continue;
        }
        if (evt.type != UART_DATA)
            continue;
        uint16_t conn = ble_gap_conn_handle();
        uint32_t epoch = ble_gap_conn_epoch();
        if (conn == BLE_HS_CONN_HANDLE_NONE) {
            /* 无连接也要真把 ring buffer 读空：只 continue 会攒满 2048 字节，
             * 之后驱动持续报 BUFFER_FULL 丢新数据，重连后第一帧还混入陈年半帧 */
            uart_read_bytes(UART_PORT, buf, evt.size, 0);
            continue;
        }
        int len = uart_read_bytes(UART_PORT, buf, evt.size, 0);
        if (len <= 0)
            continue;
        ble_gatt_notify_frame(conn, epoch, buf, (size_t)len);
    }
}

void uart_bridge_start(void)
{
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 12, NULL);
}
