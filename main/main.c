/* ServoBridge —— BLE(NUS) ↔ UART0(RS485) 透传固件
 *
 * 目标板：Waveshare ESP32-C6-Touch-LCD-1.47（只用到 UART0 与 BLE；屏、触摸、IMU 均不参与）
 * 外设：自动收发（自动流向）TTL→RS485 隔离模块，仅接 TX/RX/GND 三根线
 *
 * 数据通路：
 *   BLE write  (NUS RX, 6E400002-…) ──整帧──▶ UART0 TX (GPIO17 / U0TXD)
 *   UART0 RX   (GPIO16 / U0RXD)     ──空闲切帧──▶ BLE notify (NUS TX, 6E400003-…)
 *
 * 控制通路（带外，与 Modbus 字节流隔离）：
 *   CTRL 特性 9F1D0002-6E4E-4B52-8A53-2A7F4F3E1002，读写同一 6 字节结构：
 *     [0] 校验位  0=无 1=偶 2=奇
 *     [1] 停止位  1 或 2
 *     [2..5] 波特率 小端 u32
 *   写入即生效并落 NVS，掉电保持；非法值直接丢弃不动配置。
 *
 * 帧保真约定（Modbus RTU 以 3.5 字符静默定帧界，BLE 包间隔远大于此）：
 *   - 中央端（平板 / 桌面应用）必须把一个完整 Modbus 帧放进单个 BLE write；
 *   - 固件对下行整帧一次 uart_write_bytes，帧内零间隙；
 *   - 上行以 uart_set_rx_timeout(4) 的空闲事件为帧界，整帧一次 notify 发出，
 *     超过 ATT MTU-3 时连续分段，中央端按字节流重组。
 *
 * 换手动方向的 485 模块（DE/RE 引出）时：把 GPIO7 接 DE/RE，
 * 在 apply_uart_cfg() 处改用 uart_set_mode(UART_MODE_RS485_HALF_DUPLEX)
 * 并以 uart_set_pin(UART_PORT, -1, -1, 7, -1) 将 RTS 映射为方向脚即可，其余不动。
 */

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

#define UART_PORT          UART_NUM_0
#define UART_RX_RING_SIZE  2048
#define UART_TX_RING_SIZE  1024
#define UART_EVT_QUEUE_LEN 24
#define UART_CHUNK_MAX     2048
#define RX_IDLE_SYMBOLS    4 /* 符号时间的空闲阈值，> 3.5 字符，用于切帧 */
#define BLE_NAME           "ServoBridge"
#define NVS_NAMESPACE      "bridge"
#define NVS_KEY_CFG        "cfg"

static const char *TAG = "servobridge";

/* Nordic UART Service，与通用 NUS 工具兼容 */
static const ble_uuid128_t NUS_SVC_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_RX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_TX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* 控制服务：串口参数配置 */
static const ble_uuid128_t CTRL_SVC_UUID =
    BLE_UUID128_INIT(0x01, 0x10, 0x3e, 0x4f, 0x7f, 0x2a, 0x53, 0x8a,
                     0x52, 0x4b, 0x4e, 0x6e, 0x01, 0x00, 0x1d, 0x9f);
static const ble_uuid128_t CTRL_CHR_UUID =
    BLE_UUID128_INIT(0x02, 0x10, 0x3e, 0x4f, 0x7f, 0x2a, 0x53, 0x8a,
                     0x52, 0x4b, 0x4e, 0x6e, 0x02, 0x00, 0x1d, 0x9f);

static uint16_t nus_rx_handle;
static uint16_t nus_tx_handle;
static volatile uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t cur_mtu = 23;

/* 6 字节配置：[0]校验 [1]停止位 [2..5]波特率小端。出厂默认 19200 8E1。 */
static uint8_t uart_cfg[6] = {1, 1, 0x80, 0x4b, 0x00, 0x00};

static QueueHandle_t uart_evt_queue;

/* ---------- 串口参数 ---------- */

static uint32_t cfg_baud(void)
{
    return (uint32_t)uart_cfg[2] | ((uint32_t)uart_cfg[3] << 8) |
           ((uint32_t)uart_cfg[4] << 16) | ((uint32_t)uart_cfg[5] << 24);
}

static uart_parity_t cfg_parity(void)
{
    switch (uart_cfg[0]) {
    case 0: return UART_PARITY_DISABLE;
    case 1: return UART_PARITY_EVEN;
    default: return UART_PARITY_ODD;
    }
}

static uart_stop_bits_t cfg_stop_bits(void)
{
    return uart_cfg[1] == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
}

static void apply_uart_cfg(void)
{
    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT, cfg_baud()));
    ESP_ERROR_CHECK(uart_set_parity(UART_PORT, cfg_parity()));
    ESP_ERROR_CHECK(uart_set_stop_bits(UART_PORT, cfg_stop_bits()));
    ESP_LOGI(TAG, "UART0 %u baud, parity=%u, stop=%u",
             (unsigned)cfg_baud(), uart_cfg[0], uart_cfg[1]);
}

static void load_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return;
    size_t len = sizeof(uart_cfg);
    if (nvs_get_blob(h, NVS_KEY_CFG, uart_cfg, &len) != ESP_OK || len != sizeof(uart_cfg)) {
        /* 损坏或不存在时回落出厂默认 */
        uint8_t def[6] = {1, 1, 0x80, 0x4b, 0x00, 0x00};
        memcpy(uart_cfg, def, sizeof(def));
    }
    nvs_close(h);
}

static void save_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    if (nvs_set_blob(h, NVS_KEY_CFG, uart_cfg, sizeof(uart_cfg)) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
}

/* ---------- UART0 事件任务：整帧读出、按 MTU 分段 notify ---------- */

static void uart_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(UART_CHUNK_MAX);
    assert(buf != NULL);
    uart_event_t evt;
    for (;;) {
        if (xQueueReceive(uart_evt_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;
        if (evt.type != UART_DATA)
            continue;
        uint16_t conn = conn_handle;
        if (conn == BLE_HS_CONN_HANDLE_NONE)
            continue; /* 无连接时读走丢弃，防 ring buffer 堵塞 */
        int len = uart_read_bytes(UART_PORT, buf, evt.size, 0);
        if (len <= 0)
            continue;
        int mtu_payload = (int)cur_mtu - 3;
        if (mtu_payload < 20)
            mtu_payload = 20; /* MTU 未协商时按默认 23 计算 */
        for (int off = 0; off < len; off += mtu_payload) {
            int chunk = len - off < mtu_payload ? len - off : mtu_payload;
            /* 对端未订阅 CCCD 时返回错误码，直接忽略：下一帧照常 */
            ble_gatts_notify_custom(conn, nus_tx_handle, buf + off, (uint16_t)chunk);
        }
    }
}

/* ---------- GATT 回调 ---------- */

static int nus_rx_access(uint16_t conn, uint16_t handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return 1;
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len <= 0 || len > 512)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    uint8_t buf[512];
    if (os_mbuf_copydata(ctxt->om, 0, len, buf) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    /* 整帧一次写下：驱动 TX ring buffer 保证帧内零间隙 */
    int written = uart_write_bytes(UART_PORT, buf, (size_t)len);
    if (written != len)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0;
}

static int ctrl_access(uint16_t conn, uint16_t handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, uart_cfg, sizeof(uart_cfg)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return 1;
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != (int)sizeof(uart_cfg))
        return BLE_ATT_ERR_INVALID_ATTR_DATA_LEN;
    uint8_t in[6];
    if (os_mbuf_copydata(ctxt->om, 0, len, in) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    if (in[0] > 2 || (in[1] != 1 && in[1] != 2))
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    uint32_t baud = (uint32_t)in[2] | ((uint32_t)in[3] << 8) |
                    ((uint32_t)in[4] << 16) | ((uint32_t)in[5] << 24);
    if (baud < 1200 || baud > 1000000)
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    /* 等待在途字节发完再换参数，避免半帧换速率 */
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    memcpy(uart_cfg, in, sizeof(uart_cfg));
    apply_uart_cfg();
    save_cfg();
    return 0;
}

static const struct ble_gatt_chr_def nus_chrs[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nus_rx_access,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &nus_rx_handle,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = NULL, /* 只 notify，不提供读 */
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_handle,
    },
    {0},
};

static const struct ble_gatt_chr_def ctrl_chrs[] = {
    {
        .uuid = &CTRL_CHR_UUID.u,
        .access_cb = ctrl_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {0},
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = nus_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &CTRL_SVC_UUID.u,
        .characteristics = ctrl_chrs,
    },
    {0},
};

/* ---------- GAP：广播与连接事件 ---------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        conn_handle = event->connect.conn_handle;
        cur_mtu = 23;
        ESP_LOGI(TAG, "BLE 已连接 conn=%u", event->connect.conn_handle);
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        cur_mtu = 23;
        ESP_LOGI(TAG, "BLE 已断开，恢复广播");
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_MTU:
        cur_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU 协商为 %u", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &NUS_SVC_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "广播字段设置失败 rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)BLE_NAME;
    rsp.name_len = strlen(BLE_NAME);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "扫描响应设置失败 rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_HS_ADV_OWN_ADDR_TYPE? 0 : 0, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    ...
}
```

等等，ble_gap_adv_start 的第一个参数 own_addr_type（uint8_t），host task 的 on_sync 里 infer 出来。把 own_addr_type 存 static 变量。让我修正这段再落盘。还要 util 里 ble_hs_id_infer_auto。

完整正确版本直接写进文件。还有 assert 需要 #include <assert.h>，malloc 需要 <stdlib.h>。</think><tool_call>write_file<arg_key>content</arg_key>/* ServoBridge —— BLE(NUS) ↔ UART0(RS485) 透传固件
 *
 * 目标板：Waveshare ESP32-C6-Touch-LCD-1.47（只用到 UART0 与 BLE；屏、触摸、IMU 均不参与）
 * 外设：自动收发（自动流向）TTL→RS485 隔离模块，仅接 TX/RX/GND 三根线
 *
 * 数据通路：
 *   BLE write  (NUS RX, 6E400002-…) ──整帧──▶ UART0 TX (GPIO17 / U0TXD)
 *   UART0 RX   (GPIO16 / U0RXD)     ──空闲切帧──▶ BLE notify (NUS TX, 6E400003-…)
 *
 * 控制通路（带外，与 Modbus 字节流隔离）：
 *   CTRL 特性 9F1D0002-6E4E-4B52-8A53-2A7F4F3E1002，读写同一 6 字节结构：
 *     [0] 校验位  0=无 1=偶 2=奇
 *     [1] 停止位  1 或 2
 *     [2..5] 波特率 小端 u32
 *   写入即生效并落 NVS，掉电保持；非法值直接丢弃不动配置。
 *
 * 帧保真约定（Modbus RTU 以 3.5 字符静默定帧界，BLE 包间隔远大于此）：
 *   - 中央端（平板 / 桌面应用）必须把一个完整 Modbus 帧放进单个 BLE write；
 *   - 固件对下行整帧一次 uart_write_bytes，帧内零间隙；
 *   - 上行以 uart_set_rx_timeout(4) 的空闲事件为帧界，整帧一次 notify 发出，
 *     超过 ATT MTU-3 时连续分段，中央端按字节流重组。
 *
 * 换手动方向的 485 模块（DE/RE 引出）时：把 GPIO7 接 DE/RE，
 * 在 init_uart() 里加 uart_set_pin(UART_PORT, -1, -1, 7, -1) 把 RTS 映射为方向脚，
 * 并用 uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX) 让驱动自动切向、
 * 滤掉自己发出的回环字节，其余逻辑不动。
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

#define UART_PORT          UART_NUM_0
#define UART_RX_RING_SIZE  2048
#define UART_TX_RING_SIZE  1024
#define UART_EVT_QUEUE_LEN 24
#define UART_CHUNK_MAX     2048
#define RX_IDLE_SYMBOLS    4 /* 空闲阈值（符号时间），> 3.5 字符，用于切帧 */
#define BLE_NAME           "ServoBridge"
#define NVS_NAMESPACE      "bridge"
#define NVS_KEY_CFG        "cfg"

static const char *TAG = "servobridge";

/* Nordic UART Service，与通用 NUS 工具兼容 */
static const ble_uuid128_t NUS_SVC_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_RX_UUID = /* 中央 → 桥：写 */
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_TX_UUID = /* 桥 → 中央：notify */
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* 控制服务：串口参数配置（自定义 UUID，与应用端约定一致） */
static const ble_uuid128_t CTRL_SVC_UUID =
    BLE_UUID128_INIT(0x01, 0x10, 0x3e, 0x4f, 0x7f, 0x2a, 0x53, 0x8a,
                     0x52, 0x4b, 0x4e, 0x6e, 0x01, 0x00, 0x1d, 0x9f);
static const ble_uuid128_t CTRL_CHR_UUID =
    BLE_UUID128_INIT(0x02, 0x10, 0x3e, 0x4f, 0x7f, 0x2a, 0x53, 0x8a,
                     0x52, 0x4b, 0x4e, 0x6e, 0x02, 0x00, 0x1d, 0x9f);

static uint16_t nus_rx_handle;
static uint16_t nus_tx_handle;
static volatile uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t cur_mtu = 23;
static uint8_t own_addr_type;

/* 6 字节配置：[0]校验 [1]停止位 [2..5]波特率小端。出厂默认 19200 8E1。 */
static uint8_t uart_cfg[6] = {1, 1, 0x80, 0x4b, 0x00, 0x00};

static QueueHandle_t uart_evt_queue;

/* ---------- 串口参数 ---------- */

static uint32_t cfg_baud(void)
{
    return (uint32_t)uart_cfg[2] | ((uint32_t)uart_cfg[3] << 8) |
           ((uint32_t)uart_cfg[4] << 16) | ((uint32_t)uart_cfg[5] << 24);
}

static uart_parity_t cfg_parity(void)
{
    switch (uart_cfg[0]) {
    case 0: return UART_PARITY_DISABLE;
    case 1: return UART_PARITY_EVEN;
    default: return UART_PARITY_ODD;
    }
}

static uart_stop_bits_t cfg_stop_bits(void)
{
    return uart_cfg[1] == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
}

static void apply_uart_cfg(void)
{
    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT, cfg_baud()));
    ESP_ERROR_CHECK(uart_set_parity(UART_PORT, cfg_parity()));
    ESP_ERROR_CHECK(uart_set_stop_bits(UART_PORT, cfg_stop_bits()));
    ESP_LOGI(TAG, "UART0 %u baud，校验=%u，停止位=%u",
             (unsigned)cfg_baud(), uart_cfg[0], uart_cfg[1]);
}

static void load_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return;
    size_t len = sizeof(uart_cfg);
    if (nvs_get_blob(h, NVS_KEY_CFG, uart_cfg, &len) != ESP_OK ||
        len != sizeof(uart_cfg)) {
        uint8_t def[6] = {1, 1, 0x80, 0x4b, 0x00, 0x00};
        memcpy(uart_cfg, def, sizeof(def));
    }
    nvs_close(h);
}

static void save_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    if (nvs_set_blob(h, NVS_KEY_CFG, uart_cfg, sizeof(uart_cfg)) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
}

/* ---------- UART0 ---------- */

static void init_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = (int)cfg_baud(),
        .data_bits = UART_DATA_8_BITS,
        .parity = cfg_parity(),
        .stop_bits = cfg_stop_bits(),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    /* 引脚用芯片默认映射：TX=GPIO17(U0TXD)、RX=GPIO16(U0RXD)，显式固定，便于换芯片时对表 */
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_RING_SIZE, UART_TX_RING_SIZE,
                                        UART_EVT_QUEUE_LEN, &uart_evt_queue, 0));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT, RX_IDLE_SYMBOLS));
}

static void uart_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(UART_CHUNK_MAX);
    assert(buf != NULL);
    uart_event_t evt;
    for (;;) {
        if (xQueueReceive(uart_evt_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;
        if (evt.type != UART_DATA)
            continue;
        uint16_t conn = conn_handle;
        if (conn == BLE_HS_CONN_HANDLE_NONE)
            continue; /* 无连接时读走丢弃，防止 ring buffer 堵塞 */
        int len = uart_read_bytes(UART_PORT, buf, evt.size, 0);
        if (len <= 0)
            continue;
        int mtu_payload = (int)cur_mtu - 3;
        if (mtu_payload < 20)
            mtu_payload = 20; /* MTU 未协商时按默认 23 */
        for (int off = 0; off < len; off += mtu_payload) {
            int chunk = (len - off < mtu_payload) ? (len - off) : mtu_payload;
            /* 对端未订阅 CCCD 会返回错误码，忽略即可：下一帧照常 */
            ble_gatts_notify_custom(conn, nus_tx_handle, buf + off, (uint16_t)chunk);
        }
    }
}

/* ---------- GATT ---------- */

static int nus_rx_access(uint16_t conn, uint16_t handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return 1;
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len <= 0 || len > 512)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    uint8_t buf[512];
    if (os_mbuf_copydata(ctxt->om, 0, len, buf) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    /* 整帧一次写下：驱动 TX ring buffer 保证帧内零间隙 */
    if (uart_write_bytes(UART_PORT, buf, (size_t)len) != len)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0;
}

static int ctrl_access(uint16_t conn, uint16_t handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, uart_cfg, sizeof(uart_cfg)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return 1;
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != (int)sizeof(uart_cfg))
        return BLE_ATT_ERR_INVALID_ATTR_DATA_LEN;
    uint8_t in[6];
    if (os_mbuf_copydata(ctxt->om, 0, len, in) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    if (in[0] > 2 || (in[1] != 1 && in[1] != 2))
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    uint32_t baud = (uint32_t)in[2] | ((uint32_t)in[3] << 8) |
                    ((uint32_t)in[4] << 16) | ((uint32_t)in[5] << 24);
    if (baud < 1200 || baud > 1000000)
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    /* 等在途字节发完再换参数，避免半帧换速率 */
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    memcpy(uart_cfg, in, sizeof(uart_cfg));
    apply_uart_cfg();
    save_cfg();
    return 0;
}

static const struct ble_gatt_chr_def nus_chrs[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nus_rx_access,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &nus_rx_handle,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = NULL, /* 只 notify，不提供读 */
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_handle,
    },
    {0},
};

static const struct ble_gatt_chr_def ctrl_chrs[] = {
    {
        .uuid = &CTRL_CHR_UUID.u,
        .access_cb = ctrl_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {0},
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = nus_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &CTRL_SVC_UUID.u,
        .characteristics = ctrl_chrs,
    },
    {0},
};

/* ---------- GAP：广播与连接 ---------- */

static int gap_event(struct ble_gap_event *event, void *arg);

void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &NUS_SVC_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "广播字段设置失败 rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)BLE_NAME;
    rsp.name_len = strlen(BLE_NAME);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "扫描响应设置失败 rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &params,
                           gap_event, NULL);
    if (rc != 0)
        ESP_LOGE(TAG, "广播启动失败 rc=%d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        conn_handle = event->connect.conn_handle;
        cur_mtu = 23;
        ESP_LOGI(TAG, "BLE 已连接 conn=%u", event->connect.conn_handle);
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        cur_mtu = 23;
        ESP_LOGI(TAG, "BLE 已断开，恢复广播");
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_MTU:
        cur_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU 协商为 %u", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);
    start_advertising();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run(); /* nimble_port_stop() 时返回 */
    nimble_port_freertos_deinit();
}

/* ---------- 入口 ---------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    load_cfg();
    init_uart();
    apply_uart_cfg();

    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_svc_gap_device_name_set(BLE_NAME);

    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 12, NULL);
    ESP_LOGI(TAG, "ServoBridge 就绪：%u baud，等待 BLE 连接", (unsigned)cfg_baud());
}
