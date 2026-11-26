#include "ble_gatt.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_gap.h"
#include "bridge_cfg.h"
#include "fw_version.h"
#include "uart_bridge.h"

#define BLE_NAME SERVOBRIDGE_BLE_NAME

static const char *TAG = "ble_gatt";

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
static const ble_uuid128_t CTRL_VER_UUID = /* 只读固件版本，UTF-8 字符串 */
    BLE_UUID128_INIT(0x03, 0x10, 0x3e, 0x4f, 0x7f, 0x2a, 0x53, 0x8a,
                     0x52, 0x4b, 0x4e, 0x6e, 0x03, 0x00, 0x1d, 0x9f);

static uint16_t s_nus_rx_handle;
static uint16_t s_nus_tx_handle;

/* 在 IDF 6.x 的 NimBLE 里，任何特性（含只 notify 的）都必须带 access_cb，
 * 否则 ble_gatts_count_cfg 返回 BLE_HS_EINVAL、启动即 abort。本回调不会被调用：
 * TX 特性只声明 NOTIFY，没有 READ/WRITE 属性。 */
static int nus_tx_access(uint16_t conn, uint16_t handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)ctxt; (void)arg;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

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
    if (uart_bridge_write(buf, (size_t)len) != len)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0;
}

/* 固件版本特性：只读，返回 PROJECT_VER 对应的 UTF-8 字符串（不含结尾 0） */
static int fw_version_access(uint16_t conn, uint16_t handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR)
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    return os_mbuf_append(ctxt->om, fw_version_string(), (uint16_t)fw_version_length()) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int ctrl_access(uint16_t conn, uint16_t handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, bridge_cfg_bytes(), BRIDGE_CFG_LEN) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return 1;
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != BRIDGE_CFG_LEN)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint8_t in[BRIDGE_CFG_LEN];
    if (os_mbuf_copydata(ctxt->om, 0, len, in) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    if (bridge_cfg_set(in) != ESP_OK)
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    /* 等在途字节发完再换参数，避免半帧换速率 */
    uart_bridge_wait_tx_done();
    uart_bridge_apply_cfg();
    bridge_cfg_store();
    return 0;
}

static const struct ble_gatt_chr_def nus_chrs[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nus_rx_access,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &s_nus_rx_handle,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = nus_tx_access, /* 只 notify；6.x 要求非空回调，实际不会被调用 */
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_nus_tx_handle,
    },
    {0},
};

static const struct ble_gatt_chr_def ctrl_chrs[] = {
    {
        .uuid = &CTRL_CHR_UUID.u,
        .access_cb = ctrl_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        .uuid = &CTRL_VER_UUID.u,
        .access_cb = fw_version_access, /* 只读；6.x 要求非空 access_cb */
        .flags = BLE_GATT_CHR_F_READ,
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

const ble_uuid128_t *ble_gatt_nus_svc_uuid(void)
{
    return &NUS_SVC_UUID;
}

void ble_gatt_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_svc_gap_device_name_set(BLE_NAME);
}

int ble_gatt_notify_frame(uint16_t conn, uint32_t epoch, const uint8_t *data, size_t len)
{
    int total = (int)len;
    int sent = 0;
    while (sent < total) {
        /* 断连/重连后句柄可能被协议栈复用，不能再往「同一个句柄号」塞上一帧的剩余字节 */
        if (ble_gap_conn_handle() != conn || ble_gap_conn_epoch() != epoch)
            break;
        /* 每段重新取 MTU：协商可能在整帧发送途中完成（23 → 256），
         * 按旧 MTU 切出来的分段会超长被丢弃 */
        int payload = (int)ble_gap_mtu() - 3;
        if (payload < 20)
            payload = 20; /* MTU 未协商时按默认 23 */
        int chunk = (total - sent < payload) ? (total - sent) : payload;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data + sent, (uint16_t)chunk);
        if (om == NULL)
            break;
        /* ble_gatts_notify_custom 无论成败都接管并释放 mbuf（NimBLE 头文件原话：
         * "consumes the supplied mbuf regardless of the outcome"），这里不能再 free */
        int rc = ble_gatts_notify_custom(conn, s_nus_tx_handle, om);
        if (rc != 0) {
            ESP_LOGD(TAG, "notify 失败 rc=%d（连接已失效？）", rc);
            break; /* 连接已断，继续发只会白丢剩余分段 */
        }
        sent += chunk;
    }
    return sent;
}
