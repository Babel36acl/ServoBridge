#include "ble_gap.h"

#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

#include "ble_gatt.h"

static const char *TAG = "ble_gap";

static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t s_cur_mtu = 23;
static volatile uint32_t s_conn_epoch;
static uint8_t s_own_addr_type;

static void start_advertising(void);
static int gap_event(struct ble_gap_event *event, void *arg);
static void on_sync(void);
static void host_task(void *param);

uint16_t ble_gap_conn_handle(void)
{
    return s_conn_handle;
}

uint16_t ble_gap_mtu(void)
{
    return s_cur_mtu;
}

uint32_t ble_gap_conn_epoch(void)
{
    return s_conn_epoch;
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = ble_gatt_nus_svc_uuid();
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "广播字段设置失败 rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)SERVOBRIDGE_BLE_NAME;
    rsp.name_len = strlen(SERVOBRIDGE_BLE_NAME);
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
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params,
                           gap_event, NULL);
    if (rc != 0)
        ESP_LOGE(TAG, "广播启动失败 rc=%d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_conn_handle = event->connect.conn_handle;
        s_cur_mtu = 23;
        s_conn_epoch++;
        ESP_LOGI(TAG, "BLE 已连接 conn=%u", event->connect.conn_handle);
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_cur_mtu = 23;
        s_conn_epoch++;
        ESP_LOGI(TAG, "BLE 已断开，恢复广播");
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_MTU:
        s_cur_mtu = event->mtu.value;
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
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    assert(rc == 0);
    start_advertising();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run(); /* nimble_port_stop() 时返回 */
    nimble_port_freertos_deinit();
}

void ble_gap_start(void)
{
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
}
