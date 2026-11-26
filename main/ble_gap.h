/* GAP：广播、连接、MTU 协商，以及连接状态的唯一来源。 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 设置 sync_cb 并启动 NimBLE host task；须在 ble_gatt_init() 之后调用 */
void ble_gap_start(void);

/* 当前连接句柄；未连接时返回 BLE_HS_CONN_HANDLE_NONE（需包含 host/ble_hs.h） */
uint16_t ble_gap_conn_handle(void);

/* 当前协商 MTU，未连接/未协商时为 23 */
uint16_t ble_gap_mtu(void);

/* 连接代数：每次连接/断开都 +1。句柄值会被协议栈复用，
 * 上行分段时用它判断「还是不是当初那条连接」。 */
uint32_t ble_gap_conn_epoch(void);

#ifdef __cplusplus
}
#endif
