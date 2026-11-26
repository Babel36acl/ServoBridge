/* GATT 服务：NUS 透传（6E400001…）+ CTRL 配置（9F1D0001…）。
 *
 * 铁律：IDF 6.x 的 NimBLE 要求每个特性（含只 notify 的 NUS TX）都带非空 access_cb，
 * 否则 ble_gatts_count_cfg 返回 BLE_HS_EINVAL、启动即 abort，且编译期零告警。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 广播名 / GAP 设备名（协议契约，勿改） */
#define SERVOBRIDGE_BLE_NAME "ServoBridge"

/* 广播里要带 NUS 服务 UUID，供 GAP 模块复用 */
const ble_uuid128_t *ble_gatt_nus_svc_uuid(void);

/* 注册 GAP/GATT 基础服务 + NUS/CTRL 服务并设置设备名；须在 nimble_port_init() 之后调用 */
void ble_gatt_init(void);

/* 把一段上行字节按当前 MTU-3 分段 notify 给 conn（整帧一次调用的语义由调用方保证） */
void ble_gatt_notify_frame(uint16_t conn, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
