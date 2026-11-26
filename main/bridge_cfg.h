/* ServoBridge 串口配置：6 字节 CTRL 结构、合法性校验、NVS 持久化。
 *
 * 6 字节格式（与 Servo-Assistant/src-tauri/src/ble.rs 是硬契约）：
 *   [0] 校验位  0=无 1=偶 2=奇
 *   [1] 停止位  1 或 2
 *   [2..5] 波特率 小端 u32
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BRIDGE_CFG_LEN 6

/* 出厂默认 19200 8E1。曾错写成 80 4B（=19328），勿回退。 */
#define BRIDGE_CFG_FACTORY_DEFAULT { 1, 1, 0x00, 0x4b, 0x00, 0x00 }

/* 当前配置的只读视图，长度恒为 BRIDGE_CFG_LEN（GATT 读用） */
const uint8_t *bridge_cfg_bytes(void);

uint32_t bridge_cfg_baud(void);
uart_parity_t bridge_cfg_parity(void);
uart_stop_bits_t bridge_cfg_stop_bits(void);

/* 6 字节是否合法：校验位 0/1/2、停止位 1/2、波特率 1200..1000000 */
bool bridge_cfg_valid(const uint8_t raw[BRIDGE_CFG_LEN]);

/* 校验后写入 RAM 并返回 ESP_OK；非法返回 ESP_ERR_INVALID_ARG，当前配置不动 */
esp_err_t bridge_cfg_set(const uint8_t raw[BRIDGE_CFG_LEN]);

/* NVS → RAM。命名空间缺失/读失败/内容非法一律回落出厂默认，保证能开机 */
void bridge_cfg_load(void);

/* RAM → NVS。失败只告警不阻断（下次上电回落上次成功保存的值） */
void bridge_cfg_store(void);

#ifdef __cplusplus
}
#endif
