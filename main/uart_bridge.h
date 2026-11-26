/* UART0 ↔ BLE 数据搬运：初始化、在线改参数、空闲切帧上行。 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 参数配置 + 引脚 + 驱动安装 + 空闲切帧阈值（TX=GPIO17/U0TXD，RX=GPIO16/U0RXD） */
void uart_bridge_init(void);

/* 等在途字节发完（换参数前调用，避免半帧换速率） */
void uart_bridge_wait_tx_done(void);

/* 按 bridge_cfg 当前值设置波特率/校验/停止位并打日志 */
void uart_bridge_apply_cfg(void);

/* 下行整帧一次写入 UART0（帧内零间隙）；返回实际写入字节数，失败为负 */
int uart_bridge_write(const uint8_t *data, size_t len);

/* 创建 uart_task：UART0 空闲切帧后整帧 notify 给已连接的中央 */
void uart_bridge_start(void);

#ifdef __cplusplus
}
#endif
