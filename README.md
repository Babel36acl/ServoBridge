# ServoBridge

BLE(GATT/NUS) ↔ UART0(RS485) 透传桥固件。给 Servo-Assistant 提供「安卓平板 / 任意带 BLE 的电脑 → 无线 485 调试」的链路：

```
平板/PC (BLE GATT) ←→ ESP32-C6 (NUS 透传 + CTRL 配置) ←→ TTL→RS485 隔离模块 ←→ 伺服驱动器
```

## 硬件

- 板子：Waveshare ESP32-C6-Touch-LCD-1.47（只用到 UART0 与 BLE，屏/触摸/IMU 均不参与；换任何 ESP32-C6 板都行）
- 外设：自动收发 TTL→RS485 隔离模块（DE 自动方向，仅接三根线）

## 接线

| ESP32-C6 | TTL→485 模块 |
| --- | --- |
| GPIO17（U0TXD，芯片 TX） | 模块 RXD（接收） |
| GPIO16（U0RXD，芯片 RX） | 模块 TXD（发送） |
| GND | GND |

A/B 端接驱动器 485 总线。注意乐鑫 datasheet 里 **U0TXD=GPIO17、U0RXD=GPIO16**；Waveshare wiki 的快查表把这两个的 TX/RX 标注疑似写反，以实测为准——如果怎么接都不通，把两根信号线对调一次。

console 已改走 USB Serial/JTAG（板上 Type-C），UART0 完全干净；复位瞬间 ROM 引导可能向总线吐几个字节，从站会当错帧丢弃，无碍。

## 服务与特性

| UUID | 作用 |
| --- | --- |
| `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | NUS 服务 |
| `…0002`（NUS RX） | 中央写入 → 整帧落到 UART0 |
| `…0003`（NUS TX） | UART0 整帧 → notify 回中央 |
| `9F1D0001-6E4E-4B52-8A53-2A7F4F3E1001` | CTRL 服务 |
| `9F1D0002-6E4E-4B52-8A53-2A7F4F3E1002` | CTRL 配置特性（read + write） |

CTRL 特性是 6 字节：`[0]` 校验位（0 无 / 1 偶 / 2 奇），`[1]` 停止位（1/2），`[2..5]` 波特率小端 u32。写入即生效并落 NVS，掉电保持；非法值丢弃。出厂默认 19200 8E1。

广播名 `ServoBridge`。

## 帧保真约定（Modbus RTU 依赖 3.5 字符静默定帧界）

- 中央端**必须把一个完整 Modbus 帧放进单个 BLE write**；本固件对下行整帧一次 `uart_write_bytes`，帧内零间隙。
- 上行以 `uart_set_rx_timeout(4)` 的空闲事件切帧，整帧一次 notify；超过 ATT MTU-3 时连续分段，中央端按字节流重组。
- 配置走 CTRL 带外通道，不污染 Modbus 字节流。

## 烧录

```sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyACM0 flash monitor   # USB Serial/JTAG，无需进 boot 模式
```

## 换件说明

- **换手动方向 485 模块（DE/RE 引出）**：GPIO7 接 DE/RE，在 `init_uart()` 加 `uart_set_pin(UART_PORT, -1, -1, 7, -1)` 并 `uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX)`，驱动自动切向并滤掉自己发的回环字节，其余不动（main.c 头注释里也有这段）。
- **换芯片**：全部行为都在 `main/main.c` 一个文件里（约 400 行）：NimBLE NUS 外设 + UART0 事件循环 + 6 字节配置。换平台时按这三个块平移即可，协议本身（UUID、6 字节配置）保持不变，应用侧零改动。
