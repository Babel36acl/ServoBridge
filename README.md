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
| `9F1D0003-6E4E-4B52-8A53-2A7F4F3E1003` | CTRL 固件版本（read only，UTF-8 字符串） |

CTRL 特性是 6 字节：`[0]` 校验位（0 无 / 1 偶 / 2 奇），`[1]` 停止位（1/2），`[2..5]` 波特率小端 u32。写入即生效并落 NVS，掉电保持；非法值丢弃。出厂默认 19200 8E1。

版本特性返回 `PROJECT_VER`：CI 发版时由 `SERVOBRIDGE_VERSION=<tag 去掉 v>` 注入，本机编译取 `git describe`；同一字符串也会在启动日志里打印。

广播名 `ServoBridge`。

## 帧保真约定（Modbus RTU 依赖 3.5 字符静默定帧界）

- 中央端**必须把一个完整 Modbus 帧放进单个 BLE write**；本固件对下行整帧一次 `uart_write_bytes`，帧内零间隙。
- 上行以 `uart_set_rx_timeout(4)` 的空闲事件切帧，整帧一次 notify；超过 ATT MTU-3 时连续分段，中央端按字节流重组。
- 配置走 CTRL 带外通道，不污染 Modbus 字节流。

## 代码结构（`main/`）

| 文件 | 职责 |
| --- | --- |
| `main.c` | 只有 `app_main`，按启动顺序串联各模块 |
| `bridge_cfg.c/.h` | 6 字节 CTRL 配置、合法性校验、NVS 持久化（namespace `bridge`，key `cfg`） |
| `uart_bridge.c/.h` | UART0 初始化、在线改参数、`uart_task` 空闲切帧上行 |
| `ble_gatt.c/.h` | NUS/CTRL 服务与各 access_cb、notify 分段（含只读版本特性） |
| `ble_gap.c/.h` | 广播、连接/MTU 状态、NimBLE host task |
| `fw_version.c/.h` | `PROJECT_VER` 版本字符串（启动日志与版本特性共用） |

跨模块状态只走 getter：连接状态归 `ble_gap`，串口配置归 `bridge_cfg`。

## 烧录

- 基准版本：**ESP-IDF v6.1.1**（CI 与本机一致；v6.x 起 NimBLE 要求所有特性都带 `access_cb`，只 notify 的 NUS TX 也配了不会被调用的桩）。

```sh
idf.py set-target esp32c6
idf.py build
idf.py -p COM3 flash monitor   # USB Serial/JTAG，无需进 boot 模式
```

Windows 本机（EIM v3 布局）`export.bat` 不能直接用，需要手工拼环境：`IDF_PATH=C:\Users\A1165\.esp\.espressif\v6.1.1\esp-idf`、`IDF_TOOLS_PATH=C:\Espressif`（不是 `C:\Espressif\tools`）、`IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v6.1.1\venv`、`ESP_IDF_VERSION=6.1.1`（缺它 idf_component_manager 直接 TypeError），PATH 追加 venv Scripts、`idf-exe\1.0.3`、`cmake\4.0.3\bin`、`ninja\1.12.1`、`riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin`；`C:\Espressif\espidf.constraints.v6.1.txt` 缺失会让 idf.py rc=1，从 `tools\` 复制一份到 `C:\Espressif\` 即消。`main` 组件保持不写显式 REQUIRES（一写就失去自动依赖全部组件，`nvs.h` 会断）。

## 实测记录（2026-11-20 · Windows + ESP-IDF v6.1.1）

- 硬件：Waveshare ESP32-C6-Touch-LCD-1.47，烧录/控制台走板上 Type-C（COM3）。
- 修复两处实机暴露的问题：
  1. IDF 6.x NimBLE 对 `access_cb == NULL` 的特性在 `ble_gatts_count_cfg` 阶段直接 `BLE_HS_EINVAL` → 启动即 abort（NUS TX 补回调桩）；
  2. 出厂默认波特率字节写成 `80 4B`（=19328），与文档宣称的 19200 不符，已改回 `00 4B`。
- BLE 实测（PC 侧 bleak 直连）：扫描到 `ServoBridge` → 连接 MTU=256 → CTRL 读回 `01 01 00 4B 00 00`（19200 8E1）→ 在线写 9600 8N1 生效并回读一致 → 恢复 19200 → NUS RX 整帧写入、链路无崩溃。
- 固件侧串口日志逐条对应：`BLE 已连接`、`MTU 协商为 256`、`UART0 9600/19200 baud`、`BLE 已断开，恢复广播`。
- 未覆盖：RS485 物理链路（NUS→UART0→485 模块→伺服）尚无对端设备回包验证，接上 485 模块后用应用侧真连一次即可闭环。


## 换件说明

- **换手动方向 485 模块（DE/RE 引出）**：GPIO7 接 DE/RE，在 `uart_bridge.c` 的 `uart_bridge_init()` 里加 `uart_set_pin(UART_PORT, -1, -1, 7, -1)` 并 `uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX)`，驱动自动切向并滤掉自己发的回环字节，其余不动（main.c 头注释里也有这段）。
- **换芯片**：行为分布在 `main/` 的四个模块——`ble_gatt`（NimBLE 服务）、`ble_gap`（广播/连接）、`uart_bridge`（UART0 与切帧）、`bridge_cfg`（配置与 NVS），`main.c` 只是启动串联。换平台时按这四个块平移即可，协议本身（UUID、6 字节配置）保持不变，应用侧零改动。
