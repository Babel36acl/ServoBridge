# ServoBridge v0.1.0

ESP32-C6 的 BLE(NUS) ↔ UART0(RS485) 透传桥固件首个发版。给 Servo-Assistant 提供
「安卓平板 / 任意带 BLE 的电脑 → 无线 485 调试」链路。

## 功能

- **NUS 透传**：`6E400001-B5A3-F393-E0A9-E50E24DCCA9E` 服务，RX 写入整帧落 UART0，
  TX 按 UART0 空闲（4 符号时间）切帧后整帧 notify，超过 ATT MTU-3 自动分段。
- **CTRL 配置**：`9F1D0002-6E4E-4B52-8A53-2A7F4F3E1002` 读写 6 字节
  `[校验 0无/1偶/2奇][停止位 1/2][波特率 u32 小端]`，写入即生效并落 NVS，掉电保持；
  非法值返回 ATT 错误且不改配置。
- **出厂默认 19200 8E1**，对应字节 `01 01 00 4B 00 00`。
- **固件版本**：`9F1D0003-6E4E-4B52-8A53-2A7F4F3E1003` 只读 UTF-8 字符串，
  与启动日志打印的版本一致（本版为 `0.1.0`）。
- 广播名 `ServoBridge`，单连接，MTU 首选 256。

## 烧录

三种方式任选：

1. **合并镜像（推荐）**：`ServoBridge-0.1.0-esp32c6-merged.bin` 已包含 bootloader +
   分区表 + 应用，直接写 `0x0`：

   ```sh
   esptool.py --chip esp32c6 -p COM3 write_flash 0x0 ServoBridge-0.1.0-esp32c6-merged.bin
   ```

   或打开 [ESP Web Flasher](https://espressif.github.io/esptool-js/)（Chrome/Edge），
   选 ESP32-C6、地址填 `0x0`、选同一个 bin 烧录。板子走 USB Serial/JTAG，无需按 BOOT。

2. **分体镜像 + flash_args**：把 `-app.bin`、`-bootloader.bin`、`-partition-table.bin`
   与 `-flash_args` 放在同一目录，在该目录执行：

   ```sh
   esptool.py --chip esp32c6 write_flash @ServoBridge-0.1.0-esp32c6-flash_args
   ```

3. **源码编译**：ESP-IDF v6.1.1 下 `idf.py set-target esp32c6 && idf.py build`，
   然后 `idf.py -p COM3 flash`。

## 产物

| 文件 | 用途 |
| --- | --- |
| `ServoBridge-0.1.0-esp32c6-merged.bin` | 单文件整片镜像，烧 `0x0` |
| `ServoBridge-0.1.0-esp32c6-app.bin` | 应用分区，烧 `0x10000` |
| `ServoBridge-0.1.0-esp32c6-bootloader.bin` | bootloader，烧 `0x0` |
| `ServoBridge-0.1.0-esp32c6-partition-table.bin` | 分区表，烧 `0x8000` |
| `ServoBridge-0.1.0-esp32c6-flash_args` | 上述三个分体镜像的偏移清单，供 `write_flash @file` |

## 已验证 / 未验证

- 已验证：本机（Windows + ESP-IDF v6.1.1）`idf.py build` 通过；协议链路曾于 2026-11-20
  在实机验证（扫描 → 连接 MTU=256 → CTRL 读回 `01 01 00 4B 00 00` → 在线改 9600 8N1
  并回读 → NUS 整帧写入不崩）。本次发版是模块化重构与缺陷修复，协议与正常路径行为未变。
- 未验证：重构后的固件尚未重新烧录实机复测；RS485 物理回包也未闭环——`NUS → UART0 →
  TTL/485 模块 → 伺服驱动器 → 回包` 还没有对端设备实测，接上 485 模块后用
  Servo-Assistant 蓝牙通道真连一次即可闭环。
- 未验证：本 release 的 GitHub Actions 产物由 workflow 生成，维护者打 tag 后需确认
  Actions 日志与 release 资产。
