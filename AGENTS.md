# AGENTS.md — 给 AI 协作者的固件仓库速查

## 一句话

ESP32-C6 BLE(NUS) ↔ UART0(RS485) 透传桥固件，给 Servo-Assistant 提供无线 485 调试链路。全部逻辑在 `main/main.c` 单文件，协议（NUS UUID、6 字节 CTRL 配置）与应用侧 `Servo-Assistant/src-tauri/src/ble.rs` 是硬契约，改协议两边一起动。

## 基准与构建

- **ESP-IDF v6.1.1**（本机 EIM 安装 + CI 同版本，docker 镜像 `espressif/idf:v6.1.1`）。
- 本机（EIM v3 布局）`export.bat` 不能直接用，需要手工拼环境：
  - `IDF_PATH=C:\Users\A1165\.esp\.espressif\v6.1.1\esp-idf`
  - `IDF_TOOLS_PATH=C:\Espressif`（**不是** `C:\Espressif\tools`，IDF 6.x 拼的是 `<path>\tools\<name>\<ver>`）
  - `IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v6.1.1\venv`
  - `ESP_IDF_VERSION=6.1.1`（缺它 idf_component_manager 的 semver coerce 直接 TypeError）
  - PATH 追加 `venv\Scripts`、`idf-exe\1.0.3`、`cmake\4.0.3\bin`、`ninja\1.12.1`、`riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin`、`ccache\4.12.1\ccache-4.12.1-windows-x86_64`
  - 现成模板：`esp-idf-firmware-build` skill 的 `templates/idf-env.bat`
- `C:\Espressif\espidf.constraints.v6.1.1.txt` 缺失会让 idf.py rc=1，从 `tools\` 复制一份到 `C:\Espressif\` 根即消。
- 板子走 USB Serial/JTAG：`idf.py -p COM3 flash`，无需按 BOOT；看启动日志要自己复位（pyserial `rts=True` 0.15s 再放开）。

## 铁律

- IDF 6.x 的 NimBLE：**任何 GATT 特性都必须带非空 `access_cb`**（含只 notify 的 NUS TX），否则 `ble_gatts_count_cfg` 返回 `BLE_HS_EINVAL`、启动即 abort，编译期零告警。
- `main` 组件**别加显式 REQUIRES**：一加就失去"自动依赖全部组件"，`nvs.h` 等头会断；保持 `idf_component_register(SRCS "main.c" INCLUDE_DIRS ".")`。
- CTRL 6 字节 `[校验 0N/1E/2O][停止位 1..2][波特率 u32 LE]`，出厂默认 19200 8E1 = `01 01 00 4B 00 00`（曾错写成 `80 4B`=19328，勿回退）。
- 提交中文、原子化、显式路径暂存；本机 git 签名坏，用 `git -c commit.gpgsign=false commit`。

## 验证现状（2026-11-20）

- 编译/烧录/BLE 全链路实机通过（见 README 实测记录）；RS485 物理回包未验证，接上 485 模块后用 Servo-Assistant 蓝牙通道真连一次闭环。
