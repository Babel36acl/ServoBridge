# AGENTS.md — 给 AI 协作者的固件仓库速查

## 一句话

ESP32-C6 BLE(NUS) ↔ UART0(RS485) 透传桥固件，给 Servo-Assistant 提供无线 485 调试链路。代码按职责拆在 `main/` 下（见下表），协议（NUS UUID、6 字节 CTRL 配置）与应用侧 `Servo-Assistant/src-tauri/src/ble.rs` 是硬契约，改协议两边一起动。

## 代码结构（main/）

| 文件 | 职责 |
| --- | --- |
| `main.c` | 只做 `app_main` 串联：NVS → 配置 → UART → NimBLE → 任务 |
| `bridge_cfg.c/.h` | 6 字节 CTRL 配置、合法性校验 `bridge_cfg_valid`、NVS 读写，出厂默认的唯一来源 |
| `uart_bridge.c/.h` | UART0 初始化与重配、空闲切帧上行、溢出恢复、下行写入口 `uart_bridge_write` |
| `ble_gatt.c/.h` | NUS / CTRL / 版本特性的 UUID 与 GATT 表、access_cb、notify 分段 |
| `ble_gap.c/.h` | 广播、连接事件、MTU 与连接代数（`ble_gap_conn_epoch`）getter |
| `fw_version.c/.h` | 读 app 描述符里的 `PROJECT_VER`，供启动日志与 `9F1D0003` 只读特性 |

跨模块状态一律走 getter，不加新的全局 extern。

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
- `main` 组件**别加显式 REQUIRES**：一加就失去"自动依赖全部组件"，`nvs.h` 等头会断；`main/CMakeLists.txt` 只列全 SRCS + `INCLUDE_DIRS "."`。
- CTRL 6 字节 `[校验 0N/1E/2O][停止位 1..2][波特率 u32 LE]`，出厂默认 19200 8E1 = `01 01 00 4B 00 00`（曾错写成 `80 4B`=19328，勿回退）。
- `ble_gatts_notify_custom` 无论成败都接管 mbuf，失败后**不要**再 `os_mbuf_free_chain`（会双重释放）。
- 提交中文、原子化、显式路径暂存；提交要 SSH 签名（密钥在 Bitwarden agent），git-bash 里用 `/c/Program Files/Git/cmd/git.exe`。

## 发版

- 推 `vX.Y.Z` 标签触发 `firmware.yml` 的 release job：`SERVOBRIDGE_VERSION` 注入版本 → 编译 → `merge_bin` 合并镜像 → 发布 merged / app / bootloader / partition-table / flash_args 五件，说明取 `docs/release-notes.md`。打标签前先更新该文件。

## 验证现状（2026-11-26）

- 2026-11-20 单文件版编译/烧录/BLE 全链路实机通过（见 README 实测记录）；2026-11-26 模块化重构与 5 处修复只做了本机编译，未重新烧录。RS485 物理回包未验证，接上 485 模块后用 Servo-Assistant 蓝牙通道真连一次闭环。
