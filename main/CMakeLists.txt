# 不要给 main 加显式 REQUIRES：一加就失去「自动依赖全部组件」，nvs.h 等头会断。
idf_component_register(
    SRCS
        "main.c"
        "bridge_cfg.c"
        "uart_bridge.c"
        "ble_gatt.c"
        "ble_gap.c"
    INCLUDE_DIRS ".")
