/* 固件版本字符串：来自 PROJECT_VER（CI 是 tag 去掉 v，本机是 git describe）。
 * 运行时从 app 描述符读取，长度受 esp_app_desc_t.version[32] 限制。 */
#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *fw_version_string(void);
size_t fw_version_length(void);

#ifdef __cplusplus
}
#endif
