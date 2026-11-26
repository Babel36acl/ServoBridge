#include "fw_version.h"

#include <string.h>

#include "esp_app_desc.h"

const char *fw_version_string(void)
{
    return esp_app_get_description()->version;
}

size_t fw_version_length(void)
{
    return strlen(fw_version_string());
}
