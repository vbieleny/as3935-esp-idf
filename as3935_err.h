#pragma once

#define AS3935_CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define AS3935_CHECK_ARG(x) do { if (!(x)) return ESP_ERR_INVALID_ARG; } while (0)
