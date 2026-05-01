#pragma once
#include <stdint.h>

typedef enum {
    ESP_LOG_NONE    = 0,    /*!< No log output */
    ESP_LOG_ERROR   = 1,    /*!< Critical errors, software module can not recover on its own */
    ESP_LOG_WARN    = 2,    /*!< Error conditions from which recovery measures have been taken */
    ESP_LOG_INFO    = 3,    /*!< Information messages which describe normal flow of events */
    ESP_LOG_DEBUG   = 4,    /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ESP_LOG_VERBOSE = 5,    /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
    ESP_LOG_MAX     = 6,    /*!< Number of levels supported */
} esp_log_level_t;

#define LOG_FORMAT(letter, format)  #letter " (%lu) %s: " format "\n"

#define ESP_LOGE(tag, format, ...)   {esp_log_write(ESP_LOG_ERROR,   tag, LOG_FORMAT(E, format), esp_log_timestamp(), tag, ##__VA_ARGS__); }
#define ESP_LOGW(tag, format, ...)   {esp_log_write(ESP_LOG_WARN,    tag, LOG_FORMAT(W, format), esp_log_timestamp(), tag, ##__VA_ARGS__); }
#define ESP_LOGI(tag, format, ...)   {esp_log_write(ESP_LOG_INFO,    tag, LOG_FORMAT(I, format), esp_log_timestamp(), tag, ##__VA_ARGS__); }
#define ESP_LOGD(tag, format, ...)   {esp_log_write(ESP_LOG_DEBUG,   tag, LOG_FORMAT(D, format), esp_log_timestamp(), tag, ##__VA_ARGS__); }
#define ESP_LOGV(tag, format, ...)   {esp_log_write(ESP_LOG_VERBOSE, tag, LOG_FORMAT(V, format), esp_log_timestamp(), tag, ##__VA_ARGS__); }

// TODO(skyevg): implement in misc.c
uint32_t esp_log_timestamp(void);
void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));