#include "cjson.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "mqtt_client.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "sdkconfig.h"
#include "sht3x.h"
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include "nvs.h"
#include <nvs_flash.h>
#include <sgp40.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

// The amount of bytes that will be allocated to store the MQTT broker URI
#define MQTT_BROKER_URI_MAX_SIZE 50

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASS
#define MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY
#define BROKER_URI CONFIG_BROKER_URI
#define SERIAL_NUMBER CONFIG_SERIAL

#define LEDC_TIMER 1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (19) // Define the output GPIO
#define LEDC_CHANNEL 1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY (255) // Set duty cycle, ((2 ** n_bits) - 1) * percentage = duty cycle
#define LEDC_FREQUENCY (1000) // Frequency in Hertz. Set frequency at 10 kHz

#define I2C_BUS       0
#define I2C_SCL_PIN   22
#define I2C_SDA_PIN   21

// Separate bus for air quality sensor
#define AC_I2C_BUS 1
#define AC_SCL 32
#define AC_SDA 18

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

#define FAN_EV_NUM 5
#define HTTPD_RESP_SIZE 1000
#define MAX_CRON_SPECS 5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TASK_STACK_SIZE 5000

#define VOC_MAX_THRESHOLD_DEFAULT 140
#define BED_TEMPER_MAX_THRESHOLD_DEFAULT 83.0f

// Currently not used due to weird certficate errors
// This would be the raw bytes of the certificate for the printer server though
unsigned char bbl_ca_pem[] = {};
unsigned int bbl_ca_pem_len = 1238;

typedef enum {
  MANUAL_PRIORITY = 1,
  SENSOR_PRIORITY = 2,
  BED_TEMP_PRIORITY = 3,
  LOWEST_PRIORITY = 4
} control_priority;

typedef enum {
  FAN_ON = 1,
  FAN_OFF = 2,
} event_type;

struct fan_event {
  event_type fan;
  int fan_delay;
  int run_forever;
  int priority;
};

struct threshold_event {
  int voc_max_threshold;
  int voc_min_threshold;
  double bed_temper_max_threshold;
  double bed_temper_min_threshold;
};

struct printer_event {
  double bed_temper;
};

struct mqtt_handler_event {
  int restart;
};

static size_t
get_string_from_nvs(nvs_handle_t nvs_handle,
                    size_t max_output_size,
                    const char *key_name,
                    char *output) {
    size_t req_size = 0;

    esp_err_t nvs_get_str_err = nvs_get_str(nvs_handle, key_name, NULL, &req_size);

    if (nvs_get_str_err != ESP_OK) {
      printf("Value could not be read from nvram\n");
    }
    else if (req_size > max_output_size) {
      printf("Value stored in nvram too long, req_size = %zu, but max_output_size = %zu\n",
             req_size,
             max_output_size);
    }
    else {
      printf("Restoring value from nvram, %s = %zu\n", key_name, req_size);
      nvs_get_str_err = nvs_get_str(nvs_handle, key_name, output, &req_size);
    }
    return req_size;
}

static esp_err_t
get_int32_from_nvs(nvs_handle_t nvs_handle,
                   const char *key_name,
                   int *output) {
    esp_err_t nvs_get_i32_err = nvs_get_i32(nvs_handle, key_name, output);

    if (nvs_get_i32_err != ESP_OK) {
      printf("Value could not be read from nvram, using existing value\n");
      return nvs_get_i32_err;
    }
    printf("Restored int32 value from nvs, value = %d\n", *output);
    return nvs_get_i32_err;
}

static esp_err_t
nvs_set_f32(nvs_handle_t nvs_handle,
                const char* key_name,
                float value) {
  // Assumes floats are 32 bit, which is always going to be the case on an esp32
  uint32_t copy_to = 0;
  memcpy(&copy_to, &value, 4);
  return nvs_set_u32(nvs_handle, key_name, copy_to);
}

static esp_err_t
nvs_get_f32(nvs_handle_t nvs_handle,
            const char* key_name,
            float *output) {
  // Assumes floats are 32 bit, which is always going to be the case on an esp32
  uint32_t u32_output = 0;
  esp_err_t nvs_get_u32_err = nvs_get_u32(nvs_handle, key_name, &u32_output);
  if (nvs_get_u32_err != ESP_OK) {
    printf("f32 could not be read from nvram, using existing value\n");
    return nvs_get_u32_err;
  }
  memcpy(output, &u32_output, 4);
  printf("Restored float32 value from nvs, value = %f\n", *output);
  return nvs_get_u32_err;
}

static TickType_t
make_delay(int seconds) {
  return (1000*seconds) / portTICK_PERIOD_MS;
}
