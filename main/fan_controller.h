#include "sht3x.h"
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
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "sdkconfig.h"
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <sgp40.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

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
unsigned char bbl_ca_pem[] = {
  0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x42, 0x45, 0x47, 0x49, 0x4e, 0x20, 0x43,
  0x45, 0x52, 0x54, 0x49, 0x46, 0x49, 0x43, 0x41, 0x54, 0x45, 0x2d, 0x2d,
  0x2d, 0x2d, 0x2d, 0x0a, 0x4d, 0x49, 0x49, 0x44, 0x5a, 0x54, 0x43, 0x43,
  0x41, 0x6b, 0x32, 0x67, 0x41, 0x77, 0x49, 0x42, 0x41, 0x67, 0x49, 0x55,
  0x56, 0x31, 0x46, 0x63, 0x6b, 0x77, 0x58, 0x45, 0x6c, 0x79, 0x65, 0x6b,
  0x31, 0x6f, 0x6e, 0x46, 0x6e, 0x51, 0x39, 0x6b, 0x4c, 0x37, 0x42, 0x6b,
  0x34, 0x4e, 0x38, 0x77, 0x44, 0x51, 0x59, 0x4a, 0x4b, 0x6f, 0x5a, 0x49,
  0x68, 0x76, 0x63, 0x4e, 0x41, 0x51, 0x45, 0x4c, 0x0a, 0x42, 0x51, 0x41,
  0x77, 0x51, 0x6a, 0x45, 0x4c, 0x4d, 0x41, 0x6b, 0x47, 0x41, 0x31, 0x55,
  0x45, 0x42, 0x68, 0x4d, 0x43, 0x51, 0x30, 0x34, 0x78, 0x49, 0x6a, 0x41,
  0x67, 0x42, 0x67, 0x4e, 0x56, 0x42, 0x41, 0x6f, 0x4d, 0x47, 0x55, 0x4a,
  0x43, 0x54, 0x43, 0x42, 0x55, 0x5a, 0x57, 0x4e, 0x6f, 0x62, 0x6d, 0x39,
  0x73, 0x62, 0x32, 0x64, 0x70, 0x5a, 0x58, 0x4d, 0x67, 0x51, 0x32, 0x38,
  0x75, 0x0a, 0x4c, 0x43, 0x42, 0x4d, 0x64, 0x47, 0x51, 0x78, 0x44, 0x7a,
  0x41, 0x4e, 0x42, 0x67, 0x4e, 0x56, 0x42, 0x41, 0x4d, 0x4d, 0x42, 0x6b,
  0x4a, 0x43, 0x54, 0x43, 0x42, 0x44, 0x51, 0x54, 0x41, 0x65, 0x46, 0x77,
  0x30, 0x79, 0x4d, 0x6a, 0x41, 0x30, 0x4d, 0x44, 0x51, 0x77, 0x4d, 0x7a,
  0x51, 0x79, 0x4d, 0x54, 0x46, 0x61, 0x46, 0x77, 0x30, 0x7a, 0x4d, 0x6a,
  0x41, 0x30, 0x4d, 0x44, 0x45, 0x77, 0x0a, 0x4d, 0x7a, 0x51, 0x79, 0x4d,
  0x54, 0x46, 0x61, 0x4d, 0x45, 0x49, 0x78, 0x43, 0x7a, 0x41, 0x4a, 0x42,
  0x67, 0x4e, 0x56, 0x42, 0x41, 0x59, 0x54, 0x41, 0x6b, 0x4e, 0x4f, 0x4d,
  0x53, 0x49, 0x77, 0x49, 0x41, 0x59, 0x44, 0x56, 0x51, 0x51, 0x4b, 0x44,
  0x42, 0x6c, 0x43, 0x51, 0x6b, 0x77, 0x67, 0x56, 0x47, 0x56, 0x6a, 0x61,
  0x47, 0x35, 0x76, 0x62, 0x47, 0x39, 0x6e, 0x61, 0x57, 0x56, 0x7a, 0x0a,
  0x49, 0x45, 0x4e, 0x76, 0x4c, 0x69, 0x77, 0x67, 0x54, 0x48, 0x52, 0x6b,
  0x4d, 0x51, 0x38, 0x77, 0x44, 0x51, 0x59, 0x44, 0x56, 0x51, 0x51, 0x44,
  0x44, 0x41, 0x5a, 0x43, 0x51, 0x6b, 0x77, 0x67, 0x51, 0x30, 0x45, 0x77,
  0x67, 0x67, 0x45, 0x69, 0x4d, 0x41, 0x30, 0x47, 0x43, 0x53, 0x71, 0x47,
  0x53, 0x49, 0x62, 0x33, 0x44, 0x51, 0x45, 0x42, 0x41, 0x51, 0x55, 0x41,
  0x41, 0x34, 0x49, 0x42, 0x0a, 0x44, 0x77, 0x41, 0x77, 0x67, 0x67, 0x45,
  0x4b, 0x41, 0x6f, 0x49, 0x42, 0x41, 0x51, 0x44, 0x4c, 0x33, 0x70, 0x6e,
  0x44, 0x64, 0x78, 0x47, 0x4f, 0x6b, 0x35, 0x5a, 0x36, 0x76, 0x75, 0x67,
  0x69, 0x54, 0x34, 0x64, 0x70, 0x4d, 0x30, 0x6a, 0x75, 0x2b, 0x33, 0x58,
  0x61, 0x74, 0x78, 0x7a, 0x30, 0x39, 0x55, 0x59, 0x37, 0x6d, 0x62, 0x6a,
  0x34, 0x74, 0x6b, 0x49, 0x64, 0x62, 0x79, 0x34, 0x48, 0x0a, 0x6f, 0x65,
  0x45, 0x64, 0x69, 0x59, 0x53, 0x5a, 0x6a, 0x63, 0x35, 0x4c, 0x4a, 0x6e,
  0x67, 0x4a, 0x75, 0x43, 0x48, 0x77, 0x74, 0x45, 0x62, 0x42, 0x4a, 0x74,
  0x31, 0x42, 0x72, 0x69, 0x52, 0x64, 0x53, 0x56, 0x72, 0x46, 0x36, 0x4d,
  0x39, 0x44, 0x32, 0x55, 0x61, 0x42, 0x44, 0x79, 0x61, 0x6d, 0x45, 0x6f,
  0x30, 0x64, 0x78, 0x77, 0x53, 0x61, 0x56, 0x78, 0x5a, 0x69, 0x44, 0x56,
  0x57, 0x43, 0x0a, 0x65, 0x65, 0x43, 0x50, 0x64, 0x45, 0x4c, 0x70, 0x46,
  0x5a, 0x64, 0x45, 0x68, 0x53, 0x4e, 0x54, 0x61, 0x54, 0x34, 0x4f, 0x37,
  0x7a, 0x67, 0x76, 0x63, 0x6e, 0x46, 0x73, 0x66, 0x48, 0x4d, 0x61, 0x2f,
  0x30, 0x76, 0x4d, 0x41, 0x6b, 0x76, 0x45, 0x37, 0x69, 0x30, 0x71, 0x70,
  0x33, 0x6d, 0x6a, 0x45, 0x7a, 0x59, 0x4c, 0x66, 0x7a, 0x36, 0x30, 0x61,
  0x78, 0x63, 0x44, 0x6f, 0x4a, 0x4c, 0x6b, 0x0a, 0x70, 0x37, 0x6e, 0x36,
  0x78, 0x4b, 0x58, 0x49, 0x2b, 0x63, 0x4a, 0x62, 0x41, 0x34, 0x49, 0x6c,
  0x54, 0x6f, 0x46, 0x6a, 0x70, 0x53, 0x6c, 0x64, 0x50, 0x6d, 0x43, 0x2b,
  0x79, 0x6e, 0x4f, 0x6f, 0x37, 0x59, 0x41, 0x4f, 0x73, 0x58, 0x74, 0x37,
  0x41, 0x59, 0x4b, 0x59, 0x36, 0x47, 0x6c, 0x7a, 0x30, 0x42, 0x77, 0x55,
  0x56, 0x7a, 0x53, 0x4a, 0x78, 0x55, 0x2b, 0x2f, 0x2b, 0x56, 0x46, 0x79,
  0x0a, 0x2f, 0x51, 0x72, 0x6d, 0x59, 0x47, 0x4e, 0x77, 0x6c, 0x72, 0x51,
  0x74, 0x64, 0x52, 0x45, 0x48, 0x65, 0x52, 0x69, 0x30, 0x53, 0x4e, 0x4b,
  0x33, 0x32, 0x78, 0x31, 0x2b, 0x62, 0x4f, 0x6e, 0x64, 0x66, 0x4a, 0x50,
  0x30, 0x73, 0x6f, 0x6a, 0x75, 0x49, 0x72, 0x44, 0x6a, 0x4b, 0x73, 0x64,
  0x43, 0x4c, 0x79, 0x65, 0x35, 0x43, 0x53, 0x5a, 0x49, 0x76, 0x71, 0x6e,
  0x62, 0x6f, 0x77, 0x77, 0x57, 0x0a, 0x31, 0x6a, 0x52, 0x77, 0x5a, 0x67,
  0x54, 0x42, 0x52, 0x32, 0x39, 0x5a, 0x70, 0x32, 0x6e, 0x7a, 0x43, 0x6f,
  0x78, 0x4a, 0x59, 0x63, 0x55, 0x39, 0x54, 0x53, 0x51, 0x70, 0x2f, 0x34,
  0x4b, 0x5a, 0x75, 0x57, 0x4e, 0x56, 0x41, 0x67, 0x4d, 0x42, 0x41, 0x41,
  0x47, 0x6a, 0x55, 0x7a, 0x42, 0x52, 0x4d, 0x42, 0x30, 0x47, 0x41, 0x31,
  0x55, 0x64, 0x44, 0x67, 0x51, 0x57, 0x42, 0x42, 0x53, 0x50, 0x0a, 0x4e,
  0x45, 0x4a, 0x6f, 0x33, 0x47, 0x64, 0x4f, 0x6a, 0x38, 0x51, 0x69, 0x6e,
  0x73, 0x56, 0x38, 0x53, 0x65, 0x57, 0x72, 0x33, 0x55, 0x53, 0x2b, 0x48,
  0x6a, 0x41, 0x66, 0x42, 0x67, 0x4e, 0x56, 0x48, 0x53, 0x4d, 0x45, 0x47,
  0x44, 0x41, 0x57, 0x67, 0x42, 0x53, 0x50, 0x4e, 0x45, 0x4a, 0x6f, 0x33,
  0x47, 0x64, 0x4f, 0x6a, 0x38, 0x51, 0x69, 0x6e, 0x73, 0x56, 0x38, 0x53,
  0x65, 0x57, 0x72, 0x0a, 0x33, 0x55, 0x53, 0x2b, 0x48, 0x6a, 0x41, 0x50,
  0x42, 0x67, 0x4e, 0x56, 0x48, 0x52, 0x4d, 0x42, 0x41, 0x66, 0x38, 0x45,
  0x42, 0x54, 0x41, 0x44, 0x41, 0x51, 0x48, 0x2f, 0x4d, 0x41, 0x30, 0x47,
  0x43, 0x53, 0x71, 0x47, 0x53, 0x49, 0x62, 0x33, 0x44, 0x51, 0x45, 0x42,
  0x43, 0x77, 0x55, 0x41, 0x41, 0x34, 0x49, 0x42, 0x41, 0x51, 0x41, 0x42,
  0x6c, 0x42, 0x49, 0x54, 0x35, 0x5a, 0x65, 0x47, 0x0a, 0x66, 0x67, 0x63,
  0x4b, 0x31, 0x4c, 0x4f, 0x68, 0x31, 0x43, 0x4e, 0x39, 0x73, 0x54, 0x7a,
  0x78, 0x4d, 0x43, 0x4c, 0x62, 0x74, 0x54, 0x50, 0x46, 0x46, 0x31, 0x4e,
  0x47, 0x47, 0x41, 0x31, 0x33, 0x6d, 0x41, 0x70, 0x75, 0x36, 0x6a, 0x31,
  0x68, 0x35, 0x59, 0x45, 0x4c, 0x62, 0x53, 0x4b, 0x63, 0x55, 0x71, 0x66,
  0x58, 0x7a, 0x4d, 0x6e, 0x56, 0x65, 0x41, 0x62, 0x30, 0x36, 0x48, 0x74,
  0x75, 0x0a, 0x33, 0x43, 0x6f, 0x43, 0x6f, 0x65, 0x2b, 0x77, 0x6a, 0x37,
  0x4c, 0x4f, 0x4e, 0x54, 0x46, 0x4f, 0x2b, 0x2b, 0x76, 0x42, 0x6d, 0x32,
  0x2f, 0x69, 0x66, 0x36, 0x4a, 0x74, 0x2f, 0x44, 0x55, 0x77, 0x31, 0x43,
  0x41, 0x45, 0x63, 0x4e, 0x79, 0x71, 0x65, 0x68, 0x36, 0x45, 0x53, 0x30,
  0x4e, 0x58, 0x38, 0x4c, 0x4a, 0x52, 0x56, 0x53, 0x65, 0x30, 0x71, 0x64,
  0x54, 0x78, 0x50, 0x4a, 0x75, 0x41, 0x0a, 0x42, 0x64, 0x4f, 0x6f, 0x6f,
  0x39, 0x36, 0x69, 0x58, 0x38, 0x39, 0x72, 0x52, 0x50, 0x6f, 0x78, 0x65,
  0x65, 0x64, 0x31, 0x63, 0x70, 0x71, 0x35, 0x68, 0x5a, 0x77, 0x62, 0x65,
  0x6b, 0x61, 0x33, 0x2b, 0x43, 0x4a, 0x47, 0x56, 0x37, 0x36, 0x69, 0x74,
  0x57, 0x70, 0x33, 0x35, 0x55, 0x70, 0x35, 0x72, 0x6d, 0x6d, 0x55, 0x71,
  0x72, 0x6c, 0x79, 0x51, 0x4f, 0x72, 0x2f, 0x57, 0x61, 0x78, 0x36, 0x0a,
  0x69, 0x74, 0x6f, 0x73, 0x49, 0x7a, 0x47, 0x30, 0x4d, 0x66, 0x68, 0x67,
  0x55, 0x7a, 0x55, 0x35, 0x31, 0x41, 0x32, 0x50, 0x2f, 0x68, 0x53, 0x6e,
  0x44, 0x33, 0x4e, 0x44, 0x4d, 0x58, 0x76, 0x2b, 0x77, 0x55, 0x59, 0x2f,
  0x41, 0x76, 0x71, 0x67, 0x49, 0x4c, 0x37, 0x75, 0x37, 0x66, 0x62, 0x44,
  0x4b, 0x6e, 0x6b, 0x75, 0x31, 0x47, 0x7a, 0x45, 0x4b, 0x49, 0x6b, 0x66,
  0x48, 0x38, 0x68, 0x6d, 0x0a, 0x52, 0x73, 0x36, 0x64, 0x38, 0x53, 0x43,
  0x55, 0x38, 0x39, 0x78, 0x79, 0x72, 0x77, 0x7a, 0x51, 0x30, 0x50, 0x52,
  0x38, 0x35, 0x33, 0x69, 0x72, 0x48, 0x61, 0x73, 0x33, 0x57, 0x72, 0x48,
  0x56, 0x71, 0x61, 0x62, 0x33, 0x50, 0x2b, 0x71, 0x4e, 0x77, 0x52, 0x30,
  0x59, 0x69, 0x72, 0x4c, 0x30, 0x51, 0x6b, 0x37, 0x58, 0x74, 0x2f, 0x71,
  0x33, 0x4f, 0x31, 0x67, 0x72, 0x69, 0x4e, 0x67, 0x32, 0x0a, 0x42, 0x6c,
  0x62, 0x6a, 0x67, 0x33, 0x6f, 0x62, 0x70, 0x48, 0x6f, 0x39, 0x0a, 0x2d,
  0x2d, 0x2d, 0x2d, 0x2d, 0x45, 0x4e, 0x44, 0x20, 0x43, 0x45, 0x52, 0x54,
  0x49, 0x46, 0x49, 0x43, 0x41, 0x54, 0x45, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d,
  0x0a, 0x00
};
unsigned int bbl_ca_pem_len = 1238;

typedef enum {
  MANUAL_PRIORITY = 1,
  SENSOR_PRIORITY = 2,
  BED_TEMP_PRIORITY = 3,
  LOWEST_PRIORITY = 4
} control_priority;

void runfans(int);

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

static void wifi_init_sta(void);
static void run_fans_forever();
static void run_fans(int, int);
static void stop_running_fans(int);
static void obtain_time(void);
static void initialize_sntp(void);
