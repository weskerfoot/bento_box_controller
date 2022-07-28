#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>
#include "driver/spi_common.h"
#include "driver/uart.h"

#define LEDC_TIMER_0  0
#define LEDC_TIMER_1 1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_1  (18) // Define the output GPIO
#define LEDC_OUTPUT_IO_2  (19) // Define the output GPIO
#define LEDC_CHANNEL_1  0
#define LEDC_CHANNEL_2  1
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (8191) // Set duty to 100%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

#define PUMP_EV_NUM 5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TASK_STACK_SIZE 4000
