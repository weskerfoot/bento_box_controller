#include "./fan_controller.h"

static const char *TAG = "fan_controller";
static sht3x_sensor_t* sensor;
static sgp40_t air_q_sensor;

// Timer type definitions
const TickType_t fan_TIMER_DELAY = (1000*60) / portTICK_PERIOD_MS;
const TickType_t sensor_TIMER_DELAY = 1000 / portTICK_PERIOD_MS;
const TickType_t fan_CB_PERIOD = (1000*10) / portTICK_PERIOD_MS;

// Queue type definitions
static uint8_t fanQueueStorage[FAN_EV_NUM*sizeof (struct fan_event)];
static StaticQueue_t fanEvents;
static QueueHandle_t fanEventsHandle;

static uint8_t thresholdQueueStorage[10*sizeof (struct fan_event)];
static StaticQueue_t thresholdEvents;
static QueueHandle_t thresholdEventsHandle;

static uint8_t printerEventsQueueStorage[10*sizeof (struct fan_event)];
static StaticQueue_t printerEvents;
static QueueHandle_t printerEventsHandle;

// Task type definitions

static StaticTask_t fanRunnerTaskBuffer;
static StackType_t fanRunnerTaskStack[TASK_STACK_SIZE];

static StaticTask_t sensorManagerTaskBuffer;
static StackType_t sensorManagerTaskStack[TASK_STACK_SIZE];

SemaphoreHandle_t sensorSemaphore = NULL; // Used to control access to sensors

static void
set_fan(int fan_num, int state) {
    // Set duty to 100%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, fan_num, state == 1 ? LEDC_DUTY: 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, fan_num));
}

static void
fan_on() {
  set_fan(1, 1);
}

static void
fans_off() {
  set_fan(1, 0);
}

static void
initSGP40() {
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(sgp40_init_desc(&air_q_sensor, AC_I2C_BUS, AC_SDA, AC_SCL));
    ESP_ERROR_CHECK(sgp40_init(&air_q_sensor));
    ESP_LOGI(TAG, "SGP40 initilalized. Serial: 0x%04x%04x%04x",
            air_q_sensor.serial[0], air_q_sensor.serial[1], air_q_sensor.serial[2]);
}

// MQTT callback
static void
mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  cJSON *mqtt_data;
  char *json_st;

  // This might be inefficient
  static double bed_temper = 0.0;

  // whether the printer is currently running or not
  static int printer_state = 0;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      msg_id = esp_mqtt_client_subscribe(client, "device/" SERIAL_NUMBER "/report", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
      break;
  case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      break;

  case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
      ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
      break;
  case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;
  case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;
  case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");
      if (event->topic_len > 0 && event->data_len > 0) {
      #ifdef CONFIG_DEBUG_MODE_ENABLED
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      #endif
        mqtt_data = cJSON_ParseWithLength(event->data, event->data_len);
        if (mqtt_data != NULL) {

          #ifdef CONFIG_DEBUG_MODE_ENABLED
          json_st = cJSON_Print(mqtt_data);
          if (json_st != NULL) {
            printf("json_st = %s\n", json_st);
            free(json_st);
          }
          #endif

          cJSON *print_object = cJSON_GetObjectItemCaseSensitive(mqtt_data, "print");
          if (cJSON_IsObject(print_object)) {
            cJSON *gcode_state_val = cJSON_GetObjectItemCaseSensitive(print_object, "gcode_state");
            cJSON *bed_temper_val = cJSON_GetObjectItemCaseSensitive(print_object, "bed_temper");

            if (cJSON_IsString(gcode_state_val) && gcode_state_val->valuestring != NULL) {
              int gcode_str_len = strlen(gcode_state_val->valuestring);

              // TODO handle gcode states properly, this seems flaky, sometimes it sends a RUNNING message after it's actually done
              /*
              if (strncmp(gcode_state_val->valuestring, "RUNNING", gcode_str_len) &&
                  bed_temper > 83.0) {
                printf("Starting air filter fans\n");
                run_fans_forever(BED_TEMP_PRIORITY);
              }

              if (strncmp(gcode_state_val->valuestring, "FINISH", gcode_str_len)) {
                printf("Stopping air filter fans\n");
              }
              */
            }

            if (cJSON_IsNumber(bed_temper_val) && bed_temper_val->valuedouble != 0) {
              bed_temper = bed_temper_val->valuedouble;
              struct printer_event printerEventMessage = {0};
              printerEventMessage.bed_temper = bed_temper;

              if (printerEventsHandle != NULL) {
                xQueueSend(printerEventsHandle, (void*)&printerEventMessage, (TickType_t)0);
              }
            }

          }
          if (mqtt_data != NULL) { cJSON_Delete(mqtt_data); }
        }
      }
      break;
  case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
          ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
          ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
          ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                   strerror(event->error_handle->esp_transport_sock_errno));
      } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
          ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
      } else {
          ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
      }
      break;
  default:
      ESP_LOGI(TAG, "Other event id:%d", event->event_id);
      break;
  }
}

static void
mqtt_app_start(void) {
  const esp_mqtt_client_config_t mqtt_cfg = {
    .broker = {
      .address.uri = CONFIG_BROKER_URI
      //.verification.certificate = (const char *)bbl_ca_pem
    },
  };

  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(client);

}

static TickType_t
make_delay(int seconds) {
  return (1000*seconds) / portTICK_PERIOD_MS;
}

static void
sensor_manager_task_function(void *params) {
  int voc_max_threshold = VOC_MAX_THRESHOLD_DEFAULT;
  int voc_min_threshold = voc_max_threshold > 10 ? voc_max_threshold - 10 : 0;

  float bed_temper_min_threshold = 83.0;
  float bed_temper_max_threshold = 83.0;

  double bed_temper = 0.0f;

  struct threshold_event thresholdMessage = {0};
  struct printer_event printerEventMessage = {0};
  while (1) {
    if (fanEventsHandle != NULL) {
      if (xQueueReceive(printerEventsHandle, &printerEventMessage, (TickType_t)sensor_TIMER_DELAY) == pdPASS) {
        if (printerEventMessage.bed_temper > 0.0f) {
          bed_temper = printerEventMessage.bed_temper;
          printf("Got bed temper in sensor manager, bed_temper = %f\n", bed_temper);
        }
      }
    }

    if (thresholdEventsHandle != NULL) {
      if (xQueueReceive(thresholdEventsHandle, &thresholdMessage, (TickType_t)sensor_TIMER_DELAY) == pdPASS) {
        if (thresholdMessage.voc_max_threshold > 0 && thresholdMessage.voc_max_threshold <= 500) {
          voc_max_threshold = thresholdMessage.voc_max_threshold;
        }
        else {
        #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set voc_max_threshold to %d\n", thresholdMessage.voc_max_threshold);
          printf("current voc_max_threshold = %d, current voc_min_threshold = %d\n", voc_max_threshold, voc_min_threshold);
        #endif
        }
        if (thresholdMessage.voc_min_threshold > 0 && thresholdMessage.voc_min_threshold < voc_max_threshold) {
          voc_min_threshold = thresholdMessage.voc_min_threshold;
        }
        else {
        #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set voc_min_threshold to %d\n", thresholdMessage.voc_min_threshold);
          printf("current voc_max_threshold = %d, current voc_min_threshold = %d\n", voc_max_threshold, voc_min_threshold);
        #endif
        }

        if (thresholdMessage.bed_temper_min_threshold > 0.0f && thresholdMessage.bed_temper_min_threshold <= bed_temper_max_threshold) {
          bed_temper_min_threshold = thresholdMessage.bed_temper_min_threshold;
        }
        else {
        #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set bed_temper_min_threshold to %f\n", thresholdMessage.bed_temper_min_threshold);
          printf("current bed_temper_max_threshold = %f, current bed_temper_min_threshold = %f\n",
                 bed_temper_max_threshold,
                 bed_temper_min_threshold);
        #endif
        }

        if (thresholdMessage.bed_temper_max_threshold > 0.0f && thresholdMessage.bed_temper_max_threshold >= bed_temper_min_threshold) {
          bed_temper_max_threshold = thresholdMessage.bed_temper_max_threshold;
        }
        else {
        #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set bed_temper_max_threshold to %f\n", thresholdMessage.bed_temper_max_threshold);
          printf("current bed_temper_max_threshold = %f, current bed_temper_min_threshold = %f\n",
                 bed_temper_max_threshold,
                 bed_temper_min_threshold);
        #endif
        }

      }
    }

    vTaskDelay(make_delay(2));
    if (sensorSemaphore != NULL) {
      if (xSemaphoreTake(sensorSemaphore, (TickType_t)4) == pdTRUE) {

        float temperature = 0.0;
        float humidity = 0.0;
        int32_t voc_index = 0;
        uint16_t raw_voc = 0;

        if (sht3x_measure(sensor, &temperature, &humidity)) {
        #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("temperature = %f\n", (double)temperature);
          printf("humidity = %f\n", (double)humidity);
        #endif

          esp_err_t sgp40_status = sgp40_measure_voc(&air_q_sensor,
                                                     humidity,
                                                     temperature,
                                                     &voc_index);

          esp_err_t sgp40_status_raw = sgp40_measure_raw(&air_q_sensor,
                                                         humidity,
                                                         temperature,
                                                         &raw_voc);

          if (sgp40_status == ESP_OK) {
          #ifdef CONFIG_DEBUG_MODE_ENABLED
            printf("voc_index = %ld\n", voc_index);
          #endif
            if (voc_index > voc_max_threshold) { // TODO, make threshold configurable, test with ABS, etc
              run_fans_forever(SENSOR_PRIORITY);
            }
            if (voc_index <= voc_min_threshold) {
              stop_running_fans(SENSOR_PRIORITY);
            }
          }

          #ifdef CONFIG_DEBUG_MODE_ENABLED
          if (sgp40_status_raw == ESP_OK) {
            printf("raw_voc = %d\n", raw_voc);
          }
          #endif
          if (bed_temper > bed_temper_max_threshold) {
            run_fans_forever(BED_TEMP_PRIORITY);
          }

          if (bed_temper < bed_temper_min_threshold) {
            stop_running_fans(BED_TEMP_PRIORITY);
          }

        }

        xSemaphoreGive(sensorSemaphore);
      }
      else {
        printf("failed to acquire sensor semaphore in manager task\n");
      }
    }
  }
}

static void
fan_runner_task_function(void *params) {
  struct fan_event fanMessage;
  int current_priority = LOWEST_PRIORITY;

  printf("Task started\n");

  configASSERT( ( uint32_t ) params == 1UL );

  while (1) {
    if (fanEventsHandle != NULL) {
      // The queue exists and is created
      if (xQueueReceive(fanEventsHandle, &fanMessage, (TickType_t)fan_TIMER_DELAY) == pdPASS) {
        if (fanMessage.fan == FAN_ON && fanMessage.priority <= current_priority) {
          current_priority = fanMessage.priority;
          fan_on();

          // If it should run on a delay, then delay and turn them off
          if (fanMessage.run_forever != 1) {
            vTaskDelay(fanMessage.fan_delay);
            current_priority = LOWEST_PRIORITY;
            fans_off();
          }
        }

        if (fanMessage.fan == FAN_OFF && fanMessage.priority <= current_priority) {
          fans_off();
          current_priority = LOWEST_PRIORITY;
        }
      }
    }
  }
}

static void
createSensorManagerTask(void) {
  xTaskCreateStatic(sensor_manager_task_function,
                    "sensormant",
                     TASK_STACK_SIZE,
                     (void*)1,
                     tskIDLE_PRIORITY + 2,
                     sensorManagerTaskStack,
                     &sensorManagerTaskBuffer);
}
static void
createfanRunnerTask(void) {
  xTaskCreateStatic(fan_runner_task_function,
                    "fant",
                     TASK_STACK_SIZE,
                     (void*)1,
                     tskIDLE_PRIORITY + 2,
                     fanRunnerTaskStack,
                     &fanRunnerTaskBuffer);
}

static void
run_fans(int delay, int priority) {
  struct fan_event message;
  message.fan = FAN_ON;
  message.priority = priority;
  message.fan_delay = make_delay(delay);
  message.run_forever = 0;

  xQueueSend(fanEventsHandle, (void*)&message, (TickType_t)0);
}

static void
stop_running_fans(int priority) {
  struct fan_event message;
  message.fan = FAN_OFF;
  message.priority = priority;
  xQueueSend(fanEventsHandle, (void*)&message, (TickType_t)0);
}

static void
run_fans_forever(int priority) {
  struct fan_event message;
  message.fan = FAN_ON;
  message.fan_delay = -1;
  message.run_forever = 1;
  message.priority = priority;

  xQueueSend(fanEventsHandle, (void*)&message, (TickType_t)0);
}


static esp_err_t
set_sensor_thresholds_handler(httpd_req_t *req) {
  printf("set sensor thresholds handler executed\n");
  char content[HTTPD_RESP_SIZE];
  char resp[] = "Set thresholds";

  size_t recv_size = MIN(req->content_len, (sizeof(content)-1));

  // Receive body and do error handling
  int ret = httpd_req_recv(req, content, recv_size);

  // if ret == 0 then no data
  if (ret < 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }

  struct threshold_event thresholdMessage;
  thresholdMessage.voc_max_threshold = -1;
  thresholdMessage.voc_min_threshold = -1; // -1 means no change
  thresholdMessage.bed_temper_min_threshold = -1.0f;
  thresholdMessage.bed_temper_max_threshold = -1.0f; // -1 means no change

  cJSON *json = cJSON_ParseWithLength(content, recv_size);

  if (json != NULL) {
    if (cJSON_IsObject(json)) {
      cJSON *voc_max_j = cJSON_GetObjectItemCaseSensitive(json, "voc_max_threshold");
      cJSON *voc_min_j = cJSON_GetObjectItemCaseSensitive(json, "voc_min_threshold");

      cJSON *bed_temper_max_j = cJSON_GetObjectItemCaseSensitive(json, "bed_temper_max_threshold");
      cJSON *bed_temper_min_j = cJSON_GetObjectItemCaseSensitive(json, "bed_temper_min_threshold");

      if (cJSON_IsNumber(voc_max_j) && cJSON_IsNumber(voc_min_j)) {
        thresholdMessage.voc_max_threshold = voc_max_j->valueint;
        thresholdMessage.voc_min_threshold = voc_min_j->valueint;
        if ((thresholdMessage.voc_min_threshold > thresholdMessage.voc_max_threshold) ||
            (thresholdMessage.voc_max_threshold < 0) ||
            (thresholdMessage.voc_min_threshold < 0)) {
          #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set voc values, attempted max = %d, attempted min = %d\n", voc_max_j->valueint, voc_min_j->valueint);
          printf("Setting voc_min_threshold = %d\n", VOC_MAX_THRESHOLD_DEFAULT-10);
          printf("Setting voc_max_threshold = %d\n", VOC_MAX_THRESHOLD_DEFAULT);
          #endif
          thresholdMessage.voc_max_threshold = VOC_MAX_THRESHOLD_DEFAULT;
          thresholdMessage.voc_min_threshold = VOC_MAX_THRESHOLD_DEFAULT-10;
        }
        else {
          #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Setting new voc max: voc_max_threshold = %d\n", voc_max_j->valueint);
          printf("Setting new voc min: voc_min_threshold = %d\n", voc_min_j->valueint);
          #endif
        }
      }

      if (cJSON_IsNumber(bed_temper_max_j) && cJSON_IsNumber(bed_temper_min_j)) {
        thresholdMessage.bed_temper_max_threshold = bed_temper_max_j->valuedouble;
        thresholdMessage.bed_temper_min_threshold = bed_temper_min_j->valuedouble;
        if ((thresholdMessage.bed_temper_min_threshold > thresholdMessage.bed_temper_max_threshold) ||
            (thresholdMessage.bed_temper_max_threshold < 0.0f) ||
            (thresholdMessage.bed_temper_min_threshold < 0.0f)) {
          #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Could not set bed temper values, attempted max = %f, attempted min = %f\n",
                 bed_temper_max_j->valuedouble,
                 bed_temper_min_j->valuedouble);

          printf("Setting bed_temper_max_threshold = %f\n", BED_TEMPER_MAX_THRESHOLD_DEFAULT-2);
          printf("Setting bed_temper_min_threshold = %f\n", BED_TEMPER_MAX_THRESHOLD_DEFAULT);
          #endif

          thresholdMessage.bed_temper_max_threshold = BED_TEMPER_MAX_THRESHOLD_DEFAULT;
          thresholdMessage.bed_temper_min_threshold = BED_TEMPER_MAX_THRESHOLD_DEFAULT-2;
        }
        else {
          #ifdef CONFIG_DEBUG_MODE_ENABLED
          printf("Setting new bed temper max: bed_temper_max_threshold = %f\n", bed_temper_max_j->valuedouble);
          printf("Setting new bed temper min: bed_temper_min_threshold = %f\n", bed_temper_min_j->valuedouble);
          #endif
        }
      }
    }
  }

  else {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_status(req, HTTPD_200);
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

  if (json != NULL) { cJSON_Delete(json); }

  if (thresholdEventsHandle != NULL) {
    xQueueSend(thresholdEventsHandle, (void*)&thresholdMessage, (TickType_t)0);
  }
  return ESP_OK;
}

static esp_err_t
get_sensor_data_handler(httpd_req_t *req) {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  float temperature;
  float humidity;
  int32_t voc_index;
  uint16_t raw_voc;

  char resp[HTTPD_RESP_SIZE] = {0};
  cJSON *resp_object_j = cJSON_CreateObject();

  if (xSemaphoreTake(sensorSemaphore, (TickType_t)10) == pdTRUE) {
    if (sht3x_measure(sensor, &temperature, &humidity)) {
      cJSON_AddNumberToObject(resp_object_j, "temperature", (double)temperature);
      cJSON_AddNumberToObject(resp_object_j, "humidity", (double)humidity);

      esp_err_t sgp40_status = sgp40_measure_voc(&air_q_sensor,
                                                 humidity,
                                                 temperature,
                                                 &voc_index);

      esp_err_t sgp40_status_raw = sgp40_measure_raw(&air_q_sensor,
                                                     humidity,
                                                     temperature,
                                                     &raw_voc);


      if (sgp40_status == ESP_OK) {
        cJSON_AddNumberToObject(resp_object_j, "voc_index", voc_index);
      }

      if (sgp40_status_raw == ESP_OK) {
        cJSON_AddNumberToObject(resp_object_j, "raw_voc", raw_voc);
      }

    }

    cJSON_AddNumberToObject(resp_object_j, "hour", (double)timeinfo.tm_hour);
    cJSON_AddNumberToObject(resp_object_j, "minute", (double)timeinfo.tm_min);

    cJSON_PrintPreallocated(resp_object_j, resp, HTTPD_RESP_SIZE, false);

    if (resp_object_j != NULL) { cJSON_Delete(resp_object_j); }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    xSemaphoreGive(sensorSemaphore);
    return ESP_OK;
  }
  else {
    printf("failed to acquire sensor semaphore in web request\n");
    return ESP_FAIL;
  }
}

static esp_err_t
fans_on_handler(httpd_req_t *req) {
  printf("fans_on_handler executed\n");
  char req_body[HTTPD_RESP_SIZE+1] = {0};
  char resp[HTTPD_RESP_SIZE] = {1};

  size_t body_size = MIN(req->content_len, (sizeof(req_body)-1));

  int ret = httpd_req_recv(req, req_body, body_size);

  // if ret == 0 then no data
  if (ret < 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }

  cJSON *json = cJSON_ParseWithLength(req_body, HTTPD_RESP_SIZE);
  cJSON *fan_time_j = NULL;

  if (json != NULL) {
    if (cJSON_IsObject(json)) {
      fan_time_j = cJSON_GetObjectItemCaseSensitive(json, "fan");
      if (cJSON_IsNumber(fan_time_j)) {
        printf("Running fans: time = %d\n", fan_time_j->valueint);
        run_fans(fan_time_j->valueint, MANUAL_PRIORITY);
      }
    }
  }

  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

  if (json != NULL) { cJSON_Delete(json); }
  return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t set_sensor_thresholds = {
    .uri      = "/sensor",
    .method   = HTTP_POST,
    .handler  = set_sensor_thresholds_handler,
    .user_ctx = NULL
};

/* URI handler structure for GET /uri */
httpd_uri_t get_sensor_data = {
    .uri      = "/sensor",
    .method   = HTTP_GET,
    .handler  = get_sensor_data_handler,
    .user_ctx = NULL
};

/* URI handler structure for POST /fans_on */
httpd_uri_t fans_on = {
    .uri      = "/fans_on",
    .method   = HTTP_POST,
    .handler  = fans_on_handler,
    .user_ctx = NULL
};

/* Function for starting the webserver */
httpd_handle_t
start_webserver(void) {
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &get_sensor_data);
        httpd_register_uri_handler(server, &set_sensor_thresholds);
        httpd_register_uri_handler(server, &fans_on);
    }
    /* If server failed to start, handle will be NULL */
    ESP_LOGI(TAG, "webserver started");
    return server;
}

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static void
time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void
obtain_time(void) {
    /**
     * NTP server address could be aquired via DHCP,
     * see following menuconfig options:
     * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
     * 'LWIP_SNTP_DEBUG' - enable debugging messages
     *
     */

    wifi_init_sta();
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);\
}

static void
initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);

/*
 * If 'NTP over DHCP' is enabled, we set dynamic pool address
 * as a 'secondary' server. It will act as a fallback server in case that address
 * provided via NTP over DHCP is not accessible
 */
#if LWIP_DHCP_GET_NTP_SRV && SNTP_MAX_SERVERS > 1
    sntp_setservername(1, "pool.ntp.org");

#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2          // statically assigned IPv6 address is also possible
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // ipv6 ntp source "ntp.netnod.se"
        sntp_setserver(2, &ip6);
    }
#endif  /* LWIP_IPV6 */

#else   /* LWIP_DHCP_GET_NTP_SRV && (SNTP_MAX_SERVERS > 1) */
    // otherwise, use DNS address from a pool
    sntp_setservername(0, "pool.ntp.org");
#endif

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    sntp_init();

    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void
event_handler(void *arg,
              esp_event_base_t event_base,
              int32_t event_id,
              void *event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      if (s_retry_num < MAXIMUM_RETRY) {
          esp_wifi_connect();
          s_retry_num++;
          ESP_LOGI(TAG, "retry to connect to the AP");
      }
      else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
      ESP_LOGI(TAG,"connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void
wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);      // accept NTP offers from DHCP server, if any
#endif

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	          .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);

        httpd_handle_t server;
        printf("Trying to start webserver\n");
        server = start_webserver();
        mqtt_app_start();

    }
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void
ledc_init(int gpio_num,
          int ledc_channel_num,
          int ledc_timer_num) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = ledc_timer_num,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = ledc_channel_num,
        .timer_sel      = ledc_timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_num,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void
app_main(void) {
    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    // fan stuff
    // Set the LEDC peripheral configuration
    ledc_init(LEDC_OUTPUT_IO, LEDC_CHANNEL, LEDC_TIMER);

    sensorSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(sensorSemaphore);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);

    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %jd sec: %li ms: %li us",
                        (intmax_t)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
    fanEventsHandle = xQueueCreateStatic(FAN_EV_NUM, sizeof (struct fan_event), fanQueueStorage, &fanEvents);
    thresholdEventsHandle = xQueueCreateStatic(10, sizeof (struct threshold_event), thresholdQueueStorage, &thresholdEvents);
    printerEventsHandle = xQueueCreateStatic(10, sizeof (struct printer_event), printerEventsQueueStorage, &printerEvents);

    configASSERT(fanEventsHandle);

    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_100K);

    // Create the sensors, multiple sensors are possible.
    sensor = sht3x_init_sensor(I2C_BUS, SHT3x_ADDR_1);
    initSGP40();

    createfanRunnerTask();
    createSensorManagerTask();
}
