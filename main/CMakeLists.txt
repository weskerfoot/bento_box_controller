idf_component_register(SRCS "plant_water.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_http_server" "nvs_flash" "esp_http_client" "esp_eth" "driver" "esp8266_wrapper" "sht3x" "cjson")
