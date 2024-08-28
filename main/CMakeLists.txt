idf_component_register(SRCS "fpm_webserver.c" "ota.c" "sntp.c" "wifiap.c" "fpm_modbus.c" "main.c" "ethernet.c" "spiffs.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)