#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "total_app.h"

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    esp_log_level_set("*", ESP_LOG_NONE); 
    esp_log_level_set("WEB", ESP_LOG_INFO); 
    spiffs(); 
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    init_fpm_swsockets();
    ESP_ERROR_CHECK(WebServerStart());
    init_fpm_modbus(WAGO_SET_INFO);
    while(fpm_modbus_read_jSON("&console#inform=", metermsg_infoconfig, &metermsg_infoconfig_len) != MODBUSREAD_JSON_READY){vTaskDelay(pdMS_TO_TICKS(2));}
    init_fpm_modbus(WAGO_SET_ELEC);
    while(fpm_modbus_read_jSON("&console#rdmeter=", metermsg_electrical, &metermsg_electrical_len) != MODBUSREAD_JSON_READY){vTaskDelay(pdMS_TO_TICKS(2));}
    sensor_timestamp = xTaskGetTickCount(); 
    wifiap();
    strcpy(ethernet_status_msg, "Not Connected");
    if(strcmp(ethen, "Yes") == 0)
    {
        strcpy(ethernet_status_msg, "Connecting..");
        ethernet_setup(ethsen, ethsip, ethsgway, ethssub);
        ethernet_init_timestamp = xTaskGetTickCount();
    }
    
    while(1)
    {
        if((reset_instruction == true) && (xTaskGetTickCount() - reset_instruction_timestamp > 1000))
        {
            esp_restart();
        }
        ethernet();
        sntp_GeneratePsw();
        WsClientsProcessData();
        WsClientsAuthenticationInit();
        if(AsyncClientProcess() == ASYNC_IDLE)
        {
            WsClientsSend_AppendCntID();
            CleanSockets_IdleWsClients();
        }
        WsClientsAutoMsg();
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
