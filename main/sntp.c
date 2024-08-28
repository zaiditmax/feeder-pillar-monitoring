#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include "total_app.h"

#define SYNC_PERIOD_SECONDS 20
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

static const char *TAG = "SNTP";

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void print_servers(void)
{
    ESP_LOGI(TAG, "List of configured NTP servers:");
    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i)
    {
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

void sntp_GeneratePsw(void)
{
    static time_t now;
    struct tm timeinfo = { 0 };
    static char datatime_string[20];
    static uint8_t sntp_init = 0;
    static uint32_t sync_period_cnt = 0;
    static uint32_t sntp_stamp = 0;

    if(strcmp(ethernet_status_msg, "Connected") == 0)
    {
        if(sntp_init == 0)
        {
            if(xTaskGetTickCount() - sntp_stamp > 1000)
            {
                ESP_LOGI(TAG, "Initializing and starting SNTP");
                esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2, ESP_SNTP_SERVER_LIST(CONFIG_SNTP_TIME_SERVER, "pool.ntp.org" ) );
                config.sync_cb = time_sync_notification_cb;
                esp_netif_sntp_init(&config);
                print_servers();
                setenv("TZ", "CST-8", 1);
                tzset();
                sntp_init = 1;
                sntp_stamp = xTaskGetTickCount();
            }
        }
        else if(sntp_init == 1)
        {
            if(xTaskGetTickCount() - sntp_stamp > 1000)
            {
                if(esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Time sync.");
                    sntp_init = 2;
                }
                sntp_stamp = xTaskGetTickCount();
            }
        }
        else if(sntp_init == 2)
        {   
            if(xTaskGetTickCount() - sntp_stamp > 1000)
            {
                esp_netif_sntp_deinit();
                sntp_init = 3;
                sntp_stamp = xTaskGetTickCount();
            }
        }
        else if(sntp_init == 3)
        {
            if(xTaskGetTickCount() - sntp_stamp > 1000)
            {
                sync_period_cnt++;
                if(sync_period_cnt >= SYNC_PERIOD_SECONDS)
                {
                    sntp_init = 0;
                    sync_period_cnt = 0;
                    sntp_init = 0;
                }
                sntp_stamp = xTaskGetTickCount();
            }
        }
    }

    time(&now);
    localtime_r(&now, &timeinfo);
    sprintf(datatime_string, "%02i", timeinfo.tm_hour);
    strcpy(userpsw_adminx,datatime_string);
    sprintf(datatime_string, "%02i", timeinfo.tm_min);
    strcat(userpsw_adminx,datatime_string);
    sprintf(datatime_string, "%02i", timeinfo.tm_mday);
    strcat(userpsw_adminx,datatime_string);
    sprintf(datatime_string, "%02i", timeinfo.tm_mon + 1);
    strcat(userpsw_adminx,datatime_string);
    sprintf(datatime_string, "%i", (timeinfo.tm_year + 1900));
    strcat(userpsw_adminx,datatime_string);
}