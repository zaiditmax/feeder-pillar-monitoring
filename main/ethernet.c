#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth_driver.h"
#include "esp_check.h"
#include "esp_mac.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "total_app.h"

#define ETH_PHY_ADDR             0
#define ETH_PHY_RST_GPIO         5
#define ETH_MDC_GPIO            23
#define ETH_MDIO_GPIO           18

esp_eth_handle_t eth_handle;
esp_netif_t * eth_netif;
esp_eth_phy_t *phy;

#define ETHERNET_CONNECTED_BIT  BIT0
#define ETHERNET_FAIL_BIT       BIT1

static const char *TAG = "ETH";

eth_phy_config_t phy_config;
esp_eth_handle_t eth_handle;

static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        sprintf(ethmac, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %s", ethmac);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        ethernet_link_down_timestamp = xTaskGetTickCount();
        ethernet_link_down = 1;
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        ethernet_link_down = 1;
        ethernet_link_down_timestamp = xTaskGetTickCount();
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    sprintf(ethip, IPSTR, IP2STR(&ip_info->ip));
    sprintf(ethsub, IPSTR, IP2STR(&ip_info->netmask));
    sprintf(ethgway, IPSTR, IP2STR(&ip_info->gw));
    strcpy(ethernet_status_msg, "Connected");
    ethernet_link_down = 0;
    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:%s", ethip);
    ESP_LOGI(TAG, "ETHMASK:%s", ethsub);
    ESP_LOGI(TAG, "ETHGW:%s", ethgway);
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr         = ETH_PHY_ADDR;
    phy_config.reset_gpio_num   = ETH_PHY_RST_GPIO;
    eth_esp32_emac_config_t esp32_emac_config = {
        .smi_mdc_gpio_num  = ETH_MDC_GPIO,                       
        .smi_mdio_gpio_num = ETH_MDIO_GPIO,                      
        .interface = EMAC_DATA_INTERFACE_RMII,        
        .clock_config =                               
        {                                             
            .rmii =                                   
            {                                         
                .clock_mode = EMAC_CLK_OUT,             // Output clk from ESP32
                .clock_gpio = EMAC_CLK_OUT_180_GPIO     // Choose GPIO17   
            }                                         
        },                                            
        .dma_burst_len = ETH_DMA_BURST_LEN_32 
    };
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    phy = esp_eth_phy_new_lan87xx(&phy_config);
    eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK, NULL, err, TAG, "Ethernet driver install failed");

    if (mac_out != NULL) {
        *mac_out = mac;
    }
    if (phy_out != NULL) {
        *phy_out = phy;
    }
    return eth_handle;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}

void ethernet_setup(char* en, char* ip, char* gw, char*mask)
{
    eth_handle = eth_init_internal(NULL, NULL);
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);
    
    if(strcmp(en, "Yes") == 0)
    {
        if (esp_netif_dhcpc_stop(eth_netif) != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to stop dhcp client");
            return;
        }
        esp_netif_ip_info_t info_t;
        memset(&info_t, 0, sizeof(esp_netif_ip_info_t));
        ipaddr_aton((const char *)ip, (ip_addr_t*)&info_t.ip.addr);
        ipaddr_aton((const char *)gw, (ip_addr_t*)&info_t.gw.addr);
        ipaddr_aton((const char *)mask, (ip_addr_t*)&info_t.netmask.addr);
        if(esp_netif_set_ip_info(eth_netif, &info_t) != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to set ip info");
        }
        char dns[] = "8.8.8.8";
        esp_netif_dns_info_t dns_t;
        ipaddr_aton((const char *)dns, (ip_addr_t*)&dns_t.ip);
        esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, &dns_t);
    }
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}