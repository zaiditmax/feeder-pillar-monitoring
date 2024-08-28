#include "esp_err.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_wifi_types_generic.h"
#include "esp_eth_phy.h"
#include "esp_eth_driver.h"
#include "time.h"
#include "hal/uart_types.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"

#define MODBUS_TXD_PIN (GPIO_NUM_2)
#define MODBUS_RXD_PIN (GPIO_NUM_5)

#define ASYNC_IDLE 0
#define ASYNC_BUSY 1

#define APP_SOCKET_ALLOCATION (7)
#define SIMULTANEOUS_CLIENTS 2
#define MAX_WS_CLIENTS (SIMULTANEOUS_CLIENTS + 1)
#define APP_ASSOC_SOCKET_ALLOCATION 10
#define WS_URI_ALLOCATION (MAX_WS_CLIENTS * 2)

#define WS_CLIENT_TXTMSG_BFFR_CNT 18
#define WS_CLIENT_TXTMSG_BFFR_SIZE_450 450

#define MODBUS_READ 0
#define MODBUS_WRITE 1

#define WAGO_SET_ELEC 0
#define WAGO_SET_INFO 1

#define HTTPD_DEFAULT_CONFIG_FPM() {                        \
        .task_priority      = tskIDLE_PRIORITY+5,       \
        .stack_size         = 4096,                     \
        .core_id            = tskNO_AFFINITY,           \
        .task_caps          = (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),       \
        .server_port        = 80,                       \
        .ctrl_port          = ESP_HTTPD_DEF_CTRL_PORT,  \
        .max_open_sockets   = 7,                        \
        .max_uri_handlers   = 13,                        \
        .max_resp_headers   = 8,                        \
        .backlog_conn       = 5,                        \
        .lru_purge_enable   = false,                    \
        .recv_wait_timeout  = 5,                        \
        .send_wait_timeout  = 5,                        \
        .global_user_ctx = NULL,                        \
        .global_user_ctx_free_fn = NULL,                \
        .global_transport_ctx = NULL,                   \
        .global_transport_ctx_free_fn = NULL,           \
        .enable_so_linger = false,                      \
        .linger_timeout = 0,                            \
        .keep_alive_enable = false,                     \
        .keep_alive_idle = 0,                           \
        .keep_alive_interval = 0,                       \
        .keep_alive_count = 0,                          \
        .open_fn = NULL,                                \
        .close_fn = NULL,                               \
        .uri_match_fn = NULL                            \
}

typedef enum
{
    OTA_SAME_VERSION,
    OTA_BEGIN_FAILED,
    OTA_INVALID_LENGTH,
    OTA_WRITE_ERROR,
    OTA_RCV_ERR,
    OTA_TRANSPORT_ERR,
    OTA_WRITE_OK,
    OTA_VALIDATION_FAIL,
    OTA_END_FAIL,
    OTA_BOOT_FAIL,
    OTA_END_OK
}ota_return_t;

typedef enum
{
    MODBUSREAD_JSON_NOT_READY,
    MODBUSREAD_JSON_READY,
    MODBUSREAD_JSON_UARTFREE
}_enum_fpm_modbus_read;

typedef enum
{
    MODBUSWRITE_DEFAULT,
    MODBUSWRITE_SEND,
    MODBUSWRITE_WRITE_SEND,
    MODBUSWRITE_CMD_ERROR,
    MODBUSWRITE_WAIT_RESPONSE,
    MODBUSWRITE_NOT_OK,
    MODBUSWRITE_OK
}_enum_fpm_modbus_write;

typedef struct
{
    httpd_handle_t *handle;
    int fd;
    int assoc_fd[APP_ASSOC_SOCKET_ALLOCATION];
    char *uri;
    unsigned long key;
    uint8_t auth_status;
    uint32_t access;
    unsigned long textmessage_in_cntid;
    unsigned long textmessage_out_cntid;
    unsigned long textmessage_in_time_stamp;
    unsigned long textmessage_out_time_stamp;
    uint8_t ws_startup_init_done;
    uint8_t get_started;
    uint8_t rdmeter_confirm_get;
    uint8_t wrmeter_confirm_get;
    uint8_t setting_confirm_get;
    uint8_t infor_confirm_get;
    uint8_t inform_confirm_get;
    bool expecting_response;
    char *textmessage_out_priority;
    char textmessage_out[WS_CLIENT_TXTMSG_BFFR_CNT][WS_CLIENT_TXTMSG_BFFR_SIZE_450];
    char textmessage_in[WS_CLIENT_TXTMSG_BFFR_CNT][WS_CLIENT_TXTMSG_BFFR_SIZE_450];
    uint8_t textmessage_in_idx_write;
    uint8_t textmessage_in_idx_read;
    uint8_t textmessage_out_idx_write;
    uint8_t textmessage_out_idx_read;
    bool send_meter_infoconfig;
    bool send_meter_electrical;
    uint8_t pending_close;
    uint64_t entry_number;
    uint32_t time_persistent_timestamp;
}fpm_wsockets_t;

typedef struct
{
    int fd;
    int assoc_fd[APP_ASSOC_SOCKET_ALLOCATION];
}fpm_socket_t;

extern char wifiapip[30];
extern char ethen[5];
extern char wifiappsw[30];
extern char ethmac[20];
extern char ethgway[20];
extern char ethsub[20];
extern char ethip[20];
extern char wifimac[20];
extern char wifiappsw[30];
extern char serial[30];
extern char ethsen[5];
extern char ethsgway[20];
extern char ethssub[20];
extern char ethsip[20];
extern char userpsw_adminx[30];
extern char ethernet_status_msg[20];
extern char metermsg_infoconfig[12200];
extern char metermsg_electrical[12200];
extern uint8_t ethernet_link_down;
extern _enum_fpm_modbus_write enum_modbus_write;
extern uint32_t ethernet_init_timestamp;
extern uint32_t ethernet_link_down_timestamp;
extern uint32_t sensor_timestamp;
extern uint16_t metermsg_electrical_len;
extern uint16_t metermsg_infoconfig_len;

extern void spiffs(void);
extern void wifiap(void);
extern void ethernet_setup(char* en, char* ip, char* gw, char*mask);
extern esp_err_t WebServerStart(void);
extern _enum_fpm_modbus_read fpm_modbus_read_jSON(char *msg_init, char * json_string, uint16_t *json_str_len);
extern _enum_fpm_modbus_write fpm_modbus_write(_enum_fpm_modbus_write _modbus_write);
extern uint8_t global_modbus_operation;
extern httpd_handle_t server;
extern fpm_wsockets_t fpm_wsockets[MAX_WS_CLIENTS];
extern fpm_socket_t fpm_sockets[APP_SOCKET_ALLOCATION];
extern char modbus_write_str[100];
extern bool reset_instruction;
extern uint32_t reset_instruction_timestamp;
extern uint32_t modbus_error;

extern void sntp_GeneratePsw(void);
extern void init_fpm_modbus(uint8_t set);
extern void init_fpm_swsockets(void);
extern ota_return_t write_ota_boot(int data_read, char *ota_write_data);
extern ota_return_t write_ota_spiffs(int data_read, char *ota_write_data);
extern ota_return_t end_ota_boot(void);
extern ota_return_t end_ota_spiffs(void);
esp_err_t esp_ota_begin_spiffs(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle);
extern void ota_boot_init(void);
extern void ota_spiffs_init(void);
extern bool AsyncClientProcess(void);
extern void modbus_restart_cid(void);

extern void WsClientsProcessData(void);
extern void WsClientsSend_AppendCntID(void);
extern void WsClientsAuthenticationInit(void);
extern void WsClientsAutoMsg(void);
extern void ethernet(void);
extern void CleanSockets_IdleWsClients(void);

