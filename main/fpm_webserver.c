/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* HTTP File Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_eth_phy_802_3.h"
#include "time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "http_parser.h"
#include "total_app.h"

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define SCRATCH_BUFSIZE  8192
#define MAX_FILE_SIZE   1220000
#define MAX_FILE_SIZE_STR "1220K"
#define CLEAN_SOCKETS_INTERVAL 5000
#define FAST_SEND_UI_TEXT_MESSAGE_DELAY 80
#define SLOW_SEND_UI_TEXT_MESSAGE_DELAY 200
#define AUTHENTICATED_FALSE 0
#define AUTHENTICATED_TRUE 1
#define WRITE_SETTING 0
#define READ_SETTING 1
#define WRITE_FILE 0
#define READ_FILE 1
#define THIS_CLIENT 0
#define OTHER_CLIENT 1
#define ALL_CLIENT 2
#define ONESECOND_TIME_PERSISTENT_PERIOD 1000
#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

typedef enum
{
    SUPERVISOR_ACCESS,
    ADMINISTRATOR_ACCESS,
    MAX_ADMIN,
    BAD_CREDENTIAL
}enum_validate_result_t;

typedef esp_err_t (*httpd_req_handler_t)(httpd_req_t *req);

typedef struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
}async_resp_arg_t;

typedef struct {
    httpd_req_t* req;
    httpd_req_handler_t handler;
} httpd_async_req_t;

typedef struct
{
    char data[SCRATCH_BUFSIZE];
    int len;
}ota_data_t;

static const char *TAG = "WEB";

struct file_server_data {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
};

const char *uri_html = "/";
const char *uri_logo = "/logo.png";
const char *uri_favicon = "/favicon.ico";
const char *uri_login = "/login";
const char *uri_custommsg = "/custommsg";
const char *uri_bootupdate = "/bootupdate/*";
const char *uri_dataupdate = "/dataupdate/*";

char FirmVer[15] = "V1.00";
char wifiapip[30] = " ";
char networkmode[30] = " ";
char ethen[5] = "Yes";
char wifiappsw[30] = "password";
char ethsen[5] = "Yes";
char ethsgway[20] = "192.168.0.1";
char ethssub[20] = "255.255.255.0";
char ethsip[20] = "192.168.0.50";
char serial[30] = " ";
char ethgway[20] = " ";
char ethsub[20] = " ";
char ethip[20] = " ";
char wifimac[20] = " ";
char ethmac[20] = " ";
char username_admin[30] = "admin";
char userpsw_admin[30] = "admin";
char username_svisor[30] = "supervisor";
char userpsw_svisor[30] = "supervisor";
char username_adminx[30] = "superadmin";
char userpsw_adminx[30] = "101010102024";

char key_str[WS_URI_ALLOCATION][20];
uint32_t key_uint32[WS_URI_ALLOCATION];
char page_access_str[WS_URI_ALLOCATION][15];
uint32_t page_access_number[WS_URI_ALLOCATION];
char dynamic_ws_uri[WS_URI_ALLOCATION][30];
httpd_uri_t ws[WS_URI_ALLOCATION];
uint8_t uri_idx_int = 0;
char uri_idx_str[10] = "0";

bool ethernet_try = 0;
uint8_t mcu_init = 0;
uint32_t time_key;
uint8_t fpm_wsockets_idx = 0;
uint8_t ethernet_link_down = 0;
char metermsg_infoconfig[12200];
char metermsg_electrical[12200];
uint16_t metermsg_infoconfig_len;
uint16_t metermsg_electrical_len;
uint8_t replace_slot = 1;
uint8_t replace_fd = 0;
uint16_t ct1;
uint16_t ct2;
fpm_wsockets_t *modbuswrite_xclient;
uint32_t ethernet_init_timestamp;
uint32_t send_ui_textmessages_timestamp;
uint32_t sensor_timestamp;
uint32_t sensor_elapsed;
uint32_t ethernet_link_down_timestamp;
uint32_t reset_instruction_timestamp;
bool reset_instruction = false;
uint32_t uploadtimeout;
unsigned long target_send_ui_text_message_delay;
char ethernet_status_msg[20] = "Not Connected";
char back_ethernet_status_msg[20] = "Not Connected";
fpm_wsockets_t fpm_wsockets[MAX_WS_CLIENTS];
fpm_socket_t fpm_sockets[APP_SOCKET_ALLOCATION];
httpd_handle_t server = NULL;
struct file_server_data *server_data = NULL;
volatile bool async_now = ASYNC_IDLE;
uint64_t entry_number = 0;
bool save_new_key = false;

static esp_err_t _ws_handler(httpd_req_t *req);

void ClientResetQueTextMessageOut(fpm_wsockets_t* fpm_wsocket)
{
    if((fpm_wsocket != NULL) && (fpm_wsocket -> fd != 0))
    {
        fpm_wsocket->textmessage_out_idx_write = fpm_wsocket->textmessage_out_idx_read;
        fpm_wsocket->send_meter_electrical = false;
    }
}

void ClientResetQueTextMessageIn(fpm_wsockets_t* fpm_wsocket)
{
    if((fpm_wsocket != NULL) && (fpm_wsocket -> fd != 0))
    {
        fpm_wsocket->textmessage_in_idx_write = fpm_wsocket->textmessage_in_idx_read;
    }
}

void ClientQueTextMessageOut(fpm_wsockets_t* fpm_wsocket, char* str)
{
    if((fpm_wsocket != NULL) && (fpm_wsocket -> fd != 0))
    {
        strcpy(&fpm_wsocket->textmessage_out[fpm_wsocket->textmessage_out_idx_write][0], str);
        fpm_wsocket->textmessage_out_idx_write++;
        if(fpm_wsocket->textmessage_out_idx_write >=  WS_CLIENT_TXTMSG_BFFR_CNT)
        {
            fpm_wsocket->textmessage_out_idx_write = 0;
        }
    }
}

void ClientQueTextMessageIn(fpm_wsockets_t* fpm_wsocket, char* str)
{
    if((fpm_wsocket != NULL) && (fpm_wsocket -> fd != 0))
    {
        strcpy(&fpm_wsocket->textmessage_in[fpm_wsocket->textmessage_in_idx_write][0], str);
        fpm_wsocket->textmessage_in_idx_write++;
        if(fpm_wsocket->textmessage_in_idx_write >= WS_CLIENT_TXTMSG_BFFR_CNT)
        {
            fpm_wsocket->textmessage_in_idx_write = 0;
        }
    }
}

void WSClientsQueText(fpm_wsockets_t *xclient, char * text, uint8_t direction)
{
    static uint8_t i;
    if((direction == ALL_CLIENT) || (xclient == NULL))
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if(fpm_wsockets[i].fd != 0){ClientQueTextMessageOut(&fpm_wsockets[i], text);}
        }
    }
    else if(direction == THIS_CLIENT)
    {
        ClientQueTextMessageOut(xclient, text);
    }
    else if(direction == OTHER_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if((&fpm_wsockets[i] != xclient) && (fpm_wsockets[i].fd != 0)){ClientQueTextMessageOut(&fpm_wsockets[i], text);}
        }
    }
}

bool IsClientTextMessageOutQueEmpty(fpm_wsockets_t *xclient)
{
    if((xclient->textmessage_out_idx_read == xclient->textmessage_out_idx_write) && (xclient->send_meter_electrical == false))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool IsClientTextMessageInQueEmpty(fpm_wsockets_t *xclient)
{
    if(xclient->textmessage_in_idx_read == xclient->textmessage_in_idx_write)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

esp_err_t settings_file_json(char* file_name, char* parameter_name, char* value_char_string, uint8_t mode)
{
    static char* json_string;
    static FILE* f;
    static char file_copy_buffer[100];
    static long unsigned int len;
    cJSON *json_item;   
    cJSON *valuejSON;
    if(mode == WRITE_SETTING)
    {
        f = fopen(file_name, "w");
        if(f == NULL)
        {
            ESP_LOGI(TAG, "Failed to open file for writing.");
            return ESP_FAIL;
        }
        else
        {
            cJSON* json_obj = cJSON_CreateObject();
            json_item = cJSON_CreateString(value_char_string);
            cJSON_AddItemToObject(json_obj, parameter_name, json_item);
            json_string = cJSON_Print(json_obj);
            fprintf(f, "%s", json_string);
            ESP_LOGI(TAG,"Write Setting\r\nwrite value=\r\n%s\r\nwrite file=\r\n%s\r\n", value_char_string, json_string);
            fclose(f);
            cJSON_free(json_string);
            cJSON_Delete(json_obj);
        }
    }
    else if(mode == READ_SETTING)
    {
        f = fopen(file_name, "r");
        if(f == NULL)
        {
            ESP_LOGI(TAG, "Failed to open file for reading, then write.");
            f = fopen(file_name, "w");
            if(f == NULL)
            {
                ESP_LOGI(TAG, "Failed to open file for writing.");
                return ESP_FAIL;
            }
            else
            {
                cJSON* json_obj = cJSON_CreateObject();
                json_item = cJSON_CreateString(value_char_string);
                cJSON_AddItemToObject(json_obj, parameter_name, json_item);
                json_string = cJSON_Print(json_obj);
                fprintf(f, "%s", json_string);
                ESP_LOGI(TAG,"Write Setting\r\nwrite value=\r\n%s\r\nwrite file=\r\n%s\r\n", value_char_string, json_string);
                fclose(f);
                cJSON_free(json_string);
                cJSON_Delete(json_obj);
                return ESP_OK;
            }
        }
        else
        {
            len = fread(file_copy_buffer, 1, sizeof(file_copy_buffer), f);
            file_copy_buffer[len] = 0;
            cJSON* json_parse = cJSON_Parse(file_copy_buffer);
            valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, parameter_name);
            strcpy(value_char_string, valuejSON->valuestring);
            ESP_LOGI(TAG, "Read Setting\r\ncopy file=\r\n%s\r\nget_value=\r\n%s\r\n ", file_copy_buffer, value_char_string);
            fclose(f);
            cJSON_Delete(json_parse);
        }
    }
    return ESP_OK;
}

static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    } else if (IS_FILE_EXT(filename, ".css")) {
        return httpd_resp_set_type(req, "text/css");
    } else if (IS_FILE_EXT(filename, ".js")) {
        return httpd_resp_set_type(req, "application/javascript");
    }
    return httpd_resp_set_type(req, "text/plain");
}

static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);
    const char *quest = strchr(uri, '?');
    if(quest)
    {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if(hash)
    {
        pathlen = MIN(pathlen, hash - uri);
    }
    if(base_pathlen + pathlen + 1 > destsize) 
    {
        return NULL;
    }
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);
    return dest + base_pathlen;
}

bool match_fpm_websocket(int fd)
{
    static uint8_t i;
    static uint8_t j;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fd == fpm_wsockets[i].fd)
        {
            return true;
        }
        for(j = 0; j < APP_ASSOC_SOCKET_ALLOCATION; j++)
        {
            if(fd == fpm_wsockets[i].assoc_fd[j])
            {
                return true;
            }
        } 
    }
    return false;
}

bool match_fpm_socket(int fd)
{
    static uint8_t i;
    static uint8_t j;
    for(i = 0; i < APP_SOCKET_ALLOCATION; i++)
    {
        if(fd == fpm_sockets[i].fd)
        {
            return true;
        }
        for(j = 0; j < APP_ASSOC_SOCKET_ALLOCATION; j++)
        {
            if(fd == fpm_sockets[i].assoc_fd[j])
            {
                return true;
            }
        } 
    }
    return false;
}

bool match_fpm_socketws(int fd)
{
    if(match_fpm_websocket(fd)){return true;}
    if(match_fpm_socket(fd)){return true;}
    return false;
}

enum_validate_result_t validate_credential(char *json_str)
{
    cJSON *json_parse = cJSON_Parse(json_str);
    if(json_parse != NULL)
    {
        cJSON* valuejSON_username = cJSON_GetObjectItemCaseSensitive(json_parse, "username");
        cJSON* valuejSON_password = cJSON_GetObjectItemCaseSensitive(json_parse, "password");
        if((valuejSON_username != NULL) && (valuejSON_password != NULL))
        {
            if(strcmp(username_admin, valuejSON_username->valuestring) == 0)
            {
                if(strcmp(userpsw_admin, valuejSON_password->valuestring) == 0)
                {
                    return ADMINISTRATOR_ACCESS;
                }
            }
            if(strcmp(username_svisor, valuejSON_username->valuestring) == 0)
            {
                if(strcmp(userpsw_svisor, valuejSON_password->valuestring) == 0)
                {
                    static uint8_t i;
                    static uint8_t admin_cnt;
                    admin_cnt = 0;
                    for(i = 0; i < MAX_WS_CLIENTS; i++)
                    {
                        if((fpm_wsockets[i].fd != 0) && (fpm_wsockets[i].access == ADMINISTRATOR_ACCESS))
                        {
                            admin_cnt++;
                        }
                    }
                    
                    if(admin_cnt >= SIMULTANEOUS_CLIENTS)
                    {
                        return MAX_ADMIN;
                    }
                    else
                    {
                        return SUPERVISOR_ACCESS; 
                    }
                }
            }
            if(strcmp(username_adminx, valuejSON_username->valuestring) == 0)
            {
                if(strcmp(userpsw_adminx, valuejSON_password->valuestring) == 0)
                {
                    return ADMINISTRATOR_ACCESS;
                }
            }
        }
    }
    return BAD_CREDENTIAL;
}

void register_new_socket(httpd_req_t *req)
{
    static uint8_t i, j, k;
    static int fd;
    size_t clients = APP_ASSOC_SOCKET_ALLOCATION;
    int client_fds[APP_ASSOC_SOCKET_ALLOCATION];   
    fd = httpd_req_to_sockfd(req);
    for(i = 0; i < APP_SOCKET_ALLOCATION; i++)
    {
        if(fpm_sockets[i].fd == 0)
        {
            fpm_sockets[i].fd = fd;
            ESP_LOGI(TAG,"Register socket fd = %d\r\n", fpm_sockets[i].fd);
            k = 0;
            if(httpd_get_client_list(server, &clients, client_fds) == ESP_OK) 
            {
                ESP_LOGI(TAG,"Fd list get success");
                for (j = 0; j < clients; j++) 
                {
                    int sock = client_fds[j];
                    if(match_fpm_socketws(sock) == false)
                    {
                        fpm_sockets[i].assoc_fd[k] = sock;
                        ESP_LOGI(TAG,"Register socket assoc_fd[%d] = %d\r\n", j, fpm_sockets[i].assoc_fd[j]);
                        k++;
                    }
                }
            }
            break;
        }
    } 
}

void remove_socket(httpd_req_t *req)
{
    static uint8_t i, j, k;
    static int fd;
    fd = httpd_req_to_sockfd(req);
    for(i = 0; i < APP_SOCKET_ALLOCATION; i++)
    {
        if(fpm_sockets[i].fd == fd)
        {
            httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
            ESP_LOGI(TAG,"Remove socket fd = %d\r\n", fpm_sockets[i].fd);
            fpm_sockets[i].fd = 0;
            k = 0;
            for(j = 0; j < APP_ASSOC_SOCKET_ALLOCATION; j++)
            {
                if(fpm_sockets[i].assoc_fd[j] != 0)
                {
                    httpd_sess_trigger_close(req->handle, fpm_sockets[i].assoc_fd[j]);
                    ESP_LOGI(TAG,"Remove socket assoc_fd[%d] = %d\r\n", k, fpm_sockets[i].assoc_fd[j]);
                    k++;
                    fpm_sockets[i].assoc_fd[j] = 0;
                }
            }
            break;
        }
    }
}

uint32_t make_keyuri_access(uint32_t _access)
{
    static uint8_t j;
    time_key = xTaskGetTickCount();
    if(time_key == 0)
    {
        time_key++;
    }
    for(j = 0; j < WS_URI_ALLOCATION; j++)
    {
        if(key_uint32[j] == time_key)
        {
            time_key++;
            if(time_key == 0){time_key++;}
            j = 0;
        }
    }
    static char build_filename[40];
    sprintf(build_filename, "/data/key%d.json", uri_idx_int);
    sprintf(key_str[uri_idx_int], "%lu", time_key);
    settings_file_json(build_filename, "key", key_str[uri_idx_int], WRITE_SETTING);
    key_uint32[uri_idx_int] = time_key;
    
    itoa(_access, page_access_str[uri_idx_int], 10);
    sprintf(build_filename, "/data/access%d.json", uri_idx_int);    
    settings_file_json(build_filename, "access", page_access_str[uri_idx_int], WRITE_SETTING);
    page_access_number[uri_idx_int] = _access;
    
    httpd_unregister_uri(server, dynamic_ws_uri[uri_idx_int]);
    sprintf(dynamic_ws_uri[uri_idx_int], "/ws%s", key_str[uri_idx_int]);  
    sprintf(build_filename, "/data/uri%d.json", uri_idx_int);
    settings_file_json(build_filename, "uri", dynamic_ws_uri[uri_idx_int], WRITE_SETTING);
    httpd_register_uri_handler(server, &ws[uri_idx_int]);
    uri_idx_int++;
    if(uri_idx_int >= WS_URI_ALLOCATION)
    {
        uri_idx_int = 0;
    }
    sprintf(uri_idx_str, "%d", uri_idx_int);
    settings_file_json("/data/uriidx", "idx", uri_idx_str, WRITE_SETTING);
    return time_key;
}

static esp_err_t login_handler(httpd_req_t *req)
{
    static char filepath[FILE_PATH_MAX];
    static const char *filename;
    static char buf[100];
    static int ret, remaining;
    static char number[40];
    static char grant_msg[100];
    static char key_resp_json[250];
    static enum_validate_result_t validate_result;
    filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path, req->uri, sizeof(filepath)); 
    if(!filename)
    {
        ESP_LOGI(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if(strcmp(filename, uri_login) == 0)
    {           
        remaining = req->content_len;
        while (remaining > 0)
        {
            if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0)
            {
                if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                {
                    continue;
                }
                return ESP_FAIL;
            }
            remaining -= ret;
        }   
        validate_result = validate_credential(buf);
        if(validate_result != BAD_CREDENTIAL)
        {
            if(validate_result == ADMINISTRATOR_ACCESS)
            {
                sprintf(number, "%lu", make_keyuri_access(validate_result));
                strcpy(grant_msg, "Administrator");  
            } 
            else if(validate_result == SUPERVISOR_ACCESS)
            {
                sprintf(number, "%lu", make_keyuri_access(validate_result));
                strcpy(grant_msg, "Supervisor");  

            }
            else if(validate_result == MAX_ADMIN)
            {
                strcpy(number, "0");
                strcpy(grant_msg, "No empty slot available for Supervisor access.");  
            }
            cJSON *key_json_obj = cJSON_CreateObject();

            cJSON *valuejSON = cJSON_CreateString(number);
            cJSON_AddItemToObject(key_json_obj, "key", valuejSON);

            valuejSON = cJSON_CreateString(grant_msg);
            cJSON_AddItemToObject(key_json_obj, "msg", valuejSON);

            char *json_print = cJSON_Print(key_json_obj);
            sprintf(key_resp_json,"%s", json_print);
            cJSON_free(json_print);
            cJSON_Delete(key_json_obj);
            if(httpd_resp_send_chunk(req, key_resp_json, strlen(key_resp_json)) != ESP_OK)
            {
                ESP_LOGI(TAG, "Sending response failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send response");
                return ESP_FAIL;
            }
            #ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
            httpd_resp_set_hdr(req, "Connection", "close");
            #endif
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Bad login");
    return ESP_FAIL;
}

static esp_err_t get_handler(httpd_req_t *req)
{
    static char filepath[FILE_PATH_MAX];
    static FILE *fd = NULL;
    static struct stat file_stat;
    static char filename_html[30];
    static char filepath_html[30];
    static const char *filename;
    strcpy(filename_html, "/index.html");
    strcpy(filepath_html, "/data/index.html");
    filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path, req->uri, sizeof(filepath));
    if(!filename)
    {
        ESP_LOGI(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if(strcmp(filename, "/") == 0)
    {
        filename = filename_html;
        strcpy(filepath, filepath_html);
    }
    if(stat(filepath, &file_stat) == -1)
    {
        ESP_LOGI(TAG, "Failed to stat file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "File open..");
    fd = fopen(filepath, "r");
    if (!fd)
    {
        ESP_LOGI(TAG, "Failed to read existing file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);
        if (chunksize > 0)
        {
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) 
            {
                fclose(fd);
                ESP_LOGI(TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
           }
        }
    } while (chunksize != 0);
    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

void GETALL(fpm_wsockets_t *xclient)
{
    ClientQueTextMessageOut(xclient, "&console#get=start");
    ClientQueTextMessageOut(xclient, "&console#get=set_loginad");
    ClientQueTextMessageOut(xclient, "&console#get=set_loginsv");
    ClientQueTextMessageOut(xclient, "&console#get=set_ap_password");
    ClientQueTextMessageOut(xclient, "&console#get=set_ethernet");
    ClientQueTextMessageOut(xclient, "&console#get=end");
}

void SetSensorSend(fpm_wsockets_t *xclient, uint8_t direction)
{
    static uint8_t i;
    if((direction == ALL_CLIENT) || (xclient == NULL))
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if((fpm_wsockets[i].fd != 0) && (fpm_wsockets[i].ws_startup_init_done == 1))
            {
                fpm_wsockets[i].send_meter_electrical = true;
            }
        }
    }
    else if(direction == OTHER_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if((&fpm_wsockets[i] != xclient) && (fpm_wsockets[i].fd != 0)){fpm_wsockets[i].send_meter_electrical = true;}
        }
    }
    else if(direction == THIS_CLIENT){xclient->send_meter_electrical = true;}
}

void QueClientUIWrMeter(fpm_wsockets_t *xclient, uint8_t direction)
{
    static char wrmeter_str_admin[100];
    static char wrmeter_str_svisor[100];
    static uint8_t i;
    static char access_title[20];

    char *json_print;
    cJSON* wrmeter_json_obj; 
    cJSON *valuejSON;
    wrmeter_json_obj = cJSON_CreateObject();
    strcpy(access_title, "administrator");
    valuejSON = cJSON_CreateString(access_title);
    cJSON_AddItemToObject(wrmeter_json_obj, "access", valuejSON);
    strcpy(wrmeter_str_admin, "&console#wrmeter=");
    json_print = cJSON_Print(wrmeter_json_obj);
    strcat(wrmeter_str_admin, json_print);
    cJSON_free(json_print);
    cJSON_Delete(wrmeter_json_obj);

    wrmeter_json_obj = cJSON_CreateObject();
    strcpy(access_title, "supervisor");
    valuejSON = cJSON_CreateString(access_title);
    cJSON_AddItemToObject(wrmeter_json_obj, "access", valuejSON);
    strcpy(wrmeter_str_svisor, "&console#wrmeter=");
    json_print = cJSON_Print(wrmeter_json_obj);
    strcat(wrmeter_str_svisor, json_print);
    cJSON_free(json_print);
    cJSON_Delete(wrmeter_json_obj);

    if(direction == ALL_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if(fpm_wsockets[i].fd != 0)
            {
                if(fpm_wsockets[i].access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], wrmeter_str_admin);}
                else if(fpm_wsockets[i].access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], wrmeter_str_svisor);}
            }
        }
    }
    else if(direction == THIS_CLIENT)
    {
        if(xclient->access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(xclient, wrmeter_str_admin);}
        else if(xclient->access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(xclient, wrmeter_str_svisor);}
    }
    else if(direction == OTHER_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if((fpm_wsockets[i].fd != 0) && (xclient != &fpm_wsockets[i]))
            {
                if(fpm_wsockets[i].access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], wrmeter_str_admin);}
                else if(fpm_wsockets[i].access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], wrmeter_str_svisor);}
            }
        }
    }
}

void QueClientUISetting(fpm_wsockets_t *xclient, uint8_t direction)
{
    static char setting_str_admin[450];
    static char setting_str_svisor[100];
    static uint8_t i;

    char *json_print;
    cJSON *valuejSON;
    cJSON* setting_json_obj;
    setting_json_obj = cJSON_CreateObject();
    valuejSON = cJSON_CreateString("administrator");
    cJSON_AddItemToObject(setting_json_obj, "access", valuejSON);
    valuejSON = cJSON_CreateString(username_admin);
    cJSON_AddItemToObject(setting_json_obj, "username_admin", valuejSON);
    valuejSON = cJSON_CreateString(userpsw_admin);
    cJSON_AddItemToObject(setting_json_obj, "userpsw_admin", valuejSON);
    valuejSON = cJSON_CreateString(username_svisor);
    cJSON_AddItemToObject(setting_json_obj, "username_svisor", valuejSON);
    valuejSON = cJSON_CreateString(userpsw_svisor);
    cJSON_AddItemToObject(setting_json_obj, "userpsw_svisor", valuejSON);
    valuejSON = cJSON_CreateString(wifiappsw);
    cJSON_AddItemToObject(setting_json_obj, "wifiappsw", valuejSON);
    valuejSON = cJSON_CreateString(ethsgway);
    cJSON_AddItemToObject(setting_json_obj, "ethsgway", valuejSON);
    valuejSON = cJSON_CreateString(ethsen);
    cJSON_AddItemToObject(setting_json_obj, "ethsen", valuejSON);
    valuejSON = cJSON_CreateString(ethsip);
    cJSON_AddItemToObject(setting_json_obj, "ethsip", valuejSON);
    valuejSON = cJSON_CreateString(ethssub);
    cJSON_AddItemToObject(setting_json_obj, "ethssub", valuejSON);
    valuejSON = cJSON_CreateString(ethsen);
    cJSON_AddItemToObject(setting_json_obj, "ethsen", valuejSON);
    valuejSON = cJSON_CreateString(ethernet_status_msg);
    cJSON_AddItemToObject(setting_json_obj, "ethernet_status_msg", valuejSON);
    valuejSON = cJSON_CreateString(ethen);
    cJSON_AddItemToObject(setting_json_obj, "ethen", valuejSON);

    strcpy(setting_str_admin, "&console#setting=");
    json_print = cJSON_Print(setting_json_obj);
    strcat(setting_str_admin, json_print);
    cJSON_free(json_print);
    cJSON_Delete(setting_json_obj);

    setting_json_obj = cJSON_CreateObject();
    valuejSON = cJSON_CreateString("supervisor");
    cJSON_AddItemToObject(setting_json_obj, "access", valuejSON);
    strcpy(setting_str_svisor, "&console#setting=");
    json_print = cJSON_Print(setting_json_obj);
    strcat(setting_str_svisor, json_print);
    cJSON_free(json_print);
    cJSON_Delete(setting_json_obj);
    if(direction == ALL_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if(fpm_wsockets[i].fd != 0)
            {
                if(fpm_wsockets[i].access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], setting_str_admin);}
                else if(fpm_wsockets[i].access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], setting_str_svisor);}
            }
        }
    }
    else if(direction == THIS_CLIENT)
    {
        if(xclient->access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(xclient, setting_str_admin);}
        else if(xclient->access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(xclient, setting_str_svisor);}
    }
    else if(direction == OTHER_CLIENT)
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if((fpm_wsockets[i].fd != 0) && (xclient != &fpm_wsockets[i]))
            {
                if(fpm_wsockets[i].access == ADMINISTRATOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], setting_str_admin);}
                else if(fpm_wsockets[i].access == SUPERVISOR_ACCESS){ClientQueTextMessageOut(&fpm_wsockets[i], setting_str_svisor);}
            }
        }
    }
}

void QueClientUIInfor(fpm_wsockets_t *xclient, uint8_t direction)
{
    static char infor_str[350];
    cJSON *valuejSON;
    cJSON* infor_json_obj;

    infor_json_obj = cJSON_CreateObject();
    valuejSON = cJSON_CreateString(FirmVer);
    cJSON_AddItemToObject(infor_json_obj, "FirmVer", valuejSON);
    memset(networkmode, 0, sizeof(networkmode));
    if(strcmp(wifiapip, " ") != 0)
    {
        strcat(networkmode, "[Wifi AP]");
    }
    if(strcmp(ethip, " ") != 0)
    {
        strcat(networkmode, "[Ethernet]");
    }
    valuejSON = cJSON_CreateString(networkmode);
    cJSON_AddItemToObject(infor_json_obj, "networkmode", valuejSON);
    valuejSON = cJSON_CreateString(wifimac);
    cJSON_AddItemToObject(infor_json_obj, "wifimac", valuejSON);
    valuejSON = cJSON_CreateString(wifiapip);
    cJSON_AddItemToObject(infor_json_obj, "wifiapip", valuejSON);
    valuejSON = cJSON_CreateString(ethmac);
    cJSON_AddItemToObject(infor_json_obj, "ethmac", valuejSON);
    valuejSON = cJSON_CreateString(ethip);
    cJSON_AddItemToObject(infor_json_obj, "ethip", valuejSON);
    valuejSON = cJSON_CreateString(ethgway);
    cJSON_AddItemToObject(infor_json_obj, "ethgway", valuejSON);
    valuejSON = cJSON_CreateString(ethsub);
    cJSON_AddItemToObject(infor_json_obj, "ethsub", valuejSON);
    valuejSON = cJSON_CreateString(serial);
    cJSON_AddItemToObject(infor_json_obj, "serial", valuejSON);

    strcpy(infor_str, "&console#infor=");
    char *json_print = cJSON_Print(infor_json_obj);
    strcat(infor_str, json_print);
    WSClientsQueText(xclient, infor_str, direction);
    cJSON_free(json_print);
    cJSON_Delete(infor_json_obj);
}

void SUBMITALL(fpm_wsockets_t *xclient)
{ 
    QueClientUISetting(xclient, THIS_CLIENT);
    QueClientUIInfor(xclient, THIS_CLIENT);
    SetSensorSend(xclient, THIS_CLIENT);
}

void REFRESHCONNECTION(fpm_wsockets_t *xclient)
{
    static uint8_t i;
    static char restart_string[30];
    
    strcpy(restart_string, "&console#start");
    if(xclient != NULL)
    {
        ClientResetQueTextMessageIn(xclient);
        ClientResetQueTextMessageOut(xclient);
        if(xclient->ws_startup_init_done == 1)
        {
            xclient->get_started = 1;
            GETALL(xclient);
        }
        ClientQueTextMessageOut(xclient, restart_string);
    }
    else
    {
        for(i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if(fpm_wsockets[i].fd != 0)
            {
                ClientResetQueTextMessageIn(&fpm_wsockets[i]);
                ClientResetQueTextMessageOut(&fpm_wsockets[i]);
                if(fpm_wsockets[i].ws_startup_init_done == 1)
                {
                    fpm_wsockets[i].get_started = 1;
                    GETALL(&fpm_wsockets[i]);
                }
                ClientQueTextMessageOut(&fpm_wsockets[i], restart_string);
            }
        }
    }
}

void clear_new_ws_client(fpm_wsockets_t *fpm_wsocket)
{
    static uint8_t i;
    fpm_wsocket->fd = 0;
    fpm_wsocket->handle =NULL;
    for(i = 0; i < APP_ASSOC_SOCKET_ALLOCATION; i++)
    {
        fpm_wsocket->assoc_fd[i] = 0;
    }
    fpm_wsocket->key = 0;
    fpm_wsocket->auth_status = AUTHENTICATED_FALSE;
    fpm_wsocket->access = SUPERVISOR_ACCESS;
    fpm_wsocket->textmessage_out_cntid = fpm_wsocket->textmessage_in_cntid = 0;
    fpm_wsocket->textmessage_in_time_stamp = fpm_wsocket->textmessage_out_time_stamp = xTaskGetTickCount();
    ClientResetQueTextMessageIn(fpm_wsocket);
    ClientResetQueTextMessageOut(fpm_wsocket);
    fpm_wsocket->textmessage_out_priority = NULL;
    fpm_wsocket->ws_startup_init_done = 0;
    fpm_wsocket->expecting_response = false;
    fpm_wsocket->rdmeter_confirm_get = 0;
    fpm_wsocket->setting_confirm_get = 0;
    fpm_wsocket->infor_confirm_get = 0;
    fpm_wsocket->send_meter_infoconfig = false;
    fpm_wsocket->send_meter_electrical = false;
    fpm_wsocket->pending_close = false;
    fpm_wsocket->textmessage_in_idx_write = 0;
    fpm_wsocket->textmessage_in_idx_read = 0;
    fpm_wsocket->textmessage_out_idx_write = 0;
    fpm_wsocket->textmessage_out_idx_read = 0;
    fpm_wsocket->entry_number = 0;
    fpm_wsocket->time_persistent_timestamp = xTaskGetTickCount();
}

int get_prior_fd(fpm_wsockets_t* curr_client)
{
    static uint8_t i;
    static int the_fd;
    static uint64_t least_entry_number;
    the_fd = curr_client->fd;
    least_entry_number = curr_client->entry_number;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if((least_entry_number > fpm_wsockets[i].entry_number) && (fpm_wsockets[i].fd != 0))
        {
            least_entry_number = fpm_wsockets[i].entry_number;
            the_fd = fpm_wsockets[i].fd;
        }
    }
    return the_fd;
}

void register_new_ws(httpd_req_t *req)
{
    static uint8_t i, j, k, in_fd_cnt;
    size_t clients = APP_ASSOC_SOCKET_ALLOCATION;
    int client_fds[APP_ASSOC_SOCKET_ALLOCATION];
    static int fd;
    static uint8_t new_ws_register_slot;
    fd = httpd_req_to_sockfd(req);

    new_ws_register_slot = MAX_WS_CLIENTS;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd == 0)
        {
            new_ws_register_slot = i;
            break;
        }
    }
    if(new_ws_register_slot < MAX_WS_CLIENTS)
    {
        clear_new_ws_client(&fpm_wsockets[new_ws_register_slot]);
        fpm_wsockets[new_ws_register_slot].fd = fd;
        fpm_wsockets[new_ws_register_slot].handle = req->handle;
        fpm_wsockets[new_ws_register_slot].entry_number = entry_number;
        entry_number++;
        ESP_LOGI(TAG,"Register websocket fd = %d\r\n", fpm_wsockets[new_ws_register_slot].fd);
        k = 0;
        if(httpd_get_client_list(server, &clients, client_fds) == ESP_OK) 
        {
            ESP_LOGI(TAG,"Fd list get success");
            for (j = 0; j < clients; j++) 
            {
                int sock = client_fds[j];
                if((match_fpm_socketws(sock) == false) && (sock != 0))
                {
                    fpm_wsockets[new_ws_register_slot].assoc_fd[k] = sock;
                    ESP_LOGI(TAG,"Register websocket assoc_fd[%d] = %d\r\n", j, fpm_wsockets[new_ws_register_slot].assoc_fd[k]);
                    k++;
                }
            }
        }
        ESP_LOGI(TAG, "Handshake done, the new connection was opened, assigned fd = %d.", fd);
    }
    else
    {
        register_new_socket(req);
        remove_socket(req);
    }
    in_fd_cnt = 0;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd != 0)
        {
            in_fd_cnt++;
        }
    }
    if(in_fd_cnt >= MAX_WS_CLIENTS)
    {
        if(replace_fd == 0)
        {
            replace_fd = get_prior_fd(&fpm_wsockets[new_ws_register_slot]);
        }
    }
}

void remove_websocket(int fd)
{
    static uint8_t i, j, k;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd == fd)
        {
            httpd_sess_trigger_close(server, fd);
            ESP_LOGI(TAG,"Remove websocket fd = %d\r\n", fpm_wsockets[i].fd);
            fpm_wsockets[i].fd = 0;
            k = 0;
            for(j = 0; j < APP_ASSOC_SOCKET_ALLOCATION; j++)
            {
                if(fpm_wsockets[i].assoc_fd[j] != 0)
                {
                    httpd_sess_trigger_close(server, fpm_wsockets[i].assoc_fd[j]);
                    ESP_LOGI(TAG,"Remove websocket assoc_fd[%d] = %d\r\n", k, fpm_wsockets[i].assoc_fd[j]);
                    k++;
                    fpm_wsockets[i].assoc_fd[j] = 0;
                }
            }
            break;
        }
    }
}

fpm_wsockets_t *get_ws_client_from_sock_descriptor(int id)
{
    static uint8_t i;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(id == fpm_wsockets[i].fd)
        {
            return &fpm_wsockets[i];
        }
    }
    return NULL;
}

esp_err_t ws_handler(httpd_req_t *req)
{
   static int fd;
    if (req->method == HTTP_GET)
    {
        register_new_ws(req);
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    static httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0); 
    if (ret != ESP_OK)
    {
        fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "httpd_ws_recv_frame failed to get frame len with %d, fd = %d", ret, fd);
        remove_websocket(fd);
        return ret;
    }
    fd = httpd_req_to_sockfd(req); 
    if (ws_pkt.len) 
    {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGI(TAG, "Failed to calloc memory for buf");
            remove_websocket(fd);
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            remove_websocket(fd);
            return ret;
        }
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT || ws_pkt.type == HTTPD_WS_TYPE_PING || ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
        {
            ClientQueTextMessageIn(get_ws_client_from_sock_descriptor(fd), (char*)ws_pkt.payload);
            free(buf);
            return ESP_OK;
        } 
        else
        {
            if (ws_pkt.type == HTTPD_WS_TYPE_PING)
            {
                ESP_LOGI(TAG, "Got a WS PING frame, Replying PONG");
                ws_pkt.type = HTTPD_WS_TYPE_PONG;
                ret = httpd_ws_send_frame(req, &ws_pkt);
                get_ws_client_from_sock_descriptor(fd)->expecting_response = false;
                if (ret != ESP_OK)
                {
                    ESP_LOGI(TAG, "httpd_ws_send_frame failed with %d", ret);
                }
            } 
            else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
            {
                ws_pkt.len = 0;
                ws_pkt.payload = NULL;
                remove_websocket(fd);
            }
        }
        free(buf);
        return ret;
    }
    free(buf);
    return ESP_OK;
}

static esp_err_t custommsg_handler(httpd_req_t *req)
{
    static char filepath[FILE_PATH_MAX];
    static const char *filename;
    static char buf[100];
    static int ret, remaining;
    static uint8_t i;
    filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path, req->uri, sizeof(filepath)); 
    if(!filename)
    {
        ESP_LOGI(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if(strcmp(filename, uri_custommsg) == 0)
    {           
        remaining = req->content_len;
        while (remaining > 0)
        {
            if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0)
            {
                if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                {
                    continue;
                }
                return ESP_FAIL;
            }
            remaining -= ret;
        }   
        cJSON *json_parse = cJSON_Parse(buf);
        if(json_parse != NULL)
        {
            cJSON* valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "custommsg");
            if(valuejSON != NULL)
            {
                if(strcmp("usercnt", valuejSON->valuestring) == 0)
                {
                    static uint8_t active_user_cnt;
                    static char active_user_str[15];
                    static char max_user_str[15];
                    static char users_str[50];
                    
                    active_user_cnt = 0;
                    for(i = 0; i < MAX_WS_CLIENTS; i++)
                    {
                        if(fpm_wsockets[i].fd != 0)
                        {
                            active_user_cnt++;
                        }
                    }
                    
                    cJSON *users_json_obj = cJSON_CreateObject();

                    itoa(active_user_cnt, active_user_str, 10);
                    valuejSON = cJSON_CreateString(active_user_str);
                    cJSON_AddItemToObject(users_json_obj, "active_user", valuejSON);
                    
                    itoa(SIMULTANEOUS_CLIENTS, max_user_str, 10);
                    valuejSON = cJSON_CreateString(max_user_str);
                    cJSON_AddItemToObject(users_json_obj, "max_user", valuejSON);
                    
                    char *json_print = cJSON_Print(users_json_obj);
                    sprintf(users_str, "%s", json_print);
                    cJSON_free(json_print);
                    cJSON_Delete(users_json_obj);
                    if(httpd_resp_send_chunk(req, users_str, strlen(users_str)) != ESP_OK)
                    {
                        ESP_LOGI(TAG, "Sending response failed!");
                        httpd_resp_sendstr_chunk(req, NULL);
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send response");
                        return ESP_FAIL;
                    }
                    httpd_resp_send_chunk(req, NULL, 0);
                    return ESP_OK;
                }
            }
        }
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Bad post");
    return ESP_FAIL;
}

static esp_err_t bootupdate_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    static ota_return_t ota_return;
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/bootupdate") - 1, sizeof(filepath));
    if (!filename)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if (filename[strlen(filename) - 1] == '/')
    {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    if (req->content_len > MAX_FILE_SIZE)
    {
        ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File size must be less than " MAX_FILE_SIZE_STR "!");
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "Filepath : %s", filepath);
    ESP_LOGI(TAG, "Filename : %s", filename);
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
    int received;
    int remaining = req->content_len;
    if(xTaskGetTickCount() - uploadtimeout > 5000)
    {
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Connection", "close");
        httpd_resp_sendstr(req, "Init timeout");
        return ESP_FAIL;
    }
    ota_boot_init();
    while (remaining > 0)
    {   
        ESP_LOGI(TAG, "Remaining size : %d", remaining);
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGE(TAG, "HTTPD Socket timeout!");
                continue;
            }
            ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }
        ota_return = write_ota_boot(received, buf);
        if(ota_return != OTA_WRITE_OK)
        {
            httpd_resp_set_status(req, "303 See Other");
            httpd_resp_set_hdr(req, "Connection", "close");
            ESP_LOGI(TAG, "Ota error = %u\r\n", ota_return);
            switch(ota_return)
            {
                case OTA_SAME_VERSION: httpd_resp_sendstr(req, "Newer FW version is required"); break;
                case OTA_BEGIN_FAILED: httpd_resp_sendstr(req, "Failed to initialise"); break;
                case OTA_INVALID_LENGTH: httpd_resp_sendstr(req, "Frame length error"); break;
                case OTA_WRITE_ERROR: httpd_resp_sendstr(req, "Flash write error"); break;
                case OTA_RCV_ERR: httpd_resp_sendstr(req, "Data read error"); break;
                case OTA_TRANSPORT_ERR: httpd_resp_sendstr(req, "Data transport error"); break;
                default: httpd_resp_sendstr(req, "Unknown error"); break;
            }
            return ESP_FAIL;
        }
        remaining -= received;
    }
    ota_return = end_ota_boot();
    ESP_LOGI(TAG, "File reception complete");
    httpd_resp_set_status(req, "303 See Other");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    switch(ota_return)
    {
        case OTA_VALIDATION_FAIL: httpd_resp_sendstr(req, "Validation fail"); break;
        case OTA_END_FAIL: httpd_resp_sendstr(req, "End fail"); break;
        case OTA_BOOT_FAIL: httpd_resp_sendstr(req, "Boot set fail"); break;
        case OTA_END_OK: httpd_resp_sendstr(req, "Completed"); break;
        default: httpd_resp_sendstr(req, "Unknown error"); break;
    }
    return ESP_OK;
}

static esp_err_t dataupdate_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    static ota_return_t ota_return;
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/dataupdate") - 1, sizeof(filepath));
    if (!filename)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }
    if (filename[strlen(filename) - 1] == '/')
    {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    if (req->content_len > MAX_FILE_SIZE)
    {
        ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File size must be less than " MAX_FILE_SIZE_STR "!");
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "Filepath : %s", filepath);
    ESP_LOGI(TAG, "Filename : %s", filename);
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
    int received;
    int remaining = req->content_len;
    if(xTaskGetTickCount() - uploadtimeout > 5000)
    {
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Connection", "close");
        httpd_resp_sendstr(req, "Init timeout");
        return ESP_FAIL;
    }
    ota_spiffs_init();
    while (remaining > 0)
    {   
        ESP_LOGI(TAG, "Remaining size : %d", remaining);
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGE(TAG, "HTTPD Socket timeout!");
                continue;
            }
            ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }
        ota_return = write_ota_spiffs(received, buf);
        if(ota_return != OTA_WRITE_OK)
        {
            httpd_resp_set_status(req, "303 See Other");
            httpd_resp_set_hdr(req, "Connection", "close");
            ESP_LOGI(TAG, "Ota error = %u\r\n", ota_return);
            switch(ota_return)
            {
                case OTA_SAME_VERSION: httpd_resp_sendstr(req, "Newer FW version is required"); break;
                case OTA_BEGIN_FAILED: httpd_resp_sendstr(req, "Failed to initialise"); break;
                case OTA_INVALID_LENGTH: httpd_resp_sendstr(req, "Frame length error"); break;
                case OTA_WRITE_ERROR: httpd_resp_sendstr(req, "Flash write error"); break;
                case OTA_RCV_ERR: httpd_resp_sendstr(req, "Data read error"); break;
                case OTA_TRANSPORT_ERR: httpd_resp_sendstr(req, "Data transport error"); break;
                default: httpd_resp_sendstr(req, "Unknown error"); break;
            }
            return ESP_FAIL;
        }

        remaining -= received;
    }
    ota_return = end_ota_spiffs();
    ESP_LOGI(TAG, "File reception complete");
    httpd_resp_set_status(req, "303 See Other");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    switch(ota_return)
    {
        case OTA_VALIDATION_FAIL: httpd_resp_sendstr(req, "Validation fail"); break;
        case OTA_END_FAIL: httpd_resp_sendstr(req, "End fail"); break;
        case OTA_END_OK: httpd_resp_sendstr(req, "Completed"); break;
        default: httpd_resp_sendstr(req, "Unknown error"); break;
    }
    return ESP_OK;
}


bool has_clients(void)
{
    static uint8_t i;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd != 0)
        {
            return true;
        }
    }
    return false;
}

void CleanSockets_IdleWsClients(void)
{
    static uint8_t i;
    for (i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd != 0)
        {
            if((xTaskGetTickCount() - fpm_wsockets[i].textmessage_out_time_stamp >= CLEAN_SOCKETS_INTERVAL) && (xTaskGetTickCount() - fpm_wsockets[i].textmessage_in_time_stamp >= CLEAN_SOCKETS_INTERVAL))
            {
                ESP_LOGI(TAG,"Persistent remove fd = %d", fpm_wsockets[i].fd);
                remove_websocket(fpm_wsockets[i].fd);
                target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;
                send_ui_textmessages_timestamp = xTaskGetTickCount();
            }
        }
    }
}

void WsClientsAutoMsg(void)
{
    static bool modbus_toggle_ReadWrite = MODBUS_READ;
    if(strcmp(ethernet_status_msg, back_ethernet_status_msg) != 0)
    {
        QueClientUISetting(NULL, ALL_CLIENT);
        QueClientUIInfor(NULL, ALL_CLIENT);
        strcpy(back_ethernet_status_msg, ethernet_status_msg);
    }
    sensor_elapsed = xTaskGetTickCount() - sensor_timestamp;

    if(modbus_toggle_ReadWrite == MODBUS_READ)
    {
        if(sensor_elapsed > 1000)
        {
            static _enum_fpm_modbus_read enum_modbus_read;
            enum_modbus_read = fpm_modbus_read_jSON("&console#rdmeter=", metermsg_electrical, &metermsg_electrical_len);
            switch(enum_modbus_read)
            {
                case MODBUSREAD_JSON_READY:
                    SetSensorSend(NULL, ALL_CLIENT);
                    sensor_timestamp = xTaskGetTickCount();
                    if(enum_modbus_write == MODBUSWRITE_SEND)
                    {
                        modbus_toggle_ReadWrite = MODBUS_WRITE;
                    }
                break;    
                case MODBUSREAD_JSON_UARTFREE:
                    if(enum_modbus_write == MODBUSWRITE_SEND)
                    {
                        modbus_toggle_ReadWrite = MODBUS_WRITE;
                    }
                break;
                default:
                break;
            }
        }
    }
    else if(modbus_toggle_ReadWrite == MODBUS_WRITE)
    {
        enum_modbus_write = fpm_modbus_write(enum_modbus_write);
        static char modbus_write_return_msg[100];
        switch(enum_modbus_write)
        {
            case MODBUSWRITE_OK:
                modbus_toggle_ReadWrite = MODBUS_READ;
                modbus_restart_cid();
                ClientQueTextMessageOut(modbuswrite_xclient, "&console#mbresp=Write Successful");
                enum_modbus_write = MODBUSWRITE_DEFAULT;                    
            break;
            case MODBUSWRITE_NOT_OK:
            case MODBUSWRITE_CMD_ERROR:
                modbus_toggle_ReadWrite = MODBUS_READ;
                sprintf(modbus_write_return_msg, "&console#mbresp=Error(%lu)", modbus_error);
                ClientQueTextMessageOut(modbuswrite_xclient, modbus_write_return_msg);
                enum_modbus_write = MODBUSWRITE_DEFAULT;
            break;
            default:
            break;    
        }
    }

}

void WsClientsAuthenticationInit(void)
{
    static uint8_t i;
    static char validate_str[30];
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if((fpm_wsockets[i].fd != 0)
        && (fpm_wsockets[i].auth_status == AUTHENTICATED_TRUE)
        && (fpm_wsockets[i].ws_startup_init_done == 0))
        {
            if(fpm_wsockets[i].rdmeter_confirm_get == 1)
            {
                fpm_wsockets[i].ws_startup_init_done = 1;
                strcpy(validate_str, "&console#validate=1");
                ClientQueTextMessageOut(&fpm_wsockets[i], validate_str);
            }
        }
    }
}

void ProcessWsDataFD(fpm_wsockets_t *xclient, char *textmessage)
{
    static uint8_t i;
    static char out_str[50];
    static char *dmmyptr;
    static char *delimiter_nxt_location;
    static uint32_t key;
    static uint32_t cntID;
    cJSON *json_parse;
    cJSON *valuejSON;
    ESP_LOGI(TAG, "Client %d ui -> %s", xclient->fd, textmessage);

    if(memcmp((char*)textmessage, "&console", 8) == 0)
    { 
        delimiter_nxt_location = strchr((char*)textmessage, '*');
        *delimiter_nxt_location = 0;
        delimiter_nxt_location++;
        key = (uint32_t)strtoul(delimiter_nxt_location, &dmmyptr, 10);
        delimiter_nxt_location = strchr(delimiter_nxt_location, '*');
        delimiter_nxt_location++;
        cntID = (uint32_t)strtoul(delimiter_nxt_location, &dmmyptr, 10);
        for(i = 0; i < WS_URI_ALLOCATION; i++)
        {
            if((key == key_uint32[i]) && (key != 0))
            {
                xclient->auth_status = AUTHENTICATED_TRUE;
                xclient->access = page_access_number[i];
                if((cntID != xclient->textmessage_in_cntid) && (xclient->textmessage_in_cntid != 0))
                {
                    xclient->textmessage_in_cntid = cntID;
                    REFRESHCONNECTION(xclient);
                }
                xclient->key = key;
            }
        }
        memset(out_str, 0, sizeof(out_str));
        strcpy(out_str, "&console#");
        if(memcmp((char*)&textmessage[8], "#new_ws=", 8) == 0)
        {
            if(xclient->auth_status == AUTHENTICATED_TRUE)
            {
                if(xclient->get_started == 1)
                {
                    GETALL(xclient);
                }
                else
                {
                    strcat(out_str, "start");
                    ClientQueTextMessageOut(xclient, out_str);
                    xclient->textmessage_in_cntid = cntID;
                    SetSensorSend(xclient, THIS_CLIENT);
                    xclient->send_meter_infoconfig = true;
                }                
            }
            else
            {
                strcat(out_str, "load=logon");
                ClientQueTextMessageOut(xclient, out_str);
            }
        }
        else if(memcmp((char*)&textmessage[8], "#start", 6) == 0)
        {
            ClientResetQueTextMessageOut(xclient);
            if(xclient->get_started == 1)
            {
                GETALL(xclient);
            }
            else
            {
                if(xclient->auth_status == AUTHENTICATED_TRUE)
                {
                    SUBMITALL(xclient);
                }
                else
                {
                    strcat(out_str, "load=logon");
                    ClientQueTextMessageOut(xclient, out_str);
                }
            }
        }
        else if(memcmp((char*)&textmessage[8], "#reset", 6) == 0)
        {
            reset_instruction = true;
            reset_instruction_timestamp = xTaskGetTickCount();
        }
        else if(memcmp((char*)&textmessage[8], "#close", 6) == 0){}
        else if(memcmp((char*)&textmessage[8], "#persistent", 11) == 0){}
        else if(memcmp((char*)&textmessage[8], "#validate=1z", 12) == 0){}
        else if(memcmp((char*)&textmessage[8], "#get=start", 10) == 0){}
        else if(memcmp((char*)&textmessage[8], "#get=end", 8) == 0)
        {
            strcat(out_str, "clients_get_end");
            WSClientsQueText(xclient, out_str, ALL_CLIENT);
            xclient->get_started = 0;
        }
        else if(xclient->auth_status == AUTHENTICATED_TRUE)
        {
            if(memcmp((char*)&textmessage[8], "#waitupload", 11) == 0)
            {
                uploadtimeout = xTaskGetTickCount(); 
                strcat(out_str, "uploadnow");
                ClientQueTextMessageOut(xclient, out_str);
            }
            else if(memcmp((char*)&textmessage[8], "#modbuswr?", 10) == 0)
            {
                printf("Get modbus IN = %s\r\n", &textmessage[18]);
                strcpy(modbus_write_str, &textmessage[18]);
                if(enum_modbus_write == MODBUSWRITE_DEFAULT)
                {
                    modbuswrite_xclient = xclient;
                    enum_modbus_write = MODBUSWRITE_SEND;
                }
                else
                {
                    ClientQueTextMessageOut(xclient, "&console#mbresp=Write Not Successful");
                }
            }
            else if(memcmp((char*)&textmessage[8], "#setting=", 9) == 0)
            {
                if(memcmp((char*)&textmessage[17], "logonad?", 8) == 0)
                {
                    json_parse = cJSON_Parse(&textmessage[25]);
                    if(json_parse == NULL)
                    {
                        ESP_LOGI(TAG, "Web server script error.");
                    }
                    else
                    {
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "username");
                        if(valuejSON != NULL)
                        {
                            strcpy(username_admin, valuejSON->valuestring);
                            settings_file_json("/data/username_admin.json", "username", username_admin, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "userpsw");
                        if(valuejSON != NULL)
                        {
                            strcpy(userpsw_admin, valuejSON->valuestring);
                            settings_file_json("/data/userpsw_admin.json", "userpsw", userpsw_admin, WRITE_SETTING);
                        }
                        cJSON_Delete(json_parse);
                    }
                    strcat(out_str, "confirm_set_logonad");
                    ClientQueTextMessageOut(xclient, out_str);
                }
                else if(memcmp((char*)&textmessage[17], "logonsv?", 8) == 0)
                {
                    json_parse = cJSON_Parse(&textmessage[25]);
                    if(json_parse == NULL)
                    {
                        ESP_LOGI(TAG, "Web server script error.");
                    }
                    else
                    {
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "username");
                        if(valuejSON != NULL)
                        {
                            strcpy(username_svisor, valuejSON->valuestring);
                            settings_file_json("/data/username_svisor.json", "username", username_svisor, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "userpsw");
                        if(valuejSON != NULL)
                        {
                            strcpy(userpsw_svisor, valuejSON->valuestring);
                            settings_file_json("/data/userpsw_svisor.json", "userpsw", userpsw_svisor, WRITE_SETTING);
                        }
                        cJSON_Delete(json_parse);
                    }
                    strcat(out_str, "confirm_set_logonsv");
                    ClientQueTextMessageOut(xclient, out_str);
                }
                else if(memcmp((char*)&textmessage[17], "wifipass?", 9) == 0)
                {
                    json_parse = cJSON_Parse(&textmessage[26]);
                    if(json_parse == NULL)
                    {
                        ESP_LOGI(TAG, "Web server script error.");
                    }
                    else
                    {
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "wifiappsw");
                        if(valuejSON)
                        {
                            strcpy(wifiappsw, valuejSON->valuestring);
                            settings_file_json("/data/wifiappsw.json", "wifiappsw", wifiappsw, WRITE_SETTING);
                        }
                        cJSON_Delete(json_parse);
                    }
                    strcat(out_str, "confirm_set_wifiap");
                    ClientQueTextMessageOut(xclient, out_str);
                }
                else if(memcmp((char*)&textmessage[17], "ethernet?", 9) == 0)
                {
                    json_parse = cJSON_Parse(&textmessage[26]);
                    if(json_parse == NULL)
                    {
                        ESP_LOGI(TAG, "Web server script error.");
                    }
                    else
                    {
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "ethen");
                        if(valuejSON)
                        {
                            strcpy(ethen, valuejSON->valuestring);
                            settings_file_json("/data/ethen.json", "ethen", ethen, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "ethsgway");
                        if(valuejSON)
                        {
                            strcpy(ethsgway, valuejSON->valuestring);
                            settings_file_json("/data/ethsgway.json", "ethsgway", ethsgway, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "ethssub");
                        if(valuejSON)
                        {
                            strcpy(ethssub, valuejSON->valuestring);
                            settings_file_json("/data/ethssub.json", "ethssub", ethssub, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "ethsen");
                        if(valuejSON)
                        {
                            strcpy(ethsen, valuejSON->valuestring);
                            settings_file_json("/data/ethsen.json", "ethsen", ethsen, WRITE_SETTING);
                        }
                        valuejSON = cJSON_GetObjectItemCaseSensitive(json_parse, "ethsip");
                        if(valuejSON)
                        {
                            strcpy(ethsip, valuejSON->valuestring);
                            settings_file_json("/data/ethsip.json", "ethsip", ethsip, WRITE_SETTING);
                        }
                    }
                    strcat(out_str, "confirm_set_ethernet");
                    ClientQueTextMessageOut(xclient, out_str);
                }
                QueClientUISetting(xclient, OTHER_CLIENT);  
            }
            else if(memcmp((char*)&textmessage[8], "#rdmeter?", 9) == 0){SetSensorSend(xclient, THIS_CLIENT);} 
            else if(memcmp((char*)&textmessage[8], "#rdmeterz", 9) == 0){xclient->rdmeter_confirm_get = 1;} 
            else if(memcmp((char*)&textmessage[8], "#setting?", 9) == 0){QueClientUISetting(xclient, THIS_CLIENT);}
            else if(memcmp((char*)&textmessage[8], "#settingz", 9) == 0){xclient->setting_confirm_get = 1;}
            else if(memcmp((char*)&textmessage[8], "#wrmeter?", 9) == 0){QueClientUIWrMeter(xclient, THIS_CLIENT); }
            else if(memcmp((char*)&textmessage[8], "#wrmeterz", 9) == 0){xclient->wrmeter_confirm_get = 1;}
            else if(memcmp((char*)&textmessage[8], "#infor?", 7) == 0){QueClientUIInfor(xclient, THIS_CLIENT);}
            else if(memcmp((char*)&textmessage[8], "#inforz", 7) == 0){xclient->infor_confirm_get = 1;}
            else if(memcmp((char*)&textmessage[8], "#informz", 8) == 0){xclient->inform_confirm_get = 1;}
        }
        else
        {
            strcat(out_str, "load=logon");
            ClientQueTextMessageOut(xclient, out_str);
        }
        xclient->textmessage_in_cntid++;
    } 
}

void WsClientsProcessData(void)
{
    static uint8_t process_idx = 0;
    if(fpm_wsockets[process_idx].fd != 0)
    {
        if(fpm_wsockets[process_idx].textmessage_in_idx_write != fpm_wsockets[process_idx].textmessage_in_idx_read)
        {
            ProcessWsDataFD(&fpm_wsockets[process_idx], &fpm_wsockets[process_idx].textmessage_in[fpm_wsockets[process_idx].textmessage_in_idx_read][0]);
            fpm_wsockets[process_idx].textmessage_in_idx_read++;
            if(fpm_wsockets[process_idx].textmessage_in_idx_read >= WS_CLIENT_TXTMSG_BFFR_CNT)
            {
                fpm_wsockets[process_idx].textmessage_in_idx_read = 0;
            }
            fpm_wsockets[process_idx].textmessage_in_time_stamp = xTaskGetTickCount();
            fpm_wsockets[process_idx].expecting_response = false;
        }
    }
    process_idx++;
    if(process_idx >= MAX_WS_CLIENTS)
    {
        process_idx = 0;
    }

}

bool AsyncClientProcess(void)
{
    return async_now;
}

bool AllClientsExpectingResponse(void)
{
    static uint8_t i;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        if(fpm_wsockets[i].fd != 0)
        {
            if(fpm_wsockets[i].expecting_response == true)
            {
                return true;
            }
        }
    }
    return false;
}

bool clientSendWs(fpm_wsockets_t* xclient, char *txtmsg)
{
    static char cntid_str[20];
    static httpd_ws_frame_t frame;
    static char short_str_buffer[100];
    strcat(txtmsg, "*");
    sprintf(cntid_str, "%lu", xclient->textmessage_out_cntid);
    strcat(txtmsg, cntid_str);
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t*)txtmsg;
    frame.len = strlen(txtmsg);
    xclient->textmessage_out_time_stamp = xTaskGetTickCount();
    if(httpd_ws_send_data(server, xclient->fd, &frame) == ESP_OK)
    {
        xclient->expecting_response = true;
        memset(short_str_buffer, 0, sizeof(short_str_buffer));
        memcpy(short_str_buffer, txtmsg, 30);
        strcat(short_str_buffer, "...");
        strcat(short_str_buffer, cntid_str);
        //ESP_LOGI(TAG, "Client %d ui <- %s", fpm_wsockets[fpm_wsockets_idx].fd, _send_ptr);
        ESP_LOGI(TAG, "Client %d ui <- %s", xclient->fd, short_str_buffer);
        xclient->textmessage_out_cntid++;
        return 1;
    }
    return 0;
}

char *build_persistent_str(void)
{
    static char datatime_string[20];
    static char persistent_str[200];
    static uint8_t sensor_seconds_count;
    cJSON *valuejSON;
    static time_t now;
    static struct tm timeinfo = { 0 };

    time(&now);
    localtime_r(&now, &timeinfo);
    cJSON* persistent_json_obj = cJSON_CreateObject();
    sprintf(datatime_string, "%02i", timeinfo.tm_hour);
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "hour", valuejSON);
    sprintf(datatime_string, "%02i", timeinfo.tm_min);
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "minute", valuejSON);
    sprintf(datatime_string, "%02i", timeinfo.tm_sec);
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "second", valuejSON);
    sprintf(datatime_string, "%02i", timeinfo.tm_mday);
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "day", valuejSON);
    switch(timeinfo.tm_wday)
    {
        case 0: strcpy(datatime_string, "Sunday"); break;
        case 1: strcpy(datatime_string, "Monday"); break;
        case 2: strcpy(datatime_string, "Tuesday"); break;
        case 3: strcpy(datatime_string, "Wednesday"); break;
        case 4: strcpy(datatime_string, "Thursday"); break;
        case 5: strcpy(datatime_string, "Friday"); break;
        case 6: strcpy(datatime_string, "Saturday"); break;
        default: strcpy(datatime_string, " "); break;
    }
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "weekday", valuejSON);
    switch(timeinfo.tm_mon)
    {
        case 0: strcpy(datatime_string, "January"); break;
        case 1: strcpy(datatime_string, "February"); break;
        case 2: strcpy(datatime_string, "March"); break;
        case 3: strcpy(datatime_string, "April"); break;
        case 4: strcpy(datatime_string, "May"); break;
        case 5: strcpy(datatime_string, "June"); break;
        case 6: strcpy(datatime_string, "July"); break;
        case 7: strcpy(datatime_string, "August"); break;
        case 8: strcpy(datatime_string, "Semptember"); break;
        case 9: strcpy(datatime_string, "October"); break;
        case 10: strcpy(datatime_string, "November"); break;
        case 11: strcpy(datatime_string, "December"); break;
        default: strcpy(datatime_string, " "); break;    
    }
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "month", valuejSON);
    sprintf(datatime_string, "%i", (timeinfo.tm_year + 1900));
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "year", valuejSON);
    sensor_seconds_count = sensor_elapsed/1000;
    sprintf(datatime_string, "%d", sensor_seconds_count);
    valuejSON = cJSON_CreateString(datatime_string);
    cJSON_AddItemToObject(persistent_json_obj, "sensorelapsed", valuejSON);
    char *json_print = cJSON_Print(persistent_json_obj);
    sprintf(persistent_str, "&console#persistent=%s", json_print);
    cJSON_free(json_print);
    cJSON_Delete(persistent_json_obj);

    return persistent_str;
    
}
void WsClientsSend_AppendCntID(void)
{
    static httpd_ws_frame_t frame;
    if(xTaskGetTickCount() - send_ui_textmessages_timestamp >= target_send_ui_text_message_delay)
    {
        if(replace_fd)
        {
            frame.type = HTTPD_WS_TYPE_TEXT;
            frame.payload = (uint8_t*)"&console#close";
            frame.len = strlen((char*)frame.payload);
            if(httpd_ws_send_data(server, replace_fd, &frame) == ESP_OK)
            {
                ESP_LOGI(TAG, "Client %d ui <- %s", replace_fd, frame.payload);
                replace_fd = 0;
            }
            target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;       
            send_ui_textmessages_timestamp = xTaskGetTickCount();
        }
        else if((fpm_wsockets[fpm_wsockets_idx].fd != 0) && (fpm_wsockets[fpm_wsockets_idx].expecting_response == false))
        {
            if((xTaskGetTickCount() - fpm_wsockets[fpm_wsockets_idx].time_persistent_timestamp >= ONESECOND_TIME_PERSISTENT_PERIOD) && (fpm_wsockets[fpm_wsockets_idx].ws_startup_init_done == 1))
            {
                if(clientSendWs(&fpm_wsockets[fpm_wsockets_idx], build_persistent_str()) == 1)
                {
                    fpm_wsockets[fpm_wsockets_idx].time_persistent_timestamp = xTaskGetTickCount();
                }
                target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;
                send_ui_textmessages_timestamp = xTaskGetTickCount();
            }
            else if(fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_read != fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_write)
            {
                if(clientSendWs(&fpm_wsockets[fpm_wsockets_idx], fpm_wsockets[fpm_wsockets_idx].textmessage_out[fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_read]) == 1)
                {
                    fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_read++;
                    if(fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_read >= WS_CLIENT_TXTMSG_BFFR_CNT)
                    {
                        fpm_wsockets[fpm_wsockets_idx].textmessage_out_idx_read = 0;
                    }
                }
                target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;
                send_ui_textmessages_timestamp = xTaskGetTickCount();
            }
            else if((fpm_wsockets[fpm_wsockets_idx].send_meter_infoconfig == true) || (fpm_wsockets[fpm_wsockets_idx].send_meter_electrical == true))
            {   
                if(fpm_wsockets[fpm_wsockets_idx].send_meter_electrical == true)
                {
                    metermsg_electrical[metermsg_electrical_len] = 0;
                    if(clientSendWs(&fpm_wsockets[fpm_wsockets_idx], metermsg_electrical) == 1)
                    {
                        fpm_wsockets[fpm_wsockets_idx].send_meter_electrical = false;
                    }
                }
                else if(fpm_wsockets[fpm_wsockets_idx].send_meter_infoconfig == true)
                {
                    metermsg_infoconfig[metermsg_infoconfig_len] = 0;
                    if(clientSendWs(&fpm_wsockets[fpm_wsockets_idx], metermsg_infoconfig) == 1)
                    {
                        fpm_wsockets[fpm_wsockets_idx].send_meter_infoconfig = false;
                    }
                }
                target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;
                send_ui_textmessages_timestamp = xTaskGetTickCount();
            }     
        }
        fpm_wsockets_idx++;
        if(fpm_wsockets_idx >= MAX_WS_CLIENTS)
        {
            fpm_wsockets_idx = 0;
        }       
    }
}

void ethernet(void)
{
    if(ethernet_link_down == 1)
    {
        if(xTaskGetTickCount() - ethernet_link_down_timestamp > 10000)
        {
            strcpy(ethmac, " ");
            strcpy(ethip, " ");
            strcpy(ethsub, " ");
            strcpy(ethgway, " ");
            esp_restart();
        }
    }
}

void init_fpm_swsockets(void)
{
    static uint8_t i;
    static uint8_t j;
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        clear_new_ws_client(&fpm_wsockets[i]);
    }
    for(i = 0; i < MAX_WS_CLIENTS; i++)
    {
        fpm_sockets[i].fd = 0;
        for(j = 0; j < APP_ASSOC_SOCKET_ALLOCATION; j++)
        {
            fpm_sockets[i].assoc_fd[j] = 0;
        }
    }
    for(i = 0; i < WS_URI_ALLOCATION; i++)
    {
        strcpy(key_str[i], "0");
        sprintf(page_access_str[i], "%d", SUPERVISOR_ACCESS);
        sprintf(dynamic_ws_uri[i], "/ws%d", i);
    }
    for(i = 0; i < WS_URI_ALLOCATION; i++)
    {
        static char build_filename[40];

        sprintf(build_filename, "/data/key%d.json", i);
        settings_file_json(build_filename, "key", key_str[i], READ_SETTING);
        key_uint32[i] = atol(key_str[i]);
        sprintf(build_filename, "/data/access%d.json", i);
        settings_file_json(build_filename, "access", page_access_str[i], READ_SETTING);
        page_access_number[i] = atol(page_access_str[i]);
        sprintf(build_filename, "/data/uri%d.json", i);
        settings_file_json(build_filename, "uri", dynamic_ws_uri[i], READ_SETTING);
    }

    settings_file_json("/data/uriidx", "idx", uri_idx_str, READ_SETTING);
    uri_idx_int = (uint8_t)atol(uri_idx_str);

    settings_file_json("/data/ethen.json", "ethen", ethen, READ_SETTING);
    settings_file_json("/data/wifiappsw.json", "wifiappsw", wifiappsw, READ_SETTING);
    if(strcmp(ethen, "No") == 0)
    {
        strcpy(ethsen, "No");
        settings_file_json("/data/ethsen.json", "ethsen", ethsen, WRITE_SETTING);
    }
    else
    {
        settings_file_json("/data/ethsen.json", "ethsen", ethsen, READ_SETTING);
    }
    settings_file_json("/data/ethsgway.json", "ethsgway", ethsgway, READ_SETTING);
    settings_file_json("/data/ethssub.json", "ethssub", ethssub, READ_SETTING);
    settings_file_json("/data/ethsip.json", "ethsip", ethsip, READ_SETTING);
    settings_file_json("/data/username_admin.json", "username", username_admin, READ_SETTING);
    settings_file_json("/data/userpsw_admin.json", "userpsw", userpsw_admin, READ_SETTING);
    settings_file_json("/data/username_svisor.json", "username", username_svisor, READ_SETTING);
    settings_file_json("/data/userpsw_svisor.json", "userpsw", userpsw_svisor, READ_SETTING);
    target_send_ui_text_message_delay = SLOW_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    sensor_timestamp = xTaskGetTickCount();
    ethernet_link_down_timestamp = xTaskGetTickCount();
}

static esp_err_t _get_handler_favicon(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    ret = get_handler(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _get_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    register_new_socket(req);
    ret = get_handler(req);
    remove_socket(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _ws_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    ret = ws_handler(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _login_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    register_new_socket(req);
    ret = login_handler(req);
    remove_socket(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _custommsg_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    register_new_socket(req);
    ret = custommsg_handler(req);
    remove_socket(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _bootupdate_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    register_new_socket(req);
    ret = bootupdate_handler(req);
    remove_socket(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

static esp_err_t _dataupdate_handler(httpd_req_t *req)
{
    static esp_err_t ret;
    async_now = ASYNC_BUSY;
    register_new_socket(req);
    ret = dataupdate_handler(req);
    remove_socket(req);
    async_now = ASYNC_IDLE;
    target_send_ui_text_message_delay = FAST_SEND_UI_TEXT_MESSAGE_DELAY;
    send_ui_textmessages_timestamp = xTaskGetTickCount();
    return ret;
}

esp_err_t WebServerStart(void)
{
    static uint8_t i;
    static char* base_path = "/data";
    if(server_data)
    {
        ESP_LOGI(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data)
    {
        ESP_LOGI(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path, sizeof(server_data->base_path));
    httpd_config_t config = HTTPD_DEFAULT_CONFIG_FPM();
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }
    static httpd_uri_t file_html;
    file_html.uri       = uri_html;
    file_html.method    = HTTP_GET;
    file_html.handler   = _get_handler;
    file_html.user_ctx  = server_data;
    httpd_register_uri_handler(server, &file_html);
    
    static httpd_uri_t file_logo;
    file_logo.uri       = uri_logo;
    file_logo.method    = HTTP_GET;
    file_logo.handler   = _get_handler;
    file_logo.user_ctx  = server_data;
    httpd_register_uri_handler(server, &file_logo);

    static httpd_uri_t favicon;
    favicon.uri       = uri_favicon;
    favicon.method    = HTTP_GET;
    favicon.handler   = _get_handler_favicon;
    favicon.user_ctx  = server_data;
    httpd_register_uri_handler(server, &favicon);

    static httpd_uri_t login;  
    login.uri        = uri_login;
    login.method     = HTTP_POST;
    login.handler    = _login_handler;
    login.user_ctx   = server_data;
    httpd_register_uri_handler(server, &login);
    
    static httpd_uri_t custommsg;  
    custommsg.uri        = uri_custommsg;
    custommsg.method     = HTTP_POST;
    custommsg.handler    = _custommsg_handler;
    custommsg.user_ctx   = server_data;
    httpd_register_uri_handler(server, &custommsg);
    
    static httpd_uri_t bootupdate;
    bootupdate.uri       = uri_bootupdate;
    bootupdate.method    = HTTP_POST;
    bootupdate.handler   = _bootupdate_handler;
    bootupdate.user_ctx  = server_data;
    httpd_register_uri_handler(server, &bootupdate);
    
    static httpd_uri_t dataupdate;
    dataupdate.uri       = uri_dataupdate;
    dataupdate.method    = HTTP_POST;
    dataupdate.handler   = _dataupdate_handler;
    dataupdate.user_ctx  = server_data;
    httpd_register_uri_handler(server, &dataupdate);

    for(i = 0; i < WS_URI_ALLOCATION; i++)
    {
        ws[i].uri        = dynamic_ws_uri[i];
        ws[i].method     = HTTP_GET;
        ws[i].handler    = _ws_handler;
        ws[i].user_ctx   = NULL;
        ws[i].is_websocket = true;
        ws[i].handle_ws_control_frames = true;
        httpd_register_uri_handler(server, &ws[i]);
    }
    return ESP_OK;
}