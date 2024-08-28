#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "errno.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/queue.h"
#include "esp_app_format.h"
#include "errno.h"
#include "spi_flash_mmap.h"
#include "total_app.h"
#include "esp_flash_encrypt.h"
#include "sys/param.h"
#include "esp_image_format.h"

typedef struct ota_ops_entry_ {
    uint32_t handle;
    const esp_partition_t *part;
    bool need_erase;
    uint32_t wrote_size;
    uint8_t partial_bytes;
    WORD_ALIGNED_ATTR uint8_t partial_data[16];
    LIST_ENTRY(ota_ops_entry_) entries;
} ota_ops_entry_t;

int binary_file_length = 0;
esp_err_t err;
esp_ota_handle_t update_flash_handle = 0 ;
esp_ota_handle_t update_spiffs_handle = 0 ;
const esp_partition_t *update_partition = NULL;
const esp_partition_t *configured;
const esp_partition_t *running;
bool image_header_was_checked;

static const char *TAG = "ota";
ota_ops_entry_t *spiffs_partition_ota;

const esp_partition_t *esp_ota_get_spiffs_partition(void)
{
    return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
}

void ota_boot_init(void)
{
    update_flash_handle = 0;
    update_partition = NULL;
    image_header_was_checked = false;
    configured = esp_ota_get_boot_partition();
    running = esp_ota_get_running_partition();
    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32, configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08"PRIx32")", running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32, update_partition->subtype, update_partition->address);
}

void ota_spiffs_init(void)
{
    update_spiffs_handle = 1;
    update_partition = NULL;
    image_header_was_checked = false;
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
}

ota_return_t write_ota_boot(int data_read, char *ota_write_data)
{
    if (data_read < 0)
    {
        ESP_LOGI(TAG, "Error: SSL data read error");
        return OTA_RCV_ERR;
    } 
    else if (data_read > 0)
    {
        if (image_header_was_checked == false)
        {
            esp_app_desc_t new_app_info;
            if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);
                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                }
                const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                esp_app_desc_t invalid_app_info;
                if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                }
                if (last_invalid_app != NULL)
                {
                    if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        ESP_LOGW(TAG, "New version is the same as invalid version.");
                        ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                        ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                        esp_ota_abort(update_flash_handle);
                        return OTA_SAME_VERSION;
                    }
                }
                image_header_was_checked = true;
                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_flash_handle);
                if (err != ESP_OK)
                {
                    ESP_LOGI(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    esp_ota_abort(update_flash_handle);
                    return OTA_BEGIN_FAILED;
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");
            }
            else
            {
                ESP_LOGI(TAG, "received package is not fit len");
                esp_ota_abort(update_flash_handle);
                return OTA_INVALID_LENGTH;
            }
        }
        err = esp_ota_write( update_flash_handle, (const void *)ota_write_data, data_read);
        if (err != ESP_OK)
        {
            esp_ota_abort(update_flash_handle);
            return OTA_WRITE_ERROR;
        }
        binary_file_length += data_read;
        ESP_LOGD(TAG, "Written image length %d", binary_file_length);       
    }
    else if(data_read == 0)
    {
        if (errno == ECONNRESET || errno == ENOTCONN)
        {
            ESP_LOGI(TAG, "Connection closed, errno = %d", errno);
            esp_ota_abort(update_flash_handle);
            return OTA_TRANSPORT_ERR;
        }
    }
    return OTA_WRITE_OK;
}

esp_err_t esp_ota_write_spiffs(esp_ota_handle_t handle, const void *data, size_t size, ota_ops_entry_t *it)
{
    const uint8_t *data_bytes = (const uint8_t *)data;
    esp_err_t ret;
    if (data == NULL) {
        ESP_LOGE(TAG, "write data is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    if (size == 0) {
        ESP_LOGD(TAG, "write data size is 0");
        return ESP_OK;
    }
    if (it->handle == handle)
    {
        if (it->need_erase)
        {
            uint32_t first_sector = it->wrote_size / SPI_FLASH_SEC_SIZE;
            uint32_t last_sector = (it->wrote_size + size - 1) / SPI_FLASH_SEC_SIZE;

            ret = ESP_OK;
            if ((it->wrote_size % SPI_FLASH_SEC_SIZE) == 0) {
                ret = esp_partition_erase_range(it->part, it->wrote_size, ((last_sector - first_sector) + 1) * SPI_FLASH_SEC_SIZE);
            } else if (first_sector != last_sector) {
                ret = esp_partition_erase_range(it->part, (first_sector + 1) * SPI_FLASH_SEC_SIZE, (last_sector - first_sector) * SPI_FLASH_SEC_SIZE);
            }
            if (ret != ESP_OK) {
                return ret;
            }
        }
        if (esp_flash_encryption_enabled())
        {
            size_t copy_len;
            if (it->partial_bytes != 0)
            {
                copy_len = MIN(16 - it->partial_bytes, size);
                memcpy(it->partial_data + it->partial_bytes, data_bytes, copy_len);
                it->partial_bytes += copy_len;
                if (it->partial_bytes != 16)
                {
                    return ESP_OK;
                }
                ret = esp_partition_write(it->part, it->wrote_size, it->partial_data, 16);
                if (ret != ESP_OK)
                {
                    return ret;
                }
                it->partial_bytes = 0;
                memset(it->partial_data, 0xFF, 16);
                it->wrote_size += 16;
                data_bytes += copy_len;
                size -= copy_len;
            }

            it->partial_bytes = size % 16;
            if (it->partial_bytes != 0)
            {
                size -= it->partial_bytes;
                memcpy(it->partial_data, data_bytes + size, it->partial_bytes);
            }
        }
        ret = esp_partition_write(it->part, it->wrote_size, data_bytes, size);
        if(ret == ESP_OK)
        {
            it->wrote_size += size;
        }
        return ret;
    }
    ESP_LOGE(TAG,"not found the handle");
    return ESP_ERR_INVALID_ARG;
}

ota_return_t write_ota_spiffs(int data_read, char *ota_write_data)
{
    if (data_read < 0)
    {
        ESP_LOGI(TAG, "Error: SSL data read error");
        return OTA_RCV_ERR;
    } 
    else if (data_read > 0)
    {
        if (image_header_was_checked == false)
        {
            if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                image_header_was_checked = true;
                err = esp_ota_begin_spiffs(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_spiffs_handle);
                if (err != ESP_OK)
                {
                    ESP_LOGI(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    esp_ota_abort(update_spiffs_handle);
                    return OTA_BEGIN_FAILED;
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");
            }
            else
            {
                ESP_LOGI(TAG, "received package is not fit len");
                esp_ota_abort(update_spiffs_handle);
                return OTA_INVALID_LENGTH;
            }
        }
        err = esp_ota_write_spiffs( update_spiffs_handle, (const void *)ota_write_data, data_read, spiffs_partition_ota);
        if (err != ESP_OK)
        {
            esp_ota_abort(update_spiffs_handle);
            return OTA_WRITE_ERROR;
        }
        binary_file_length += data_read;
        ESP_LOGD(TAG, "Written image length %d", binary_file_length);       
    }
    else if(data_read == 0)
    {
        if (errno == ECONNRESET || errno == ENOTCONN) {
            ESP_LOGI(TAG, "Connection closed, errno = %d", errno);
            esp_ota_abort(update_spiffs_handle);
            return OTA_TRANSPORT_ERR;
        }
    }
    return OTA_WRITE_OK;
}

esp_err_t esp_ota_begin_spiffs(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle)
{
    esp_err_t ret = ESP_OK;
    if ((partition == NULL) || (out_handle == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    partition = esp_partition_verify(partition);
    if (partition == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (image_size != OTA_WITH_SEQUENTIAL_WRITES)
    {
        if ((image_size == 0) || (image_size == OTA_SIZE_UNKNOWN))
        {
            ret = esp_partition_erase_range(partition, 0, partition->size);
        }
        else
        {
            const int aligned_erase_size = (image_size + SPI_FLASH_SEC_SIZE - 1) & ~(SPI_FLASH_SEC_SIZE - 1);
            ret = esp_partition_erase_range(partition, 0, aligned_erase_size);
        }
        if (ret != ESP_OK)
        {
            return ret;
        }
    }
    spiffs_partition_ota = (ota_ops_entry_t *) calloc(sizeof(ota_ops_entry_t), 1);
    if (spiffs_partition_ota == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    spiffs_partition_ota->part = partition;
    spiffs_partition_ota->handle = *out_handle;
    spiffs_partition_ota->need_erase = (image_size == OTA_WITH_SEQUENTIAL_WRITES);
    *out_handle = spiffs_partition_ota->handle;
    return ESP_OK;
}

ota_return_t end_ota_boot(void)
{
    err = esp_ota_end(update_flash_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGI(TAG, "Image validation failed, image is corrupted");
            return OTA_VALIDATION_FAIL;
        }
        else
        {
            ESP_LOGI(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
            return OTA_END_FAIL;
        }
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return OTA_BOOT_FAIL;
    }
    return OTA_END_OK;
}

esp_err_t esp_ota_end_spiffs(ota_ops_entry_t *it)
{
    esp_err_t ret = ESP_OK;

    if (it == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    free(it);
    return ret;
}

ota_return_t end_ota_spiffs(void)
{
    if(spiffs_partition_ota)    
    {
        free(spiffs_partition_ota);
    }
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGI(TAG, "Image validation failed, image is corrupted");
            return OTA_VALIDATION_FAIL;
        }
        else
        {
            ESP_LOGI(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
            return OTA_END_FAIL;
        }
    }
    return OTA_END_OK;
}