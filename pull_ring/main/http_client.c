#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#define MY_TAG	"moding"

extern int time_count;
extern FILE *writefile;
extern unsigned char picture_num; //图片数量

static const char *TAG = "http_client";

#define MAX_HTTP_RECV_BUFFER 4096

//update van
unsigned char epd_data[4096]; //该数组最大可以设置为55928
extern int url_length;

extern void DK_ByO(void);
extern void DK_ByT(void);
extern void DK_ROflesh(void);
extern void DK_RTflesh(void);
extern void Write_CO(uint8_t value);
extern void Write_CT(uint8_t value);
extern void Hal_UpGraghScreen3();
extern void Hal_UpGraghScreen4();
extern void Hal_UpGraghScreen1(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);
extern void Hal_UpGraghScreen2(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);
/**************************************************/

/**********************************************/
//update van pull_ring
#include "display_pic.h"
unsigned char pull_ring[4096];
extern void Acep_loadPIC1_init();
extern void Acep_loadPIC2_init();
extern void Acep_loadPIC1_end();
extern void Acep_loadPIC2_end();
extern void Acep_loadPIC1_test(unsigned char* pic_data3,int max_data);
extern void Acep_loadPIC2_test(unsigned char* pic_data4,int max_data);
extern void moding_sleep();
/**********************************************/

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void http_test_task(char *dpwn_url, char *filename)
{
	//note: don`t change char
	int data_read_size = 0, bytes_read = 0;

    char *temp_buffer = malloc(100);
    if (temp_buffer == NULL)
    {
        ESP_LOGE(TAG, "Cannot malloc temp_buffer");
        return;
    }
    char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Cannot malloc http receive buffer");
        return;
    }

//    memset(temp_buffer, 0, 100);
//    sprintf(temp_buffer, "%s%s", dpwn_url, filename);
//    ESP_LOGE(TAG, "url=%s", temp_buffer);
//    esp_http_client_config_t config = {
//        .url = temp_buffer, //temp_buffer
//        .event_handler = _http_event_handler,
//    };
    //update van
    ESP_LOGW(MY_TAG, "url=%s", dpwn_url);
    esp_http_client_config_t config = {
    	.url = dpwn_url, //temp_buffer
        .event_handler = _http_event_handler,
    };
	/************************/
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err;
    if ((err = esp_http_client_open(client, 0)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        free(buffer);
        return;
    }


    int content_length = esp_http_client_fetch_headers(client);
    int total_read_len = 0, read_len;
    data_read_size = esp_http_client_get_content_length(client);
    ESP_LOGW(MY_TAG, "data_read_size:%d",data_read_size);
    ESP_LOGW(MY_TAG, "content_length:%d",content_length);
    ESP_LOGE(TAG, "start_earse flash size");
    sf_WriteBuffer(&data_read_size, info_page * 4096 + picture_num * 50 + 41, sizeof(int));       //存储文件大小
    spi_flash_erase_range((picture_page + picture_cap * picture_num) * 4096, picture_cap * 4096); //清除flash内存
    ESP_LOGE(TAG, "earse flash end");
    ESP_LOGE(TAG, "filepath = %s", temp_buffer);
    ESP_LOGE(TAG, "start_download");
    if (total_read_len < content_length && content_length <= MAX_HTTP_RECV_BUFFER)
    {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0)
        {
            ESP_LOGE(TAG, "Error read data");
        }
        buffer[read_len] = 0;
        ESP_LOGD(TAG, "read_len = %d", read_len);
    }
    else
    {
        while (data_read_size > 0)
        {
            // ESP_LOGE(TAG, "read %d", bytes_read);
//            time_count = 0;
//            read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
//            spi_flash_write((picture_page + picture_cap * picture_num) * 4096 + bytes_read * MAX_HTTP_RECV_BUFFER, buffer, read_len);
//            bytes_read++;
//            data_read_size -= read_len;

        	/****************************************************/
        	//update van pull_ring
            time_count = 0;
            if(data_read_size<MAX_HTTP_RECV_BUFFER)
            {
            	read_len = esp_http_client_read(client, buffer, data_read_size);
            }
            else
            {
            	read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
            }
            spi_flash_write((picture_page + picture_cap * picture_num) * 4096 + bytes_read * MAX_HTTP_RECV_BUFFER, buffer, read_len);
			bytes_read++;
			data_read_size -= read_len;
			/***************************************************/
        }
    }
    ESP_LOGW(MY_TAG, "filename:%s",filename);
    ESP_LOGE(TAG, "write end");
    ESP_LOGI(TAG, "read_times = %d", bytes_read);
    ESP_LOGI(TAG, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(buffer);
    free(temp_buffer);
    ESP_LOGE(TAG, "download_finished");
    /********************************************/
    //update van pull_ring
    ESP_LOGW(MY_TAG, "start_display_picture");
#ifdef on_off
    int ret = search_in_flash(filename,0x02,134400);
    ESP_LOGW(MY_TAG, "search_in_flash(%s,0x02,%d)=%d",filename,content_length,ret);
    ESP_LOGW(MY_TAG, "display_picture_done");
#else

    int acount=0;
    memset(pull_ring,0,MAX_HTTP_RECV_BUFFER);
    char *get_dpwn_url_fifth_from_the_bottom;
	get_dpwn_url_fifth_from_the_bottom=dpwn_url+url_length-5;
	if(*get_dpwn_url_fifth_from_the_bottom=='1')
    {
		Acep_loadPIC1_init();
		for(;bytes_read>1;bytes_read--)
		{
			spi_flash_read((picture_page + picture_cap * picture_num) * 4096 + acount * MAX_HTTP_RECV_BUFFER,pull_ring,4096);
			Acep_loadPIC1_test(pull_ring,4096);
			acount++;
		}
		spi_flash_read((picture_page + picture_cap * picture_num) * 4096 + acount * MAX_HTTP_RECV_BUFFER,pull_ring,3328);
		Acep_loadPIC1_test(pull_ring,3328);
		Acep_loadPIC1_end();
		ESP_LOGW(MY_TATG,"picture 1 end");
    }
	else if(*get_dpwn_url_fifth_from_the_bottom=='2')
    {
		Acep_loadPIC2_init();
		for(;bytes_read>1;bytes_read--)
		{
			spi_flash_read((picture_page + picture_cap * picture_num) * 4096 + acount * MAX_HTTP_RECV_BUFFER,pull_ring,4096);
			Acep_loadPIC2_test(pull_ring,4096);
			acount++;
		}
		spi_flash_read((picture_page + picture_cap * picture_num) * 4096 + acount * MAX_HTTP_RECV_BUFFER,pull_ring,3328);
		Acep_loadPIC2_test(pull_ring,3328);
		Acep_loadPIC2_end();
		ESP_LOGW(MY_TGAG,"picture 2 end");
		esp_timer_start_periodic(periodic_timer, 1*1000 * 1000); //1s定时器
    }
    else
    {
    	ESP_LOGW(MY_TAG,"not 1.bin or 2.bin");
    }
#endif

    /********************************************/
}
