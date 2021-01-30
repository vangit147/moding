/*
   服务端发送广播，根据特征值的读和通知属性发送0x55,0x02,0x55
   SPI_flash需要先擦除再写入 4096 页
   4000记录图片个数，4001-4096存储路径
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "gatts_server.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "http_client.h"
#include "esp_spi_flash.h"



/************************************/
//update van pull_ring
#include "display_pic.h"
extern esp_timer_handle_t periodic_timer_moding;
extern unsigned char flag[4096];
extern unsigned char pic_is_loop_display;
char url_state=0;
extern int time_from_server;
/************************************/

extern int READ_BIT;
extern int CLEAR_BIT;
extern int DOWNLOAD_BIT;
extern unsigned char isconnected; //wifi连接标志位
extern esp_vfs_spiffs_conf_t conf;
extern QueueHandle_t Message_Queue; //信息队列句柄
extern esp_timer_handle_t periodic_timer;
extern esp_timer_handle_t periodic_timer2;
extern EventGroupHandle_t EventGroupHandler; //事件标志组句柄

extern void ble_senddata(unsigned char *data);

FILE *writefile;
int write_id = 0;
int remian_total; //剩余电量
int time_count = 0;   //计时
//char receive_data[200];
unsigned char total = 0;
unsigned int rec_data = 0;
uint16_t data_conn_id = 0xffff;
esp_gatt_if_t data_gatts_if = 0xff;
//char download_url[100], file_name[32];
//unsigned char device_info[] = {0x55, 0xfa, 0x0A, 0x03, 0x00, 0x34, 0x01, 0x01, 0x11, 0x22, 0x0A, 0x01, 0xfa, 0x55};

#define GATTS_TAG "composite_DEMO"
#define MY_TAG	"moding"

//update van
extern int url_length;

char wifi_ssid_first[32];
char wifi_pssd_first[64];
char wifi_ssid_tmp[32]={0};
char wifi_pssd_tmp[64]={0};
char wifi_ssid_cache[32]={0};
char wifi_pssd_cache[64]={0};
char wifi_ssid_is_empty[32]={0};
char wifi_pssd_is_empty[64]={0};
char download_url_cache[200]={0};
char download_url[200]={0}, file_name[40]={0};
unsigned char device_info[26] = {0x00, 0x34, 0x01, 0x01, 0x11, 0x22, 0x0A, 0x01, 0x00,0x00,0x00,0x00,0x00,0x00};
//unsigned char device_info[26] = {0x55, 0xfa, 0x0A, 0x03, 0x00, 0x34, 0x01, 0x01, 0x11, 0x22, 0x0A, 0x01, 0xfa, 0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*********************/


//profile 处理事件
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 4

//定义设备名称
#define TEST_DEVICE_NAME "Mdlahu"
#define TEST_MANUFACTURER_DATA_LEN 17
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024

extern unsigned char receive_buffer[512]; //数据接收缓冲区
// 特征初始值  特征初始值必须是非空对象，特征长度必须始终大于零，否则堆栈将返回错误
static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
    {
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
        .attr_len = sizeof(char1_str),
        .attr_value = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,
    0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd};
static uint8_t raw_scan_rsp_data[] = {
    0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
    0x45, 0x4d, 0x4f};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

//adv 有效负载最多可以包含31个数据字节
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,    //将adv_data设置为扫描响应
    .include_name = true,     //adv_data 是否包含设备名称
    .include_txpower = false, //是否包含发送功率
    .min_interval = 0x0006,   //连接最小时间间隔 Time = min_interval * 1.25 msec
    .max_interval = 0x0010,   //连接最大事件间隔, Time = max_interval * 1.25 msec
    .appearance = 0x00,
	//    .manufacturer_len = 14,                                               //制造商数据长度
	//update van
	.manufacturer_len = 26,                                               //制造商数据长度
	/*********************************************/
    .p_manufacturer_data = device_info,                                   //制造商数据点
    .service_data_len = 0,                                                //服务数据长度
    .p_service_data = NULL,                                               //服务数据
    .service_uuid_len = sizeof(adv_service_uuid128),                      //服务UUID长度
    .p_service_uuid = adv_service_uuid128,                                //服务的UUID
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), //发现模式的标记
};
// 扫描恢复数据
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
//static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }
            else
            {
                if (param->write.offset > PREPARE_BUF_MAX_SIZE)
                {
                    status = ESP_GATT_INVALID_OFFSET;
                }
                else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
                {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }
            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        printf("come here\n");
        if (prepare_write_env->prepare_buf[0] == 0x55 && prepare_write_env->prepare_buf[1] == 0xfa && prepare_write_env->prepare_buf[prepare_write_env->prepare_len - 1] == 0x55 && prepare_write_env->prepare_buf[prepare_write_env->prepare_len - 2] == 0xfa)
        {
        	printf("come in \n");
            if (prepare_write_env->prepare_buf[3] == 0x01)
            {
            	printf("come in 0x01 \n");
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
				 esp_timer_stop(periodic_timer);
				 printf("wifi_connect......\r\n");

//				wifi_config_t wifi_config;
//				bzero(&wifi_config, sizeof(wifi_config_t));
//				memcpy(wifi_config.sta.ssid, prepare_write_env->prepare_buf + 5, prepare_write_env->prepare_buf[4]);
//				//memcpy(ssid, receive_buffer + 5, receive_buffer[4]);
//				printf("ssid=%s\r\n", wifi_config.sta.ssid); //17
//				int j = 4 + prepare_write_env->prepare_buf[4] + 1;
//				memcpy(wifi_config.sta.password, prepare_write_env->prepare_buf + 5 + prepare_write_env->prepare_buf[4] + 1, prepare_write_env->prepare_buf[j]);
//				//memcpy(pssd, receive_buffer + 5 + receive_buffer[4] + 1, receive_buffer[j]);
//				printf("pssd=%s\r\n", wifi_config.sta.password);
//				ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//				esp_wifi_disconnect();
//				esp_wifi_connect();

				//udpate van
				printf("/*******************prepare_buf****************************/\n");
				int j = 4 + prepare_write_env->prepare_buf[4] + 1;
				memset(wifi_ssid_first,0,sizeof(wifi_ssid_first));
				memset(wifi_pssd_first,0,sizeof(wifi_pssd_first));
				memcpy(wifi_ssid_first, prepare_write_env->prepare_buf + 5, prepare_write_env->prepare_buf[4]);
				printf("wifi_ssid_first=%s\r\n", wifi_ssid_first);
				memcpy(wifi_pssd_first, prepare_write_env->prepare_buf + 5 + prepare_write_env->prepare_buf[4] + 1, prepare_write_env->prepare_buf[j]);
			    printf("wifi_pssd_first=%s\r\n", wifi_pssd_first);
				if(strcmp(wifi_ssid_first,wifi_ssid_is_empty)&&strcmp(wifi_pssd_first,wifi_pssd_is_empty)&&(strcmp(wifi_ssid_tmp,wifi_ssid_first)||strcmp(wifi_pssd_tmp,wifi_pssd_first)))
				{
					strcpy(wifi_ssid_tmp, wifi_ssid_first);
					strcpy(wifi_pssd_tmp, wifi_pssd_first);
					printf("wifi_ssid_tmp=%s\r\n",wifi_ssid_tmp);
					printf("wifi_pssd_tmmp=%s\r\n",wifi_pssd_tmp);
					if(strcmp(wifi_ssid_cache,wifi_ssid_tmp)||strcmp(wifi_pssd_cache, wifi_pssd_tmp))
					{
					    wifi_config_t wifi_config;
						bzero(&wifi_config, sizeof(wifi_config_t));
						//需要再更改一下代码
						strcpy((char *)wifi_config.sta.ssid,wifi_ssid_tmp);
						strcpy((char *)wifi_config.sta.password,wifi_pssd_tmp);
//						memcpy(wifi_config.sta.ssid, prepare_write_env->prepare_buf + 5, prepare_write_env->prepare_buf[4]);
//						memcpy(wifi_config.sta.password, prepare_write_env->prepare_buf + 5 + prepare_write_env->prepare_buf[4] + 1, prepare_write_env->prepare_buf[j]);
						printf("ssid=%s\r\n", wifi_config.sta.ssid);
						printf("pssd=%s\r\n", wifi_config.sta.password);
						ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
						esp_wifi_disconnect();
						esp_wifi_connect();
						strcpy(wifi_ssid_cache, wifi_ssid_tmp);
						strcpy(wifi_pssd_cache, wifi_pssd_tmp);
						printf("wifi_ssid_cache=%s\r\n",wifi_ssid_cache);
					    printf("wifi_pssd_cache=%s\r\n",wifi_pssd_cache);
					}
				}
				/***********************************************************/

				ble_senddata(prepare_write_env->prepare_buf);
				ESP_LOGI(GATTS_TAG, "open timer");
	            esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
            }
            else if (prepare_write_env->prepare_buf[3] == 0x02)
            {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                esp_timer_stop(periodic_timer);
                ESP_LOGI(GATTS_TAG, "httpdownload");
//                char filenum = prepare_write_env->prepare_buf[4]; //下载文件个数
                int download_len = prepare_write_env->prepare_buf[5];
                int file_name_len = prepare_write_env->prepare_buf[6];
                ESP_LOGI(GATTS_TAG, "download_len= %d", download_len);
				ESP_LOGI(GATTS_TAG, "file_name_len= %d", file_name_len);
//              memset(download_url, 0, 100);
//				memcpy(download_url, prepare_write_env->prepare_buf + 6, download_len);
//				download_url[download_len] = 0;
//				memcpy(file_name, (char *)(prepare_write_env->prepare_buf + 6 + download_len + 1), prepare_write_env->prepare_buf[6 + download_len]);
//				if (isconnected == 1)
//				{
//					ESP_LOGI(GATTS_TAG, "setbits");
//					xEventGroupSetBits(EventGroupHandler, DOWNLOAD_BIT);
//				}
//				else
//				{
//					ESP_LOGI(GATTS_TAG, "wifi not connect");
//					xEventGroupClearBits(EventGroupHandler, DOWNLOAD_BIT);
//				}

				/*******************************************************/
				//update van
                memset(download_url_cache, 0, 200);
			    url_length=download_len;
			    memcpy(download_url_cache, prepare_write_env->prepare_buf + 6, download_len);
			    ESP_LOGI(GATTS_TAG, "download_url_cache= %s", download_url_cache);
			    ESP_LOGI(GATTS_TAG, "download_url= %s", download_url);
				if((strcmp(download_url,download_url_cache) && url_state<2) && download_len>0)
				{
					memset(download_url, 0, 200);
					memcpy(download_url, prepare_write_env->prepare_buf + 6, download_len);
					download_url[download_len] = 0;
					memset(file_name, 0, 40);
					memcpy(file_name, (char *)(prepare_write_env->prepare_buf + 6 + download_len + 1), prepare_write_env->prepare_buf[6 + download_len]);
					ESP_LOGI(GATTS_TAG, "file_name= %s", file_name);
					if (isconnected == 1)
					{
						ESP_LOGI(GATTS_TAG, "setbits");
						xEventGroupSetBits(EventGroupHandler, DOWNLOAD_BIT);
					}
					else
					{
						ESP_LOGI(GATTS_TAG, "wifi not connect");
						xEventGroupClearBits(EventGroupHandler, DOWNLOAD_BIT);
					}
				}
				else
				{
					getdeviceinfo();
					esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
				}
				/*******************************************************/
                //将收到的数据返回给微信小程序
                ble_senddata(prepare_write_env->prepare_buf);
            }
            else if (prepare_write_env->prepare_buf[3] == 0x07)
		   {//display picture what you want
#ifdef on_off
				esp_timer_stop(periodic_timer);
				char pic_name[40];
				memset(pic_name, 0, 40);
				int pic_name_length=prepare_write_env->prepare_buf[4];
				memcpy(pic_name, prepare_write_env->prepare_buf + 5, pic_name_length);
				ESP_LOGW(MY_TAG,"the picture what you want to display is %s",pic_name);
				ble_senddata(prepare_write_env->prepare_buf);
				int ret=search_in_flash(pic_name,0x07,134400);
				ESP_LOGW(MY_TAG, "search_in_flash(pic_name,0x07,134400)=%d",ret);
                ESP_LOGI(GATTS_TAG, "open timer");
               	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s

#endif
		   }
			else if(prepare_write_env->prepare_buf[3] == 0x08)
			{
#ifdef on_off
				esp_timer_stop(periodic_timer);
				//是否轮询显示图片
				pic_is_loop_display=prepare_write_env->prepare_buf[4];
				ESP_LOGW(MY_TAG,"pic_is_loop_display=%d\n",pic_is_loop_display);
				getdeviceinfo();
				sf_WriteBuffer(&pic_is_loop_display, info_page * 4096+ 81*50, sizeof(unsigned char));
				esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
				spi_flash_read(info_page * 4096,&flag,4096);
				if(flag[1]==0xaa&&flag[5]==0xaa)
				{
					flag[2]=0x11;
					flag[3]=prepare_write_env->prepare_buf[5];
					flag[4]=prepare_write_env->prepare_buf[6];
					spi_flash_erase_sector(info_page);
					spi_flash_write(info_page * 4096, &flag, 4096);
					spi_flash_read(info_page * 4096,&flag,4096);
					ESP_LOGW(MY_TAG,"flag 1~5 :");
					for(int i=1;i<=5;i++)
					{
						ESP_LOGW(MY_TAG,"flag[%d]=%x",i,flag[i]);
					}
				}
				loop_display_picture(prepare_write_env->prepare_buf[4]);
				if(prepare_write_env->prepare_buf[4] == 0x00)
				{
					ESP_LOGI(GATTS_TAG, "open timer");
					esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
				}
				else
				{
					ESP_LOGW(MY_TAG, "display_finished open timer 4s");
					esp_timer_start_periodic(periodic_timer, 4*1000 * 1000);
				}
#endif
		   }
            else if (prepare_write_env->prepare_buf[3] == 0x09)
            {//delete picture what you want
#ifdef on_off
				esp_timer_stop(periodic_timer);
				unsigned char temp;
				unsigned char picnum;
				unsigned char picture_num_real; //实际图片数量
				char temp_name[40];
				char pic_name[40];
				memset(pic_name, 0, 40);
				memset(temp_name, 0, 40);
				int pic_name_length=prepare_write_env->prepare_buf[4];
				ble_senddata(prepare_write_env->prepare_buf);
				memcpy(pic_name, prepare_write_env->prepare_buf + 5, pic_name_length);
				ESP_LOGW(MY_TAG,"pic_name=%s",pic_name);
				spi_flash_read(info_page * 4096, &picnum, sizeof(unsigned char));
				spi_flash_read(info_reboot_times * 4096, &picture_num_real, sizeof(unsigned char));
				ESP_LOGW(MY_TAG,"picture_num_real=%d",picture_num_real);
				if(picnum==0xff)
				{
					picnum=0;
				}
				ESP_LOGW(MY_TAG,"picnum=%d",picnum);
				unsigned char i=1;
				if(i>picnum&&picnum==0)
				{
					ESP_LOGW(MY_TAG,"flash no pic to let you delete");
				}
				else
				{
					for (; i <= picnum; i++)
					{
						spi_flash_read(info_page * 4096 + i * 50, temp_name, 40*sizeof(unsigned char));
						if(i>picture_num_real)
						{
							temp_name[39]='\0';
							ESP_LOGW(MY_TAG,"temp_name=%s",temp_file_name);
							if(strcmp(temp_name,temp_file_name)==0)
							{
								ESP_LOGW(MY_TAG,"no picture what you want to delete");
								break;
							}
						}
						if (strcmp(temp_name, pic_name) == 0) //有重名文件
						{
							ESP_LOGW(MY_TAG,"find picture what you want to delete: %s",temp_name);
							spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
							ESP_LOGW(MY_TAG,"first read temp=%d i=%d",temp,i);
							if(temp==0)
							{
								ESP_LOGE(MY_TAG,"picture which is %d in flash was deleted already ",i);
								break;
							}
							unsigned char temp=0;
							sf_WriteBuffer(&temp, info_page * 4096 + i * 50 + 40, sizeof(unsigned char));
							spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
							ESP_LOGW(MY_TAG,"after delete pic read temp=%d i=%d",temp,i);
							break;
						}
					}
				}
				getdeviceinfo();
				esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
				ESP_LOGW(MY_TAG,"CLEAR ONE PIC: esp_ble_gap_config_adv_data(&adv_data)");
				ESP_LOGI(MY_TAG, "open timer");
				esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
#endif
			}
            else if(prepare_write_env->prepare_buf[3]==0x10)
			{//get and set time from network
            	union longstrvalue{
					char time_sec[4];
					long i;
				}longstr;
				longstr.time_sec[0]=receive_buffer[4];
				longstr.time_sec[1]=receive_buffer[5];
				longstr.time_sec[2]=receive_buffer[6];
				longstr.time_sec[3]=receive_buffer[7];
				ESP_LOGW(MY_TAG,"time_sec[0]=%x,time_sec[1]=%x,time_sec[2]=%x,time_sec[3]=%x ",longstr.time_sec[0],longstr.time_sec[1],longstr.time_sec[2],longstr.time_sec[3]);
				ESP_LOGW(MY_TAG,"longstr.i=%ld",longstr.i);
				update_set_time(longstr.i);
				ESP_LOGI(GATTS_TAG, "open timer");
				esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
		   }
        }
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        //update van
//        char deviceName[30]="Moding_";
 //        uint8_t mac[6];
 //        char r[20];
 //        esp_read_mac(mac, ESP_MAC_BT);
 //        HexToStr(r,mac, 6);
 //        strcat(deviceName,r);
 //        printf("devieName=%s\r\n", deviceName);
 //        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(deviceName); //设置设备名称
        /****************************************/

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME); //设置设备名称
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //配置广播数据
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //配置扫描响应数据
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		getdeviceinfo();
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 14;
//        rsp.attr_value.value[0] = 0x55;
//        rsp.attr_value.value[1] = 0x02;
//        rsp.attr_value.value[2] = 0x55;
        for(uint8_t t=0;t<14;t++)
        {
             rsp.attr_value.value[t] = device_info[t];
        }
        //rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        write_id = param->write.handle;
        esp_log_buffer_hex("receive data", receive_buffer, strlen((char *)receive_buffer));
        printf("run  here \n");
        if (!param->write.is_prep)
        {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            //esp_log_buffer_hex("receive data.....", param->write.value, param->write.len);
            //将接收到的数据发送出去
            //rec_data = param->write.len;
            if (param->write.len == 2 && (param->write.value[0] == 0x01) && param->write.value[2] == 0x00)
            {
                printf("notidy data......\r\n");
            }
            else
            {
                if (param->write.value[param->write.len - 1] != 0x55)
                {
                    //数据没有接收完
                    strcat((char *)receive_buffer, (char *)param->write.value);
                }
                else
                {
                    //数据接收完
                	//2020/10/27
                    //update yuqiang
                    //strcat((char *)receive_buffer, (char *)param->write.value);
                    uint16_t len=param->write.len;
                    for(int l=0;l<len;l++)
                    {
                    	receive_buffer[l]=*(param->write.value+l);
                    }
                    //esp_log_buffer_hex("receive data", receive_buffer, strlen((char *)receive_buffer));
                    //int i = strlen((char *)receive_buffer);

					//update yuqiang
                    esp_log_buffer_hex("receive data", receive_buffer, len);
                    int i=len;
                    ESP_LOGI(GATTS_TAG, "receive_buffer len= %d", i);
                    /********************************************/

                    if (receive_buffer[0] == 0x55 && receive_buffer[1] == 0xfa && receive_buffer[i - 1] == 0x55 && receive_buffer[i - 2] == 0xfa)
                    {
                        printf("data is prefect\r\n");
                        //心跳数据
                        if (receive_buffer[3] == 0x04)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);

#endif
                            esp_timer_stop(periodic_timer);
                            ESP_LOGW(MY_TAG,"heart data ");
                            /********************************************/
                            //update van pull_ring
                            getdeviceinfo();
                            esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
                            /********************************************/
                            time_count = 0;                                         //收到心跳数据，计数清零
                            ble_senddata(receive_buffer);                           //心跳数据直接发送出去即可
							memset((char *)receive_buffer, 0, 512);
                            ESP_LOGI(GATTS_TAG, "open timer");
                           	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                        }
                        //获取设备信息
                        else if (receive_buffer[3] == 0x03)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                            esp_timer_stop(periodic_timer);
                            ESP_LOGI(GATTS_TAG, "return device info");
							// unsigned char total;
                            // spi_flash_read(info_page * 4096, &total, sizeof(unsigned char));
                            // if (total == 0xff)
                            //     total = 0;
                            // ESP_LOGI(GATTS_TAG, "total=%d", total);
                            // remian_total = 11534336 - total * picture_cap * 4096;
                            // device_info[6] = (unsigned char)((remian_total & 0xff000000) >> 8 >> 8 >> 8);
                            // device_info[7] = (unsigned char)((remian_total & 0xff0000) >> 8 >> 8);
                            // device_info[8] = (unsigned char)((remian_total & 0xff00) >> 8);
                            // device_info[9] = (unsigned char)(remian_total & 0xff);
                            // device_info[10] = total;
                            // device_info[11] = isconnected;
                            getdeviceinfo();
                            ble_senddata(device_info);
                            esp_log_buffer_hex("device_info", device_info, 14);
                            ESP_LOGI(GATTS_TAG, "open timer");
                            esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                        }
                        //下载数据
                        // else
                        else if (receive_buffer[3] == 0x05)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                            esp_timer_stop(periodic_timer);
                            ESP_LOGI(GATTS_TAG, "delete picture");
                            xEventGroupSetBits(EventGroupHandler, CLEAR_BIT);
                            ble_senddata(receive_buffer);
                            ESP_LOGI(GATTS_TAG, "open timer");
                            esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                        }
                        else if (receive_buffer[3] == 0x06)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                            esp_timer_stop(periodic_timer);
                            ESP_LOGI(GATTS_TAG, "read picture");
                            xEventGroupSetBits(EventGroupHandler, READ_BIT);
                            ble_senddata(receive_buffer);
                            ESP_LOGI(GATTS_TAG, "read data");
                        }
                        else if (receive_buffer[3] == 0x01)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
							esp_timer_stop(periodic_timer);
							ESP_LOGW(MY_TAG,"wifi_connect......");

//							wifi_config_t wifi_config;
//							bzero(&wifi_config, sizeof(wifi_config_t));
//							memcpy(wifi_config.sta.ssid, receive_buffer + 5, receive_buffer[4]);
//						   //memcpy(ssid, receive_buffer + 5, receive_buffer[4]);
//							printf("/*************wifi_config.sta.ssid/password*********************/\n");
//							printf("ssid=%s\r\n", wifi_config.sta.ssid); //17
//							int j = 4 + receive_buffer[4] + 1;
//							memcpy(wifi_config.sta.password, receive_buffer + 5 + receive_buffer[4] + 1, receive_buffer[j]);
//							//memcpy(pssd, receive_buffer + 5 + receive_buffer[4] + 1, receive_buffer[j]);
//							printf("pssd=%s\r\n", wifi_config.sta.password);
//							ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//							esp_wifi_disconnect();
//							esp_wifi_connect();

							//update van
							ESP_LOGW(MY_TAG,"/*******************receive_buf****************************/");
							int j = 4 + receive_buffer[4] + 1;
							memset(wifi_ssid_first,0,sizeof(wifi_ssid_first));
							memset(wifi_pssd_first,0,sizeof(wifi_pssd_first));
							memcpy(wifi_ssid_first, receive_buffer + 5, receive_buffer[4]);
							ESP_LOGW(MY_TAG,"wifi_ssid_first=%s", wifi_ssid_first);
							memcpy(wifi_pssd_first, receive_buffer + 5 + receive_buffer[4] + 1, receive_buffer[j]);
							ESP_LOGW(MY_TAG,"wifi_pssd_first=%s", wifi_pssd_first);
							if(strcmp(wifi_ssid_first,wifi_ssid_is_empty)&&strcmp(wifi_pssd_first,wifi_pssd_is_empty)&&(strcmp(wifi_ssid_tmp,wifi_ssid_first)||strcmp(wifi_pssd_tmp,wifi_pssd_first)))
							{
								strcpy(wifi_ssid_tmp, wifi_ssid_first);
								strcpy(wifi_pssd_tmp, wifi_pssd_first);
								ESP_LOGW(MY_TAG,"wifi_ssid_tmp=%s",wifi_ssid_tmp);
								ESP_LOGW(MY_TAG,"wifi_pssd_tmmp=%s",wifi_pssd_tmp);
								if(strcmp(wifi_ssid_cache,wifi_ssid_tmp)||strcmp(wifi_pssd_cache, wifi_pssd_tmp))
								{
									wifi_config_t wifi_config;
									bzero(&wifi_config, sizeof(wifi_config_t));
									//需要再更改一下代码
									strcpy((char *)wifi_config.sta.ssid,wifi_ssid_tmp);
									strcpy((char *)wifi_config.sta.password,wifi_pssd_tmp);
//									memcpy(wifi_config.sta.ssid, receive_buffer + 5, receive_buffer[4]);
//									memcpy(wifi_config.sta.password, receive_buffer + 5 + receive_buffer[4] + 1, receive_buffer[j]);
									ESP_LOGW(MY_TAG,"ssid=%s", wifi_config.sta.ssid);
									ESP_LOGW(MY_TAG,"pssd=%s", wifi_config.sta.password);
									ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
									esp_wifi_disconnect();
									esp_wifi_connect();
									strcpy(wifi_ssid_cache, wifi_ssid_tmp);
									strcpy(wifi_pssd_cache, wifi_pssd_tmp);
									ESP_LOGW(MY_TAG,"wifi_ssid_cache=%s",wifi_ssid_cache);
									ESP_LOGW(MY_TAG,"wifi_pssd_cache=%s",wifi_pssd_cache);
								}
							}
							/**********************************************/
							 ble_senddata(receive_buffer);
							 ESP_LOGI(GATTS_TAG, "open timer");
					         esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                        }
                        else if (receive_buffer[3] == 0x02)
                        {
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                            esp_timer_stop(periodic_timer);
                            ESP_LOGI(GATTS_TAG, "httpdownload");
//                            char filenum = receive_buffer[4]; //下载文件个数
                            int download_len = receive_buffer[5];
                            int file_name_len = receive_buffer[6];
                            ESP_LOGI(GATTS_TAG, "download_len= %d", download_len);
                            ESP_LOGW(MY_TAG, "file_name_len= %d", file_name_len);
							  /*
							memset(download_url, 0, 100);
                            memcpy(download_url, receive_buffer + 6, download_len);
                            download_url[download_len] = 0;
                            memcpy(file_name, (char *)(receive_buffer + 6 + download_len + 1), receive_buffer[6 + download_len]);

                            if (isconnected == 1)
                            {
                                ESP_LOGI(GATTS_TAG, "setbits");
                                xEventGroupSetBits(EventGroupHandler, DOWNLOAD_BIT);
                            }
                            else
                            {
                                ESP_LOGI(GATTS_TAG, "wifi not connect");
                                xEventGroupClearBits(EventGroupHandler, DOWNLOAD_BIT);
                            }
							*/

							//update van
                            memset(download_url_cache, 0, 200);
                            url_length=download_len;
                            memcpy(download_url_cache, receive_buffer + 6, download_len);
                            ESP_LOGW(MY_TAG, "download_url_cache= %s", download_url_cache);
                            ESP_LOGW(MY_TAG, "download_url= %s", download_url);
							if((strcmp(download_url,download_url_cache) && url_state<2) && download_len>0)
							{
								memset(download_url, 0, 200);
								memcpy(download_url, receive_buffer + 6, download_len);
								download_url[download_len] = 0;
								memset(file_name, 0, 40);
								memcpy(file_name, (char *)(receive_buffer + 6 + download_len + 1), receive_buffer[6 + download_len]);
								ESP_LOGW(MY_TAG, "file_name= %s", file_name);
								if (isconnected == 1)
								{
									ESP_LOGI(GATTS_TAG, "setbits");
									xEventGroupSetBits(EventGroupHandler, DOWNLOAD_BIT);
								}
								else
								{
									ESP_LOGI(GATTS_TAG, "wifi not connect");
									xEventGroupClearBits(EventGroupHandler, DOWNLOAD_BIT);
								}
							}
							else
							{
							    getdeviceinfo();
							    esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
							}
							/*******************************************************/
                            ble_senddata(receive_buffer); //将收到的数据返回给微信小程序
                        }

                        else if (receive_buffer[3] ==  0x07)
                        {//display picture what you want
#ifdef on_off
                        	esp_timer_stop(periodic_timer);
                        	char pic_name[40];
                        	memset(pic_name, 0, 40);
                        	int  pic_name_length=receive_buffer[4];
                        	memcpy(pic_name, receive_buffer + 5, pic_name_length);
                        	ESP_LOGW(MY_TAG,"the picture what you want to display is %s",pic_name);
							ble_senddata(receive_buffer);
							int ret=search_in_flash(pic_name,0x07,134400);
							ESP_LOGW(MY_TAG, "search_in_flash(pic_name,0x07,134400)=%d",ret);
                            ESP_LOGW(MY_TAG, "open timer");
                           	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
#endif
                        }
                        else if(receive_buffer[3] == 0x08)
                        {
#ifdef on_off
                        	esp_timer_stop(periodic_timer);
                        	//是否轮询显示图片
                        	pic_is_loop_display=receive_buffer[4];
                        	ESP_LOGW(MY_TAG,"pic_is_loop_display=%d",pic_is_loop_display);
							getdeviceinfo();
							sf_WriteBuffer(&pic_is_loop_display, info_page * 4096+ 81*50, sizeof(unsigned char));
							esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
							spi_flash_read(info_page * 4096,&flag,4096);
							if(flag[1]==0xaa&&flag[5]==0xaa)
							{
								flag[2]=0x11;
								flag[3]=receive_buffer[5];//睡眠时长
								flag[4]=receive_buffer[6];//启动时长
								spi_flash_erase_sector(info_page);
								spi_flash_write(info_page * 4096, &flag, 4096);
								spi_flash_read(info_page * 4096,&flag,4096);
								printf("flag 1~5 :\n");
								for(int i=1;i<=5;i++)
								{
									ESP_LOGW(MY_TAG,"flag[%d]=%x",i,flag[i]);
								}
							}
							loop_display_picture(receive_buffer[4]);
							if(receive_buffer[4] == 0x00)
							{
								ESP_LOGI(GATTS_TAG, "open timer");
								esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
							}
							else
							{
								ESP_LOGW(MY_TAG,"display_finished open timer 4s");
								esp_timer_start_periodic(periodic_timer, 4*1000 * 1000);
							}
#endif
                        }
                        else if (receive_buffer[3] == 0x09)
                        {//delete picture what you want
#ifdef on_off
                        	esp_timer_stop(periodic_timer);
							unsigned char temp;
							unsigned char picnum;
							unsigned char picture_num_real; //实际图片数量
							char temp_name[40];
							char pic_name[40];
							memset(pic_name, 0, 40);
							memset(temp_name, 0, 40);
							int pic_name_length=receive_buffer[4];
							ble_senddata(receive_buffer);
							memcpy(pic_name, receive_buffer + 5, pic_name_length);
							ESP_LOGW(MY_TAG,"pic_name=%s",pic_name);
							spi_flash_read(info_page * 4096, &picnum, sizeof(unsigned char));
							spi_flash_read(info_reboot_times * 4096, &picture_num_real, sizeof(unsigned char));
							printf("picture_num_real=%d\n",picture_num_real);
							if(picnum==0xff)
							{
								picnum=0;
							}
							ESP_LOGW(MY_TAG,"picnum=%d",picnum);
							unsigned char i=1;
							if(i>picnum&&picnum==0)
							{
								ESP_LOGW(MY_TAG,"flash no pic to let you delete");
							}
							else
							{
								for (; i <= picnum; i++)
								{
									spi_flash_read(info_page * 4096 + i * 50, temp_name, 40);
									if(i>picture_num_real)
									{
										temp_name[39]='\0';
										ESP_LOGW(MY_TAG,"temp_name=%s",temp_file_name);
										if(strcmp(temp_name,temp_file_name)==0)
										{
											ESP_LOGW(MY_TAG,"no picture what you want to delete");
											break;
										}
									}
									if (strcmp(temp_name, pic_name) == 0) //有重名文件
									{
										ESP_LOGW(MY_TAG,"find picture what you want to delete: %s",temp_name);
										spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
										ESP_LOGW(MY_TAG,"first read temp=%d i=%d",temp,i);
										if(temp==0)
										{
											ESP_LOGE(MY_TAG,"picture which is %d in flash was deleted already ",i);
											break;
										}
										unsigned char temp=0;
										sf_WriteBuffer(&temp, info_page * 4096 + i * 50 + 40, sizeof(unsigned char));
										spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
										ESP_LOGW(MY_TAG,"after delete pic read temp=%d,i=%d",temp,i);
										break;
									}
								}
								getdeviceinfo();
								esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
							}
							ESP_LOGI(MY_TAG,"CLEAR ONE PIC: esp_ble_gap_config_adv_data(&adv_data)");
							ESP_LOGI(GATTS_TAG, "open timer");
							esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
#endif
                        }
                        else if(receive_buffer[3]==0x10)
                        {//get and set time from network
                        	union longstrvalue{
								char time_sec[4];
								long i;
							}longstr;
							longstr.time_sec[0]=receive_buffer[5];
							longstr.time_sec[1]=receive_buffer[6];
							longstr.time_sec[2]=receive_buffer[7];
							longstr.time_sec[3]=receive_buffer[8];
							ESP_LOGW(MY_TAG,"time_sec[0]=%x,time_sec[1]=%x,time_sec[2]=%x,time_sec[3]=%x ",longstr.time_sec[0],longstr.time_sec[1],longstr.time_sec[2],longstr.time_sec[3]);
							ESP_LOGW(MY_TAG,"longstr.i=%ld",longstr.i);
							update_set_time(longstr.i);
                        	ESP_LOGI(GATTS_TAG, "open timer");
                        	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                        }
                        memset((char *)receive_buffer, 0, 512);
                    }
                }
            }
            //将收到的数据返回去
            // esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
            //  param->write.len, param->write.value, false);
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        memset((char *)receive_buffer, 0, 512);
                        //通知消息发送的数据
                        // uint8_t notify_data[3] = {0x55, 0x02, 0x55};
                        getdeviceinfo();
                        // for (unsigned char i = 0; i < 14; i++)
                        // {
                        //     rsp.attr_value.value[i] = device_info[i];
                        // }
                        //for (int i = 0; i < sizeof(notify_data); ++i)
                        //{
                        //    notify_data[i] = i%0xff;
                        // }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    14, device_info, false);
                    }
                }
                else if (descr_value == 0x0002)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }
            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        //特征值属性，可读可写可通知
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL)
        {
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }
        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for (int i = 0; i < length; i++)
        {
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        data_gatts_if = gatts_if;
        data_conn_id = param->connect.conn_id;
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        //esp_ble_gap_stop_advertising();
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void GattServers_Init(void)
{
    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    //创建一个esp_bt_controller_config_t由BT_CONTROLLER_INIT_CONFIG_DEFAULT()
    //宏生成的默认设置命名的BT控制器配置结构来初始化BT控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //初始化蓝牙控制器
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    /*使能蓝牙控制器：并工作在BLE模式
    支持四种蓝牙模式：
    ESP_BT_MODE_IDLE：蓝牙空闲模式
    ESP_BT_MODE_BLE：BLE模式
    ESP_BT_MODE_CLASSIC_BT：BT经典模式
    ESP_BT_MODE_BTDM：双模式（BLE + BT Classic）
   */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //初始化蓝牙堆栈
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //启用堆栈
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //注册GATTS事件处理程序
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    //注册GAT事件处理程序
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    //使用应用程序ID注册应用程序配置文件应用程序ID是用户分配的编号，用于标识每个配置文件。
    //这样，多个应用程序配置文件可以在一台服务器上运行。
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}
void getdeviceinfo(void)
{
    unsigned char total;
    spi_flash_read(info_page * 4096, &total, sizeof(unsigned char));
    if (total == 0xff)
	{
		total = 0;
	}
//    ESP_LOGI(GATTS_TAG, "total=%d", total);
//    remian_total = 11534336 - total * picture_cap * 4096;
//    device_info[6] = (unsigned char)((remian_total & 0xff000000) >> 8 >> 8 >> 8);
//    device_info[7] = (unsigned char)((remian_total & 0xff0000) >> 8 >> 8);
//    device_info[8] = (unsigned char)((remian_total & 0xff00) >> 8);
//    device_info[9] = (unsigned char)(remian_total & 0xff);
//    device_info[10] = total;
//    device_info[11] = isconnected;
    //update van
#ifdef on_off
    unsigned char delete_pic[total+1];
    spi_flash_read(info_reboot_times * 4096, &delete_pic, sizeof(unsigned char));
    if(delete_pic[0]==0xff)
    {
    	delete_pic[0]=0;
    }
    delete_pic[0]=0;//实际存在的图片数量
    ESP_LOGW(MY_TAG,"---------");
    ESP_LOGW(MY_TAG,"getdeviceinfo");
    printf("total=%d\n",total);
    for(unsigned char i=1;i<=total;i++)
    {
    	unsigned char temp;
    	spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
    	if(temp!=i)
    	{
    		delete_pic[0]++;
    	}
    	delete_pic[i]=temp;
    	ESP_LOGW(MY_TAG,"delete_pic[%d]=%x",i,temp);
    }
    delete_pic[0]=total-delete_pic[0];
#endif
	remian_total = 11534336 -  delete_pic[0] * picture_cap * 4096;
	device_info[2] = (unsigned char)((remian_total & 0xff000000) >> 8 >> 8 >> 8);
    device_info[3] = (unsigned char)((remian_total & 0xff0000) >> 8 >> 8);
    device_info[4] = (unsigned char)((remian_total & 0xff00) >> 8);
    device_info[5] = (unsigned char)(remian_total & 0xff);
    device_info[6] =  delete_pic[0];
    device_info[7] = isconnected;
    device_info[14] = pic_is_loop_display;
    sf_WriteBuffer(delete_pic, info_reboot_times * 4096,(total+1) * sizeof(unsigned char));
    ESP_LOGE(GATTS_TAG, "picture number total=%d", device_info[6]);
    ESP_LOGW(MY_TAG,"picture number total location is in %d sector and first %d bytes", info_reboot_times,sizeof(unsigned char));
    ESP_LOGW(MY_TAG,"picture loop display(0 no ,1 yes) :%d",device_info[14]);
    ESP_LOGW(MY_TAG,"*********");
}
//周期定时器回调函数
