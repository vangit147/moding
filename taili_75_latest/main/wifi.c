#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "esp_err.h"
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "gatts_server.h"
#include "calendar.h"
#define wifi_tag	"wifi"
extern esp_ble_adv_data_t adv_data;
unsigned char isconnected = 0;
/* 系统事件循环处理函数 */
// static esp_err_t example_net_event_handler(void *ctx, system_event_t *event)
// {
//     wifi_mode_t mode;
//     switch (event->event_id)
//     {
//     case SYSTEM_EVENT_STA_START:
//         esp_wifi_connect();
//         break;
//     case SYSTEM_EVENT_STA_GOT_IP:
//     {
//         printf("GOT_IP:WIFI connectted\r\n");
//         // esp_wifi_get_mode(&mode);
//         break;
//     }
//     case SYSTEM_EVENT_STA_CONNECTED:
//         printf("connected WIFI connectted\r\n");
//         break;
//     case SYSTEM_EVENT_STA_DISCONNECTED:
//         /* This is a workaround as ESP32 WiFi libs don't currently
//            auto-reassociate. */
//         printf("WIFI disconnectted\r\n");
//         break;
//     default:
//         break;
//     }
//     return ESP_OK;
// }
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	static int retry_num = 0; /* 记录wifi重连次数 */
	/* 系统事件为WiFi事件 */
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START) /* 事件id为STA开始 */
        {
        	//esp_wifi_connect();
        	//update van
        	esp_err_t  err_code;
        	err_code=esp_wifi_connect();
        	printf("err_code=%d\n",err_code);
        	/********************************/
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) /* 事件id为失去STA连接 */
        {
        	//esp_wifi_connect();
        	//update van;
        	esp_err_t  err_code;
        	err_code=esp_wifi_connect();
        	printf("err_code=%d\n",err_code);
        	/*****************************/
        	isconnected = 0;
        	retry_num++;
        	printf("retry to connect to the AP %d times. \n", retry_num);
            if (retry_num > 10) /* WiFi重连次数大于10 超时*/
            {
                retry_num = 0;
                printf("WIFI CONNECTED FAIL ....RECONNCET.....\n");
            }
        }
    }
    /* 系统事件为ip地址事件，且事件id为成功获取ip地址 */
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data; /* 获取IP地址信息*/
        printf("WIFI CONNECTED SUCCESS.....\n");
        printf("got ip:%d.%d.%d.%d \n", IP2STR(&event->ip_info.ip)); /* 打印ip地址*/
        /* WiFi重连次数清零 */
        retry_num = 0;
        isconnected = 1;
        getdeviceinfo();
        esp_ble_gap_config_adv_data(&adv_data);
    }
}
void wifi_init_sta(void)
{
    tcpip_adapter_init(); //初始化TCP/IP协议栈
    /* 创建默认事件循环,*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ESP_ERROR_CHECK(esp_event_loop_init(example_net_event_handler, NULL));
    //upate van
    /*下面两行带代码用来初始化WIFI环境 */
    /**************************/
    /* 使用WIFI_INIT_CONFIG_DEFAULT() 来获取一个默认的wifi配置参数结构体变量*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    /* 根据cfg参数初始化wifi连接所需要的资源 */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //忽略之前存储的wifi ssid和pwd
//    esp_wifi_restore();
    //update van
    // 配置STA 模式，配置wifi名称以及密码
    // 使用ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config))函数是配置生效
	/**************************/
     wifi_config_t wifi_config = {
         .sta = {
			  .ssid = "moding_wifi",
			  .password = "modingtech.com"},
     };
     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
     /* 将事件处理程序注册到系统默认事件循环，分别是WiFi事件、IP地址事件及smartconfig事件 */
       ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
       ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
       //update van
       //设置当前工作模式可以使用esp_wifi_get_mode()获取当前工作模式
       //esp即可处在AP模式，也可处在STA模式，也可STA+AP模式
       /*****************************/
       ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
       //update van
       /*启动wifi功能，使我们的配置全部生效*/
   	/****************************************/
       ESP_ERROR_CHECK(esp_wifi_start());
       //esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
       //esp_wifi_set_auto_connect();

    ESP_LOGW(wifi_tag,"wifi_init_sta finished. \n");
}
