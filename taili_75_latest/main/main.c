//spiffsgen.py 0xA00000 spiffs_image spiffs.bin esptool.py --chip esp32 --port COM5 write_flash -z 0x410000 spiffs.bin

#include "pic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "wifi.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "driver/rtc_io.h"
#include "calendar.h"
#include "aenc.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t  *adc_chars;
uint32_t adc_reading=0;
uint32_t voltage=0;

extern int write_id;
extern SFLASH_T g_tSF;
extern uint16_t data_conn_id;
extern esp_gatt_if_t data_gatts_if;
extern esp_ble_adv_data_t adv_data;

static const char *main_tag = "main";
unsigned char receive_buffer[512];

void ble_senddata(unsigned char *data);  //函数声明

//extern void e_init(void);//10.2  台历
//extern esp_c_t  CN_Init(void);//5.65  拉环
extern void LDK_init(void);//7.5 台历
//主函数入口相关内容初始化
void app_main()
{
    g_tSF.TotalSize = 16 * 1024 * 1024; /* 总容量 = 16M */
    g_tSF.PageSize = 4 * 1024;          /* 页面大小 = 4K */

    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

//    spi_flash_erase_sector(info_page);
//    spi_flash_erase_sector(info_page_backup);
//    spi_flash_erase_sector(low_network_wifi_picture_page);
//    esp_deep_sleep_start();

    ESP_LOGW(main_tag,"get voltage");
	adc1_config_width(3);
	adc1_config_channel_atten(6,3);
	adc_chars = calloc(1,sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(1,3,3,1100,adc_chars);
	voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(6),adc_chars)*2;
	ESP_LOGW(main_tag,"Voltage:%d",voltage);

    ESP_LOGW(main_tag,"device MAC address");
    uint8_t mac[6];
	esp_read_mac(mac, ESP_MAC_BT);
	for(int i=0;i<6;i++){
		device_info[9+i]=mac[i];
		if(i==5)
		{
			if((int)mac[i]<16)
			{
				printf("0");
			}
			printf("%x\n",mac[i]);
		}
		else
		{
			if((int)mac[i]<16)
			{
				printf("0");
			}
			printf("%x:",mac[i]);
		}
	}

	ESP_LOGW(main_tag, "read_write_init");
	read_write_init();

	ESP_LOGW(main_tag,"wifi init_sta");
	wifi_init_sta();

	ESP_LOGW(main_tag, "LDK_init desk_calender 7.5");
	LDK_init();
	ESP_LOGW(main_tag, "LDK_init desk_calender 7.5 done");

	vTaskDelay(2000 / portTICK_PERIOD_MS);
	if(voltage<=220)
	{
//		display_low_network_wifi_picture(0);
		ESP_LOGW(main_tag,"low voltage ---> deep sleep start");
		esp_deep_sleep_start();
	}
	else
	{
		ESP_LOGW(main_tag,"voltage>220");
		find_wakeup_cause();
//		check_wifi_httpdownload_pic('0');
//		char *dwurl="https://aink.net/devices/resources/c4:4f:33:79:7a:eb/20201201.bin";
//		 char *dwurl="https://aink.net/devices/download/pic/c4:4f:33:79:7a:eb?picname=20210101.bin&picsize=0&voltage=328&action=0&timestamp=1609430400";
//		http_test_task(dwurl);
	}

	return;
}

void ble_senddata(unsigned char *data)
{
    int len = 0;
    len = data[2] + 4;
    ESP_LOGW(main_tag,"len=%d", len);
    if (len <= 20)
    {
        printf("GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", data_gatts_if, data_conn_id, write_id);
        esp_ble_gatts_send_indicate(data_gatts_if, data_conn_id, write_id,
                                    len, (uint8_t *)data, false);
    }
}
