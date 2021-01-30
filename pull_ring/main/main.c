//spiffsgen.py 0xA00000 spiffs_image spiffs.bin esptool.py --chip esp32 --port COM5 write_flash -z 0x410000 spiffs.bin
//#include "pic.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
#include "time.h"
#include "gatts_server.h"
#include "wifi.h"
#include "esp_wifi.h"
#include "spiff.h"
#include "http_client.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#define MY_TAG	"moding"
/****************************/
//update yuqiang
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
//static esp_adc_cal_characteristics_t  *adc_chars;
uint32_t adc_reading=0;
uint32_t voltage=0;
/*****************************/
/**********************************************/
//update van pull_ring
#include "aenc.h"
#include "esp_attr.h"
#include "display_pic.h"
extern void moding_sleep();
extern  esp_c_t  CN_Init(void);
extern void ncolor_display(uint16_t index);
extern void ncolor_display_diff(uint16_t index);
extern unsigned char pull_ring[4096];
extern void Acep_loadPIC1_init();
extern void Acep_loadPIC2_init();
extern void Acep_loadPIC1_end();
extern void Acep_loadPIC2_end();
extern void Acep_loadPIC1_test(unsigned char* pic_data3,int max_data);
extern void Acep_loadPIC2_test(unsigned char* pic_data4,int max_data);
extern void moding_sleep();
extern void Timer_Config();
extern void DELAY(UWORD DATA);
#ifdef on_off
unsigned char flag[4096];
#endif
extern char url_state;
#define GATTS_TAG "composite_DEMO"
unsigned char pic_is_loop_display;
char temp_file_name[40];
/**********************************************/

extern int write_id;
extern int time_count;
extern SFLASH_T g_tSF;
extern unsigned char total;
extern unsigned int rec_data;
extern uint16_t data_conn_id;
extern esp_gatt_if_t data_gatts_if;
extern esp_ble_adv_data_t adv_data;
extern esp_ble_adv_params_t adv_params;
extern esp_timer_handle_t periodic_timer;
extern esp_timer_handle_t periodic_timer_moding;
extern struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];
//extern char download_url[100], file_name[32];

int READ_BIT = BIT2;
int CLEAR_BIT = BIT1;
int DOWNLOAD_BIT = BIT0;
unsigned char picture_num; //图片数量
unsigned char receive_buffer[512];

//宏定义
#define REC_LEN 300 //发送数据的消息队列的数量
#define MESSAGE_Q_NUM 1
#define EVENTGROUP_TASK_PRIO 20  //任务优先级
#define EVENTGROUP_STK_SIZE 4096 * 2  //任务堆栈大小

EventBits_t EventValue;
QueueHandle_t Message_Queue; //信息队列句柄
TaskHandle_t EventGroupTask_Handler;  //任务句柄
EventGroupHandle_t EventGroupHandler; //事件标志组句柄

void ble_senddata(unsigned char *data);  //函数声明
void eventgroup_task(void *pvParameters);  //任务函数

//update van
extern unsigned char device_info[26];
extern char download_url[200], file_name[32];

int url_length;

//#define G_SIZE  30720

extern void LDK_init(void);
//extern void DK_ByO(void);
//extern void DK_ByT(void);
//extern void DK_ROflesh(void);
//extern void DK_RTflesh(void);
//extern void Hal_UpGraghScreen3();
//extern void Hal_UpGraghScreen4();
//extern void Write_CT(uint8_t cmd);
//extern void Write_CO(uint8_t cmd);
//extern void Hal_UpGraghScreen1(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);
//extern void Hal_UpGraghScreen2(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);
/**********************/
char *day[7]={
		"Sun","Mon","Tue","Wed","Thur","Fri","Sat"
	};
//主函数入口相关内容初始化
struct timeval stime;
struct tm *p;
time_t time_now;
void app_main()
{
	/********************************/
	//读取时间

//	gettimeofday(&stime,NULL);
//	time_now=stime.tv_sec;
//	p=localtime(&time_now);
//	printf("/*****************************/\n");
//	ESP_LOGW(MY_TAG,"%ld",time_now);
//	ESP_LOGW(MY_TAG,"%d/%d/%d %s %d:%d:%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,day[p->tm_wday],(p->tm_hour+8),p->tm_min,p->tm_sec);

//	printf("/*****************************/\n");
//	if(stime.tv_sec==0)
//	{
//		stime.tv_sec=160000000;
//		settimeofday(&stime,NULL);
//		time_now=stime.tv_sec;
//		p=localtime(&time_now);
////		printf("stime_tv_sec:%ld,stime_tv_usec:%ld\n",stime.tv_sec,stime.tv_usec);
//		printf("/*****************************/\n");
//		printf("%d/%d/%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday);
//		printf("%d  %d:%d:%d\n",p->tm_wday,(p->tm_hour+8),p->tm_min,p->tm_sec);
//		printf("/*****************************/\n");
//	}

	/********************************/

	ESP_LOGW(MY_TAG,"---------");
	ESP_LOGW(MY_TAG,"Init NVS");
    esp_err_t ret;
    // unsigned char test_receive[1024];
    g_tSF.TotalSize = 16 * 1024 * 1024; /* 总容量 = 16M */
    g_tSF.PageSize = 4 * 1024;           /* 页面大小 = 4K */
    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGW(MY_TAG,"*********");

//    ESP_LOGW(MY_TAG,"---------");
//    ESP_LOGW(MY_TAG,"clear picture information(info_page sector) and picture capacity of sector");
//    spi_flash_erase_sector(info_page);
//    for(unsigned char i=1;i<=70;i++)
//    {
//    	spi_flash_erase_range((picture_page + picture_cap * i) * 4096, picture_cap * 4096); //清除flash内存
//    	ESP_LOGW(MY_TAG,"clear sector:%d",i);
//    }
//    ESP_LOGW(MY_TAG,"clear sector done");
//    ESP_LOGW(MY_TAG,"*********");
//
//    for(int i=0;i<39;i++)
//	{
//		temp_file_name[i]=0xff;
//	}
//    temp_file_name[39]='\0';
//    ESP_LOGW(MY_TAG,"do temp_file_name[i]=0xff;");
//
//	ESP_LOGW(MY_TAG,"---------");
//    ESP_LOGW(MY_TAG,"Init ink bottle");
//    CN_Init();
//    ESP_LOGW(MY_TAG,"*********");
    //墨水屏直接显示图片
    //	ncolor_display(1);
    //	ncolor_display(0);
    //	ncolor_display_diff(1);
    //	ncolor_display_diff(0);


//#ifdef on_off
//    ESP_LOGW(MY_TAG,"---------");
//    //开始与结束的标志位:flag[1] flag[5]
//	//有效信息:flag[2] flag[3] flag[4]
//	//是否设置睡眠与启动时间:flag[2]
//	//设置睡眠时间:flag[3]
//	//设置启动时间:flag[4]
//	//是否轮询显示图片:pic_is_loop_display
//	spi_flash_read(info_page * 4096,&flag,4096);
//	if(flag[1]!=0xaa)
//	{
//		unsigned char temp=flag[0];
//		flag[1]=0xaa;
//		flag[2]=0x0a;
//		flag[5]=0xaa;
//		flag[0]=temp;
//		spi_flash_erase_sector(info_page);
//		spi_flash_write(info_page * 4096, &flag, 4096);
//		spi_flash_read(info_page * 4096,&flag,4096);
//		ESP_LOGW(MY_TAG,"flag[1]=%x	flag[5]=%x",flag[1],flag[5]);
//	}
//	else
//	{
////		char *s="5FE1670C";
////		update_set_time(s);
//		ESP_LOGW(MY_TAG,"flag 0~5 is :");
//		for(int i=0;i<=5;i++)
//		{
//			ESP_LOGW(MY_TAG,"flag[%d]=%x",i,flag[i]);
//		}
//
////		struct MY_STRUCT
////		{
////			unsigned char a;
////			unsigned char b;
////			int c;
////		}my_struct;
////		ESP_LOGW(MY_TAG,"sizeof(unsigned char)=%d sizeof(char)=%d sizeof(int)=%d sizeof(my_struct)=%d",sizeof(unsigned char),sizeof(char),sizeof(int),sizeof(my_struct));
////		spi_flash_read(info_page*4096,&my_struct,sizeof(my_struct));
////		ESP_LOGW(MY_TAG,"%x %x %x",my_struct.a,my_struct.b,my_struct.c);
//	}
//	ESP_LOGW(MY_TAG,"*********");
//
//	ESP_LOGW(MY_TAG,"---------");
//	ESP_LOGW(MY_TAG,"look pic_is_loop_display value after boot ");
//    spi_flash_read(info_page * 4096+ 81*50,&pic_is_loop_display, sizeof(unsigned char));
//    if(pic_is_loop_display==0xff)
//	{
//		pic_is_loop_display=0;
//		sf_WriteBuffer(&pic_is_loop_display, info_page * 4096+ 81*50, sizeof(unsigned char));
//		spi_flash_read(info_page * 4096+ 81*50,&pic_is_loop_display, sizeof(unsigned char));
//		ESP_LOGW(MY_TAG,"pic_is_loop_display=%d",pic_is_loop_display);
//	}
//    getdeviceinfo();
//	if(pic_is_loop_display==1)
//	{
//		loop_display_picture(0x01);
//		ESP_LOGW(MY_TAG,"start deep sleep: %lld us", esp_timer_get_time());
//	}
//	ESP_LOGW(MY_TAG,"*********");
//#endif

    //update van
    uint8_t mac[6];
	esp_read_mac(mac, ESP_MAC_BT);
	for(int i=0;i<6;i++){
		device_info[8+i]=mac[i];
	}

	ESP_LOGW(MY_TAG,"---------");
	ESP_LOGW(MY_TAG,"timer wifi gatt_server init");
	Timer_Config();
    wifi_init_sta(); //WIFI初始化
    GattServers_Init();
    ESP_LOGW(MY_TAG,"*********");

    /*****************************************************************/
//    update van
//    uint8_t mac[6];
//    esp_read_mac(mac, ESP_MAC_WIFI_STA);
//    printf("esp_read_mac()11: %02x:%02x:%02x:%02x:%02x:%02x \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
//    esp_read_mac(mac, ESP_MAC_BT);
//	  printf("esp_read_mac()22: %02x:%02x:%02x:%02x:%02x:%02x \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
//	  esp_read_mac(mac, ESP_MAC_ETH);
//    printf("esp_read_mac()33: %02x:%02x:%02x:%02x:%02x:%02x \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      /***********************************************************************/

    // ESP_LOGE("main", "flash test");
    // for (int i = 0; i < 1024; i++)
    // {
    //     test_buffer[i] = 2 + 0x30;
    // }
    // sf_WriteBuffer(test_buffer, 1280 * 4096, 1024);
    // memset(test_buffer, 0, 1024);
    // printf("read data is %s", test_buffer);
    // spi_flash_read(1280 * 4096, test_buffer, 1024);
    // printf("read data is %s", test_buffer);

    // rtc_gpio_pulldown_en(34);
    // rtc_gpio_pullup_en(GPIO_NUM_34); //保持上拉
    // switch (esp_sleep_get_wakeup_cause())
    // {
    // case ESP_SLEEP_WAKEUP_EXT1:
    // {
    //     uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    //     if (wakeup_pin_mask != 0)
    //     {
    //         int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
    //         printf("Wake up from GPIO %d\n", pin);
    //     }
    //     else
    //     {
    //         printf("Wake up from GPIO\n");
    //     }
    //     break;
    // }
    // case ESP_SLEEP_WAKEUP_UNDEFINED:
    // default:
    //     printf("Not a deep sleep reset\n");
    // }
    // //第一个参数：RTCIO的编号
    // //第二个参数：是高电平唤醒还是低电平唤醒
    const int ext_wakeup_pin_1 = 25;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    // printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    //esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
    //esp_deep_sleep_start();
    // switch (esp_sleep_get_wakeup_cause())
    // {
    // case ESP_SLEEP_WAKEUP_EXT0:
    //     printf("wake up.........\n");
    //     break;
    // case ESP_SLEEP_WAKEUP_UNDEFINED:
    // default:
    //     printf("not sleep...............\n");
    //     break;
    // }
    // //第一个参数：RTCIO的编号
    // //第二个参数：是高电平唤醒还是低电平唤醒
    // esp_sleep_enable_ext0_wakeup(34, 1);

    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    EventGroupHandler = xEventGroupCreate(); //创建事件标志组

    //创建事件标志组处理任务
    xTaskCreate((TaskFunction_t)eventgroup_task,
                (const char *)"eventgroup_task",
                (uint16_t)EVENTGROUP_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)EVENTGROUP_TASK_PRIO,
                (TaskHandle_t *)&EventGroupTask_Handler);



//    ESP_LOGW(MY_TAG,"---------");
//    ESP_LOGW(MY_TAG,"get voltage");
//	adc1_config_width(3);
//	adc1_config_channel_atten(6,3);
//	adc_chars = calloc(1,sizeof(esp_adc_cal_characteristics_t));
//	esp_adc_cal_characterize(1,3,3,1100,adc_chars);
//	while(1)
//	{
//		voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(6),adc_chars)*2;
//		ESP_LOGW(MY_TAG,"Voltage:%d",voltage);
//		vTaskDelay(1000/portTICK_RATE_MS);
//	}
//	ESP_LOGW(MY_TAG,"*********");

    return;
}
//static unsigned int total_num = 0;
//static unsigned int current_num = 0;
char send_buf[20];

//事件标志组处理任务
void eventgroup_task(void *pvParameters)
{
    unsigned char cmp_name[40];
    while (1)
    {
        if (EventGroupHandler != NULL)
        {
            //等待事件组中的相应事件位
            EventValue = xEventGroupWaitBits((EventGroupHandle_t)EventGroupHandler,
                                             (EventBits_t)DOWNLOAD_BIT | CLEAR_BIT | READ_BIT,
                                             (BaseType_t)pdTRUE,
                                             (BaseType_t)pdFALSE,
                                             (TickType_t)portMAX_DELAY);
            //wifi连接成功准备下载图片
            if (EventValue & DOWNLOAD_BIT)
            {
            	ESP_LOGW(MY_TAG,"---------");
            	url_state=2;
//            	unsigned char pic_total;
            	ESP_LOGW(MY_TAG,"time to download picture");
                //1280页也就是5M出开始写
                //下载图片前的工作
                spi_flash_read(info_page * 4096, &picture_num, sizeof(unsigned char));
                if (picture_num == 0xff) //没有图片
                {
//                	 unsigned char i;
//                	 for(i=1;i<=70;i++)
//					 {
//						unsigned char temp;
//						spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
//						if(temp==i)
//						{
//							break;
//						}
//					 }
//                	 if(i==71)
//                	 {
                		 picture_num = 0;
//                	 }
//                	 else
//                	 {
//                		picture_num=70;
//                	 }
//                	 pic_total=picture_num;
                }

                ESP_LOGW(MY_TAG, "picture_num(you pushed before this pic)=%d", picture_num);

                //判断是否有重名文件
                unsigned char temp_name[40];
                unsigned char i;
                for(i=0;i<40;i++)
                {
                	temp_name[i]=0xff;
                }
                for (i = 1; i <= picture_num; i++)
                {
                    spi_flash_read(info_page * 4096 + i * 50, cmp_name, 40);
                    if( strcmp((char *)cmp_name,(char *)temp_name)==0)
					 {
                    	ESP_LOGW(MY_TAG, "i to the end!!!!!!!");
						 break;
					 }
                    if (strcmp((char *)cmp_name, file_name) == 0) //有重名文件
                    {
                    	ESP_LOGW(MY_TAG, "find same name picture");
                       	//把文件名写到重名的地址
                        break;
                    }
                }
                ESP_LOGW(MY_TAG, "picture_num=%d", picture_num);

//                    update van pull_ring
				/*****************************/
				//判断图片是否被删掉
				unsigned char j;
				for(j=1;j<=picture_num;j++)
				{
					spi_flash_read(info_page * 4096 + j * 50 + 40, &picture_num_temp, sizeof(unsigned char));
					 if( picture_num_temp==0xff)
					 {
						 ESP_LOGW(MY_TAG, "j to the end!!!!!!!");
						 break;
					 }
					if(picture_num_temp==0)
					{
						ESP_LOGW(MY_TAG, "find picture was be deleted");
						break;
					}
				}
				picture_num++;
				sf_WriteBuffer(&picture_num, info_page * 4096, sizeof(unsigned char));         //写入文件个数
//				pic_total++;
//				sf_WriteBuffer(&pic_total, info_reboot_times * 4096+sizeof(unsigned char), 2*sizeof(unsigned char));
				if(j==picture_num&&i==picture_num)
				{
					 if(picture_num!=1)
					 {
						 ESP_LOGW(MY_TAG, "not find same name picture");
						 ESP_LOGW(MY_TAG, "not find picture was be deleted");
					 }
					 ESP_LOGW(MY_TAG, "j>picture_num&&i>picture_num current picture_num_real=%d", picture_num);
				}
				else
				{
					//若有重名，先把文件名写到重名的地址
					if(i<picture_num)
					{
						picture_num = i;
					}
					else
					{
					//若无重名，有被删掉的图片，把文件写到被删掉的图片的地址
						picture_num = j;
						unsigned char picture_num_real; //实际图片数量
						spi_flash_read(info_reboot_times * 4096, &picture_num_real, sizeof(unsigned char));
						picture_num_real++;
						sf_WriteBuffer(&picture_num_real, info_reboot_times * 4096, sizeof(unsigned char));
						spi_flash_read(info_reboot_times * 4096, &picture_num_real, sizeof(unsigned char));
						ESP_LOGW(MY_TAG, "current picture_num_real=%d", picture_num_real);
					}
				}
			    /*****************************/
				sf_WriteBuffer((uint8_t *)file_name, info_page * 4096 + picture_num * 50, 40); //存储文件名称长度40
				sf_WriteBuffer(&picture_num, info_page * 4096+ picture_num * 50+40, sizeof(unsigned char));
                ESP_LOGI("http task", "read file_name= %s", file_name);
                ESP_LOGE("http task", "closr timer");
                esp_timer_stop(periodic_timer);
#ifndef on_off
				 esp_timer_stop(periodic_timer_moding);
#endif
                http_test_task(download_url, file_name);
                ESP_LOGE("http task", "start timer");
                getdeviceinfo();
                esp_ble_gap_config_adv_data(&adv_data);//图片完成后广播
                ESP_LOGI(GATTS_TAG,"open timer");
               	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                url_state=1;
                ESP_LOGW(MY_TAG,"*********");
            }
            else if (EventValue & CLEAR_BIT)
            {
                //清空所有图片
            	ESP_LOGW(MY_TAG,"---------");
                esp_wifi_stop();
                ESP_LOGE("http task", "clear picture");
                spi_flash_erase_sector(info_page);
                spi_flash_erase_sector(info_reboot_times);
                getdeviceinfo();
                esp_ble_gap_config_adv_data(&adv_data);
                esp_wifi_start();
                esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
                ESP_LOGW(MY_TAG,"*********");
            }
            else if (EventValue & READ_BIT)
            {
                //清空所有图片
                esp_wifi_stop();
                spi_flash_read(info_page * 4096, &picture_num, sizeof(unsigned char));
                ESP_LOGI("read task", "picture_num=%d", picture_num);
                int size;
                unsigned int bytes_read1 = 0;
                for (char i = 1; i <= picture_num; i++)
                {
                    spi_flash_read(info_page * 4096 + i * 50 + 41, &size, sizeof(int));
                    ESP_LOGI("read task", "size=%d", size);
                    spi_flash_read(info_page * 4096 + i * 50, file_name, 40);
                    ESP_LOGI("read task", "filename=%s", file_name);
                    ESP_LOGI("read task", "read data:");
                    while (size > 0)
                    {
                        spi_flash_read((picture_page + picture_cap * i) * 4096 + bytes_read1 * 512, receive_buffer, 512);
                        ESP_LOGI("read task", "add=%d", (picture_page + picture_cap * i) * 4096 + bytes_read1 * 512);
                        bytes_read1++;
                        size -= 512;
                        esp_log_buffer_hex("receive data", receive_buffer, 512);
                    }
                    ESP_LOGI("read task", "bytes_read=%d", bytes_read1);
                }
                esp_wifi_start();
                ESP_LOGI(GATTS_TAG,"open timer");
               	esp_timer_start_periodic(periodic_timer,60*1000 * 1000); //设置定时器的定时周期为60s
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_RATE_MS); //延时10ms，也就是10个时钟节拍
        }
    }
}
// ble 一次只能发送20个字节，分包进行数据发送
void ble_senddata(unsigned char *data)
{
    int len = 0;
    //len = strlen((char *)data);
    len = data[2] + 4;
    ESP_LOGW(MY_TAG,"len=%d", len);
//    if (len <= 20)
//    {
    ESP_LOGW(MY_TAG,"GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", data_gatts_if, data_conn_id, write_id);
//        esp_ble_gatts_send_indicate(data_gatts_if, data_conn_id, write_id,
//                                           len, (uint8_t *)data, false);
        esp_ble_gatts_send_indicate(data_gatts_if, data_conn_id, write_id,
                                    10, (uint8_t *)data, false);
//    }
//    else if (len > 20)
//    {
//        if (len % 20 == 0)
//        {
//            total_num = len / 20;
//        }
//        else
//        {
//            total_num = len / 20 + 1;
//        }
//        current_num = 1;
//        while (current_num <= total_num)
//        {
//            if (current_num < total_num)
//            {
//                memcpy(send_buf, receive_buffer + (current_num - 1) * 20, 20);
//                esp_ble_gatts_send_indicate(data_gatts_if, data_conn_id, write_id,
//                                            20, (uint8_t *)send_buf, false);
//            }
//            else if (current_num == total_num)
//            {
//                memset(send_buf, 0, 20);
//                // printf("remin_data=%d\r\n", (len - (current_num - 1) * 20));
//                memcpy(send_buf, receive_buffer + (current_num - 1) * 20, (len - (current_num - 1) * 20));
//                esp_ble_gatts_send_indicate(data_gatts_if, data_conn_id, write_id,
//                                            (len - (current_num - 1) * 20), (uint8_t *)send_buf, false);
//            }
//            current_num++;
//        }
//    }
}



























