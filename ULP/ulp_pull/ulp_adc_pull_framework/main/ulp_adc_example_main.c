/* 
   x,z轴分方向的，所以定义两个阈值
   变量说明 :
   阈值需要在初始化ULP的时候赋值
   p_ax    x正轴阈值
   m_ax    x负轴阈值 
   sysrun_times:ulp运行的次数
   moving_times：运动的次数
   move_flags：当前是否运动的标志位
   在ulp中设置了唤醒主CPU后不停止工作，ULP会一直在运行
*/

#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "timer.h"
#include "bleinit.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <time.h>
#include "esp_spi_flash.h"

#include "spiff.h"
RTC_DATA_ATTR unsigned char current_mode = 0; //0-runmode 1-demomode
RTC_DATA_ATTR unsigned char Hiber_time = 0;   //H状态标志位设置 1设置，0未设置



typedef enum
{
    Run_Mode = 0,
    Demo_Mode
};
typedef enum
{
    Hibernation = 1,
    Display,
    Update,
    Response
}pull_mode;
typedef struct Mode_t
{
	pull_mode mode;
    unsigned char status;
};
RTC_DATA_ATTR struct Mode_t t_mode; //将结构体放入RTC内存中
struct tm timeinfo;                 //时间变量结构体
time_t now;

//添加宏 RTCRTC_DATA_ATTR
//将变量定义在(RTC)内存
//深睡时变量不会清零
RTC_DATA_ATTR static unsigned char mode_union[48];


extern esp_timer_handle_t periodic_timer;
int HIBER_BIT =  BIT0;
int DISPLAY_BIT = BIT1;
int UPDATE_BIT = BIT2;
int RESPONSE_BIT = BIT3;
int TIMER_BIT = BIT4;
int STARTSCREEN = BIT5;
//任务相关定义

#define the_time_periods_of_the_day 48
#define EVENTGROUP_TASK_PRIO 10           //任务优先级
#define EVENTGROUP_STK_SIZE 4096 * 2      //任务堆栈大小
TaskHandle_t EventGroupTask_Handler;      //任务句柄
void eventgroup_task(void *pvParameters); //任务函数

EventGroupHandle_t EventGroupHandler; //事件标志组句柄
EventBits_t EventValue;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");
const gpio_num_t GPIO_SCL = GPIO_NUM_32;
const gpio_num_t GPIO_SDA = GPIO_NUM_33;
const gpio_num_t gpio_led = GPIO_NUM_2;

// function
static void init_ulp_program();
static void start_ulp_program();
static void getcurrenttime(void);
static void write_modetoflash(void);
short aacx, aacy, aacz; //加速度传感器原始数据
float AccX, AccY, AccZ;

void start_ulp();
void hibernation();
void display_picture();
void update();
unsigned char value;
void app_main()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    //开机时判断当前的运行模式为演示模式还是运行模式,当深度唤醒时,初始化的函数不会再次执行,会进入
    //esp_wake_deep_sleep执行代码
    setenv("TZ", "CST-8", 1);
    getcurrenttime();

    printf("hours=%d ,minute=%d ,second=%d \n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    write_modetoflash();
    current_mode=0;
    if (current_mode == 0)
    {
	    printf("current mode is Run_mode\r\n");
	    t_mode.mode = Run_Mode; //运行模式
	    spi_flash_read(1500 * 4096, (uint8_t *)&mode_union, 48);
	    printf("%d\n",mode_union[0]);
    }
    else if (current_mode == 1)
    {
    	printf("current mode is Demo_mode\r\n");
	    t_mode.mode = Demo_Mode; //演示模式
	    spi_flash_read(1500 * 4096 + 200, (uint8_t *)&mode_union, 48);
    }

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch(cause)
    {
    	case ESP_SLEEP_WAKEUP_TIMER:
    		if(t_mode.status==Hibernation)
    		{
    			if(value==1)
    			{
    				printf("after first boot and deep sleep,now wakeup by timer , it will be in Hibernation mode\n");
    			}
    			else if(value==2)
    			{
    				printf("wakeup by timer in Hibernation mode,just start ULP and deep sleep, will be in Display mode in 5 seconds\n");
					start_ulp();
					esp_deep_sleep_start();
    			}
    		}
    		else if(t_mode.status==Display)
    		{
    			if(value==3)
    			{
    				printf("wakeup by timer in Display mode\n");
    			}
    		}
    		else if(t_mode.status==Response)
    		{
    			if(value==5)
				{
					printf("wakeup by timer in Response mode\n");
				}
    		}
    		if(value==4)
    		{
    			printf("wakeup by timer from Display mode,now,it will be in Update mode\n");
    		}
			break;
    	case ESP_SLEEP_WAKEUP_ULP:
			printf( "ULP wakeup\n");
			aacx = ((unsigned short)ulp_ac << 8) | ulp_acl;
			aacz = ((unsigned short)ulp_az << 8) | ulp_azl;
			AccX = aacx / 16384.0; // X-axis value
			AccZ = aacz / 16384.0;
			printf("the aacx is %d \n",aacx);
			printf("the aacz is %d \n",aacz);
			printf("the AccX is %lf \n",AccX);
			printf("the AccZ is %lf \n",AccZ);
			printf("the compare_times is %x \n",ulp_compare_times);
			printf("moving_times=%x,sysrun_times=%x,move_flags=%x\r\n", ulp_moving_times, ulp_sysrun_times, ulp_move_flags);
			break;
    	case ESP_SLEEP_WAKEUP_UNDEFINED:
    		value=1;
    		esp_sleep_enable_timer_wakeup(1000000);
    		break;
    	default:
			printf("other wakeup cause ----> deep sleep start");
			break;
    }

	EventGroupHandler = xEventGroupCreate(); //创建事件标志组                                             //创建事件标志组处理任务
	xTaskCreate((TaskFunction_t)eventgroup_task,
				(const char *)"eventgroup_task",
				(uint16_t)EVENTGROUP_STK_SIZE,
				(void *)NULL,
				(UBaseType_t)EVENTGROUP_TASK_PRIO,
				(TaskHandle_t *)&EventGroupTask_Handler);
	 switch (t_mode.status)
	 {
	    case Hibernation:
	    	printf("hiber mode setting bits!\n");
	        xEventGroupSetBits(EventGroupHandler, HIBER_BIT); //设置hiber模式标志位
	        break;
	    case Display:
	    	printf("Display mode setting timer!\n");
	        xEventGroupSetBits(EventGroupHandler, DISPLAY_BIT); //设置DISPLAY_BIT模式标志位
	        break;
	    case Update:
	    	printf("Update mode setting timer!\n");
	        xEventGroupSetBits(EventGroupHandler, UPDATE_BIT); //设置UPDATE_BIT模式标志位
	        break;
	    case Response:
	    	printf("Response mode setting timer!\n");
	        xEventGroupSetBits(EventGroupHandler, RESPONSE_BIT);
	        break;
	    default:
	        break;
	  }
//    esp_sleep_enable_timer_wakeup(1000000);
    esp_deep_sleep_start();
}

void start_ulp()
{
	printf("start ULP\n");
	init_ulp_program();                             //初始化ulp
	start_ulp_program();                            //启动ulp
	ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup()); //设置允许ulp唤醒CPU
}
void hibernation()
{
	unsigned char i,j=0;
	uint64_t sleep_time;
	getcurrenttime();
	i = timeinfo.tm_hour;
	if(timeinfo.tm_min > 30)
	{
		i = timeinfo.tm_hour + 1;
	}
	for(;i<the_time_periods_of_the_day;i++)
	{
		if(mode_union[i]==1)
		{
			j++;
		}
		else
		{
			break;
		}
	}
	unsigned char temp = 60/(the_time_periods_of_the_day/24);
	sleep_time = j*temp*60-55;
	value=2;
	esp_sleep_enable_timer_wakeup(sleep_time);
}

void display_picture()
{
	printf("start loop display picture\n");
	vTaskDelay(2000 / portTICK_RATE_MS);
	printf("loop display picture end\n");
}

void update()
{
	printf("http_download picture\n");
//	http_download();
	vTaskDelay(2000 / portTICK_RATE_MS);
	printf("http_download picture end\n");
}
static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
    rtc_gpio_init(gpio_led);
    rtc_gpio_set_direction(gpio_led, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_SCL);
    rtc_gpio_set_direction(GPIO_SCL, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(GPIO_SDA);
    rtc_gpio_set_direction(GPIO_SDA, RTC_GPIO_MODE_INPUT_ONLY);
    ulp_p_ax = 1000;
    ulp_p_az = 10000;
    ulp_m_ax = 50;
    ulp_m_az = 0;
    ulp_compare_times = 6; //设置ulp的比较运行次数
    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 1000 * 1000);
}

static void start_ulp_program()
{
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

//事件标志组处理任务
void eventgroup_task(void *pvParameters)
{
    while (1)
    {

        if (EventGroupHandler != NULL)
        {
            //等待事件组中的相应事件位
            EventValue = xEventGroupWaitBits((EventGroupHandle_t)EventGroupHandler,
                                             (EventBits_t)HIBER_BIT | DISPLAY_BIT | UPDATE_BIT | RESPONSE_BIT,
                                             (BaseType_t)pdTRUE,
                                             (BaseType_t)pdFALSE,
                                             (TickType_t)portMAX_DELAY);
            //  printf("EventGroupvalue:%d\r\n", EventValue);

            if (EventValue & HIBER_BIT) //hibernation 模式
            {
                printf("hibernation Task Start.......\r\n");
                if (Hiber_time == 0) //未设置
                {
                    Hiber_time = 1;
                    esp_sleep_enable_timer_wakeup(1000000); //设置唤醒时间
//                    hibernation();
                    printf("enter deep sleep\n");
                    esp_deep_sleep_start();                 //进入深睡模式
                }
                else if (Hiber_time == 1) //设置过Timer
                {
                	 printf("enter deep sleep\n");
                    esp_deep_sleep_start(); ////进入深睡模式
                }
            }
            else if (EventValue & DISPLAY_BIT) //Display 模式
            {
                printf("display Task\r\n");
                vTaskDelay(2000 / portTICK_RATE_MS);
                if(timeinfo.tm_hour==23)
				{
					printf("i am in Display mode,but it time is to go in Update mode \n");
					xEventGroupSetBits(EventGroupHandler, UPDATE_BIT); //设置UPDATE_BIT模式标志位
				}
				else
				{
					if (ulp_move_flags == 0) //ulp状态 未动
					{
						printf("from display mode enter deepsleep\r\n ");
						start_ulp();
						value=4;
						esp_sleep_enable_timer_wakeup(1000000);
						printf("enter deep sleep\n");
						esp_deep_sleep_start(); ////进入深睡模式
					}
					else if (ulp_move_flags == 1) //ulp状态 动
					{
						printf("from display mode enter deepsleep\r\n ");
						getcurrenttime();
						display_picture();
						getcurrenttime();
						if(timeinfo.tm_hour==23)
						{
							printf("i am in Display mode,but it time is to go in Update mode \n");
							xEventGroupSetBits(EventGroupHandler, UPDATE_BIT); //设置UPDATE_BIT模式标志位
						}
						else
						{
							value=3;
							esp_sleep_enable_timer_wakeup(1000000); //设置唤醒时间(采样周期所使用的时间与图片刷新时间综合考虑此唤醒时间的设置)
							start_ulp();
							printf("enter deep sleep\n");
							esp_deep_sleep_start();                 ////进入深睡模式
						}
					}
				}
            }
            else if (EventValue & UPDATE_BIT)
            {
                printf("update mode run.....\r\n ");
                vTaskDelay(2000 / portTICK_RATE_MS);
                printf("from UPDATE mode enter deepsleep\r\n ");
                printf("enter deep sleep\n");
                esp_deep_sleep_start();
            }
            else if (EventValue & RESPONSE_BIT)
            {
                printf("RESPONSE mode run.....\r\n ");
                vTaskDelay(2000 / portTICK_RATE_MS);
                if(timeinfo.tm_hour==23)
				{
					printf("i am in Display mode,but it time is to go in Update mode \n");
					xEventGroupSetBits(EventGroupHandler, UPDATE_BIT); //设置UPDATE_BIT模式标志位
				}
				else
				{
					if (ulp_move_flags == 0) //ulp状态 未动
					{
//						loop_display_picture();
					}
					else if(ulp_move_flags == 1)
					{
//						wifi_init_sta();
						GattServers_Init();
						esp_timer_start_periodic(periodic_timer, 7000 * 1000);
					}
					getcurrenttime();
					if(timeinfo.tm_hour==23)
					{
						printf("i am in Display mode,but it time is to go in Update mode \n");
						xEventGroupSetBits(EventGroupHandler, UPDATE_BIT); //设置UPDATE_BIT模式标志位
					}
					else
					{
						value=5;
						esp_sleep_enable_timer_wakeup(30000000); //30s
						start_ulp();
						printf("enter deep sleep\n");
						esp_deep_sleep_start();                 ////进入深睡模式
					}
				}
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_RATE_MS); //延时10ms，也就是10个时钟节拍
        }
    }
}
//获取当前系统时间
void getcurrenttime(void)
{
    time(&now);
    tzset();
    localtime_r(&now, &timeinfo);
//    printf("hours=%d ,minute=%d ,second=%d \n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    vTaskDelay(1000 / portTICK_RATE_MS);
}
//写入数据到内存,测试时使用
static void write_modetoflash(void)
{
    unsigned char run_mode_data[48] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4};
    unsigned char demo_mode_data[48] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4};
    spi_flash_erase_sector(1500); //将数据要写入的数据清空
    spi_flash_write(1500 * 4096, (uint8_t *)&run_mode_data, 48);
    spi_flash_write(1500 * 4096 + 200, (uint8_t *)&demo_mode_data, 48);
    spi_flash_write(1500 * 4096 + 300, (uint8_t *)&current_mode, 1);
}
//在API指南中 深度睡眠唤醒存根章节中 有此函数的说明
//唤醒后需要执行的代码
void RTC_IRAM_ATTR esp_wake_deep_sleep(void)
{
    esp_default_wake_deep_sleep();
	static RTC_RODATA_ATTR const char t_mode_vlaue[] = "t_mode.status=%d\n";
    //以30分钟为一个时间段，进行判断
    if (timeinfo.tm_min > 30)
    {
        t_mode.status = mode_union[timeinfo.tm_hour + 1];
        ets_printf(t_mode_vlaue,t_mode.status);
    }
    else if (timeinfo.tm_min <= 30)
    {
        t_mode.status = mode_union[timeinfo.tm_hour];
        ets_printf(t_mode_vlaue,t_mode.status);
    }
}
