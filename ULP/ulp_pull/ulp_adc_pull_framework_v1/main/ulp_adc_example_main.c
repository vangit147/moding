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
#include "esp_gap_ble_api.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "bleinit.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "time.h"
#include "esp_spi_flash.h"
#include "spiff.h"
#include "aenc.h"
RTC_DATA_ATTR unsigned char current_mode = 0; //0-runmode 1-demomode
//RTC_DATA_ATTR unsigned char Hiber_time = 0;   //H状态标志位设置 1设置，0未设置
typedef enum
{
    Run_Mode = 0,
    Demo_Mode
}MODE;
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
}Mode_t;
RTC_DATA_ATTR struct Mode_t t_mode; //将结构体放入RTC内存中
RTC_DATA_ATTR struct timeval stime;
RTC_DATA_ATTR struct tm timeinfo;                 //时间变量结构体
RTC_DATA_ATTR time_t now;
RTC_DATA_ATTR char strftime_buf[64];

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

#define number_of_time_periods_in_a_day 48
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

extern int settimeofday (const struct timeval *, const struct timezone *);

extern void ncolor_display(uint16_t index);
extern esp_c_t  CN_Init(void);

void sleep_and_storage_time();
void init_flag_bit();
void set_flag_bit(int i);
void read_data_from_1279_sector();
void write_data_to_1279_sector();
void write_data_to_1280_sector();
void read_data_from_1280_sector();
void display_data_from_1280_sector();
void set_time();
void start_ulp();
void sleep_time(unsigned char mode);
void hibernation();
void display(int mode);
void update();
void int_to_string(long value, char * output);
void storage_current_time_to_flash();
void set_mode();
unsigned char temp_read_from_1280_flash[15];
unsigned char acount_battery=0;//更换电池的次数

void app_main()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
//    esp_pm_config_esp32_t pm_config ={
//    		.max_freq_mhz = CONFIG_MAX_CPU_FREQ_MHZ,
//			.min_freq_mhz = CONFIG_MIN_CPU_FREQ_MHZ,
//			.light_sleep_enable = true;
//    };
    //开机时判断当前的运行模式为演示模式还是运行模式,当深度唤醒时,初始化的函数不会再次执行,会进入
    //esp_wake_deep_sleep执行代码

//    spi_flash_erase_sector(info_page);
//    spi_flash_erase_sector(info_page_temp);
//    printf("enter deep sleep\n");
//    esp_deep_sleep_start();

    read_data_from_1279_sector();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("cause=%d\n",cause);
    if(cause!=ESP_SLEEP_WAKEUP_UNDEFINED)
    {
    	set_mode();
    }
    switch(cause)
    {
    	case ESP_SLEEP_WAKEUP_TIMER:
    	{
    		if(t_mode.status==Hibernation)
    		{
    			if(flag_data.flag_bit[0]==0)
    			{
    				printf("need set wakeup time in Hibernation\n");
    			}
    			else if(flag_data.flag_bit[0]==1)
    			{
    				printf("has been set wakeup time in Hibernation \n");
    				sleep_and_storage_time();
    			}
    		}
    		else if(t_mode.status==Display)
    		{
    			if(flag_data.flag_bit[1]==0)
    			{
    				printf("need set ULP in Display\n");
    				start_ulp();
    				esp_sleep_enable_timer_wakeup(180000000);//3 min
//    				sleep_time(2);
    				set_flag_bit(1);
    				write_data_to_1279_sector();
    				sleep_and_storage_time();
    			}
    		}
    		else if(t_mode.status==Update)
    		{
    			if(flag_data.flag_bit[2]==0)
    			{
    				printf("wait for downloading picture in Update\n");
    			}
    		}
			break;
    	}
    	case ESP_SLEEP_WAKEUP_ULP:
    	{
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
    	}
    	case ESP_SLEEP_WAKEUP_UNDEFINED:
    	{
    		set_time();
    		init_flag_bit();
    		read_data_from_1280_sector();
    		if(acount_battery!=0xff)
			{
    			display_data_from_1280_sector();
    			if((int)acount_battery!=255)
    			{
    				(int)acount_battery++;
    			}
    			else
    			{
    				acount_battery=0;
    			}
			}
    		else
    		{
    			acount_battery=0;
    		}
    		write_data_to_1280_sector();
			spi_flash_read(info_page * 4096, &acount_battery,1);
			printf("acount_battery=%d\n",acount_battery);

    		write_modetoflash();
			spi_flash_read(1500 * 4096 + 300, &current_mode, 1);
			printf("current_mode=%d\n",current_mode);
			if (current_mode == 0)
			{
				printf("current mode is Run_mode\r\n");
				t_mode.mode = Run_Mode; //运行模式
				spi_flash_read(1500 * 4096, (uint8_t *)mode_union, 48);
				for(unsigned char i=0;i<48;i++)
				{
					if(i%2==0)
					{
						printf(" ");
					}
					printf("%d",mode_union[i]);
					if(i==23||i==47)
					{
						printf("\n");
					}
				}
			}
			else if (current_mode == 1)
			{
				printf("current mode is Demo_mode\r\n");
				t_mode.mode = Demo_Mode; //演示模式
				spi_flash_read(1500 * 4096 + 200, (uint8_t *)mode_union, 48);
				printf("%d	",mode_union[0]);
			}
			set_mode();

			esp_sleep_enable_timer_wakeup(1000000);
    		sleep_and_storage_time();                 ////进入深睡模式
    		break;
    	}
    	default:
			printf("other wakeup cause ----> deep sleep start");
			sleep_and_storage_time();
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

//	 sleep_and_storage_time();
	 printf("wait wait....\n");
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
                hibernation();
                sleep_and_storage_time();
            }
            else if (EventValue & DISPLAY_BIT) //Display 模式
            {
                printf("display Task\r\n");
                getcurrenttime();
                CN_Init();
				printf("CN_Init done\n");
				display(0);
				getcurrenttime();
				printf("set timer and ulp before entering deepsleep\n");
				printf("ulp_move_flags=%d\n",ulp_move_flags&0xff);
				if ((ulp_move_flags&0xff) == 1) //ulp状态 动
				{
					esp_sleep_enable_timer_wakeup(180000000);//3 min
				}
				else if((ulp_move_flags&0xff) == 0)
				{
					esp_sleep_enable_timer_wakeup(180000000);//3 min
//					sleep_time(2);
				}
				start_ulp();
				sleep_and_storage_time();
            }
            else if (EventValue & UPDATE_BIT)
            {
                printf("update mode run.....\r\n ");
                vTaskDelay(2000 / portTICK_RATE_MS);
                printf("from UPDATE mode enter deepsleep\r\n ");
                update();
                sleep_and_storage_time();
            }
            else if (EventValue & RESPONSE_BIT)
            {
                printf("RESPONSE mode run.....\r\n ");
                vTaskDelay(2000 / portTICK_RATE_MS);
                sleep_and_storage_time();               ////进入深睡模式
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_RATE_MS); //延时10ms，也就是10个时钟节拍
        }
    }
}

void sleep_and_storage_time()
{
	printf("in sleep_and_storage_time\n");
	storage_current_time_to_flash();
	write_data_to_1279_sector();
	read_data_from_1279_sector();

	read_data_from_1280_sector();
	memcpy(&flag_data_backup[acount_battery].before_sleep_time,&flag_data.before_sleep_time,13);
	memcpy(&flag_data_backup[acount_battery].start_test_time,&flag_data.start_test_time,13);
	printf("flag_data_backup[%d].before_sleep_time = %s\n",acount_battery,flag_data_backup[acount_battery].before_sleep_time);
	printf("flag_data_backup[%d].start_test_time   = %s\n",acount_battery,flag_data_backup[acount_battery].start_test_time);
	write_data_to_1280_sector();
	display_data_from_1280_sector();

	printf("enter deep sleep \n");
	printf("out sleep_and_storage_time\n");
	esp_deep_sleep_start();
}
void init_flag_bit()
{
	memset(&flag_data,0,sizeof(flag_data));
	flag_data.flag_bit[0]=0;//set wakeup time in Hibrenation
	flag_data.flag_bit[1]=0;//set ULP in Display
	flag_data.flag_bit[2]=0;//picture downlaod done
	flag_data.flag_bit[3]=0;//loop display by 0/1
	storage_current_time_to_flash();
	memcpy(&flag_data.start_test_time,flag_data.before_sleep_time,13);
	write_data_to_1279_sector();
	read_data_from_1279_sector();
}

void set_flag_bit(int i)
{
	//j = 0 set wakeup time in Hibrenation
	//j = 1 set ULP in Display
	//j = 2 picture downlaod done
	for(int j=0;j<3;j++)
	{
		if(j==i)
		{
			flag_data.flag_bit[j]=1;
		}
		else
		{
			flag_data.flag_bit[j]=0;
		}
	}
	if(i==3)
	{
		flag_data.flag_bit[3]=!flag_data.flag_bit[3];//loop display by 0/1
	}
}

void read_data_from_1279_sector()
{
	//check data if storage in flash 1279 sector
	printf("read_data_from_1279_sector\n");
	memset(&flag_data,0,sizeof(flag_data));
	spi_flash_read(info_page_temp * 4096, &flag_data, sizeof(flag_data));
	for(unsigned char i=0;i<4;i++)
	{
		printf("flag_data.flag_bit[%d]=%d\n",i,flag_data.flag_bit[i]);
	}
	if(flag_data.before_sleep_time[0]!=0xff&&flag_data.start_test_time[0]!=0xff)
	{
		printf("flag_data.before_sleep_time = %s\n",flag_data.before_sleep_time);
		printf("flag_data.start_test_time   = %s\n",flag_data.start_test_time);
	}
	else
	{
		printf("no save time \n");
	}
}

void write_data_to_1279_sector()
{
	spi_flash_erase_sector(info_page_temp);//info_page_temp 1279
	spi_flash_write(info_page_temp * 4096, &flag_data, sizeof(flag_data));
}

void write_data_to_1280_sector()
{
	spi_flash_erase_sector(info_page);//info_page_temp 1279
	spi_flash_write(info_page * 4096,&acount_battery,1);
	spi_flash_write(info_page * 4096+1, &flag_data_backup, sizeof(flag_data_backup));
}

void read_data_from_1280_sector()
{
	printf("read_data_from_1280_sector\n");
	memset(&flag_data_backup,0,sizeof(flag_data_backup));
	spi_flash_read(info_page * 4096, &acount_battery,1);
	printf("acount_battery=%d  sizeof(flag_data_backup)=%d\n",acount_battery,sizeof(flag_data_backup));
	spi_flash_read(info_page * 4096+1, &flag_data_backup,sizeof(flag_data_backup));
}
void display_data_from_1280_sector()
{
	for(unsigned char i=0;i<=acount_battery;i++)
	{
		if(flag_data_backup[i].before_sleep_time[0]!=0xff&&flag_data_backup[i].start_test_time[0]!=0xff)
		{
			printf("Current battery replacement times : %d \n",i+1);
			printf("flag_data_backup[%d].before_sleep_time=%s\n",i,flag_data_backup[i].before_sleep_time);
			printf("flag_data_backup[%d].start_test_time=%s\n",i,flag_data_backup[i].start_test_time);
		}
		else
		{
			printf("%d no data in flash\n",i+1);
		}
	}
}

void set_time()
{
//	1609451940 2021-01-01 05:59:00
//  1609430400 2021-01-01 00:00:00
//	1609450200 2021-01-01 05:30:00
	stime.tv_sec=1609450200;
	settimeofday(&stime,NULL);
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
    ulp_p_az = 15000;
    ulp_m_ax = 50;
    ulp_m_az = 0;
    ulp_compare_times = 10; //设置ulp的比较运行次数
    //采样周期为10的话，一个周期执行完大概需要25s
    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 1000 * 1000);
}

static void start_ulp_program()
{
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void start_ulp()
{
	printf("start ULP\n");
	init_ulp_program();                             //初始化ulp
	start_ulp_program();                            //启动ulp
	ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup()); //设置允许ulp唤醒CPU
}

void sleep_time(unsigned char mode)
{
	uint64_t sleep_time;
	int remain_time;
	int number_of_time_periods=0;
	unsigned char i;
	getcurrenttime();
	i = timeinfo.tm_hour;
	if(timeinfo.tm_min >= 30)
	{
		i = timeinfo.tm_hour*2 + 1;
		remain_time=60-timeinfo.tm_min;
	}
	else
	{
		i = timeinfo.tm_hour*2;
		remain_time=30-timeinfo.tm_min;
	}
	printf("remain_time=%d\n",remain_time);
	for(unsigned char j=0;j<number_of_time_periods_in_a_day;j++)
	{
		if(++i==47)
		{
			i=0;
		}
		if(mode_union[i]==mode)
		{
			number_of_time_periods++;
		}
		else
		{
			printf("number_of_time_periods=%d mode=%d\n",number_of_time_periods,mode);
			break;
		}

	}
	unsigned char temp = 60/(number_of_time_periods_in_a_day/24);
	int min = number_of_time_periods*temp+remain_time;
	printf("wakeup in %d s (%d h %d min)\n",min*60,min/60,min);
	sleep_time = (uint64_t)min*60*1000*1000;
	esp_sleep_enable_timer_wakeup(sleep_time);
}

void hibernation()
{
	sleep_time(1);
	set_flag_bit(0);
	printf("**********\n");
}

void display(int mode)
{
	if(mode==1)
	{
		printf("display_picture_mode_1\n");
	}
	else if(mode==0)
	{
		printf("display_picture_mode_0\n");
//		CN_Init();
//		vTaskDelay(2000 / portTICK_RATE_MS);
//		printf("CN_Init done\n");
		if(flag_data.flag_bit[3]==1)
		{
			ncolor_display(1);
			set_flag_bit(3);
			set_flag_bit(1);
			write_data_to_1279_sector();
		}
		else if(flag_data.flag_bit[3]==0)
		{
			ncolor_display(0);
			set_flag_bit(3);
			set_flag_bit(1);
			write_data_to_1279_sector();
		}
	}
}

void update()
{
	//http_download_pic_done
	sleep_time(3);
	set_flag_bit(2);
	write_data_to_1279_sector();
	//http_download_pic_failed
	esp_sleep_enable_timer_wakeup(180000000);//3 min
}

void int_to_string(long value, char * output)
{
    int index = 0;
    char temp;
    if(value == 0)
    {
    	output[0] = value + '0';
    }
    else
    {
        while(value)
        {
        	output[index] = value % 10 + '0';
            index ++;
            value /= 10;
        }
    }
    for(unsigned char i=0;i<index/2;i++)
    {
    	temp=output[i];
    	output[i]=output[index-i-1];
    	output[index-i-1]=temp;
    }
}

void storage_current_time_to_flash()
{
	getcurrenttime();
	int date[5];
	unsigned char temp_time[13];
	date[0]=timeinfo.tm_year+1900;
	date[1]=timeinfo.tm_mon+1;
	date[2]=timeinfo.tm_mday;
	date[3]=timeinfo.tm_hour;
	date[4]=timeinfo.tm_min;
	memset(temp_time,0,sizeof(temp_time));
	int_to_string(date[0],(char*)temp_time);
	if(date[1]<10)
	{
		temp_time[4]='0';
		int_to_string(date[1],(char*)&temp_time[5]);
	}
	else
	{
		int_to_string(date[2],(char*)&temp_time[4]);
	}
	if(date[2]<10)
	{
		temp_time[6]='0';
		int_to_string(date[2],(char*)&temp_time[7]);
	}
	else
	{
		int_to_string(date[2],(char*)&temp_time[6]);
	}
	if(date[3]<10)
	{
		temp_time[8]='0';
		int_to_string(date[3],(char*)&temp_time[9]);
	}
	else
	{
		int_to_string(date[3],(char*)&temp_time[8]);
	}
	if(date[4]<10)
	{
		temp_time[10]='0';
		int_to_string(date[4],(char*)&temp_time[11]);
	}
	else
	{
		int_to_string(date[4],(char*)&temp_time[10]);
	}
	memcpy(&flag_data.before_sleep_time,temp_time,13);
}

//获取当前系统时间
void getcurrenttime(void)
{
	time(&now);
	// Set timezone to China Standard Time
	setenv("TZ", "CST-8", 1);
	tzset();
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	printf("The current date/time in Shanghai is: %s\n", strftime_buf);
}

//写入数据到内存,测试时使用
static void write_modetoflash(void)
{
    unsigned char run_mode_data[48] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1};
    unsigned char demo_mode_data[48] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4};
    spi_flash_erase_sector(1500); //将数据要写入的数据清空
    spi_flash_write(1500 * 4096, (uint8_t *)run_mode_data, 48);
    spi_flash_write(1500 * 4096 + 200, (uint8_t *)demo_mode_data, 48);
    spi_flash_write(1500 * 4096 + 300, (uint8_t *)&current_mode, 1);
    spi_flash_read(1500 * 4096 + 300, &current_mode, 1);
}
//在API指南中 深度睡眠唤醒存根章节中 有此函数的说明
//唤醒后需要执行的代码
void RTC_IRAM_ATTR esp_wake_deep_sleep(void)
{
    esp_default_wake_deep_sleep();
	static RTC_RODATA_ATTR const char t_mode_vlaue[] = "t_mode.status=%d\n";
	static RTC_RODATA_ATTR const char str1[] = "%d";
	static RTC_RODATA_ATTR const char str2[] = " ";
	static RTC_RODATA_ATTR const char str3[] = "\n";
    //以30分钟为一个时间段，进行判断
	 for(unsigned char i=0;i<48;i++)
	{
		if(i%2==0)
		{
			 ets_printf(str2);
		}
		ets_printf(str1,mode_union[i]);
		if(i==23||i==47)
		{
			 ets_printf(str3);
		}
	}
	ets_printf(t_mode_vlaue,timeinfo.tm_hour);
    if (timeinfo.tm_min >= 30)
    {
        t_mode.status = mode_union[timeinfo.tm_hour*2 + 1];
        ets_printf(t_mode_vlaue,t_mode.status);
    }
    else if (timeinfo.tm_min < 30)
    {
        t_mode.status = mode_union[timeinfo.tm_hour*2];
        ets_printf(t_mode_vlaue,t_mode.status);
    }
}
void set_mode()
{
	getcurrenttime();
	if (timeinfo.tm_min >= 30)
	{
		t_mode.status = mode_union[timeinfo.tm_hour*2 + 1];
		printf("t_mode.status=%d\n",t_mode.status);
	}
	else if (timeinfo.tm_min < 30)
	{
		t_mode.status = mode_union[timeinfo.tm_hour*2];
		printf("t_mode.status=%d\n",t_mode.status);
	}
}

