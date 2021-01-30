//深度睡眠时电流是6uA,蓝牙wifi初始化之后电流是81.7ua

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
/*************************/
//update van pull_ring
#include "spiff.h"
#include "time.h"
#include "esp_spi_flash.h"
#include "display_pic.h"
/************************/
#define MY_TAG	"moding"

extern struct timeval stime;
extern struct tm *p;
extern time_t time_now;
extern int time_count;
//static const char *TAG = "timer";

// esp_timer_handle_t periodic_timer2;
//100ms 定时器回调函数
//static void periodic_timer_callback2(void *arg)
//{
//	printf("100ms timer interrupt:%d\r\n", time_count);
//	printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
//	esp_deep_sleep_start();
//}

//update van
esp_timer_handle_t periodic_timer;
extern unsigned char flag[4096];
extern unsigned char pic_is_loop_display;
//定时器回调函数
static void periodic_timer_callback(void *arg)
{
    time_count++;
    gettimeofday(&stime,NULL);
	time_now=stime.tv_sec;
	p=localtime(&time_now);
	printf("/*****************************/\n");
	printf("%d/%d/%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday);
	printf("%d  %d:%d:%d\n",p->tm_wday,(p->tm_hour+8),p->tm_min,p->tm_sec);
    printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
    time_count = 0;
    esp_timer_stop(periodic_timer);//关闭定时器
    esp_deep_sleep_start();
    /*********************************/
    //update van pull_ring
    //esp_deep_sleep_start();
    /*********************************/
    //esp_light_sleep_start();
}
/*******************************/


/*******************************/
//update van pull_ring
esp_timer_handle_t periodic_timer_moding;
#ifndef on_off
static void periodic_timer_callback_moding(void *arg)
{

	//打开定时器 若5秒内没有wifi要连接，则查看是否内存中有图片若有则10秒轮询显示，若没有图片，则开启5秒后休眠的定时器
    time_count++;
    printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
    time_count = 0;
    esp_timer_stop(periodic_timer_moding); //关闭定时器;
    esp_deep_sleep_start();
//    spi_flash_read(info_page * 4096,&test_pic_num, sizeof(unsigned char));
//    ESP_LOGW(MY_TAG,"test_pic_num=%d\n",test_pic_num);
//    if(test_pic_num==2)
//    {
//    	moding_pull_ring_display_4();
//    }
//    else if(test_pic_num==1)
//    {
//    	moding_pull_ring_display_3();
//    }
//    else
//    {
//    	printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
//    	esp_deep_sleep_start();
//    }
}
#endif
/*******************************/

//创建周期定时器
void Timer_Config(void)
{
//    const esp_timer_create_args_t periodic_timer_args2 = {
//         .callback = &periodic_timer_callback2,
//         .name = "periodic2"};
//    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args2, &periodic_timer2));
//    esp_timer_start_periodic(periodic_timer2, 8000 * 1000);  //设置定时器的定时周期为100ms
//    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(5000000)); //设置睡眠唤醒时间

	//update van
	const esp_timer_create_args_t periodic_timer_args = {
	        .callback = &periodic_timer_callback,
	        .name = "periodic"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
#ifdef on_off
    spi_flash_read(info_page * 4096,&flag,4096);
    if(flag[1]==0xaa&&flag[5]==0xaa)
    {
    	if(flag[2]==0x11)
    	{
    		ESP_LOGW(MY_TAG,"flag[3]=%x flag[4]=%x",flag[3],flag[4]);
    		ESP_LOGW(MY_TAG,"flag[3]=%d flag[4]=%d",flag[3],flag[4]);
//			if(pic_is_loop_display==1)
//			{
//				flag[4]=0x04;
//				ESP_LOGW(MY_TAG,"flag[3]=%x flag[4]=%x",flag[3],flag[4]);
//				ESP_LOGW(MY_TAG,"flag[3]=%d flag[4]=%d",flag[3],flag[4]);
//			}
    		gettimeofday(&stime,NULL);
			time_now=stime.tv_sec;
			p=localtime(&time_now);
			printf("/*****************************/\n");
			ESP_LOGW(MY_TAG,"%ld",time_now);
			ESP_LOGW(MY_TAG,"%d/%d/%d %s %d:%d:%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,day[p->tm_wday],(p->tm_hour+8),p->tm_min,p->tm_sec);
			ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(flag[3]*1000*1000)); //睡眠唤醒时间为90s(自定义)
			printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
			esp_timer_start_periodic(periodic_timer, flag[4]*1000 * 1000);//设置并打开定时器10s(自定义)
    	}
    	else
    	{
    		ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(5*1000*1000)); //睡眠唤醒时间为10秒
    		esp_timer_start_periodic(periodic_timer, 15*1000 * 1000);//设置并打开定时器5s
    	}
    }
#else
    const esp_timer_create_args_t periodic_timer_args_moding = {
       				.callback = &periodic_timer_callback_moding,
       				.name = "periodic_moding"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_moding, &periodic_timer_moding));
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(15*1000*1000)); //睡眠唤醒时间为15秒
#endif
    /*******************************************************/
}

