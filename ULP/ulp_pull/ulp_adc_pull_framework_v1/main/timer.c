#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
extern EventGroupHandle_t EventGroupHandler;
esp_timer_handle_t periodic_timer;
extern int TIMER_BIT;
extern short aacx, aacy, aacz;
extern float AccX, AccY, AccZ;
static void periodic_timer_callback(void *arg)
{
    // BaseType_t xHigherPriorityTaskWoken;
    // xEventGroupSetBitsFromISR(EventGroupHandler, TIMER_BIT, &xHigherPriorityTaskWoken); //定时器时间到标志位
    /*如果需要查看调试信息可把以下代码打开，开启定时器定时查看*/
    // printf("moving_times=%x,sysrun_times=%x,move_flags=%x\r\n", ulp_moving_times, ulp_sysrun_times, ulp_move_flags);
    // aacx = ((unsigned short)ulp_ac << 8) | ulp_acl;
    // aacz = ((unsigned short)ulp_az << 8) | ulp_azl;
    // AccX = aacx / 16384.0; // X-axis value
    // AccZ = aacz / 16384.0;
    // printf("the aacx is %d \t", aacx);
    // printf("the aacz is %d  \t", aacz);
    // printf("the aacx is %lf  \t", AccX);
    // printf("the aacz is %lf  \n", AccZ);
	printf("close wifi and ble\n");
//	loop_display_picture();
}

//创建周期定时器
void Timer_Config(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    esp_timer_start_periodic(periodic_timer, 3000 * 1000);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(3000000)); //设置睡眠唤醒时间  3s，调试使用
                                                             // esp_timer_start_periodic(periodic_timer2, 8000 * 1000);  //设置定时器的定时周期为100ms
}
