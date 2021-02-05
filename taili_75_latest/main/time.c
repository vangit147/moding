//锟斤拷锟剿拷锟绞憋拷锟斤拷锟斤拷锟�6uA,锟斤拷锟斤拷wifi锟斤拷始锟斤拷之锟斤拷锟斤拷锟斤拷锟�81.7ua
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "calendar.h"

esp_timer_handle_t periodic_timer;
static const char *timer_tag = "timer";

static void periodic_timer_callback(void *arg)
{
    esp_timer_stop(periodic_timer);
    int64_t  time_run = esp_timer_get_time()/1000000;
    ESP_LOGE(timer_tag,"start deep sleep: %lld s", time_run);
    ESP_LOGW(timer_tag,"Perform function: sleep_for_next_wakeup()");
    sleep_for_next_wakeup();
}

void Timer_Config(void)
{
	const esp_timer_create_args_t periodic_timer_args = {
	        .callback = &periodic_timer_callback,
	        .name = "periodic"};
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_LOGW(timer_tag,"set five minutes");
    esp_timer_start_periodic(periodic_timer,3*1000*1000);
}

