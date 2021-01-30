/* Deep sleep wake up example

 分区16M  14M用于存储 2M用于APP和OTA

 SPI-flash分区表如下： 前36K系统使用分区表存放在0x8000处

nvs,      data, nvs,     0x9000,  0x4000,   16K

otadata,  data, ota,     0xd000,  0x2000,   8K

phy_init, data, phy,     0xf000,  0x1000,   4K

ota_0,    0,    ota_0,   0x10000,  1M,      1M  --要保证地址与0x10000对齐多以分配1M

ota_1,    0,    ota_1,   0x110000, 0xF0000   960K

共 2M 其余14M用于存储

*/



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "soc/sens_periph.h"
#include "soc/rtc.h"
/********************************/
//update van pull_ring
#include  "spiff.h"
/********************************/


void moding_sleep()
{
	ESP_LOGI("MODING_SLEEP", "esp_sleep_enable_timer_wakeup  START1");
	vTaskDelay(1*1000 / portTICK_PERIOD_MS);
	ESP_LOGI("MODING_SLEEP", "esp_sleep_enable_timer_wakeup  START2");
	esp_sleep_enable_timer_wakeup(5*1000000);
	esp_deep_sleep_start();
}
