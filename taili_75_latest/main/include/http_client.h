#ifndef __HTTP_CLIENT_H__
#define __HTTP_CLIENT_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "calendar.h"

char wakeup_cause;

//char BUFFER_temp[15][4096];
char BUFFER[4096];
char PICNAME[20];
unsigned char picture_index;


esp_http_client_handle_t client;

esp_err_t _http_event_handler(esp_http_client_event_t *evt);

int http_test_task(char *dpwn_url);

void search_dis_pic();

typedef uint8_t   DK_T;
extern void Hal_UpGraghScreen3();
extern void  Write_CT(DK_T value);
extern void DK_ByT(void);
extern void DK_RTflesh(void);


#endif
