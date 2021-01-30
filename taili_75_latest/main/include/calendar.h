#ifndef _CALENDAR_H_
#define	_CALENDAR_H_

#include "common.h"


#define AND '&'
#define QUES '?'

extern unsigned char isconnected;
extern esp_ble_adv_data_t adv_data;
extern unsigned char device_info[26];
extern esp_timer_handle_t periodic_timer;
int64_t time_account;


cJSON * item;
unsigned char failed_bit;
unsigned char failed_times;
unsigned char wifi_config_page;
char URL_download[200];

void find_wakeup_cause();

void sleep_for_next_wakeup();

void read_write_init();

void set_time_to_wakeup_by_timer(int hour,int min);

void init_default_data();

void analysis_data();

void updated_esp_time();

void updated_data_to_flash();

void check_wifi_httpdownload_pic(char wakeup_cause);

void download_pic_url_composite(cJSON * item);


void download_pic_finished_url_composite(cJSON * item,char *url_temp);

void delete_pic_finished_url_composite(cJSON * item);

void charconnectuchar(char a[],unsigned char b[]);

void mac_to_device_info();

#endif
