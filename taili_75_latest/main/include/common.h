#ifndef _COMMON_H_
#define	_COMMON_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys/time.h"
#include "esp_log.h"
#include "gatts_server.h"
#include "spiff.h"
#include "http_client.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_gap_ble_api.h"


#include "cJSON.h"
#include "esp_spi_flash.h"

#define sector_size 4096

struct tm *p;
time_t time_now;
struct timeval stime;


time_t now;
char strftime_buf[64];
struct tm timeinfo;



extern void display_picture(char *buffer);
extern void display_picture_temp(int display_index,int picture_page_index);
extern void ncolor_display(uint16_t index,unsigned char pic_data4);



struct my_data
{
	/*
	 * default value :
	 * 			check_head[0] = 0x55
	 * 			check_head[1] = 0x56;
	 */
	unsigned char check_head[2];
	char pic_number;
	/*
	 * low_power_state:
	 * 			-1	没有此图片
	 * 			0	存在此图片
	 */
	char low_power_state;
	/*
	 * pic_name : xy01.bin
	 * 			x : x > 2021
	 * 			y : 01~10 11~12
	 * examples:20210101.bin 20301201.bin
	 */
	char pic_name[20];
	/*
	 * pic_name_current : xyz.bin
	 * 			x : x > 2021
	 * 			y : 01~10 11~12
	 * 			z : 1~31
	 * examples : 20210123.bin 20401215.bin
	 */
	char pic_name_current[20];
	char wifi_ssid[40];
	char wifi_pssd[40];
	char server_add_get_down_pic[50];
	/*
	 * network_wrong_state :
	 * 			-1	没有此图片
	 * 			0	存在此图片
	 */
	char network_wrong_state;
	/*
	 * config_wifi_state :
	 * 			0	没有此图片
	 * 			1	存在此图片
	 */
	char config_wifi_state;
	char server_add_tell_down_ok[50];
	char server_add_tell_dele_ok[50];
	char server_add_to_downlo_pic[50];
	/*
	 * default value :
	 * 			check_tail[0] = 0x55
	 * 			check_tail[1] = 0x56;
	 */
	unsigned char check_tail[2];
	long time_stamp;
	long picture_time_stamp;
	/*
	 *      			  total space : 16M
	 * 			         program size : 4M
	 *       write data start address : 5M
	 * 	          picture_information : 1280 sector and 1281 sector
	 * low_network_wificonfig picture : 3*picture_capactiy  1282 sector
	 *  		              picture : 1283 sector
	 * 			         remain_space : (16-5)M-picture_number*picture_capactiy
	 */
	long remain_space;
};
struct my_data current_data;

void cJSON_data(char *json_str);

void int_to_string(long value, char * output);

int  string_to_int(char * string,int index);

#endif
