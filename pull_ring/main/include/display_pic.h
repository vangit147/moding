#ifndef _DISPLAY_PIC_H_
#define _DISPLAY_PIC_H_
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "string.h"
#include "spiff.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "time.h"
#include "esp_wifi.h"




extern esp_timer_handle_t periodic_timer;
extern unsigned char picture_num; //Í¼Æ¬ÊýÁ¿
extern unsigned char pull_ring[4096];
extern char temp_file_name[40];
extern void Acep_loadPIC1_init();
extern void Acep_loadPIC2_init();
extern void Acep_loadPIC1_end();
extern void Acep_loadPIC2_end();
extern void ncolor_display(uint16_t index);
extern void ncolor_display_diff(uint16_t index);
extern void Acep_loadPIC1_test(unsigned char* pic_data3,int max_data);
extern void Acep_loadPIC2_test(unsigned char* pic_data4,int max_data);


/********************************/
//update van pull_ring
#define info_reboot_times 1279


#define on_off	1

unsigned char picture_num_temp;
#define MY_TAG	"moding"
char *day[7];

extern struct timeval stime;
extern struct tm *p;
extern time_t time_now;
/*******************************/

int hightolower(int c);
int lowertohigh(int c);
int hextoint(char s[]);
void update_set_time(long time_now);
void erase_write_read_timer(int * a);
int search_in_flash(char * pic_name,unsigned char revceive_data_from_blue,int pic_size);
void display(char pic_name[32],unsigned char temp,int pic_size);
void display_Acep_loadPIC1(int num);
void display_Acep_loadPIC2(int num);
void moding_pull_ring_display_4();
void moding_pull_ring_display_3();
void loop_display_picture(unsigned char receive_4);

#endif
