#include "calendar.h"

static const char *my_tag = "calender";
char *deletepic="deletepic=";
char *picname="picname=";
char *picsize="picsize=";
char *Voltage="voltage=";
char *action="action=";
char *timestamp="timestamp=";
unsigned char wifi_config_page=0;
char default_wifi_ssid[40]="van";
char default_wifi_pssd[40]="1234567";
long default_time_stamp=1609430400;// 2021/1/1  00:00:00
unsigned char failed_bit=0;
unsigned char failed_times=0;
extern void Timer_Config(void);

void find_wakeup_cause()
{
	ESP_LOGW(my_tag,"find wakeup_cause");

	printf("Wake up  %d\n",esp_sleep_get_wakeup_cause());
	switch (esp_sleep_get_wakeup_cause())
	{
		case ESP_SLEEP_WAKEUP_EXT1:
		{
			uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
			int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			printf("Wake up from GPIO %d\n", pin);
			if(pin == 14)
			{
				check_wifi_httpdownload_pic('2');//next page
			}
			else if(pin == 25)
			{
				check_wifi_httpdownload_pic('0');//auto_update
			}
			else
			{
				sleep_for_next_wakeup();
			}
			break;
		}
		case ESP_SLEEP_WAKEUP_EXT0:
			check_wifi_httpdownload_pic('1');//prev page
			break;
		case ESP_SLEEP_WAKEUP_TIMER:
			check_wifi_httpdownload_pic('0');//auto_update by timer
			break;
		case ESP_SLEEP_WAKEUP_UNDEFINED:
			if(isconnected==0)
			{
				ESP_LOGW(my_tag,"Timer_Config()");
				Timer_Config();

				ESP_LOGW(my_tag,"gattserver(ble) init");
				GattServers_Init();
//				display picture network wrong
//				display_low_network_wifi_picture(1);
			}
			break;
		default:
			ESP_LOGW(my_tag,"other wakeup cause ----> deep sleep start");
			sleep_for_next_wakeup();
			break;

	}
}


void sleep_for_next_wakeup()
{
	esp_sleep_enable_ext0_wakeup(0ULL<<0x00, 0);

	const int ext_wakeup_pin_1 = 25;
	const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
	const int ext_wakeup_pin_2 = 14;
	const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

	printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
	esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

	ESP_LOGW(my_tag,"Go to sleep immediately");
//	set_time_to_wakeup_by_timer(17,59);
	esp_deep_sleep_start();
}

void read_write_init()
{
	memset(&current_data,0,sizeof(current_data));
	spi_flash_read(info_page*sector_size,&current_data,sizeof(current_data));
	if(current_data.check_head[0]==0x55&&current_data.check_head[1]==0x56&&current_data.check_tail[0]==0x55&&current_data.check_tail[1]==0x56)
	{
		 ESP_LOGW(my_tag, "Start Interpreting data");
	}
	else
	{
		memset(&current_data,0,sizeof(current_data));
		spi_flash_read(info_page_backup*sector_size,&current_data,sizeof(current_data));
		if(current_data.check_head[0]==0x55&&current_data.check_head[1]==0x56&&current_data.check_tail[0]==0x55&&current_data.check_tail[1]==0x56)
		{
			spi_flash_write(info_page*sector_size,&current_data,sizeof(current_data));
			ESP_LOGW(my_tag,"Data recovery complete");
			read_write_init();
		}
		else
		{
			init_default_data();
			read_write_init();
		}
	}
}
void init_default_data()
{
	ESP_LOGW(my_tag,"Assign the system default value to an array");
	struct my_data default_data;
	memset(&default_data,0,sizeof(default_data));

	default_data.check_head[0]=0x55;
	default_data.check_head[1]=0x56;
	default_data.pic_number=0;
	default_data.low_power_state=0;
	strcpy(default_data.pic_name,"20210101.bin");
	strcpy(default_data.pic_name_current,"20210101.bin");
	strcpy(default_data.wifi_ssid,default_wifi_ssid);
	strcpy(default_data.wifi_pssd,default_wifi_pssd);
	strcpy(default_data.server_add_get_down_pic,"https://aink.net/devices/download/pic/");
	default_data.network_wrong_state=0;
	default_data.config_wifi_state=0;
	strcpy(default_data.server_add_tell_down_ok,"https://aink.net/devices/finished/pic/");
	strcpy(default_data.server_add_tell_dele_ok,"https://aink.net/devices/deleted/pic/");
	strcpy(default_data.server_add_to_downlo_pic,"https://aink.net/devices/resources/");
	default_data.check_tail[0]=0x55;
	default_data.check_tail[1]=0x56;
	default_data.time_stamp=default_time_stamp;
	stime.tv_sec = 	default_data.time_stamp;
	settimeofday(&stime,NULL);
	default_data.picture_time_stamp=default_time_stamp;
	default_data.remain_space=11534336-2*4*1024-picture_cap*(default_data.pic_number+default_data.low_power_state+default_data.config_wifi_state+default_data.network_wrong_state);

	spi_flash_erase_sector(info_page);
	spi_flash_erase_sector(info_page_backup);
	spi_flash_write(info_page*sector_size,&default_data,sizeof(default_data));
	spi_flash_write(info_page_backup*sector_size,&default_data,sizeof(default_data));
	ESP_LOGW(my_tag,"write default data to array done and write flash 1280 1281 done");
}

void analysis_data()
{
//	ESP_LOGW(my_tag,"sizeof(default_data)=%d",sizeof(data));
	printf("\n");
	ESP_LOGW(my_tag, "analysis data from sector 1280:");
	ESP_LOGW(my_tag,"current_data.check_head[0]=%x",current_data.check_head[0]);
	ESP_LOGW(my_tag,"current_data.check_head[1]=%x",current_data.check_head[1]);
	ESP_LOGW(my_tag,"current_data.pic_number=%d",current_data.pic_number);
	ESP_LOGW(my_tag,"current_data.low_power_state=%d",current_data.low_power_state);
	ESP_LOGW(my_tag,"current_data.pic_name=%s",current_data.pic_name);
	ESP_LOGW(my_tag,"current_data.pic_name_current=%s",current_data.pic_name_current);
	ESP_LOGW(my_tag,"current_data.wifi_ssid=%s",current_data.wifi_ssid);
	ESP_LOGW(my_tag,"current_data.wifi_pssd=%s",current_data.wifi_pssd);
	ESP_LOGW(my_tag,"current_data.server_add_get_down_pic=%s",current_data.server_add_get_down_pic);
	ESP_LOGW(my_tag,"current_data.network_wrong_state=%d",current_data.network_wrong_state);
	ESP_LOGW(my_tag,"current_data.config_wifi_state=%d",current_data.config_wifi_state);
	ESP_LOGW(my_tag,"current_data.server_add_tell_down_ok=%s",current_data.server_add_tell_down_ok);
	ESP_LOGW(my_tag,"current_data.server_add_tell_dele_ok=%s",current_data.server_add_tell_dele_ok);
	ESP_LOGW(my_tag,"current_data.server_add_to_downlo_pic=%s",current_data.server_add_to_downlo_pic);
	ESP_LOGW(my_tag,"current_data.check_tail[0]=%x",current_data.check_head[0]);
	ESP_LOGW(my_tag,"current_data.check_tail[1]=%x",current_data.check_head[1]);
	ESP_LOGW(my_tag,"current_data.time_stamp=%ld",current_data.time_stamp);
	ESP_LOGW(my_tag,"current_data.picture_time_stamp=%ld",current_data.picture_time_stamp);
	ESP_LOGW(my_tag,"current_data.remain_space=%ld",current_data.remain_space);
	printf("\n");
//	unsigned char temp_data;
//	for(int i=0;i<4096;i++)
//	{
//		 spi_flash_read(info_page*sector_size+i,&temp_data,1);
//		 if(temp_data==0xff)
//		 {
//			 break;
//		 }
//		 ESP_LOGI(my_tag,"temp_data %d =%x", i+1,temp_data);
//	}
}

void updated_esp_time()
{
	time(&now);
	// Set timezone to China Standard Time
	setenv("TZ", "CST-8", 1);
	tzset();
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(my_tag, "The current date/time in Shanghai is: %s", strftime_buf);
}
void set_time_to_wakeup_by_timer(int hour,int min)
{
	gettimeofday(&stime,NULL);
	current_data.time_stamp=stime.tv_sec;
	gettimeofday(&stime,NULL);
	time_now=stime.tv_sec;
	p=localtime(&time_now);
	printf("p->tm_hour=%d p->tm_min=%d p->tm_sec=%d ",p->tm_hour,p->tm_min,p->tm_sec);
	int temp_hour=0,temp_min=0,temp_second=0;
	if(p->tm_hour>hour)
	{
		temp_hour=24+hour-p->tm_hour-1;
		temp_min=60-p->tm_min-1+min;
	}
	else if(p->tm_hour<hour)
	{
		temp_hour=hour-p->tm_hour-1;
		temp_min=60-p->tm_min-1+min;
	}
	else
	{
		if(p->tm_min==min+1)
		{
			temp_hour=24+hour-p->tm_hour-1;
			temp_min=60-p->tm_min-1-1+min;
		}
		else if(p->tm_min<min)
		{
			temp_hour=0;
			temp_min=min-p->tm_min-1;
		}
		else if(p->tm_min==min)
		{
			if(p->tm_sec>0)
			{
				temp_hour=24+hour-p->tm_hour-1;
				temp_min=60-p->tm_min-1+min;
			}
		}
		else
		{
			temp_hour=24+hour-p->tm_hour-1;
			temp_min=60-p->tm_min-1-1+min;
		}
	}
	if(temp_min>=60)
	{
		temp_hour++;
		temp_min-=60;
	}
	temp_second=60-p->tm_sec;
	printf("%d\n",temp_hour*60*60+temp_min*60+temp_second);
	ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)(temp_hour*60*60+temp_min*60+temp_second)*1000*1000));
	ESP_LOGW(my_tag,"After %d hours %d minutes %d seconds , it will be wakeup by timer",temp_hour,temp_min,temp_second);
}

void updated_data_to_flash()
{
	updated_esp_time();
	spi_flash_erase_sector(info_page);
	spi_flash_erase_sector(info_page_backup);
	spi_flash_write(info_page*4096,&current_data,sizeof(current_data));
	spi_flash_write(info_page_backup*4096,&current_data,sizeof(current_data));
	printf("\n");
	ESP_LOGW(my_tag,"udpated data to flash 1280 1281 sector");
}

void check_wifi_httpdownload_pic(char wakeup_cause)
{
	if(isconnected)
	{
		memset(URL_download,'\0',200);
		strcat(URL_download,current_data.server_add_get_down_pic);
		charconnectuchar(URL_download,&device_info[9]);
		URL_download[strlen(URL_download)]=QUES;
		strcat(URL_download,picname);
		if(wakeup_cause==0)
		{
			strcat(URL_download,current_data.pic_name_current);
		}
		else
		{
			strcat(URL_download,current_data.pic_name);
		}
		URL_download[strlen(URL_download)]=AND;
		strcat(URL_download,picsize);
		char char_pic_number[2];
		int_to_string((int)current_data.pic_number,char_pic_number);
		char_pic_number[1]='\0';
		URL_download[strlen(URL_download)]=char_pic_number[0];
		URL_download[strlen(URL_download)]=AND;
		strcat(URL_download,Voltage);
		char char_voltage[4];
		int_to_string(328,char_voltage);
		char_voltage[3]='\0';
		strcat(URL_download,char_voltage);
		URL_download[strlen(URL_download)]=AND;
		strcat(URL_download,action);
		URL_download[strlen(URL_download)]=wakeup_cause;
		URL_download[strlen(URL_download)]=AND;
		strcat(URL_download,timestamp);
		char temp_time_stamp[50];
		int_to_string(default_time_stamp,temp_time_stamp);
		temp_time_stamp[10]='\0';
		strcat(URL_download,temp_time_stamp);

//		ESP_LOGW(my_tag,"Request server try to get download_picture_name");
//		ESP_LOGW(my_tag,"request_url=%s",URL_download);
		http_test_task(URL_download);
	}
	else
	{
		ESP_LOGW(my_tag,"Timer_Config()");
		Timer_Config();

		ESP_LOGW(my_tag,"gattserver(ble) init");
		GattServers_Init();
//				display picture network wrong
//				display_low_network_wifi_picture(1);
	}
}

void cJSON_data(char *json_str)
{
//	{\"picname\":\"20201222.bin\"},{\"picname\":\"20201223.bin\"},{\"picname\":\"20201224.bin\"}
//	char * json_str = "{\"timestame\":56325142,\"deletepic\":\"20201011.bin\",\"downloadlist\":[], \"lowvol\":\"lowvol.bin\", \"networkerr\":\"networkerr.bin\", \"qcode\":\"xx\"}";
	struct timeval stime;
	cJSON * root = NULL;
	item = NULL;
	root = cJSON_Parse(json_str);
	if (!root)
	{
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		sleep_for_next_wakeup();
	}
	else
	{
		item = cJSON_GetObjectItem(root, "succ");
		if(item!=NULL)
		{
			printf("%s\n", cJSON_Print(root));
			if(strcmp(item->valuestring,"ok")==0)
			{
				ESP_LOGE(my_tag, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
				esp_http_client_close(client);
				esp_http_client_cleanup(client);
				ESP_LOGW(my_tag, "close http(s) cleanup client done");
				sleep_for_next_wakeup();
			}
		}
//		item = cJSON_GetObjectItem(root, "succ");
//		if(item!=NULL)
//		{
//			if(strcmp(item->valuestring,"ok")==0)
//			{
//				failed_times=0;
//				failed_bit=1;
//			}
//			else
//			{
//				if(failed_times>2)
//				{
//					failed_times=0;
//					ESP_LOGW(my_tag,"picture download failed (more than twice)");
////					display_picture_temp(1,low_network_wifi_picture_page);
//					sleep_for_next_wakeup();
//				}
//			}
//		}
		else
		{
			printf("%s\n", cJSON_Print(root));

			printf("%s\n", "get timestamp cJSON object:");
			item = cJSON_GetObjectItem(root, "timestamp");
			if(item!=NULL)
			{
				stime.tv_sec=item->valueint;
				settimeofday(&stime,NULL);
				printf("%s\n", cJSON_Print(item));
				printf("%ld\n", stime.tv_sec);
				ESP_LOGE(my_tag,"start deep sleep: %lld us", esp_timer_get_time());
				updated_esp_time();
//				set_time_to_wakeup_by_timer(17,45);
				ESP_LOGW(my_tag,"%s:%d, and set ESP32 time done!!!!!", item->string,item->valueint);
			}

				item = cJSON_GetObjectItem(root, "picname");
				if(item!=NULL)
				{
	//				while(failed_bit==0)
	////				while(failed_times<=2)
	//				{
						printf("%s\n", "get picname cJSON object:");
						printf("%s\n", cJSON_Print(item));
	//					failed_times++;
	//				}
				}
				else
				{
					item = cJSON_GetObjectItem(root, "lowvol");
					if(item!=NULL)
					{
						printf("%s\n", "get lowvol cJSON object :");
						printf("%s\n", cJSON_Print(item));
						ESP_LOGW(my_tag,"%s:%s", item->string,item->valuestring);
					}
					else
					{
						item = cJSON_GetObjectItem(root, "networkerr");
						if(item!=NULL)
						{
							printf("%s\n", "get networkerr cJSON object :");
							printf("%s\n", cJSON_Print(item));
							ESP_LOGW(my_tag,"%s:%s", item->string,item->valuestring);
						}
						else
						{
							item = cJSON_GetObjectItem(root, "qcode");
							if(item!=NULL)
							{
								printf("%s\n", "get qcode cJSON object :");
								printf("%s\n", cJSON_Print(item));
								ESP_LOGW(my_tag,"%s:%s", item->string,item->valuestring);
							}
							else
							{
								ESP_LOGW(my_tag,"picname is NULL,no need to download picture(s)");
								sleep_for_next_wakeup();
							}
						}
					}
				}
				download_pic_url_composite(item);

			}
//			item = cJSON_GetObjectItem(root, "deletepic");
//			if(item!=NULL)
//			{
//				printf("%s\n", "get deletepic cJSON object:");
//				printf("%s\n", cJSON_Print(item));
//				ESP_LOGW(my_tag,"%s:%s", item->string,item->valuestring);
//				download_composite(item);
//			}

	}
}

void download_pic_url_composite(cJSON * item)
{
	memset(URL_download,0,200);
	ESP_LOGW(my_tag,"What picture do you want to download is %s",item->valuestring);
	strcat(URL_download,current_data.server_add_to_downlo_pic);
	charconnectuchar(URL_download,&device_info[9]);
	URL_download[strlen(URL_download)]='/';
	strcat(URL_download,item->valuestring);
	ESP_LOGW(my_tag,"Your download address is %s",URL_download);
//		http_test_task(URL_download);
}

void download_pic_finished_url_composite(cJSON * item,char *url_temp)
{
	ESP_LOGW(my_tag,"The picture %s has been downloaded and displayed.",item->valuestring);
	memset(URL_download,0,200);
	strcat(URL_download,url_temp);
	charconnectuchar(URL_download,&device_info[9]);
	URL_download[strlen(URL_download)]=QUES;
	strcat(URL_download,picname);
	strcat(URL_download,item->valuestring);
	URL_download[strlen(URL_download)]=AND;
	strcat(URL_download,timestamp);
	char temp_time_stamp[11];
	int_to_string(current_data.time_stamp,temp_time_stamp);
	temp_time_stamp[10]='\0';
	strcat(URL_download,temp_time_stamp);
	ESP_LOGW(my_tag,"url=%s",URL_download);
}

void delete_pic_finished_url_composite(cJSON * item)
{
	printf("%s\n",item->string);
	printf("%s\n",item->valuestring);
	memset(URL_download,0,200);
	strcat(URL_download,current_data.server_add_tell_dele_ok);
	charconnectuchar(URL_download,&device_info[9]);
	URL_download[strlen(URL_download)]=QUES;
	strcat(URL_download,picname);
	strcat(URL_download,item->valuestring);
	URL_download[strlen(URL_download)]=AND;
	strcat(URL_download,timestamp);
	char temp_time_stamp[11];
	int_to_string(current_data.time_stamp,temp_time_stamp);
	temp_time_stamp[10]='\0';
	strcat(URL_download,temp_time_stamp);
	ESP_LOGW(my_tag,"url=%s",URL_download);
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

void charconnectuchar(char a[],unsigned char b[])
{
	unsigned char i=0;
	while(1)
	{
		if(a[i]=='\0')
		{
//			printf("i=%d\n",i);
			break;
		}
		i++;
	}
	char arr[18];
	char *p=arr;
	arr[17]='\0';
	for(unsigned char j=0;j<6;j++)
	{
		sprintf(p,"%x",device_info[j+9]);
		p++;
		p++;
		if(j<5)
		{
			*p=':';
			p++;
		}
	}
	strcat(a,arr);
//	printf("j=%d\n",j);
//	for(j=0;j<18;j++)
//	{
//		printf("arr[%d]=%c\n",j,arr[j]);
//	}
}

void mac_to_device_info()
{
	char arr[18];
	char *p=arr;
	arr[17]='\0';
	for(unsigned char j=0;j<6;j++)
	{
		sprintf(p,"%x",device_info[j+9]);
		p++;
		p++;
		if(j<5)
		{
			*p=':';
			p++;
		}
	}
	for(unsigned char j=0;j<17;j++)
	{
		device_info[j+9]=arr[j];
//		printf("device_info_%d %c",j,device_info[j+9]);
	}
}

int string_to_int(char * string,int index)
{
    int value = 0;
    for(int i = 0;string[index] >= '0' && string[index] <= '9' && i<index; i++)
    {
        value = value * 10 + string[i] - '0';
    }
    return value;
}

//void display_low_network_wifi_picture(unsigned char index)
//{
//
//}


//	cJSON * item_temp = NULL;
//	char temp_name[20];
//			printf("%s\n", "get downloadlist cJSON object:");
//			item_temp = cJSON_GetObjectItem(root, "downloadlist");
//			ESP_LOGW(my_tag,"%s", cJSON_Print(item_temp));
//
//			printf("%s\n", "get picname cJSON object:");
//			if( NULL != item_temp )
//			{
//				cJSON *client_list  = item_temp->child;
//				if( NULL == client_list )
//				{
//					ESP_LOGW(my_tag,"picname is NULL,no need to download picture(s)");
//					ESP_LOGW(my_tag,"go to sleep");
//					esp_deep_sleep_start();
//				}
//				else
//				{
//					while( client_list != NULL )
//					{
//						failed_times++;
//						download_composite(item);
//						if(failed_bit==1)
//						{
//							client_list = client_list->next;
//						}
//					}
//					ESP_LOGW(my_tag,"all pictures downloaded");
//					ESP_LOGW(my_tag,"go to sleep");
//					esp_deep_sleep_start();
//				}
//			}
//			else
//			{
//				ESP_LOGW(my_tag,"no downloadlist string");
//				ESP_LOGW(my_tag,"go to sleep");
//				esp_deep_sleep_start();
//			}


//				 "deletepic"
//				for (unsigned char i = 0; i < current_data.pic_number; i++)
//				{
//					spi_flash_read(info_pic_name + i * 20, temp_name, 20);
//					if (strcmp((char *)temp_name, item->valuestring) == 0)
//					{
//						ESP_LOGW(my_tag, "find same name picture,now delete it");
//						spi_flash_read(info_page*4096,&current_data,sizeof(current_data));
//						current_data.pic_number--;
//						spi_flash_write(info_page*4096,&current_data,sizeof(current_data));
//						temp_name[8]='*';
//						sf_WriteBuffer((uint8_t *)temp_name,info_pic_name + i * 20,20);
//						break;
//					}
//				}



