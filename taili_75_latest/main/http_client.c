#include "http_client.h"


static const char *http_tag = "http";
int  low_power_state=0;
int config_wifi_state=0;
int  network_wrong_state=0;
int  picname_times=0;
char low_power_picname[20]="lowvol.bin";
char network_wrong_picname[20]="networkerr.bin";
char config_wifi_picname[20]="qcode.bin";

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(http_tag, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(http_tag, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(http_tag, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(http_tag, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(http_tag, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(http_tag, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(http_tag, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

int http_test_task(char *dpwn_url)
{
	int data_read_size = 0, bytes_read = 0;

    esp_http_client_config_t config = {
    	.url = dpwn_url,
        .event_handler = _http_event_handler,
    };

    client = esp_http_client_init(&config);

    while(1)
    {
        esp_err_t err;
        unsigned char http_client_open_failed_times=3;
        while(http_client_open_failed_times)
        {
        	if ((err = esp_http_client_open(client, 0)) != ESP_OK)
    		{
    			ESP_LOGE(http_tag, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    			http_client_open_failed_times--;
    			if(http_client_open_failed_times==0)
    			{
    				ESP_LOGE(http_tag, "Failed to open an HTTP connection after three attempts");
    				if(current_data.network_wrong_state==0)
    				{
    					//			display picture network wrong
    				}
    				else
    				{
    					ESP_LOGE(http_tag,"flash no low_network_wifi_picture_page to display");
    				}
    				sleep_for_next_wakeup();
    				return 0;
    			}
    		}
    		else
    		{
    			ESP_LOGE(http_tag, "esp_http_client_open succeed");
    			break;
    		}
        }

		printf("\n");
		ESP_LOGE(http_tag, "start_download_data");

		memset(BUFFER,'\0',sector_size);
		int  content_length;
		int  read_len=-1;

		unsigned char len_url=strlen(current_data.server_add_to_downlo_pic);
		ESP_LOGE(http_tag,"config.url=%s",config.url);

		content_length = esp_http_client_fetch_headers(client);
		data_read_size = esp_http_client_get_content_length(client);

		if(strncmp(config.url,current_data.server_add_to_downlo_pic,len_url)!=0)
		{
			len_url=strlen(current_data.server_add_get_down_pic);
			if(strncmp(config.url,current_data.server_add_get_down_pic,len_url)==0)
			{
				wakeup_cause=*(strstr(config.url,"action")+7);
				ESP_LOGE(http_tag,"wakeup_cause=%c",wakeup_cause);
			}

			int i=0;
			char *q=BUFFER;
			printf("\n");
			ESP_LOGE(http_tag, "Start receiving CJSON data ");
			 while(1)
			{
				if(read_len==0)
				{
					content_length=i;
					break;
				}
				read_len = esp_http_client_read(client,q++,1);
				if(read_len==1)
				{
					i++;
				}
			}
			if(esp_http_client_get_status_code(client)!=200)
			{
				ESP_LOGE(http_tag, " After three attempts, no JSON data was obtained from the server");
				esp_http_client_close(client);
				esp_http_client_cleanup(client);
				break;
			}
			else
			{
				cJSON_data(BUFFER);
				ESP_LOGE(http_tag, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
				if(esp_http_client_set_url(client,URL_download)!=ESP_OK)
				{
					ESP_LOGE(http_tag,"esp_http_set_url failed!!\n we will close current client and open a new one.");
					ESP_LOGE(http_tag, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
					esp_http_client_close(client);
					esp_http_client_cleanup(client);
					ESP_LOGE(http_tag, "close http(s) cleanup client done");
					http_test_task(URL_download);
				}
				else
				{
					ESP_LOGE(http_tag,"esp_http_client_set_url OK !");
				}
			}
		}
		else
		{
			ESP_LOGE(http_tag,"content_length(read from https)=%d",content_length);
			ESP_LOGE(http_tag,"data_read_size(read from https)=%d",data_read_size);

			memset(PICNAME,0,20);
			memcpy(PICNAME,&config.url[strlen(current_data.server_add_to_downlo_pic)+18],12);
			search_dis_pic();
			if(low_power_state==1||network_wrong_state==1||config_wifi_state==1)
			{
				printf("\n");
				ESP_LOGE(http_tag, "Start receiving data stream");
				if(data_read_size<=sector_size)
				{
					read_len = esp_http_client_read(client, BUFFER, data_read_size);
					ESP_LOGE(http_tag, "read_len %d", read_len);
					sf_WriteBuffer((uint8_t*)BUFFER, (low_network_wifi_picture_page + picture_index) * 4096 + bytes_read * sector_size, read_len);
				}
				ESP_LOGE(http_tag, "wrote low_network_wifi_picture_page_data to %d sector ",(low_network_wifi_picture_page + picture_index));
			}
			else
			{
				printf("\n");
				DK_ByT();
				Write_CT(0x10);
				ESP_LOGE(http_tag,"start deep sleep: %lld us", esp_timer_get_time());
				ESP_LOGE(http_tag, "Start receiving data stream");
				while (data_read_size > 0)
				{
					read_len = esp_http_client_read(client, BUFFER, sector_size);
					ESP_LOGE(http_tag, "read %d		read_len %d", bytes_read,read_len);
					Hal_UpGraghScreen3();
					bytes_read++;
					data_read_size -= read_len;
				}
				DK_RTflesh();
				ESP_LOGE(http_tag,"start deep sleep: %lld us", esp_timer_get_time());
				ESP_LOGE(http_tag, "read_times = %d", bytes_read);
			}
			if(esp_http_client_get_status_code(client)!=200)
			{
				failed_times++;
				if(failed_times>2)
				{
					failed_times=0;
					ESP_LOGE(http_tag,"picture download failed (more than twice)");
					esp_http_client_close(client);
					esp_http_client_cleanup(client);
					ESP_LOGE(http_tag, "close http(s) cleanup client done");
					//			display picture network wrong
					sleep_for_next_wakeup();
				}
			}
			else
			{
				updated_data_to_flash();
				analysis_data();
				download_pic_finished_url_composite(item,current_data.server_add_tell_down_ok);
				ESP_LOGE(http_tag, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
				if(esp_http_client_set_url(client,URL_download)!=ESP_OK)
				{
					ESP_LOGE(http_tag,"esp_http_set_url failed!!\n we will close current client and open a new one.");
					ESP_LOGE(http_tag, "HTTP Stream reader Status = %d, content_length = %d",esp_http_client_get_status_code(client),esp_http_client_get_content_length(client));
					esp_http_client_close(client);
					esp_http_client_cleanup(client);
					ESP_LOGE(http_tag, "close http(s) cleanup client done");
					http_test_task(URL_download);
				}
				else
				{
					ESP_LOGE(http_tag,"esp_http_client_set_url OK !");
				}
			}
		}
    }
    return 1;
}



void search_dis_pic()
{
	char time_string[20];
	gettimeofday(&stime,NULL);
	time_now=stime.tv_sec;
  	p=localtime(&time_now);
	int tm_year=p->tm_year+1900;
	int tm_mon=p->tm_mon+1;
	int tm_mday=p->tm_mday;
	int date=tm_year*10000+tm_mon*100+tm_mday;
	int value;
	char temp[4];

  	if(strcmp(PICNAME,low_power_picname)==0)
  	{
  		picture_index=0;
  		low_power_state=1;
  		current_data.low_power_state=1;
  	}
  	else if(strcmp(PICNAME,network_wrong_picname)==0)
	{
		picture_index=1;
		network_wrong_state=1;
		current_data.network_wrong_state=1;
	}
  	else if(strcmp(PICNAME,config_wifi_picname)==0)
  	{
  		picture_index=2;
  		config_wifi_state=1;
  		current_data.config_wifi_state=1;
  	}
  	else
  	{
  		if(wakeup_cause == '0')
  		{
  			date=tm_year*10000+tm_mon*100+tm_mday;
			memset(time_string,'\0',20);
			int_to_string(date,time_string);
			strcat(time_string,".bin");
  		}
  		else
  		{
  			strcpy(time_string,current_data.pic_name);
  			value=string_to_int(&(time_string[4]),2);
  			if(wakeup_cause=='1')
			{
				value--;
				int_to_string(value,temp);
				if(value<10&&value>=0)
				{
					if(value==0)
					{
						value=string_to_int(&(time_string[0]),4);
						value--;
						int_to_string(value,temp);
						for(unsigned char i=0;i<4;i++)
						{
							time_string[i]=temp[i];
						}
						time_string[4]='1';
						time_string[5]='2';
					}
					else
					{
						time_string[4]='0';
						time_string[5]=temp[0];
					}
				}
				else
				{
					time_string[4]=temp[0];
					time_string[5]=temp[1];
				}
			}
  			else if(wakeup_cause=='2')
			{
				value++;
				int_to_string(value,temp);
				if(value<10)
				{
					time_string[4]='0';
					time_string[5]=temp[0];
				}
				else
				{
					if(value==13)
					{
						value=string_to_int(&(time_string[0]),4);
						value++;
						int_to_string(value,temp);
						for(unsigned char i=0;i<4;i++)
						{
							time_string[i]=temp[i];
						}
						time_string[4]='0';
						time_string[5]='1';
					}
					else
					{
						time_string[4]=temp[0];
						time_string[5]=temp[1];
					}
				}
			}
  			if(strncmp(PICNAME,time_string,6)==0)
			{
				strcpy(time_string,PICNAME);
			}
  		}


  		if(strcmp(PICNAME,time_string)==0)
  		{
			if(wakeup_cause=='0')
			{
				if(strcmp(PICNAME,current_data.pic_name_current)!=0)
				{
					picname_times=0;
					strcpy(current_data.pic_name_current,PICNAME);
					ESP_LOGE(http_tag,"current_data.pic_name = %s",current_data.pic_name);
					ESP_LOGE(http_tag,"current_data.pic_name_current = %s",current_data.pic_name_current);
				}
				else
				{
					ESP_LOGE(http_tag,"The picture %s was updated %d times a day",PICNAME,++picname_times);
				}
			}
			else
			{
				strcpy(current_data.pic_name,PICNAME);
				ESP_LOGE(http_tag,"current_data.pic_name = %s",current_data.pic_name);
				ESP_LOGE(http_tag,"current_data.pic_name_current = %s",current_data.pic_name_current);
			}
  		}
  		else
		{
			ESP_LOGE(http_tag,"not the picture what will display, something wrong ,check your picture_time on your picture or check your picname from server");
			sleep_for_next_wakeup();
		}
  	}
}
