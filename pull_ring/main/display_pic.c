#include "display_pic.h"


/*
 * 	Function:
 * 			int hightolower(int c)
 * 	Description:
 * 				A--->a(change A to a)
 */
int hightolower(int c)
{
	if(c >= 'A' && c <= 'Z')
	{
		return c + 'a'-'A';
	}
	else
	{
		return c;
	}
}


/*
 * 	Function:
 * 			int lowertohigh(int c)
 * 	Description:
 * 				a--->A(change a to A)
 */
int lowertohigh(int c)
{
	if(c >= 'a' && c <= 'z')
	{
		return c - ('a'-'A');
	}
	else
	{
		return c;
	}
}

/*
 * 	Function:
 * 			int hextoint(char s[])
 * 	Description:
 * 				hex--->int(change hex_string to int_value)
 * 				eg.
 * 					*s="5FE1670C" ---> 1531882506
 */
int hextoint(char s[])
{
	int i;
	int n=0;
	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
	{
		i = 2;
	}
	else
	{
		i = 0;
	}
	for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >= 'A' && s[i] <= 'Z'); ++i)
	{
		if (hightolower(s[i]) > '9')
		{
			n = 16 * n + (10 + hightolower(s[i]) - 'a');
		}
		else
		{
			n = 16 * n + (hightolower(s[i]) - '0');
		}
	}
	return n;
}

/*
 * 	Function:
 * 			void update_set_time(long time_now)
 * 	Description:
 * 				set time and print the current time what you set in format(eg.2020/12/23 Mon 10:10:10)
 * 				time_now:time stamp
 */
void update_set_time(long time_now)
{
	ESP_LOGW(MY_TAG,"---------");
	struct tm *p;
	struct timeval stime;
	stime.tv_sec = time_now;
	settimeofday(&stime,NULL);
	p=localtime(&time_now);
	ESP_LOGW(MY_TAG,"%ld",time_now);
	ESP_LOGW(MY_TAG,"%d/%d/%d %s %d:%d:%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,day[p->tm_wday],(p->tm_hour+8),p->tm_min,p->tm_sec);
	ESP_LOGW(MY_TAG,"*********");
}


/*
 * 	Function:
 * 			int search_in_flash(char * pic_name,unsigned char revceive_data_from_blue,int pic_size)
 * 	Description:
 * 				just display picture for 0x02(it means that the picture is new)
 * 				find picture what you click and display it for 0x07(it means that the picture is old and in flash)
 */
int search_in_flash(char * pic_name,unsigned char revceive_data_from_blue,int pic_size)
{
	char temp_name[40];
	unsigned char temp;
	unsigned char picture_num_temp;
	unsigned char picture_num_real; //实际图片数量
//		ESP_LOGW(MY_TAG,"all pic_name in flash:");
//		for(int j=1;j<=picture_num_temp;j++)
//		{
//			spi_flash_read(info_page * 4096 + j * 50, temp_name, 40);
//			ESP_LOGW(MY_TAG,"pic_%d_name=%s\n",j,temp_name);
//		}
	switch(revceive_data_from_blue)
	{
	case 0x02:
		ESP_LOGW(MY_TAG,"pic come frome httpdownload");
		spi_flash_read(info_page * 4096 + picture_num * 50 + 40, &temp, sizeof(unsigned char));
		ESP_LOGW(MY_TAG,"let`s start_display_pic_what_you_push");
		display(pic_name,temp,pic_size);
		break;
	case 0x07:
		spi_flash_read(info_reboot_times * 4096, &picture_num_real, sizeof(unsigned char));
		if(picture_num_real==0)
		{
			ESP_LOGW(MY_TAG,"no pic in flash");
			break;
		}
		else
		{

			spi_flash_read(info_page * 4096, &picture_num_temp, sizeof(unsigned char));
			unsigned char i=1;
			for(;i<=picture_num_temp;i++)
			{
				spi_flash_read(info_page * 4096 + i * 50, temp_name, 40);
				if(i>picture_num_real)
				{
					temp_name[39]='\0';
					if(strcmp(temp_name,temp_file_name)==0)
					{
						ESP_LOGW(MY_TAG,"no picture what you want to display,");
						return -2;
					}
				}
				if(strcmp(pic_name,temp_name)==0)
				{
					spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
					ESP_LOGW(MY_TAG,"find pic what you want to display,");
					if(temp==i)
					{
						ESP_LOGW(MY_TAG,"now,let`s start_display_pic_what_you_choose");
						display(pic_name,temp,pic_size);
						break;
					}
					else
					{
						ESP_LOGW(MY_TAG,"but it was be deleted already");
						return -1;
					}
				}
			}
		}
		break;
	}
	return 0;	// display successfully
}


/*
 * 	Function:
 * 			void display(char pic_name[32],unsigned char temp,int pic_size)
 * 	Description:
 * 				get picture location in flash and screen number which picture should display in by picture name,then,display it.
 */
void display(char pic_name[32],unsigned char temp,int pic_size)
{
	char screen_num;
	int num_times=0;
	int num=pic_size/4096+1;
	int pic_size_remainder=pic_size%4096;
	screen_num=pic_name[strlen(pic_name)-5];
	ESP_LOGW(MY_TAG,"the pic will display on screen %c !",screen_num);
	ESP_LOGW(MY_TAG,"the pic loaction is %d in flash !",temp);
	switch(screen_num)
	{
		case '1':
			Acep_loadPIC1_init();
			for(int i=num;i>1;i--)
			{
				spi_flash_read((picture_page + picture_cap *temp ) * 4096 + num_times * 4096,pull_ring,4096);
				Acep_loadPIC1_test(pull_ring,4096);
				num_times++;
			}
			spi_flash_read((picture_page + picture_cap * temp) * 4096 + num_times * 4096,pull_ring,pic_size_remainder);
			Acep_loadPIC1_test(pull_ring,pic_size_remainder);
			Acep_loadPIC1_end();
			ESP_LOGW(MY_TAG,"the pic display_finished in screen 1 !");
			break;
		case '2':
			Acep_loadPIC2_init();
			for(int i=num;i>1;i--)
			{
				spi_flash_read((picture_page + picture_cap * temp) * 4096 + num_times * 4096,pull_ring,4096);
				Acep_loadPIC2_test(pull_ring,4096);
				num_times++;
			}
			spi_flash_read((picture_page + picture_cap * temp) * 4096 + num_times * 4096,pull_ring,pic_size_remainder);
			Acep_loadPIC2_test(pull_ring,pic_size_remainder);
			Acep_loadPIC2_end();
			ESP_LOGW(MY_TAG,"the pic display_finished in screen 2 !");
			break;
	}
}

/*
 * 	Function:
 * 			void loop_display_picture(unsigned char receive_4)
 * 	Description:
 * 				display picture in order
 * 				wake up on time
 */
void loop_display_picture(unsigned char receive_4)
{
	//display picture in order
	unsigned char pic_num_total;
	switch(receive_4)
	{
	case 0x00:
		ESP_LOGW(MY_TAG,"loop_dispaly_picture closed");
		break;
	case 0x01:
		spi_flash_read(info_reboot_times * 4096 , &pic_num_total, sizeof(unsigned char));
		ESP_LOGW(MY_TAG,"pic_num_total=%d",pic_num_total);
		if(pic_num_total==0)
		{
			ESP_LOGW(MY_TAG,"flash no picture");
		}
		else
		{
			unsigned char i;
			unsigned char pic_num;
			unsigned char pic_num_temp;
			unsigned char times_display=0;
			spi_flash_read( info_page * 4096+ 81*50+sizeof(unsigned char), &pic_num_temp, sizeof(unsigned char));
			ESP_LOGW(MY_TAG,"pic_num_temp=%d",pic_num_temp);
			if(pic_num_temp==0xff)
			{
				pic_num_temp=1;
				sf_WriteBuffer(&pic_num_temp, info_page * 4096+ 81*50+sizeof(unsigned char), sizeof(unsigned char));
				ESP_LOGW(MY_TAG,"pic_num_temp=%d",pic_num_temp);
			}
			spi_flash_read(info_page * 4096, &pic_num, sizeof(unsigned char));
			for(i=pic_num_temp;i<=pic_num;i++)
			{
				unsigned char temp;
				spi_flash_read(info_page * 4096 + i * 50 + 40, &temp, sizeof(unsigned char));
				if(temp==0xff)
				{
					ESP_LOGW(MY_TAG,"This space never saved picture");
					break;
				}
				if(temp!=0&&temp==i)
				{
					char temp_name[40];
					spi_flash_read(info_page * 4096 + i * 50, temp_name, 40);
					display(temp_name,temp,134400);
					times_display++;
					if(i==pic_num)
					{
						pic_num_temp=1;
						sf_WriteBuffer(&pic_num_temp, info_page * 4096+ 81*50+sizeof(unsigned char), sizeof(unsigned char));
						if(times_display==2)
						{
							ESP_LOGW(MY_TAG,"Displayed last two pictures");
						}
						else
						{
							ESP_LOGW(MY_TAG,"Displayed last one pictures");
						}
						break;
					}
					if(times_display==2)
					{
						pic_num_temp=i+1;
						sf_WriteBuffer(&pic_num_temp, info_page * 4096+ 81*50+sizeof(unsigned char), sizeof(unsigned char));
						ESP_LOGW(MY_TAG,"Displayed two pictures");
						break;
					}
				}
			}
		}
		break;
	}
	//wake up on time
	gettimeofday(&stime,NULL);
	time_now=stime.tv_sec;
	p=localtime(&time_now);
	ESP_LOGW(MY_TAG,"%ld",time_now);
	ESP_LOGW(MY_TAG,"%d/%d/%d %s %d:%d:%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,day[p->tm_wday],(p->tm_hour+8),p->tm_min,p->tm_sec);
	if(p->tm_hour+8==9&&p->tm_min<10&&p->tm_min>5)
	{
		ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(300*1000*1000)); //睡眠唤醒时间为90s(自定义)
		printf("start deep sleep: %lld us\r\n", esp_timer_get_time());
		esp_deep_sleep_start();
	}
}

