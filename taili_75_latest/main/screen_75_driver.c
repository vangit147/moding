/*桌签驱动
 *  Created on: 2020年9月21日
 *      Author: ziling
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "esp_log.h"

#define G_SIZE  30720
#define MAX_WAVE_NUM  20
#define FR_100HZ 0x3A
#define FR_86HZ  0x32
#define FR_71HZ  0x2A
#define FR_50HZ  0x3C

#define VS_3V6  0x03
#define VS_4V   0x05
#define VS_4V2  0x06
#define VS_4V4  0x07
#define VS_4V6  0x08
#define VS_4V8  0x09
#define VS_5V   0x0a
#define VS_5V2  0x0b
#define VS_5V4  0x0c
#define VS_5V6  0x0D
#define VS_5V8  0x0E
#define VS_6V   0x0f
#define VS_6V2  0x10
#define VS_6V4  0x11
#define VS_7V   0x14
#define VS_7V2  0x15
#define VS_7V4  0x16
#define VS_7V6  0x17
#define VS_7V8  0x18
#define VS_8V   0x19
#define VS_9V   0x1E
#define VS_10V  0x23
#define VS_11V  0x28
#define VS_15V  0x3c

const unsigned char Lut[]={	0x8,	0x8,	0xC,	0x3C,	0x3C,
	0x2,	0x0,	0x0,			0x17,	0xC,	0xE,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x7,	0x0,	0x0,			0x12,	0x12,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,			0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,			0xA,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x1,	0x0,	0x0,			0x0,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x4,	0x0,	0x0,			0x3,	0x19,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x5,	0x0,	0x0,			0x2,	0x13,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,			0x14,	0x2,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x20,	0x0,	0x0,	0x17,	0xC,	0xE,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x7,	0x21,	0x0,	0x0,	0x0,	0x12,	0x12,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,	0x0,	0x0,	0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x10,	0x0,	0x0,	0x0,	0xA,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x4,	0x0,	0x0,	0x0,	0x0,	0x3,	0x19,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x5,	0x0,	0x0,	0x0,	0x0,	0x2,	0x13,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x10,	0x0,	0x0,	0x14,	0x2,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x1,	0x0,	0x0,	0x0,	0x17,	0xC,	0xE,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x7,	0x21,	0x0,	0x0,	0x0,	0x12,	0x12,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x20,	0x0,	0x0,	0x0,	0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,	0x0,	0x0,	0xA,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x4,	0x0,	0x0,	0x0,	0x0,	0x3,	0x19,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x5,	0x0,	0x0,	0x0,	0x0,	0x2,	0x13,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x2,	0x0,	0x0,	0x0,	0x14,	0x2,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x20,	0x0,	0x0,	0x0,	0x17,	0xC,	0xE,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x7,	0x21,	0x0,	0x0,	0x0,	0x12,	0x12,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x0,	0x0,	0x0,	0x0,	0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x10,	0x0,	0x0,	0x0,	0xA,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x1,	0x12,	0x0,	0x0,	0x0,	0x0,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x4,	0x43,	0x0,	0x0,	0x0,	0x3,	0x19,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x5,	0x43,	0x0,	0x0,	0x0,	0x2,	0x13,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x30,	0x0,	0x0,	0x0,	0x14,	0x2,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0xE0,				0x17,	0xC,	0xE,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x7,	0xC0,				0x12,	0x12,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0x80,				0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0xC0,				0xA,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x1,	0x40,				0x0,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x4,	0xC0,				0x3,	0x19,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x5,	0xC0,				0x2,	0x13,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x2,	0xE0,				0x14,	0x2,	0x4,	0x0,	0x0,	0x0,	0x0,	0x0,
};

typedef uint16_t  DK_t;
typedef uint8_t   DK_T;
typedef uint32_t  DK_Tag;
typedef uint8_t   DK_T;
spi_device_handle_t spi;

#define ESP32SET      1
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  21
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<PIN_NUM_RST) |(1ULL<<PIN_NUM_CS))
#define PIN_NUM_BUSY  19
#define GPIO_INPUT_PIN_SEL  ((1ULL<<PIN_NUM_BUSY))
#define RY1  13
#define RY2  17
#define SOC  4
#define CS_L 0
#define CS_H 1
#define ESP32RESET    0
#define PARALLEL_LINES 16
#define  EO  26
#define  ESP_VALUEW(a,b)   gpio_set_level(a,b)
#define  ESP_VALUER(a)     gpio_get_level(a)


void DK_Init(void);
void LDK_init(void);

void DK_ByT(void);
void DK_RTflesh(void);

void DK_ByO(void);
void DK_ROflesh(void);

void DK_REO_D(void);
void DK_REO_E(void);

void DK_RET_D(void);
void DK_RET_E(void);

void  Write_CO(DK_T value);
void  Write_DO(DK_T data);

void  Write_CT(DK_T value);
void  Write_DT(DK_T data);

int READK(gpio_num_t g_num);
void WRITEDK(gpio_num_t g_num,uint32_t value);

void HLUT(unsigned char * wavedata);
void HLUTO(unsigned char * wavedata);

void delay_ms(uint16_t value);

void Hal_UpGraghScreen1(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);
void Hal_UpGraghScreen2(unsigned char * buffer1,unsigned char * buffer2,unsigned int num);




void DK_Init(void)
{
//	delay_ms(150);
//	DK_REO_D();
//	delay_ms(20);
//	DK_REO_E();
//	delay_ms(30);
//	Write_CO(0x06);
//	Write_DO(0xd7);
//	Write_DO(0x2f);
//    delay_ms(1);
//    Write_CO(0x04);
//    delay_ms(10);
//    DK_ByO();
//    Write_CO(0x00);
//	Write_DO(0x0f);
//	Write_DO(0x80);
//	Write_CO(0x50);
//	Write_DO(0x77);
//	Write_CO(0x61);
//	Write_DO(0x02);
//	Write_DO(0x80);
//	Write_DO(0x01);
//	Write_DO(0x80);
//	Write_CO(0x82);
//	Write_DO(0x28);
//	DK_ByO();
//    HLUTO((unsigned char *)Lut);

	delay_ms(150);
	DK_RET_D();
	delay_ms(20);
	DK_RET_E();
	delay_ms(30);
	Write_CT(0x06);
	Write_DT(0xd7);
	Write_DT(0x2f);
    delay_ms(1);
    Write_CT(0x04);
    delay_ms(10);
    DK_ByT();
    Write_CT(0x00);
	Write_DT(0x0f);
	Write_DT(0x80);
	Write_CT(0x50);
	Write_DT(0x77);
	Write_CT(0x61);
	Write_DT(0x02);
	Write_DT(0x80);
	Write_DT(0x01);
	Write_DT(0x80);
	Write_CT(0x82);
	Write_DT(0x28);
	DK_ByT();
    HLUT((unsigned char *)Lut);
}
void LDK_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<4) |(1ULL<<5)| (1ULL<<15) |(1ULL<<16)|(1ULL<<26) |(1ULL<<27)|(1ULL<<21) |(1ULL<<23));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL<<13) |(1ULL<<17);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    WRITEDK(4,1);
    WRITEDK(15,1);
    WRITEDK(5,1);
    WRITEDK(26,1);
    WRITEDK(21,0);
	delay_ms(100);
	DK_Init();
}

void DK_ByT(void)
{
  while(READK(RY2)==0)
	delay_ms(1);
}

void DK_RTflesh(void)
{
	Write_CT(0x12);
	delay_ms(1000);
	DK_ByT();
}

void DK_ByO(void)
{
  while(READK(RY1)==0)
	delay_ms(1);
}
void DK_ROflesh(void)
{
	Write_CO(0x12);
	delay_ms(1000);
	DK_ByO();
}

void DK_REO_D(void)
{
   WRITEDK(EO,ESP32RESET);
}
void DK_REO_E(void)
{
   WRITEDK(EO,ESP32SET);
}

void DK_RET_D(void)
{
   WRITEDK(5,0);
}
void DK_RET_E(void)
{
   WRITEDK(5,1);
}

void  Write_CO(DK_T value)
{
	WRITEDK(15,ESP32SET);
	WRITEDK(15,ESP32RESET);
	WRITEDK(27,ESP32RESET);
	ets_delay_us(5);
	WRITEDK(21, 0);
	for (int i = 0; i < 8; i++)
	{
		if ((value & 0x80) == 0)
		WRITEDK(23, ESP32RESET);
		else
		WRITEDK(23, 1);
		WRITEDK(21, 1);
		ets_delay_us(5);
		WRITEDK(21, 0);
		value <<= 1;
	}
	ets_delay_us(5);
	WRITEDK(15,ESP32SET);
}
void  Write_DO(DK_T data)
{
	WRITEDK(15,ESP32SET);
	WRITEDK(15,ESP32RESET);
	WRITEDK(27,ESP32SET);
	ets_delay_us(2);
	WRITEDK(21, ESP32RESET);
	for (int i = 0; i < 8; i++)
	{
		if ((data & 0x80) == 0)
		WRITEDK(23, ESP32RESET);
		else
		WRITEDK(23, 1);
		WRITEDK(21, 1);
		ets_delay_us(2);
		WRITEDK(21, ESP32RESET);
		data <<= 1;
	}
	ets_delay_us(2);
	WRITEDK(15,ESP32SET);
}

void  Write_CT(DK_T value)
{
	WRITEDK(4,ESP32SET);
	WRITEDK(4,ESP32RESET);
	WRITEDK(16,ESP32RESET);
	ets_delay_us(5);
	WRITEDK(21, 0);
	for (int i = 0; i < 8; i++)
	{
		if ((value & 0x80) == 0)
		WRITEDK(23, ESP32RESET);
		else
		WRITEDK(23, 1);
		WRITEDK(21, 1);
		ets_delay_us(5);
		WRITEDK(21, 0);
		value <<= 1;
	}
	ets_delay_us(5);
	WRITEDK(4,ESP32SET);
}
void  Write_DT(DK_T data)
{
	WRITEDK(4,ESP32SET);
	WRITEDK(4,ESP32RESET);
	WRITEDK(16,ESP32SET);
	ets_delay_us(2);
	WRITEDK(21, ESP32RESET);
	for (int i = 0; i < 8; i++)
	{
		if ((data & 0x80) == 0)
		WRITEDK(23, ESP32RESET);
		else
		WRITEDK(23, 1);
		WRITEDK(21, 1);
		ets_delay_us(2);
		WRITEDK(21, ESP32RESET);
		data <<= 1;
	}
	ets_delay_us(2);
	WRITEDK(4,ESP32SET);
}

int READK(gpio_num_t g_num)
{
	int value=0;
	value=ESP_VALUER(g_num);
	return value;
}
void WRITEDK(gpio_num_t g_num,uint32_t value)
{
	ESP_VALUEW(g_num,value);
}

void HLUT(unsigned char * wavedata)
{
	unsigned int count;
	unsigned int i;
    unsigned char vcomlutnum,KWlutnum,Redlutnum;
	unsigned char * tmpdata_p;
	KWlutnum=*wavedata;
	Redlutnum=*(wavedata+1);
    vcomlutnum=Redlutnum;
	wavedata+=2;
	Write_CT(0x01);
	Write_DT(0x37);
	Write_DT(0x00);
	Write_DT(*wavedata++);
	Write_DT(*wavedata++);
	Write_CT(0x30);
	Write_DT(*wavedata++);
	Write_CT(0x20);
	for(count=0;count<11*vcomlutnum ;count++)Write_DT(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-vcomlutnum);i++){
		for(count=0;count<11;count++)Write_DT(0);
	}
	Write_CT(0x21);
	tmpdata_p=wavedata;
	for(count=0;count<13*KWlutnum;count++)Write_DT(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-KWlutnum);i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x22);
	for(count=0;count<13*KWlutnum;count++)Write_DT(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-KWlutnum);i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x23);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x24);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x25);
	for(count=0;count<13*Redlutnum;count++)Write_DT(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-Redlutnum);i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x26);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x27);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x28);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DT(0);
	}
	Write_CT(0x29);
	for(count=0;count<10*vcomlutnum;count++)Write_DT(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-vcomlutnum);i++){
		for(count=0;count<10;count++)Write_DT(0);
	}
	DK_ByT();
}
void HLUTO(unsigned char * wavedata)
{
	unsigned int count;
	unsigned int i;
    unsigned char vcomlutnum,KWlutnum,Redlutnum;
	unsigned char * tmpdata_p;
	KWlutnum=*wavedata;
	Redlutnum=*(wavedata+1);
    vcomlutnum=Redlutnum;
	wavedata+=2;
	Write_CO(0x01);
	Write_DO(0x37);
	Write_DO(0x00);
	Write_DO(*wavedata++);
	Write_DO(*wavedata++);
	Write_CO(0x30);
	Write_DO(*wavedata++);
	Write_CO(0x20);
	for(count=0;count<11*vcomlutnum ;count++)Write_DO(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-vcomlutnum);i++){
		for(count=0;count<11;count++)Write_DO(0);
	}
	Write_CO(0x21);
	tmpdata_p=wavedata;
	for(count=0;count<13*KWlutnum;count++)Write_DO(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-KWlutnum);i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x22);
	for(count=0;count<13*KWlutnum;count++)Write_DO(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-KWlutnum);i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x23);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x24);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x25);
	for(count=0;count<13*Redlutnum;count++)Write_DO(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-Redlutnum);i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x26);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x27);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x28);
	for(i=0;i<MAX_WAVE_NUM;i++){
		for(count=0;count<13;count++)Write_DO(0);
	}
	Write_CO(0x29);
	for(count=0;count<10*vcomlutnum;count++)Write_DO(*wavedata++);
	for(i=0;i<(MAX_WAVE_NUM-vcomlutnum);i++){
		for(count=0;count<10;count++)Write_DO(0);
	}
	DK_ByO();
}

void delay_ms(uint16_t value)
{
	vTaskDelay(value / portTICK_RATE_MS);
}

void Hal_UpGraghScreen1(unsigned char * buffer1,unsigned char * buffer2,unsigned int num)
{
	unsigned int i;
	unsigned char j,k;
	unsigned char tempvalue;
	unsigned char tempMono,tmpRed;
	DK_ByT();
	Write_CT(0x10);
	for(i=0;i<num;i++)
	{
		if(buffer1==NULL)tempMono=0;
		else tempMono=* buffer1++;
		if(buffer2==NULL)tmpRed=0;
		else tmpRed=* buffer2++;
		for(k=0;k<4;k++)
		{
		   tempvalue=0;
		   for(j=0;j<2;j++)
		   {
			  tempvalue <<= 4;
			  if(tempMono&0x80)tempvalue&=0xf0;//black
			  else
			  {
				if(tmpRed&0x80)tempvalue|=0x04;//red
				else tempvalue|=0x03; //white
			  }
			  tempMono<<=1;
			  tmpRed<<=1;
		   }
		   Write_DT(tempvalue);
		}
	}
}

void Hal_UpGraghScreen2(unsigned char * buffer1,unsigned char * buffer2,unsigned int num)
{
	unsigned int i;
	unsigned char j;
	unsigned char tempvalue,k;
	unsigned char tempMono,tmpRed;
	DK_ByO();
	Write_CO(0x10);
	for(i=0;i<num;i++)
	{
		if(buffer1==NULL)
			tempMono=0;
		else
			tempMono=* buffer1++;
		if(buffer2==NULL)
			tmpRed=0;
		else
			tmpRed=* buffer2++;
		for(k=0;k<4;k++)
		{
		   tempvalue=0;
		   for(j=0;j<2;j++)
		   {
			  tempvalue<<=4;
			  if(tempMono&0x80)
		   	   	   tempvalue&=0xf0;//black
			  else
			  {
				if(tmpRed&0x80)
		   	   	   tempvalue|=0x04;//red
				else
		   	   	   tempvalue|=0x03;//white
			  }
			  tempMono<<=1;
			  tmpRed<<=1;
		   }

//		   udpate van
//		   tempvalue=0x33;//全白 0011
//		   tempvalue=0x00;//全黑0000
//		   tempvalue=0x44;//全红0100
/**************************************/

		   Write_DO(tempvalue);
		}
	}
}


extern char BUFFER[4096];
void Hal_UpGraghScreen3()
{
	unsigned int l;
	unsigned char j,k;
	unsigned char tempvalue;
	unsigned char tempMono,temp;

		for(l=0;l<4096;l++)
		{
			tempMono=BUFFER[l];
			for(k=0;k<2;k++)
			{
				tempvalue=0xFF;
				for(j=0;j<2;j++)
				{
					temp=tempMono;
					temp&=0xC0;
					if(j==0)
					{
						if(temp==0x00||temp==0x40)
							temp|=0x0F;
						if(temp==0xC0)
							temp=(temp|0xFF)&0x3F;
					 }
					 else
					 {
						if(temp==0x00)
							temp|=0xF0;
						if(temp==0x40)
							temp|=0xF4;
						if(temp==0xC0)
							temp=(temp|0xFF)&0xF3;
					 }
					tempMono<<=2;
					tempvalue&=temp;
				}
				Write_DT(tempvalue);
			}
	}
}

void Hal_UpGraghScreen4()
{
	unsigned int l;
	unsigned char j,k;
	unsigned char tempvalue;
	unsigned char tempMono,temp;
	for(l=0;l<4096;l++)
	{
		tempMono=BUFFER[l];
		for(k=0;k<2;k++)
		{
			tempvalue=0xFF;
			for(j=0;j<2;j++)
			{
				temp=tempMono;
				temp&=0xC0;
				if(j==0)
				{
					if(temp==0x00||temp==0x40)
						temp|=0x0F;
					if(temp==0xC0)
						temp=(temp|0xFF)&0x3F;
				 }
				 else
				 {
					if(temp==0x00)
						temp|=0xF0;
					if(temp==0x40)
						temp|=0xF4;
					if(temp==0xC0)
						temp=(temp|0xFF)&0xF3;
				 }
				tempMono<<=2;
				tempvalue&=temp;
			}
			Write_DO(0x44);
		}
	}
}
/********************************************/







