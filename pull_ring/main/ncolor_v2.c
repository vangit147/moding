/* À­»·Çý¶¯
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "aenc.h"
#include "pic_ncolor.h"

typedef union{
	UBYTE AC[10];
}ac_t;
static void OWCT_OW(UBYTE value);
esp_c_t  CN_Init(void);
const UBYTE DATATEMP[10]={0x00,0X10,0X1B,0X04,0X0C,0X0F,0X0E,0X1A,0X22,0X68};
ac_t AC_T;
static void NCI_GP(void);
static void OWTC_O(UBYTE temp);
static void OWCT_O(UBYTE temp);
static void WOCT_O(UBYTE temp);
typedef struct cdnc{
	UBYTE NCTEMP[20];
}cdnc_t;
cdnc_t CDNC_T;
static esp_c_t  GAC_Init(void)
{
	AC_T.AC[0]=0X1B;
	AC_T.AC[1]=0X04;
	AC_T.AC[2]=0X0F;
	AC_T.AC[3]=0X0C;
	AC_T.AC[4]=0X1A;
	AC_T.AC[5]=0X00;
	AC_T.AC[6]=0X10;
	AC_T.AC[7]=0X0E;
	return NRR;
}
void DELAY(UWORD DATA)
{
	vTaskDelay(DATA / portTICK_RATE_MS);
}
int  ONR_T(gpio_num_t g_num)
{
	int value=0;
	value=gpio_get_level(g_num);
	return value;
}
static void delay(UWORD value)
{
	ets_delay_us(value);
}
void WIN_O(gpio_num_t g_num,uint32_t value)
{
	gpio_set_level(g_num,value);
}
static esp_c_t CN_InitG(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = 0;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =( (1ULL<<AC_T.AC[0]) |\
                    (1ULL<<AC_T.AC[3]) |(1ULL<<AC_T.AC[2])|(1ULL<<AC_T.AC[7])|(1ULL<<AC_T.AC[1]) |(1ULL<<AC_T.AC[4])|\
				    (1ULL<<AC_T.AC[6])|\
							(1ULL<<AC_T.AC[5])
							) ;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.intr_type = 0;
    io_conf.pin_bit_mask = ((1ULL<<BNCO)\
    		 |(1ULL<<TNCB) |\
			 (1ULL<<CNOS)
			 );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    WIN_O(0X04,1);
    WIN_O(0X0F,1);
    WIN_O(0X1A,NRR);
    WIN_O(0X00,NRR);
    WIN_O(0X0C,NFR);
    DELAY(100);
    ets_delay_us(200);
    return NRR;
}
 esp_c_t  CN_Init(void)
{
	GAC_Init();
	CN_InitG();
	NCI_GP();
	return NRR;
}
static void OWTT_Init(void)
{
	WIN_O(0x04,NRR);
	WIN_O(0x04,NFR);
	WIN_O(0x10,NFR);
	delay(2);
	WIN_O(0x0C,NFR);
}
static void OWCT_O(UBYTE temp)
{
	OWTT_Init();
	OWCT_OW(temp);
}
static void TTOW_Init(void)
{
	WIN_O(AC_T.AC[2],NRR);
	WIN_O(AC_T.AC[2],NFR);
	WIN_O(AC_T.AC[0],NFR);
	delay(2);
	WIN_O(AC_T.AC[3],NFR);
}
static void OWTD_Init(void)
{
	WIN_O(0x04,NRR);
	WIN_O(0x04,NFR);
	WIN_O(0x10,NRR);
	delay(2);
	WIN_O(0x0C,NFR);
}
void WK_AC(void)
{
	CN_Init();
	DELAY(1000);
	OWCT_O(CDI);
	OWTC_O(0x37);
	WOCT_O(0X50);
	OTCO_O(0X37);
}
static void OWCT_OW(UBYTE value)
{
	for(int i=0;i<8;i++)
	{
	   if ((value & 0x80) == 0)
	   {
		   WIN_O(DNCO,NFR);
	   }else{

		   WIN_O(DNCO,NRR);
	   }
		WIN_O(0X0C,NRR);
		delay(2);
		WIN_O(NCLKC,NFR);
		value=value<<1;
	}
	delay(2);
	WIN_O(NCTC,NRR);
}
static void OWTC_O(UBYTE temp)
{
	OWTD_Init();
	OWCT_OW(temp);
}
static void TTOD_Init(void)
{
	WIN_O(AC_T.AC[2],NRR);
	WIN_O(AC_T.AC[2],NFR);
	WIN_O(AC_T.AC[0],NRR);
	delay(2);
	WIN_O(AC_T.AC[3],NFR);
}
static void OWCT_OT(UBYTE value)
{
	for(int i=0;i<8;i++)
	{
	   if ((value & 0x80) == 0)
	   {
		   WIN_O(DNCO,NFR);
	   }else{

		   WIN_O(DNCO,NRR);
	   }
		WIN_O(0X0C,NRR);
		delay(2);
		WIN_O(NCLKC,NFR);
		value=value<<1;
	}
	delay(2);
	WIN_O(NCOC,NRR);
}
static void WOCT_O(UBYTE temp)
{
	TTOW_Init();
	OWCT_OT(temp);
}
void OTCO_O(UBYTE temp)
{
	TTOD_Init();
	OWCT_OT(temp);
}
static void NCI_GP(void)
{
	DELAY(100);
	WIN_O(RNCO, NFR);
	DELAY(10);
	WIN_O(RNCO, NRR);
	DELAY(10);
	while(ONR_T(BONC)==0);
	WOCT_O(PSR);
	OTCO_O(0xEF);
	OTCO_O(0x08);
	WOCT_O(PWR);
	OTCO_O(0x37);
	OTCO_O(0x00);
	OTCO_O(0x23);
	OTCO_O(0x23);
	WOCT_O(PFS);
	OTCO_O(0x00);
	WOCT_O(BTST);
	OTCO_O(0xC7);
	OTCO_O(0xC7);
	OTCO_O(0x1D);
	WOCT_O(PLL);
	OTCO_O(0x3C);
	WOCT_O(TSE);
	OTCO_O(0x00);
	WOCT_O(CDI);
	OTCO_O(0x37);
	WOCT_O(TCON);
	OTCO_O(0x22);
	WOCT_O(TRES);
	OTCO_O(0x02);
	OTCO_O(0x58);
	OTCO_O(0x01);
	OTCO_O(0xC0);
	WOCT_O(PWS);
	OTCO_O(0xAA);
	DELAY(100);
	WIN_O(0, NFR);
	DELAY(10);
	WIN_O(0, NRR);
	DELAY(10);
	while(ONR_T(BTNC)==0);
	OWCT_O(PSR);
	OWTC_O(0xEF);
	OWTC_O(0x08);
	OWCT_O(PWR);
	OWTC_O(0x37);
	OWTC_O(0x00);
	OWTC_O(0x23);
	OWTC_O(0x23);
	OWCT_O(PFS);
	OWTC_O(0x00);
	OWCT_O(BTST);
	OWTC_O(0xC7);
	OWTC_O(0xC7);
	OWTC_O(0x1D);
	OWCT_O(PLL);
	OWTC_O(0x3C);
	OWCT_O(TSE);
	OWTC_O(0x00);
	OWCT_O(CDI);
	OWTC_O(0x37);
	OWCT_O(TCON);
	OWTC_O(0x22);
	OWCT_O(TRES);
	OWTC_O(0x02);
	OWTC_O(0x58);
	OWTC_O(0x01);
	OWTC_O(0xC0);
	OWCT_O(PWS);
	OWTC_O(0xAA);
}
void Acep_loadPIC1(unsigned char* pic_data3)
{
	uint8_t *pusFrameBuf3=pic_data3;
	OWCT_O(0x61);
	OWTC_O(0x02);
	OWTC_O(0x58);
	OWTC_O(0x01);
	OWTC_O(0xC0);
	OWCT_O(DTM);
	for(int i=0;i<448;i++)
	{
		for(int j=0;j<300;j++){
			OWTC_O(*pusFrameBuf3);
			pusFrameBuf3++;
		}
	}
	OWCT_O(PON);
	while(ONR_T(BTNC)==0);
	OWCT_O(DRF);
	while(ONR_T(BTNC)==0);
	OWCT_O(POF);
	while(ONR_T(0x11));
	DELAY(100);
}
/***********************************************/
//update van pull_ring
void Acep_loadPIC1_init()
{
	OWCT_O(0x61);
	OWTC_O(0x02);
	OWTC_O(0x58);
	OWTC_O(0x01);
	OWTC_O(0xC0);
	OWCT_O(DTM);
}
void Acep_loadPIC1_test(unsigned char* pic_data3,int max_data)
{
	uint8_t *pusFrameBuf3=pic_data3;
	for(int i=0;i<max_data;i++)
	{
		OWTC_O(*pusFrameBuf3);
		pusFrameBuf3++;
	}
}
void Acep_loadPIC1_end()
{
	OWCT_O(PON);
	while(ONR_T(BTNC)==0);
	OWCT_O(DRF);
	while(ONR_T(BTNC)==0);
	OWCT_O(POF);
	while(ONR_T(0x11));
	DELAY(100);
}
/***********************************************/
void Acep_loadPIC2(unsigned char* pic_data4)
{
	uint8_t *pusFrameBuf4=pic_data4;
	WOCT_O(0x61);
	OTCO_O(0x02);
	OTCO_O(0x58);
	OTCO_O(0x01);
	OTCO_O(0xC0);
	WOCT_O(DTM);
	for(int i=0;i<448;i++)
	{
		for(int j=0;j<300;j++){
			OTCO_O(*pusFrameBuf4);
			pusFrameBuf4++;
		}
	}
	WOCT_O(PON);
	while(ONR_T(BONC)==0);
	WOCT_O(DRF);
	while(ONR_T(BONC)==0);
	WOCT_O(POF);
	while(ONR_T(0x0D));
	DELAY(100);
}
/**************************************************************/
//update van pull_ring
void Acep_loadPIC2_init()
{
	WOCT_O(0x61);
	OTCO_O(0x02);
	OTCO_O(0x58);
	OTCO_O(0x01);
	OTCO_O(0xC0);
	WOCT_O(DTM);
}
void Acep_loadPIC2_test(unsigned char* pic_data4,int max_data)
{
	uint8_t *pusFrameBuf4=pic_data4;
	for(int i=0;i<max_data;i++)
	{
		OTCO_O(*pusFrameBuf4);
		pusFrameBuf4++;
	}
}
void Acep_loadPIC2_end()
{
	WOCT_O(PON);
	while(ONR_T(BONC)==0);
	WOCT_O(DRF);
	while(ONR_T(BONC)==0);
	WOCT_O(POF);
	while(ONR_T(0x0D));
	DELAY(100);
}
/**************************************************/
void ncolor_display(uint16_t index)
{
	//CN_Init();
	if(index==1){
		Acep_loadPIC1(pic_ncolor_0);
		Acep_loadPIC2(pic_ncolor_1);

//		Acep_loadPIC1_init();
//		Acep_loadPIC1(pic_ncolor_0);
//		Acep_loadPIC1_end();
//		Acep_loadPIC2_init();
//		Acep_loadPIC2(pic_ncolor_2);
//		Acep_loadPIC2_end();
	}else{
		Acep_loadPIC1(pic_ncolor_2);
		Acep_loadPIC2(pic_ncolor_3);

//		Acep_loadPIC1_init();
//		Acep_loadPIC1(pic_ncolor_1);
//		Acep_loadPIC1_end();
//		Acep_loadPIC2_init();
//		Acep_loadPIC2(pic_ncolor_3);
//		Acep_loadPIC2_end();
	}
	printf("ncolor_display done\n");
}

void ncolor_display_diff(uint16_t index)
{
	while(1)
	{
		if(index==1)
			{
				Acep_loadPIC1_init();
				Acep_loadPIC1_test(pic_ncolor_0,134400);
				Acep_loadPIC1_end();
				Acep_loadPIC2_init();
				Acep_loadPIC2_test(pic_ncolor_1,134400);
				Acep_loadPIC2_end();
			}
			else
			{
				Acep_loadPIC1_init();
				Acep_loadPIC1_test(pic_ncolor_2,134400);
				Acep_loadPIC1_end();
				Acep_loadPIC2_init();
				Acep_loadPIC2_test(pic_ncolor_3,134400);
				Acep_loadPIC2_end();
			}
			printf("ncolor_display_diff  done\n");
	}

}

