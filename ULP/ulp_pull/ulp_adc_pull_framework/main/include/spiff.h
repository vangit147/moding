#ifndef __spiffs_H__
#define __spiffs_H__
uint8_t sf_WriteBuffer(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
typedef struct
{
    unsigned int ChipID;    /* 芯片ID */
    char ChipName[16];      /* 芯片型号字符串，主要用于显示 */
    unsigned int TotalSize; /* 总容量 */
    unsigned int PageSize;  /* 页面大小 */
} SFLASH_T;

#define info_page 1280      //存储图片数量 名称等信息
#define info_page_temp 1279  //存储图片名称
#define picture_cap 15      //  图片所占扇区空间40 大小为40*4096
#define picture_data 1281   //存储图片数据



typedef struct PIC_data
{
	unsigned char pic_name[20];
	unsigned char pic_time[10];
	unsigned char pic_display_screen;
	unsigned char pic_index;
	unsigned char pic_display_order[2];
	unsigned char pic_display_start_end_time[4];//07 14
	int pic_display_duration;
	unsigned char pic_is_delete;
	//same time picture list
	struct pic_data *next;
	struct pic_data *prev;
}PIC_data;

PIC_data pic_data;

#endif
