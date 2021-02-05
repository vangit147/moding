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



typedef struct FLAG_data
{
	unsigned char flag_bit[4];
	unsigned char before_sleep_time[13];
	unsigned char start_test_time[13];
}FLAG_data;

FLAG_data flag_data;

typedef struct FLAG_data_backup
{
	unsigned char before_sleep_time[13];
	unsigned char start_test_time[13];
}FLAG_data_backup;

FLAG_data_backup flag_data_backup[128];//可更换电池128次

#endif
