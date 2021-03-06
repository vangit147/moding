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

#define info_page 1280    //存储图片数量 名称等信息
#define picture_cap 40    //  图片所占扇区空间40 大小为40*4096
#define picture_page 1281 //从1281扇区开始存储，每张图片占picture_cap大小


#endif
