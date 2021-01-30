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
#define info_page_backup 1281    //存储图片数量 名称等信息用于备份
#define picture_cap 15    //  图片所占扇区空间40 大小为40*4096
#define low_network_wifi_picture_page 1282		//存储低电压图片 1282
//#define picture_page 1283 //从1283扇区开始存储，每张图片占picture_cap大小

//#define info_pic_name_for_err	info_page*4096+500
//#define info_pic_name	info_page*4096+600 //从这里开始存储图片名字 每个图片名字长度为20B
//#define composite_picture_page 1342  //存储合成图片 1342~1350


#endif
