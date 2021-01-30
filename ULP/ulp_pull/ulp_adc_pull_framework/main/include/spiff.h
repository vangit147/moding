#ifndef __spiffs_H__
#define __spiffs_H__
uint8_t sf_WriteBuffer(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
typedef struct
{
    unsigned int ChipID;    /* оƬID */
    char ChipName[16];      /* оƬ�ͺ��ַ�������Ҫ������ʾ */
    unsigned int TotalSize; /* ������ */
    unsigned int PageSize;  /* ҳ���С */
} SFLASH_T;

#define info_page 1280    //�洢ͼƬ���� ���Ƶ���Ϣ
#define picture_cap 40    //  ͼƬ��ռ�����ռ�40 ��СΪ40*4096
#define picture_page 1281 //��1281������ʼ�洢��ÿ��ͼƬռpicture_cap��С


#endif
