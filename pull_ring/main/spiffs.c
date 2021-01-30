#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_spi_flash.h"
#include "spiff.h"
static const char *TAG = "spi_flash";
#define Sector 4096
#define PSector 49152

SFLASH_T g_tSF;
static uint8_t s_spiBuf[4 * 1024];
/*
*********************************************************************************************************
*	函 数 名: sf_AutoWritePage
*	功能说明: 写1个PAGE并校验,如果不正确则再重写两次。本函数自动完成擦除操作。
*	形    参:  	_pBuf : 数据源缓冲区；
*				_uiWriteAddr ：目标区域首地址
*				_usSize ：数据个数，不能超过页面大小
*	返 回 值: 0 : 错误， 1 ： 成功
*********************************************************************************************************
*/
// static uint8_t s_spiBuf[4 * 1024];

void sf_ReadBuffer(uint8_t *_pBuf, uint32_t _uiReadAddr, uint32_t _uiSize)
{
    spi_flash_read(_uiReadAddr, _pBuf, _uiSize);
}
// // uint8_t sf_WriteBuffer(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize)
// // {
// //   spi_flash_write(_uiWriteAddr, (uint8_t *)&_pBuf, _usWriteSize);
// // }
// /*
/*********************************************************************************************************
*	函 数 名: sf_CmpData
*	功能说明: 比较Flash的数据.
*	形    参:  	_ucpTar : 数据缓冲区
*				_uiSrcAddr ：Flash地址
*				_uiSize ：数据个数, 可以大于PAGE_SIZE,但是不能超出芯片总容量
*	返 回 值: 0 = 相等, 1 = 不等
*********************************************************************************************************
*/
static uint8_t sf_CmpData(uint32_t _uiSrcAddr, uint8_t *_ucpTar, uint32_t _uiSize)
{
    uint8_t ucValue[1];
    // ESP_LOGE("CmpData:", "_uiSrcAddr=%d,_uiSize=%d", _uiSrcAddr, _uiSize);
    /* 如果读取的数据长度为0或者超出串行Flash地址空间，则直接返回 */
    if ((_uiSrcAddr + _uiSize) > g_tSF.TotalSize)
    {
        return 1;
    }

    if (_uiSize == 0)
    {
        return 0;
    }

    while (_uiSize--)
    {
        /* 读一个字节 */
        sf_ReadBuffer(ucValue, _uiSrcAddr, 1);
        //ESP_LOGE("CmpData:", "_ucValue=%s,_ucpTar=%s", ucValue, _ucpTar);
        if (*_ucpTar++ != ucValue[0])
        {

            return 1;
        }
        _uiSrcAddr++;
    }
    return 0;
}
/*
*********************************************************************************************************
*	函 数 名: sf_NeedErase
*	功能说明: 判断写PAGE前是否需要先擦除。
*	形    参:   _ucpOldBuf ： 旧数据
*			   _ucpNewBuf ： 新数据
*			   _uiLen ：数据个数，不能超过页面大小
*	返 回 值: 0 : 不需要擦除， 1 ：需要擦除
*********************************************************************************************************
*/
static uint8_t sf_NeedErase(uint8_t *_ucpOldBuf, uint8_t *_ucpNewBuf, uint16_t _usLen)
{
    uint16_t i;
    uint8_t ucOld;

    /*
	算法第1步：old 求反, new 不变
	      old    new
		  1101   0101
	~     1111
		= 0010   0101

	算法第2步: old 求反的结果与 new 位与
		  0010   old
	&	  0101   new
		 =0000

	算法第3步: 结果为0,则表示无需擦除. 否则表示需要擦除
	*/

    for (i = 0; i < _usLen; i++)
    {
        ucOld = *_ucpOldBuf++;
        ucOld = ~ucOld;

        /* 注意错误的写法: if (ucOld & (*_ucpNewBuf++) != 0) */
        if ((ucOld & (*_ucpNewBuf++)) != 0)
        {
            return 1;
        }
    }
    return 0;
}

static uint8_t sf_AutoWritePage(uint8_t *_ucpSrc, uint32_t _uiWrAddr, uint16_t _usWrLen)
{
    uint16_t i;
    uint16_t j;           /* 用于延时 */
    uint32_t uiFirstAddr; /* 扇区首址 */
    uint8_t ucNeedErase;  /* 1表示需要擦除 */
    uint8_t cRet;

    /* 长度为0时不继续操作,直接认为成功 */
    if (_usWrLen == 0)
    {
        return 1;
    }

    /* 如果偏移地址超过芯片容量则退出 */
    if (_uiWrAddr >= g_tSF.TotalSize)
    {
        return 0;
    }

    /* 如果数据长度大于扇区容量，则退出 */
    if (_usWrLen > g_tSF.PageSize)
    {
        return 0;
    }

    /* 如果FLASH中的数据没有变化,则不写FLASH */
    sf_ReadBuffer(s_spiBuf, _uiWrAddr, _usWrLen);
    if (memcmp(s_spiBuf, _ucpSrc, _usWrLen) == 0)
    {
        return 1;
    }

    /* 判断是否需要先擦除扇区 */
    /* 如果旧数据修改为新数据，所有位均是 1->0 或者 0->0, 则无需擦除,提高Flash寿命 */
    ucNeedErase = 0;
    if (sf_NeedErase(s_spiBuf, _ucpSrc, _usWrLen))
    {
        ucNeedErase = 1;
    }

    uiFirstAddr = _uiWrAddr & (~(g_tSF.PageSize - 1));

    if (_usWrLen == g_tSF.PageSize) /* 整个扇区都改写 */
    {
        for (i = 0; i < g_tSF.PageSize; i++)
        {
            s_spiBuf[i] = _ucpSrc[i];
        }
    }
    else /* 改写部分数据 */
    {
        /* 先将整个扇区的数据读出 */
        sf_ReadBuffer(s_spiBuf, uiFirstAddr, g_tSF.PageSize);

        /* 再用新数据覆盖 */
        i = _uiWrAddr & (g_tSF.PageSize - 1);
        memcpy(&s_spiBuf[i], _ucpSrc, _usWrLen);
    }

    /* 写完之后进行校验，如果不正确则重写，最多3次 */
    cRet = 0;
    for (i = 0; i < 3; i++)
    {

        /* 如果旧数据修改为新数据，所有位均是 1->0 或者 0->0, 则无需擦除,提高Flash寿命 */
        if (ucNeedErase == 1)
        {
            // ESP_LOGE("auto write", "earse sector");
            spi_flash_erase_sector(uiFirstAddr / 4096); /* 擦除1个扇区 */
        }

        /* 编程一个PAGE */
        spi_flash_write(uiFirstAddr, s_spiBuf, g_tSF.PageSize);

        if (sf_CmpData(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
        {
            cRet = 1;
            break;
        }
        else
        {
            if (sf_CmpData(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
            {
                cRet = 1;
                break;
            }

            /* 失败后延迟一段时间再重试 */
            for (j = 0; j < 10000; j++)
                ;
        }
    }

    return cRet;
}
/*
*********************************************************************************************************
*	函 数 名: sf_WriteBuffer
*	功能说明: 写1个扇区并校验,如果不正确则再重写两次。本函数自动完成擦除操作。
*	形    参:  	_pBuf : 数据源缓冲区；
*				_uiWrAddr ：目标区域首地址
*				_usSize ：数据个数，不能超过页面大小
*	返 回 值: 1 : 成功， 0 ： 失败
*********************************************************************************************************
*/
uint8_t sf_WriteBuffer(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize)
{
    uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = _uiWriteAddr % g_tSF.PageSize;
    count = g_tSF.PageSize - Addr;
    NumOfPage = _usWriteSize / g_tSF.PageSize;
    NumOfSingle = _usWriteSize % g_tSF.PageSize;

    if (Addr == 0) /* 起始地址是页面首地址  */
    {
        if (NumOfPage == 0) /* 数据长度小于页面大小 */
        {
            if (sf_AutoWritePage(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
            {
                return 0;
            }
        }
        else /* 数据长度大于等于页面大小 */
        {
            while (NumOfPage--)
            {
                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, g_tSF.PageSize) == 0)
                {
                    return 0;
                }
                _uiWriteAddr += g_tSF.PageSize;
                _pBuf += g_tSF.PageSize;
            }
            if (sf_AutoWritePage(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
            {
                return 0;
            }
        }
    }
    else /* 起始地址不是页面首地址  */
    {
        if (NumOfPage == 0) /* 数据长度小于页面大小 */
        {
            if (NumOfSingle > count) /* (_usWriteSize + _uiWriteAddr) > SPI_FLASH_PAGESIZE */
            {
                temp = NumOfSingle - count;

                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, count) == 0)
                {
                    return 0;
                }

                _uiWriteAddr += count;
                _pBuf += count;

                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, temp) == 0)
                {
                    return 0;
                }
            }
            else
            {
                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
                {
                    return 0;
                }
            }
        }
        else /* 数据长度大于等于页面大小 */
        {
            _usWriteSize -= count;
            NumOfPage = _usWriteSize / g_tSF.PageSize;
            NumOfSingle = _usWriteSize % g_tSF.PageSize;

            if (sf_AutoWritePage(_pBuf, _uiWriteAddr, count) == 0)
            {
                return 0;
            }

            _uiWriteAddr += count;
            _pBuf += count;

            while (NumOfPage--)
            {
                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, g_tSF.PageSize) == 0)
                {
                    return 0;
                }
                _uiWriteAddr += g_tSF.PageSize;
                _pBuf += g_tSF.PageSize;
            }

            if (NumOfSingle != 0)
            {
                if (sf_AutoWritePage(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
                {
                    return 0;
                }
            }
        }
    }
    return 1; /* 成功 */
}
