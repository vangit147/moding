#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "spiff.h"
static const char *TAG = "spi_flash";
#define Sector 4096
#define PSector 49152

SFLASH_T g_tSF;
static uint8_t s_spiBuf[4 * 1024];
/*
*********************************************************************************************************
*	�� �� ��: sf_AutoWritePage
*	����˵��: д1��PAGE��У��,�������ȷ������д���Ρ��������Զ���ɲ���������
*	��    ��:  	_pBuf : ����Դ��������
*				_uiWriteAddr ��Ŀ�������׵�ַ
*				_usSize �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 0 : ���� 1 �� �ɹ�
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
*	�� �� ��: sf_CmpData
*	����˵��: �Ƚ�Flash������.
*	��    ��:  	_ucpTar : ���ݻ�����
*				_uiSrcAddr ��Flash��ַ
*				_uiSize �����ݸ���, ���Դ���PAGE_SIZE,���ǲ��ܳ���оƬ������
*	�� �� ֵ: 0 = ���, 1 = ����
*********************************************************************************************************
*/
static uint8_t sf_CmpData(uint32_t _uiSrcAddr, uint8_t *_ucpTar, uint32_t _uiSize)
{
    uint8_t ucValue[1];
    // ESP_LOGE("CmpData:", "_uiSrcAddr=%d,_uiSize=%d", _uiSrcAddr, _uiSize);
    /* �����ȡ�����ݳ���Ϊ0���߳�������Flash��ַ�ռ䣬��ֱ�ӷ��� */
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
        /* ��һ���ֽ� */
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
*	�� �� ��: sf_NeedErase
*	����˵��: �ж�дPAGEǰ�Ƿ���Ҫ�Ȳ�����
*	��    ��:   _ucpOldBuf �� ������
*			   _ucpNewBuf �� ������
*			   _uiLen �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 0 : ����Ҫ������ 1 ����Ҫ����
*********************************************************************************************************
*/
static uint8_t sf_NeedErase(uint8_t *_ucpOldBuf, uint8_t *_ucpNewBuf, uint16_t _usLen)
{
    uint16_t i;
    uint8_t ucOld;

    /*
	�㷨��1����old ��, new ����
	      old    new
		  1101   0101
	~     1111
		= 0010   0101

	�㷨��2��: old �󷴵Ľ���� new λ��
		  0010   old
	&	  0101   new
		 =0000

	�㷨��3��: ���Ϊ0,���ʾ�������. �����ʾ��Ҫ����
	*/

    for (i = 0; i < _usLen; i++)
    {
        ucOld = *_ucpOldBuf++;
        ucOld = ~ucOld;

        /* ע������д��: if (ucOld & (*_ucpNewBuf++) != 0) */
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
    uint16_t j;           /* ������ʱ */
    uint32_t uiFirstAddr; /* ������ַ */
    uint8_t ucNeedErase;  /* 1��ʾ��Ҫ���� */
    uint8_t cRet;

    /* ����Ϊ0ʱ����������,ֱ����Ϊ�ɹ� */
    if (_usWrLen == 0)
    {
        return 1;
    }

    /* ���ƫ�Ƶ�ַ����оƬ�������˳� */
    if (_uiWrAddr >= g_tSF.TotalSize)
    {
        return 0;
    }

    /* ������ݳ��ȴ����������������˳� */
    if (_usWrLen > g_tSF.PageSize)
    {
        return 0;
    }

    /* ���FLASH�е�����û�б仯,��дFLASH */
    sf_ReadBuffer(s_spiBuf, _uiWrAddr, _usWrLen);
    if (memcmp(s_spiBuf, _ucpSrc, _usWrLen) == 0)
    {
        return 1;
    }

    /* �ж��Ƿ���Ҫ�Ȳ������� */
    /* ����������޸�Ϊ�����ݣ�����λ���� 1->0 ���� 0->0, ���������,���Flash���� */
    ucNeedErase = 0;
    if (sf_NeedErase(s_spiBuf, _ucpSrc, _usWrLen))
    {
        ucNeedErase = 1;
    }

    uiFirstAddr = _uiWrAddr & (~(g_tSF.PageSize - 1));

    if (_usWrLen == g_tSF.PageSize) /* ������������д */
    {
        for (i = 0; i < g_tSF.PageSize; i++)
        {
            s_spiBuf[i] = _ucpSrc[i];
        }
    }
    else /* ��д�������� */
    {
        /* �Ƚ��������������ݶ��� */
        sf_ReadBuffer(s_spiBuf, uiFirstAddr, g_tSF.PageSize);

        /* ���������ݸ��� */
        i = _uiWrAddr & (g_tSF.PageSize - 1);
        memcpy(&s_spiBuf[i], _ucpSrc, _usWrLen);
    }

    /* д��֮�����У�飬�������ȷ����д�����3�� */
    cRet = 0;
    for (i = 0; i < 3; i++)
    {

        /* ����������޸�Ϊ�����ݣ�����λ���� 1->0 ���� 0->0, ���������,���Flash���� */
        if (ucNeedErase == 1)
        {
            // ESP_LOGE("auto write", "earse sector");
            spi_flash_erase_sector(uiFirstAddr / 4096); /* ����1������ */
        }

        /* ���һ��PAGE */
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

            /* ʧ�ܺ��ӳ�һ��ʱ�������� */
            for (j = 0; j < 10000; j++)
                ;
        }
    }

    return cRet;
}
/*
*********************************************************************************************************
*	�� �� ��: sf_WriteBuffer
*	����˵��: д1��������У��,�������ȷ������д���Ρ��������Զ���ɲ���������
*	��    ��:  	_pBuf : ����Դ��������
*				_uiWrAddr ��Ŀ�������׵�ַ
*				_usSize �����ݸ��������ܳ���ҳ���С
*	�� �� ֵ: 1 : �ɹ��� 0 �� ʧ��
*********************************************************************************************************
*/
uint8_t sf_WriteBuffer(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize)
{
    uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = _uiWriteAddr % g_tSF.PageSize;
    count = g_tSF.PageSize - Addr;
    NumOfPage = _usWriteSize / g_tSF.PageSize;
    NumOfSingle = _usWriteSize % g_tSF.PageSize;

    if (Addr == 0) /* ��ʼ��ַ��ҳ���׵�ַ  */
    {
        if (NumOfPage == 0) /* ���ݳ���С��ҳ���С */
        {
            if (sf_AutoWritePage(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
            {
                return 0;
            }
        }
        else /* ���ݳ��ȴ��ڵ���ҳ���С */
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
    else /* ��ʼ��ַ����ҳ���׵�ַ  */
    {
        if (NumOfPage == 0) /* ���ݳ���С��ҳ���С */
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
        else /* ���ݳ��ȴ��ڵ���ҳ���С */
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
    return 1; /* �ɹ� */
}
