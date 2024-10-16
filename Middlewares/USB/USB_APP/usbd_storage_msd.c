/*!
    \file    usbd_storage_msd.c
    \brief   this file provides the disk operations functions

    \version 2024-01-05, V1.2.0, firmware for GD32H7xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "usb_conf.h"
#include "usbd_msc_mem.h"
#include "./BSP/NORFLASH/norflash.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./BSP/SDIO/sd_conf.h"
#include "./SYSTEM/usart/usart.h"


/* �ļ�ϵͳ���ⲿFLASH����ʼ��ַ
 * ���Ƕ���SPI FLASHǰ9M���ļ�ϵͳ��, ���Ե�ַ��0��ʼ
 */
#define USB_STORAGE_FLASH_BASE       0

/* ����SPI FLASH��д���С */
#define SPI_FLASH_BLOCK_SIZE         512


/* USB����״̬
 * 0,û������;
 * 1,�Ѿ�����;
 */
volatile uint8_t g_device_state = 0;    /* Ĭ��û������ */


/* �Լ������һ�����USB״̬�ļĴ���, �����ж�USB״̬
 * bit0 : ��ʾ����������SD��д������
 * bit1 : ��ʾ��������SD����������
 * bit2 : SD��д���ݴ����־λ
 * bit3 : SD�������ݴ����־λ
 * bit4 : 1,��ʾ��������ѯ����(�������ӻ�������)
 */
volatile uint8_t g_usb_state_reg = 0;

/* USB mass storage standard inquiry data */

const int8_t STORAGE_InquiryData[] = 
{
    /* LUN 0 */
    0x00,
    0x80,
    0x00,
    0x01,
    (USBD_STD_INQUIRY_LENGTH - 5U),
    0x00,
    0x00,
    0x00,
    'G', 'D', '3', '2', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
    'S', 'P', 'I', ' ', 'F', 'l', 'a', 's', /* Product      : 16 Bytes */
    'h', ' ', 'd', 'i', 's', 'k', ' ', ' ',
    '1', '.', '0' ,'0',                     /* Version      : 4 Bytes */
};

/* USB mass storage standard inquiry data */

const int8_t STORAGE_InquiryData1[] = 
{
    /* LUN 0 */
    0x00,
    0x80,
    0x00,
    0x01,
    (USBD_STD_INQUIRY_LENGTH - 5U),
    0x00,
    0x00,
    0x00,
    'G', 'D', '3', '2', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
    'S', 'D', ' ', ' ', 'F', 'l', 'a', 's', /* Product      : 16 Bytes */
    'h', ' ', 'd', 'i', 's', 'k', ' ', ' ',
    '1', '.', '0' ,'0',                     /* Version      : 4 Bytes */
};

/* local function prototypes ('static') */
static int8_t  STORAGE_Init             (uint8_t Lun);
static int8_t  STORAGE_IsReady          (uint8_t Lun);
static int8_t  STORAGE_IsWriteProtected (uint8_t Lun);
static int8_t  STORAGE_GetMaxLun        (void);

static int8_t  STORAGE_Read             (uint8_t Lun,
                                        uint8_t *buf,
                                        uint32_t BlkAddr,
                                        uint16_t BlkLen);

static int8_t  STORAGE_Write            (uint8_t Lun,
                                        uint8_t *buf,
                                        uint32_t BlkAddr,
                                        uint16_t BlkLen);


usbd_mem_cb USBD_Internal_Storage_fops =
{
    .mem_init      = STORAGE_Init,
    .mem_ready     = STORAGE_IsReady,
    .mem_protected = STORAGE_IsWriteProtected,
    .mem_read      = STORAGE_Read,
    .mem_write     = STORAGE_Write,
    .mem_maxlun    = STORAGE_GetMaxLun,

    .mem_inquiry_data = {(uint8_t *)STORAGE_InquiryData, (uint8_t *)STORAGE_InquiryData1},
};

usbd_mem_cb *usbd_mem_fops = &USBD_Internal_Storage_fops;


/**
 * @brief       ��ʼ���洢�豸
 * @param       Lun        : �߼���Ԫ���
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD��
 * @retval      �������
 *   @arg       0    , �ɹ�
 *   @arg       ���� , �������
 */
static int8_t  STORAGE_Init (uint8_t Lun)
{
    uint8_t res = 0;
    
    switch (Lun)
    {
        case 0: /* SPI FLASH */
            usbd_mem_fops->mem_block_size[Lun] = SPI_FLASH_BLOCK_SIZE;
            usbd_mem_fops->mem_block_len[Lun] = (9 * 1024 * 1024) / SPI_FLASH_BLOCK_SIZE;   /* SPI FLASH��ǰ9M�ֽ�,�ļ�ϵͳ�� */
            norflash_init();
            break;

        case 1: /* SD�� */
            usbd_mem_fops->mem_block_size[Lun] = 512;
            usbd_mem_fops->mem_block_len[Lun] = sd_card_capacity_get() * 2;
            res = sdio_sd_init();
        
            if (res == SD_OK)  
            {
                res = 0;
            }  
            
            break;
    }
    
    return res;
}

/**
 * @brief       �鿴�洢�豸�Ƿ����
 * @param       Lun        : �߼���Ԫ���
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD��
 * @retval      ����״̬
 *   @arg       0    , ����
 *   @arg       ���� , δ����
 */
static int8_t  STORAGE_IsReady (uint8_t Lun)
{
    g_usb_state_reg |= 0X10;    /* �����ѯ */

    return 0;
}

/**
 * @brief       �鿴�洢�豸�Ƿ�д����
 * @param       Lun        : �߼���Ԫ���
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD��
 * @retval      д����״̬
 *   @arg       0    , û��д����
 *   @arg       ���� , ��д����
 */
static int8_t  STORAGE_IsWriteProtected (uint8_t Lun)
{
    return 0;
}

/**
 * @brief       �Ӵ洢�豸��ȡ����
 * @param       Lun        : �߼���Ԫ���
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD��
 * @param       buf        : ���ݴ洢���׵�ַָ��
 * @param       BlkAddr    : Ҫ��ȡ�ĵ�ַ(�ֽڵ�ַ)
 * @param       BlkLen     : Ҫ��ȡ�Ŀ���(������)
 * @retval      �������
 *   @arg       0    , �ɹ�
 *   @arg       ���� , �������
 */
static int8_t  STORAGE_Read (uint8_t Lun,
                            uint8_t *buf,
                            uint32_t BlkAddr,
                            uint16_t BlkLen)
{
    uint8_t res = 0;
    g_usb_state_reg |= 0X02;    /* ������ڶ����� */

    switch (Lun)
    {
        case 0: /* SPI FLASH */
            norflash_read(buf, USB_STORAGE_FLASH_BASE + BlkAddr, BlkLen * SPI_FLASH_BLOCK_SIZE);
            break;

        case 1: /* SD�� */
            res = sd_read_udisk(buf, BlkAddr, BlkLen);
           
            if (res == SD_OK)  
            {
                res = 0;
            }  

            break;
    }  

    if (res)
    {
        printf("rerr:%d,%d\r\n", Lun, res);
        g_usb_state_reg |= 0X08;    /* ������! */
    }
      
    return res;
}

/**
 * @brief       �Ӵ洢�豸д����
 * @param       Lun        : �߼���Ԫ���
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD��
 * @param       buf        : ���ݴ洢���׵�ַָ��
 * @param       BlkAddr    : Ҫд��ĵ�ַ(�ֽڵ�ַ)
 * @param       BlkLen     : Ҫд��Ŀ���(������)
 * @retval      �������
 *   @arg       0    , �ɹ�
 *   @arg       ���� , �������
 */
static int8_t  STORAGE_Write (uint8_t Lun,
                             uint8_t *buf,
                             uint32_t BlkAddr,
                             uint16_t BlkLen)
{
    int8_t res = 0;
    g_usb_state_reg |= 0X01;    /* �������д���� */

    switch (Lun)
    {
        case 0: /* SPI FLASH */
            norflash_write(buf, USB_STORAGE_FLASH_BASE + BlkAddr, BlkLen * SPI_FLASH_BLOCK_SIZE);
            break;

        case 1: /* SD�� */
            res = sd_write_udisk(buf, BlkAddr, BlkLen);
        
            if (res == SD_OK)  
            {
                res = 0;
            }  
            
            break;
    }
    
    if (res)
    {
        g_usb_state_reg |= 0X04;    /* д����! */
        printf("werr:%d,%d", Lun, res);
    }

    return res;
}

/**
 * @brief       ��ȡ֧�ֵ�����߼���Ԫ����
 *   @note      ע��, ���ﷵ�ص��߼���Ԫ�����Ǽ�ȥ��1��.
 *              0, �ͱ�ʾ1��; 1, ��ʾ2��; �Դ�����
 * @param       ��
 * @retval      ֧�ֵ��߼���Ԫ���� - 1
 */
static int8_t STORAGE_GetMaxLun (void)
{
    /* MEM_LUN_NUM ��usbd_conf.h���涨��, ��ʾ֧�ֵ��߼���Ԫ���� */
    return (MEM_LUN_NUM - 1);
}





