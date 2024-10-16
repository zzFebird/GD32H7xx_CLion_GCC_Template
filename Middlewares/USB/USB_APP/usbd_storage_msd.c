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


/* 文件系统在外部FLASH的起始地址
 * 我们定义SPI FLASH前9M给文件系统用, 所以地址从0开始
 */
#define USB_STORAGE_FLASH_BASE       0

/* 定义SPI FLASH读写块大小 */
#define SPI_FLASH_BLOCK_SIZE         512


/* USB连接状态
 * 0,没有连接;
 * 1,已经连接;
 */
volatile uint8_t g_device_state = 0;    /* 默认没有连接 */


/* 自己定义的一个标记USB状态的寄存器, 方便判断USB状态
 * bit0 : 表示电脑正在向SD卡写入数据
 * bit1 : 表示电脑正从SD卡读出数据
 * bit2 : SD卡写数据错误标志位
 * bit3 : SD卡读数据错误标志位
 * bit4 : 1,表示电脑有轮询操作(表明连接还保持着)
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
 * @brief       初始化存储设备
 * @param       Lun        : 逻辑单元编号
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD卡
 * @retval      操作结果
 *   @arg       0    , 成功
 *   @arg       其他 , 错误代码
 */
static int8_t  STORAGE_Init (uint8_t Lun)
{
    uint8_t res = 0;
    
    switch (Lun)
    {
        case 0: /* SPI FLASH */
            usbd_mem_fops->mem_block_size[Lun] = SPI_FLASH_BLOCK_SIZE;
            usbd_mem_fops->mem_block_len[Lun] = (9 * 1024 * 1024) / SPI_FLASH_BLOCK_SIZE;   /* SPI FLASH的前9M字节,文件系统用 */
            norflash_init();
            break;

        case 1: /* SD卡 */
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
 * @brief       查看存储设备是否就绪
 * @param       Lun        : 逻辑单元编号
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD卡
 * @retval      就绪状态
 *   @arg       0    , 就绪
 *   @arg       其他 , 未就绪
 */
static int8_t  STORAGE_IsReady (uint8_t Lun)
{
    g_usb_state_reg |= 0X10;    /* 标记轮询 */

    return 0;
}

/**
 * @brief       查看存储设备是否写保护
 * @param       Lun        : 逻辑单元编号
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD卡
 * @retval      写保护状态
 *   @arg       0    , 没有写保护
 *   @arg       其他 , 有写保护
 */
static int8_t  STORAGE_IsWriteProtected (uint8_t Lun)
{
    return 0;
}

/**
 * @brief       从存储设备读取数据
 * @param       Lun        : 逻辑单元编号
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD卡
 * @param       buf        : 数据存储区首地址指针
 * @param       BlkAddr    : 要读取的地址(字节地址)
 * @param       BlkLen     : 要读取的块数(扇区数)
 * @retval      操作结果
 *   @arg       0    , 成功
 *   @arg       其他 , 错误代码
 */
static int8_t  STORAGE_Read (uint8_t Lun,
                            uint8_t *buf,
                            uint32_t BlkAddr,
                            uint16_t BlkLen)
{
    uint8_t res = 0;
    g_usb_state_reg |= 0X02;    /* 标记正在读数据 */

    switch (Lun)
    {
        case 0: /* SPI FLASH */
            norflash_read(buf, USB_STORAGE_FLASH_BASE + BlkAddr, BlkLen * SPI_FLASH_BLOCK_SIZE);
            break;

        case 1: /* SD卡 */
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
        g_usb_state_reg |= 0X08;    /* 读错误! */
    }
      
    return res;
}

/**
 * @brief       从存储设备写数据
 * @param       Lun        : 逻辑单元编号
 *   @arg                  0, SPI FLASH
 *   @arg                  1, SD卡
 * @param       buf        : 数据存储区首地址指针
 * @param       BlkAddr    : 要写入的地址(字节地址)
 * @param       BlkLen     : 要写入的块数(扇区数)
 * @retval      操作结果
 *   @arg       0    , 成功
 *   @arg       其他 , 错误代码
 */
static int8_t  STORAGE_Write (uint8_t Lun,
                             uint8_t *buf,
                             uint32_t BlkAddr,
                             uint16_t BlkLen)
{
    int8_t res = 0;
    g_usb_state_reg |= 0X01;    /* 标记正在写数据 */

    switch (Lun)
    {
        case 0: /* SPI FLASH */
            norflash_write(buf, USB_STORAGE_FLASH_BASE + BlkAddr, BlkLen * SPI_FLASH_BLOCK_SIZE);
            break;

        case 1: /* SD卡 */
            res = sd_write_udisk(buf, BlkAddr, BlkLen);
        
            if (res == SD_OK)  
            {
                res = 0;
            }  
            
            break;
    }
    
    if (res)
    {
        g_usb_state_reg |= 0X04;    /* 写错误! */
        printf("werr:%d,%d", Lun, res);
    }

    return res;
}

/**
 * @brief       获取支持的最大逻辑单元个数
 *   @note      注意, 这里返回的逻辑单元个数是减去了1的.
 *              0, 就表示1个; 1, 表示2个; 以此类推
 * @param       无
 * @retval      支持的逻辑单元个数 - 1
 */
static int8_t STORAGE_GetMaxLun (void)
{
    /* MEM_LUN_NUM 在usbd_conf.h里面定义, 表示支持的逻辑单元个数 */
    return (MEM_LUN_NUM - 1);
}





