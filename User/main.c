/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       USB读卡器(SLAVE) 实验
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/SDRAM/sdram.h"
#include "./BSP/KEY/key.h"
#include "./MALLOC/malloc.h"
#include "./BSP/SDIO/sd_conf.h"
#include "./BSP/NORFLASH/norflash.h"
#include "drv_usb_hw.h"
#include "usbd_msc_core.h"


usb_core_driver msc_udisk;

extern volatile uint8_t g_usb_state_reg;    /* USB状态 */
extern volatile uint8_t g_device_state;     /* USB连接 情况 */


int main(void)
{
    uint8_t offline_cnt = 0;
    uint8_t tct = 0;
    uint8_t usb_sta;
    uint8_t device_sta;
    uint16_t id;
  
    sys_cache_enable();                     /* 使能CPU cache */
    delay_init(600);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化串口 */  
    led_init();							                /* 初始化LED */   
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* 初始化SDRAM */
    norflash_init();                        /* 初始化NORFLASH */

    my_mem_init(SRAMIN);                    /* 初始化内部SRAM内存池 */
    my_mem_init(SRAMEX);                    /* 初始化外部SDRAM内存池 */

    printf("GD32H757\n");
    printf("USB Card Reader TEST\n");
    printf("WKS SMART\n");

    if (SD_OK != sdio_sd_init())    /* 初始化SD卡 */
    {
        printf("SD Card Error!\n");      /* 检测SD卡错误 */
    }
    else   /* SD 卡正常 */
    {
        printf("SD Card Size: %lu MB\n", sd_card_capacity_get() >> 10);
    }
        
    id = norflash_read_id();
    
    if (id != W25Q128)
    {
        printf("SPI FLASH Error!\n");    /* 检测SPI FLASH错误 */
    }
    else   /* SPI FLASH 正常 */
    {
        printf("SPI FLASH Size:9MB\n");
    }

    printf("USB Connecting...\n");       /* 提示正在建立连接 */

    usb_rcu_config();                                                      /* 配置USB时钟 */

    usb_para_init (&msc_udisk, USBHS0, USB_SPEED_FULL);                    /* 配置USB参数，USBHS0使用全速模式 */

    usbd_init (&msc_udisk, &msc_desc, &msc_class);                         /* 初始化USB设备并添加类 */

    usb_intr_config();                                                     /* 配置USB中断 */
    delay_ms(1800);
    
    while (1)
    {        
        delay_ms(1);

        if (usb_sta != g_usb_state_reg)   /* 状态改变了 */
        {
            if (g_usb_state_reg & 0x01)   /* 正在写 */
            {
                LED1(0);
                printf("USB Writing...\n"); /* 提示USB正在写入数据 */
            }

            if (g_usb_state_reg & 0x02)   /* 正在读 */
            {
                LED1(0);
                printf("USB Reading...\n"); /* 提示USB正在读出数据 */
            }

            if (g_usb_state_reg & 0x04)
            {
                printf("USB Write Err \n"); /* 提示写入错误 */
            }
            
            if (g_usb_state_reg & 0x08)
            {
                printf("USB Read  Err \n"); /* 提示读出错误 */
            }
            
            usb_sta = g_usb_state_reg; /* 记录最后的状态 */
        }

        if (device_sta != g_device_state)
        {
            if (g_device_state == 1)
            {
                printf("USB Connected    \n");    /* 提示USB连接已经建立 */
            }
            else
            {
                printf("USB DisConnected \n");    /* 提示USB被拔出了 */
            }
            
            device_sta = g_device_state;
        }

        tct++;

        if (tct == 200)
        {
            tct = 0;
            LED1(1);        /* 关闭 LED1 */
            LED0_TOGGLE();  /* LED0 闪烁 */

            if (g_usb_state_reg & 0x10)
            {
                offline_cnt = 0;    /* USB连接了,则清除offline计数器 */
                g_device_state = 1;
            }
            else    /* 没有得到轮询 */
            {
                offline_cnt++;

                if (offline_cnt > 10)
                {
                    g_device_state = 0;/* 2s内没收到在线标记,代表USB被拔出了 */
                }
            }

            g_usb_state_reg = 0;
        }     
    }
}


