/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       USB������(SLAVE) ʵ��
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
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

extern volatile uint8_t g_usb_state_reg;    /* USB״̬ */
extern volatile uint8_t g_device_state;     /* USB���� ��� */


int main(void)
{
    uint8_t offline_cnt = 0;
    uint8_t tct = 0;
    uint8_t usb_sta;
    uint8_t device_sta;
    uint16_t id;
  
    sys_cache_enable();                     /* ʹ��CPU cache */
    delay_init(600);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ��ʼ������ */  
    led_init();							                /* ��ʼ��LED */   
    mpu_memory_protection();                /* ������ش洢���� */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* ��ʼ��SDRAM */
    norflash_init();                        /* ��ʼ��NORFLASH */

    my_mem_init(SRAMIN);                    /* ��ʼ���ڲ�SRAM�ڴ�� */
    my_mem_init(SRAMEX);                    /* ��ʼ���ⲿSDRAM�ڴ�� */

    printf("GD32H757\n");
    printf("USB Card Reader TEST\n");
    printf("WKS SMART\n");

    if (SD_OK != sdio_sd_init())    /* ��ʼ��SD�� */
    {
        printf("SD Card Error!\n");      /* ���SD������ */
    }
    else   /* SD ������ */
    {
        printf("SD Card Size: %lu MB\n", sd_card_capacity_get() >> 10);
    }
        
    id = norflash_read_id();
    
    if (id != W25Q128)
    {
        printf("SPI FLASH Error!\n");    /* ���SPI FLASH���� */
    }
    else   /* SPI FLASH ���� */
    {
        printf("SPI FLASH Size:9MB\n");
    }

    printf("USB Connecting...\n");       /* ��ʾ���ڽ������� */

    usb_rcu_config();                                                      /* ����USBʱ�� */

    usb_para_init (&msc_udisk, USBHS0, USB_SPEED_FULL);                    /* ����USB������USBHS0ʹ��ȫ��ģʽ */

    usbd_init (&msc_udisk, &msc_desc, &msc_class);                         /* ��ʼ��USB�豸������� */

    usb_intr_config();                                                     /* ����USB�ж� */
    delay_ms(1800);
    
    while (1)
    {        
        delay_ms(1);

        if (usb_sta != g_usb_state_reg)   /* ״̬�ı��� */
        {
            if (g_usb_state_reg & 0x01)   /* ����д */
            {
                LED1(0);
                printf("USB Writing...\n"); /* ��ʾUSB����д������ */
            }

            if (g_usb_state_reg & 0x02)   /* ���ڶ� */
            {
                LED1(0);
                printf("USB Reading...\n"); /* ��ʾUSB���ڶ������� */
            }

            if (g_usb_state_reg & 0x04)
            {
                printf("USB Write Err \n"); /* ��ʾд����� */
            }
            
            if (g_usb_state_reg & 0x08)
            {
                printf("USB Read  Err \n"); /* ��ʾ�������� */
            }
            
            usb_sta = g_usb_state_reg; /* ��¼����״̬ */
        }

        if (device_sta != g_device_state)
        {
            if (g_device_state == 1)
            {
                printf("USB Connected    \n");    /* ��ʾUSB�����Ѿ����� */
            }
            else
            {
                printf("USB DisConnected \n");    /* ��ʾUSB���γ��� */
            }
            
            device_sta = g_device_state;
        }

        tct++;

        if (tct == 200)
        {
            tct = 0;
            LED1(1);        /* �ر� LED1 */
            LED0_TOGGLE();  /* LED0 ��˸ */

            if (g_usb_state_reg & 0x10)
            {
                offline_cnt = 0;    /* USB������,�����offline������ */
                g_device_state = 1;
            }
            else    /* û�еõ���ѯ */
            {
                offline_cnt++;

                if (offline_cnt > 10)
                {
                    g_device_state = 0;/* 2s��û�յ����߱��,����USB���γ��� */
                }
            }

            g_usb_state_reg = 0;
        }     
    }
}


