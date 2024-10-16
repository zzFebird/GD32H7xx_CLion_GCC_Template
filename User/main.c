/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       USB���⴮��(Slave) ʵ��
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
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"


usb_core_driver cdc_acm;


int main(void)
{
    uint16_t len;
    uint16_t times = 0;
    uint8_t usbstatus = 0;
  
    sys_cache_enable();                     /* ʹ��CPU cache */
    delay_init(600);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ��ʼ������ */  
    led_init();							    /* ��ʼ��LED */   
    mpu_memory_protection();                /* ������ش洢���� */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* ��ʼ��SDRAM */

    my_mem_init(SRAMIN);                    /* ��ʼ���ڲ�SRAM�ڴ�� */
    my_mem_init(SRAMEX);                    /* ��ʼ���ⲿSDRAM�ڴ�� */

    usb_rcu_config();                                                      /* ����USBʱ�� */

    usb_para_init (&cdc_acm, USBHS0, USB_SPEED_FULL);                      /* ����USB������USBHS0ʹ��ȫ��ģʽ */

    usbd_init (&cdc_acm, &cdc_desc, &cdc_class);                           /* ��ʼ��USB�豸������� */

    usb_intr_config();                                                     /* ����USB�ж� */
    delay_ms(1000);
    
    while (1)
    {        
        if (usbstatus != cdc_acm.dev.cur_status)    /* USB����״̬�����˸ı� */
        {
            usbstatus = cdc_acm.dev.cur_status;     /* ��¼�µ�״̬ */

            if (USBD_CONFIGURED == usbstatus)
            {
                usbd_ep_recev (&cdc_acm, CDC_DATA_OUT_EP, g_usb_rx_buffer, USB_USART_REC_LEN); /* ���ӳɹ���˵�׼���������� */
                printf("USB Connected\n"); /* ��ʾUSB���ӳɹ� */
                LED1(0);                                                         /* LED1�� */
            }
            else
            {
                printf("USB disConnected\n"); /* ��ʾUSB�Ͽ� */
                LED1(1);                                                         /* LED1�� */
            }
        }
        if (USBD_CONFIGURED == cdc_acm.dev.cur_status)
        {
            if (g_usb_usart_rx_sta & 0x8000)
            {
                len = g_usb_usart_rx_sta & 0x3FFF;                               /* �õ��˴ν��յ������ݳ��� */
                usb_printf("\r\n�����͵���Ϣ����Ϊ:%d\r\n\r\n", len);
                cdc_vcp_data_tx(g_usb_usart_rx_buffer, len);
                usb_printf("\r\n\r\n");                                          /* ���뻻�� */
                g_usb_usart_rx_sta = 0;
            }
            else
            {
                times++;

                if (times % 5000 == 0)
                {
                    usb_printf("\r\n������Զ GD32H757ZMT6Сϵͳ��USB���⴮��ʵ��\r\n");
                    usb_printf("������Զ@Waiken-Smart\r\n\r\n\r\n");
                }

                if (times % 200 == 0)
                {
                    usb_printf("����������,�Իس�������\r\n");
                }

                if (times % 30 == 0)
                {
                    LED0_TOGGLE();  /* LED0��˸,��ʾϵͳ�������� */
                }
                
                delay_ms(10);
            }
        }    
    }
}


