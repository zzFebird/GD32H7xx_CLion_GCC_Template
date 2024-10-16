/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       USB虚拟串口(Slave) 实验
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
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"


usb_core_driver cdc_acm;


int main(void)
{
    uint16_t len;
    uint16_t times = 0;
    uint8_t usbstatus = 0;
  
    sys_cache_enable();                     /* 使能CPU cache */
    delay_init(600);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化串口 */  
    led_init();							    /* 初始化LED */   
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* 初始化SDRAM */

    my_mem_init(SRAMIN);                    /* 初始化内部SRAM内存池 */
    my_mem_init(SRAMEX);                    /* 初始化外部SDRAM内存池 */

    usb_rcu_config();                                                      /* 配置USB时钟 */

    usb_para_init (&cdc_acm, USBHS0, USB_SPEED_FULL);                      /* 配置USB参数，USBHS0使用全速模式 */

    usbd_init (&cdc_acm, &cdc_desc, &cdc_class);                           /* 初始化USB设备并添加类 */

    usb_intr_config();                                                     /* 配置USB中断 */
    delay_ms(1000);
    
    while (1)
    {        
        if (usbstatus != cdc_acm.dev.cur_status)    /* USB连接状态发生了改变 */
        {
            usbstatus = cdc_acm.dev.cur_status;     /* 记录新的状态 */

            if (USBD_CONFIGURED == usbstatus)
            {
                usbd_ep_recev (&cdc_acm, CDC_DATA_OUT_EP, g_usb_rx_buffer, USB_USART_REC_LEN); /* 连接成功后端点准备接收数据 */
                printf("USB Connected\n"); /* 提示USB连接成功 */
                LED1(0);                                                         /* LED1亮 */
            }
            else
            {
                printf("USB disConnected\n"); /* 提示USB断开 */
                LED1(1);                                                         /* LED1灭 */
            }
        }
        if (USBD_CONFIGURED == cdc_acm.dev.cur_status)
        {
            if (g_usb_usart_rx_sta & 0x8000)
            {
                len = g_usb_usart_rx_sta & 0x3FFF;                               /* 得到此次接收到的数据长度 */
                usb_printf("\r\n您发送的消息长度为:%d\r\n\r\n", len);
                cdc_vcp_data_tx(g_usb_usart_rx_buffer, len);
                usb_printf("\r\n\r\n");                                          /* 插入换行 */
                g_usb_usart_rx_sta = 0;
            }
            else
            {
                times++;

                if (times % 5000 == 0)
                {
                    usb_printf("\r\n慧勤智远 GD32H757ZMT6小系统板USB虚拟串口实验\r\n");
                    usb_printf("慧勤智远@Waiken-Smart\r\n\r\n\r\n");
                }

                if (times % 200 == 0)
                {
                    usb_printf("请输入数据,以回车键结束\r\n");
                }

                if (times % 30 == 0)
                {
                    LED0_TOGGLE();  /* LED0闪烁,提示系统正在运行 */
                }
                
                delay_ms(10);
            }
        }    
    }
}


