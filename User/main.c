/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       FATFS ʵ��
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
#include "./FATFS/exfuns/exfuns.h"


int main(void)
{
    uint32_t total, free;
    uint8_t t = 0;
    uint8_t res = 0;
  
    sys_cache_enable();                     /* ʹ��CPU cache */
    delay_init(600);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ��ʼ������ */  
    led_init();							    /* ��ʼ��LED */   
    mpu_memory_protection();                /* ������ش洢���� */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* ��ʼ��SDRAM */
    key_init();                             /* ��ʼ������ */
    norflash_init();                        /* ��ʼ��NORFLASH */

    my_mem_init(SRAMIN);                    /* ��ʼ���ڲ�SRAM�ڴ�� */
    my_mem_init(SRAMEX);                    /* ��ʼ���ⲿSDRAM�ڴ�� */

    exfuns_init();                          /* Ϊfatfs��ر��������ڴ� */
    res = f_mount(fs[1], "1:", 1);          /* ����FLASH */
    if (res == 0X0D)                        /* FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH */
    {
        res = f_mkfs("1:", 0, 0, FF_MAX_SS);                                         /* ��ʽ��FLASH,1:,�̷�;0,ʹ��Ĭ�ϸ�ʽ������ */
        if (res == 0)
        {
            f_setlabel((const TCHAR *)"1:WKS");                                      /* ����Flash���̵�����Ϊ��WKS */
        }
        else 
        {
        }

        delay_ms(1000);
    }

    FIL fp;
    char *str = "Hello, world!\n";
    int cnt;
    res = f_open(&fp, "1:/test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK) {
        res = f_write(&fp, str, 5, &cnt);
        if (res == FR_OK) {
            printf("%d bytes written\n", cnt);
        }
        f_close(&fp);
    } else {
        printf("file open failed %d\n", res);
    }

    while (1)
    {        
        t++;
        delay_ms(10);
        
        if (t == 20)
        {
            t = 0;            
            LED0_TOGGLE();                                      /* LED0��˸ */
        }         
    }
}


