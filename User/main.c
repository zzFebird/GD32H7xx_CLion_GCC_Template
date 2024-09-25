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
//#include "./BSP/LCD/lcd.h"
#include "./BSP/SDRAM/sdram.h"
//#include "./USMART/usmart.h"
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
    //usmart_init(300);	                    /* ��ʼ��USMART */
    led_init();							    /* ��ʼ��LED */   
    mpu_memory_protection();                /* ������ش洢���� */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* ��ʼ��SDRAM */
//    lcd_init();                             /* ��ʼ��LCD */
    key_init();                             /* ��ʼ������ */
    norflash_init();                        /* ��ʼ��NORFLASH */

    my_mem_init(SRAMIN);                    /* ��ʼ���ڲ�SRAM�ڴ�� */
    my_mem_init(SRAMEX);                    /* ��ʼ���ⲿSDRAM�ڴ�� */

    //lcd_show_string(30, 50, 200, 16, 16, "GD32H757", RED);
    //lcd_show_string(30, 70, 200, 16, 16, "FATFS TEST", RED);
    //lcd_show_string(30, 90, 200, 16, 16, "WKS SMART", RED);
    //lcd_show_string(30, 110, 200, 16, 16, "Use USMART for test", RED);

    //while (SD_OK != sdio_sd_init())         /* ��ⲻ��SD�� */
    //{
    //    //lcd_show_string(30, 130, 200, 16, 16, "SD Card Error!", RED);
    //    delay_ms(500);
    //    //lcd_show_string(30, 130, 200, 16, 16, "Please Check! ", RED);
    //    delay_ms(500);
    //    LED0_TOGGLE();                      /* LED0��˸ */
    //}
        
    exfuns_init();                          /* Ϊfatfs��ر��������ڴ� */
    //f_mount(fs[0], "0:", 1);                /* ����SD�� */
    res = f_mount(fs[0], "0:", 1);          /* ����FLASH */
    
    if (res == 0X0D)                        /* FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH */
    {
        //lcd_show_string(30, 150, 200, 16, 16, "Flash Disk Formatting...", RED);	     /* ��ʽ��FLASH */
        res = f_mkfs("0:", 0, 0, FF_MAX_SS);                                         /* ��ʽ��FLASH,1:,�̷�;0,ʹ��Ĭ�ϸ�ʽ������ */

        if (res == 0)
        {
            f_setlabel((const TCHAR *)"0:WKS");                                      /* ����Flash���̵�����Ϊ��WKS */
            //lcd_show_string(30, 150, 200, 16, 16, "Flash Disk Format Finish", RED);  /* ��ʽ����� */
        }
        else 
        {
            //lcd_show_string(30, 150, 200, 16, 16, "Flash Disk Format Error ", RED);  /* ��ʽ��ʧ�� */
        }

        delay_ms(1000);
    }

    //lcd_fill(30, 130, 240, 150 + 16, WHITE);                                         /* �����ʾ */

    //while (exfuns_get_free((uint8_t*)"0", &total, &free))                            /* �õ�SD������������ʣ������ */
    //{
    //    //lcd_show_string(30, 150, 200, 16, 16, "SD Card Fatfs Error!", RED);
    //    delay_ms(200);
    //    //lcd_fill(30, 150, 240, 150 + 16, WHITE);                                     /* �����ʾ */
    //    delay_ms(200);          
    //    LED0_TOGGLE();                                                               /* LED0��˸ */
    //} 

    //lcd_show_string(30, 150, 200, 16, 16, "FATFS OK!", BLUE);
    //lcd_show_string(30, 170, 200, 16, 16, "SD Total Size:     MB", BLUE);
    //lcd_show_string(30, 190, 200, 16, 16, "SD  Free Size:     MB", BLUE);
    //lcd_show_num(30 + 8 * 14, 170, total >> 10, 5, 16, BLUE);   /* ��ʾSD�������� MB */
    //lcd_show_num(30 + 8 * 14, 190, free >> 10, 5, 16, BLUE);    /* ��ʾSD��ʣ������ MB */
    
    FIL fp;
    char *str = "hello";
    int cnt;
    res = f_open(&fp, "0:\\test.txt", FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK) {
        res = f_write(&fp, str, 5, &cnt);
        f_close(&fp);
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


