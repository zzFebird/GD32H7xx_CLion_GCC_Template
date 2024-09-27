/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       FATFS 实验
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
#include "./FATFS/exfuns/exfuns.h"


int main(void)
{
    uint32_t total, free;
    uint8_t t = 0;
    uint8_t res = 0;
  
    sys_cache_enable();                     /* 使能CPU cache */
    delay_init(600);                        /* 延时初始化 */
    usart_init(115200);                     /* 初始化串口 */  
    led_init();							    /* 初始化LED */   
    mpu_memory_protection();                /* 保护相关存储区域 */
    sdram_init(EXMC_SDRAM_DEVICE0);         /* 初始化SDRAM */
    key_init();                             /* 初始化按键 */
    norflash_init();                        /* 初始化NORFLASH */

    my_mem_init(SRAMIN);                    /* 初始化内部SRAM内存池 */
    my_mem_init(SRAMEX);                    /* 初始化外部SDRAM内存池 */

    exfuns_init();                          /* 为fatfs相关变量申请内存 */
    res = f_mount(fs[1], "1:", 1);          /* 挂载FLASH */
    if (res == 0X0D)                        /* FLASH磁盘,FAT文件系统错误,重新格式化FLASH */
    {
        res = f_mkfs("1:", 0, 0, FF_MAX_SS);                                         /* 格式化FLASH,1:,盘符;0,使用默认格式化参数 */
        if (res == 0)
        {
            f_setlabel((const TCHAR *)"1:WKS");                                      /* 设置Flash磁盘的名字为：WKS */
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
            LED0_TOGGLE();                                      /* LED0闪烁 */
        }         
    }
}


