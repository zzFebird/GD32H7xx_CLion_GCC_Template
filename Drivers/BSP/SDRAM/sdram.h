/**
 ****************************************************************************************************
 * @file        sdram.h
 * @version     V1.0
 * @brief       SDRAM 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */	

#ifndef __SDRAM_H
#define __SDRAM_H

#include "./SYSTEM/sys/sys.h"


#define SDRAM_DEVICE0_ADDR         ((uint32_t)(0XC0000000))           /* SDRAM开始地址 */


void sdram_init(uint32_t sdram_device);                               /* SDRAM 初始化 */
void sdram_writebuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n);   /* SDRAM 写入 */
void sdram_readbuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n);    /* SDRAM 读取 */


#endif

















