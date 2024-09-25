/**
 ****************************************************************************************************
 * @file        sdram.h
 * @version     V1.0
 * @brief       SDRAM ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	

#ifndef __SDRAM_H
#define __SDRAM_H

#include "./SYSTEM/sys/sys.h"


#define SDRAM_DEVICE0_ADDR         ((uint32_t)(0XC0000000))           /* SDRAM��ʼ��ַ */


void sdram_init(uint32_t sdram_device);                               /* SDRAM ��ʼ�� */
void sdram_writebuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n);   /* SDRAM д�� */
void sdram_readbuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n);    /* SDRAM ��ȡ */


#endif

















