/**
 ****************************************************************************************************
 * @file        malloc.h
 * @version     V1.0
 * @brief       �ڴ���� ����
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */

#ifndef __MALLOC_H
#define __MALLOC_H

#include "./SYSTEM/sys/sys.h"

/* �����ĸ��ڴ�� */
#define SRAMIN                  0       /* AXI�ڴ��,AXI��512+320KB */
#define SRAMEX                  1       /* �ⲿ�ڴ��(SDRAM),SDRAM��32MB */
#define SRAMITCM                2       /* ITCM�ڴ��,ITCM��64 KB(����ֵ),�˲����ڴ��CPU��MDMA(ͨ��AHBS)���Է���!!!! */
#define SRAMDTCM                3       /* DTCM�ڴ��,DTCM��128KB(����ֵ),�˲����ڴ��CPU��MDMA(ͨ��AHBS)���Է���!!!! */

#define SRAMBANK                4       /* ����֧�ֵ�SRAM���� */


/* �����ڴ���������,������SDRAM��ʱ�򣬱���ʹ��uint32_t���ͣ�������Զ����uint16_t���Խ�ʡ�ڴ�ռ�� */
#define MT_TYPE     uint32_t


/* �����ڴ棬�ڴ������ռ�õ�ȫ���ռ��С���㹫ʽ���£�
 * size = MEMx_MAX_SIZE + (MEMx_MAX_SIZE / MEMx_BLOCK_SIZE) * sizeof(MT_TYPE)
 * ��SRAMEXΪ����size = 26912 * 1024 + (26912 * 1024 / 64) * 4 = 28594KB

 * ��֪���ڴ�����(size)������ڴ�صļ��㹫ʽ���£�
 * MEMx_MAX_SIZE = (MEMx_BLOCK_SIZE * size) / (MEMx_BLOCK_SIZE + sizeof(MT_TYPE))
 * ��SRAMITCMΪ��, MEM3_MAX_SIZE = (64 * 64) / (64 + 4) = 60.24KB �� 60KB
 */
 
/* mem1�ڴ�����趨.mem1��H7Ƭ�ϵ�AXI SRAM�ڴ�. */
#define MEM1_BLOCK_SIZE         64                              /* �ڴ���СΪ64�ֽ� */
#define MEM1_MAX_SIZE           500 * 1024                      /* �������ڴ�500K,H7��AXI�ڴ��ܹ�512+320KB */
#define MEM1_ALLOC_TABLE_SIZE   MEM1_MAX_SIZE/MEM1_BLOCK_SIZE   /* �ڴ���С */

/* mem2�ڴ�����趨.mem2���ڴ�ش����ⲿSDRAM���� */
#define MEM2_BLOCK_SIZE         64                              /* �ڴ���СΪ64�ֽ� */
#define MEM2_MAX_SIZE           26912 * 1024                    /* �������ڴ�26912K */
#define MEM2_ALLOC_TABLE_SIZE   MEM2_MAX_SIZE/MEM2_BLOCK_SIZE   /* �ڴ���С */

/* mem3�ڴ�����趨.mem3��H7�ڲ���ITCM�ڴ�,�˲����ڴ��CPU��MDMA���Է���!!!!!! */
#define MEM3_BLOCK_SIZE         64                              /* �ڴ���СΪ64�ֽ� */
#define MEM3_MAX_SIZE           60 * 1024                       /* �������ڴ�60K,H7��ITCM��64KB(����ֵ) */
#define MEM3_ALLOC_TABLE_SIZE   MEM3_MAX_SIZE/MEM3_BLOCK_SIZE   /* �ڴ���С */

/* mem4�ڴ�����趨.mem4��H7�ڲ���DTCM�ڴ�,�˲����ڴ��CPU��MDMA���Է���!!!!!! */
#define MEM4_BLOCK_SIZE         64                              /* �ڴ���СΪ64�ֽ� */
#define MEM4_MAX_SIZE           120 * 1024                      /* �������ڴ�120K,H7��DTCM��128KB(����ֵ) */
#define MEM4_ALLOC_TABLE_SIZE   MEM4_MAX_SIZE/MEM4_BLOCK_SIZE   /* �ڴ���С */

/* ���û�ж���NULL, ����NULL */
#ifndef NULL
#define NULL 0
#endif


/* �ڴ��������� */
struct _m_mallco_dev
{
    void (*init)(uint8_t);              /* ��ʼ�� */
    uint16_t (*perused)(uint8_t);       /* �ڴ�ʹ���� */
    uint8_t *membase[SRAMBANK];         /* �ڴ�� ����SRAMBANK��������ڴ� */
    MT_TYPE *memmap[SRAMBANK];          /* �ڴ����״̬�� */
    uint8_t  memrdy[SRAMBANK];          /* �ڴ�����Ƿ���� */
};

extern struct _m_mallco_dev mallco_dev; /* ��mallco.c���涨�� */


/* �������� */
void my_mem_init(uint8_t memx);                             /* �ڴ�����ʼ������(��/�ڲ�����) */
uint16_t my_mem_perused(uint8_t memx) ;                     /* ����ڴ�ʹ����(��/�ڲ�����) */
void my_mem_set(void *s, uint8_t c, uint32_t count);        /* �ڴ����ú��� */
void my_mem_copy(void *des, void *src, uint32_t n);         /* �ڴ濽������ */
void myfree(uint8_t memx, void *ptr);                       /* �ڴ��ͷ�(�ⲿ����) */
void *mymalloc(uint8_t memx, uint32_t size);                /* �ڴ����(�ⲿ����) */
void *myrealloc(uint8_t memx, void *ptr, uint32_t size);    /* ���·����ڴ�(�ⲿ����) */

#endif













