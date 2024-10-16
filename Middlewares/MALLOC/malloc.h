/**
 ****************************************************************************************************
 * @file        malloc.h
 * @version     V1.0
 * @brief       内存管理 代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */

#ifndef __MALLOC_H
#define __MALLOC_H

#include "./SYSTEM/sys/sys.h"

/* 定义四个内存池 */
#define SRAMIN                  0       /* AXI内存池,AXI共512+320KB */
#define SRAMEX                  1       /* 外部内存池(SDRAM),SDRAM共32MB */
#define SRAMITCM                2       /* ITCM内存池,ITCM共64 KB(出厂值),此部分内存仅CPU和MDMA(通过AHBS)可以访问!!!! */
#define SRAMDTCM                3       /* DTCM内存池,DTCM共128KB(出厂值),此部分内存仅CPU和MDMA(通过AHBS)可以访问!!!! */

#define SRAMBANK                4       /* 定义支持的SRAM块数 */


/* 定义内存管理表类型,当外扩SDRAM的时候，必须使用uint32_t类型，否则可以定义成uint16_t，以节省内存占用 */
#define MT_TYPE     uint32_t


/* 单块内存，内存管理所占用的全部空间大小计算公式如下：
 * size = MEMx_MAX_SIZE + (MEMx_MAX_SIZE / MEMx_BLOCK_SIZE) * sizeof(MT_TYPE)
 * 以SRAMEX为例，size = 26912 * 1024 + (26912 * 1024 / 64) * 4 = 28594KB

 * 已知总内存容量(size)，最大内存池的计算公式如下：
 * MEMx_MAX_SIZE = (MEMx_BLOCK_SIZE * size) / (MEMx_BLOCK_SIZE + sizeof(MT_TYPE))
 * 以SRAMITCM为例, MEM3_MAX_SIZE = (64 * 64) / (64 + 4) = 60.24KB ≈ 60KB
 */
 
/* mem1内存参数设定.mem1是H7片上的AXI SRAM内存. */
#define MEM1_BLOCK_SIZE         64                              /* 内存块大小为64字节 */
#define MEM1_MAX_SIZE           500 * 1024                      /* 最大管理内存500K,H7的AXI内存总共512+320KB */
#define MEM1_ALLOC_TABLE_SIZE   MEM1_MAX_SIZE/MEM1_BLOCK_SIZE   /* 内存表大小 */

/* mem2内存参数设定.mem2的内存池处于外部SDRAM里面 */
#define MEM2_BLOCK_SIZE         64                              /* 内存块大小为64字节 */
#define MEM2_MAX_SIZE           26912 * 1024                    /* 最大管理内存26912K */
#define MEM2_ALLOC_TABLE_SIZE   MEM2_MAX_SIZE/MEM2_BLOCK_SIZE   /* 内存表大小 */

/* mem3内存参数设定.mem3是H7内部的ITCM内存,此部分内存仅CPU和MDMA可以访问!!!!!! */
#define MEM3_BLOCK_SIZE         64                              /* 内存块大小为64字节 */
#define MEM3_MAX_SIZE           60 * 1024                       /* 最大管理内存60K,H7的ITCM共64KB(出厂值) */
#define MEM3_ALLOC_TABLE_SIZE   MEM3_MAX_SIZE/MEM3_BLOCK_SIZE   /* 内存表大小 */

/* mem4内存参数设定.mem4是H7内部的DTCM内存,此部分内存仅CPU和MDMA可以访问!!!!!! */
#define MEM4_BLOCK_SIZE         64                              /* 内存块大小为64字节 */
#define MEM4_MAX_SIZE           120 * 1024                      /* 最大管理内存120K,H7的DTCM共128KB(出厂值) */
#define MEM4_ALLOC_TABLE_SIZE   MEM4_MAX_SIZE/MEM4_BLOCK_SIZE   /* 内存表大小 */

/* 如果没有定义NULL, 定义NULL */
#ifndef NULL
#define NULL 0
#endif


/* 内存管理控制器 */
struct _m_mallco_dev
{
    void (*init)(uint8_t);              /* 初始化 */
    uint16_t (*perused)(uint8_t);       /* 内存使用率 */
    uint8_t *membase[SRAMBANK];         /* 内存池 管理SRAMBANK个区域的内存 */
    MT_TYPE *memmap[SRAMBANK];          /* 内存管理状态表 */
    uint8_t  memrdy[SRAMBANK];          /* 内存管理是否就绪 */
};

extern struct _m_mallco_dev mallco_dev; /* 在mallco.c里面定义 */


/* 函数声明 */
void my_mem_init(uint8_t memx);                             /* 内存管理初始化函数(外/内部调用) */
uint16_t my_mem_perused(uint8_t memx) ;                     /* 获得内存使用率(外/内部调用) */
void my_mem_set(void *s, uint8_t c, uint32_t count);        /* 内存设置函数 */
void my_mem_copy(void *des, void *src, uint32_t n);         /* 内存拷贝函数 */
void myfree(uint8_t memx, void *ptr);                       /* 内存释放(外部调用) */
void *mymalloc(uint8_t memx, uint32_t size);                /* 内存分配(外部调用) */
void *myrealloc(uint8_t memx, void *ptr, uint32_t size);    /* 重新分配内存(外部调用) */

#endif













