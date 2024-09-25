/**
 ****************************************************************************************************
 * @file        tli.h
 * @version     V1.0
 * @brief       TLI 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */	

#ifndef __TLI_H
#define __TLI_H

#include "./SYSTEM/sys/sys.h"


/* 选择是否使用8寸800X1280RGB屏
 * 0: 没有使用8寸RGB屏
 * 1: 使用8寸RGB屏
 */
#define RGB_80_8001280       0     /* 默认不使用8寸800X1280RGB屏 */


/* LCD TLI重要参数集 */
typedef struct
{
    uint32_t pwidth;        /* LCD面板的宽度,固定参数,不随显示方向改变,如果为0,说明没有任何RGB屏接入 */
    uint32_t pheight;       /* LCD面板的高度,固定参数,不随显示方向改变 */
    uint16_t hsw;           /* 水平同步宽度 */
    uint16_t vsw;           /* 垂直同步宽度 */
    uint16_t hbp;           /* 水平后廊 */
    uint16_t vbp;           /* 垂直后廊 */
    uint16_t hfp;           /* 水平前廊 */
    uint16_t vfp;           /* 垂直前廊 */
    uint8_t activelayer;    /* 当前层编号:0/1 */
    uint8_t dir;            /* 0,竖屏;1,横屏; */
    uint16_t width;         /* LCD宽度 */
    uint16_t height;        /* LCD高度 */
    uint32_t pixsize;       /* 每个像素所占字节数 */
    uint8_t pixformat;      /* 颜色像素格式 */
} _tli_dev;

extern _tli_dev lcdtli;     /* 管理LCD TLI参数 */



#define TLI_PIXFORMAT_ARGB8888      0X00    /* ARGB8888格式 */
#define TLI_PIXFORMAT_RGB888        0X01    /* RGB888格式 */
#define TLI_PIXFORMAT_RGB565        0X02    /* RGB565格式 */
#define TLI_PIXFORMAT_ARGB1555      0X03    /* ARGB1555格式 */
#define TLI_PIXFORMAT_ARGB4444      0X04    /* ARGB4444格式 */
#define TLI_PIXFORMAT_L8            0X05    /* L8格式 */
#define TLI_PIXFORMAT_AL44          0X06    /* AL44格式 */
#define TLI_PIXFORMAT_AL88          0X07    /* AL88格式 */

/******************************************************************************************/
/* TLI_DE/VSYNC/HSYNC/PCLK/BL/RST 引脚 定义 
 * TLI_R0~R7, G0~G7, B0~B7,由于引脚太多,就不在这里定义了,直接在tli_gpio_config里面修改.所以在移植的时候,
 * 除了改这6个IO口, 还得改tli_gpio_config里面的R0~R7, G0~G7, B0~B7所在的IO口.
 */

#define TLI_DE_GPIO_PORT                GPIOF
#define TLI_DE_GPIO_PIN                 GPIO_PIN_10
#define TLI_DE_GPIO_AF                  GPIO_AF_14
#define TLI_DE_GPIO_CLK                 RCU_GPIOF   /* GPIOF时钟使能 */

#define TLI_VSYNC_GPIO_PORT             GPIOA
#define TLI_VSYNC_GPIO_PIN              GPIO_PIN_4
#define TLI_VSYNC_GPIO_AF               GPIO_AF_14
#define TLI_VSYNC_GPIO_CLK              RCU_GPIOA   /* GPIOA时钟使能 */

#define TLI_HSYNC_GPIO_PORT             GPIOC
#define TLI_HSYNC_GPIO_PIN              GPIO_PIN_6
#define TLI_HSYNC_GPIO_AF               GPIO_AF_14
#define TLI_HSYNC_GPIO_CLK              RCU_GPIOC   /* GPIOC时钟使能 */

#define TLI_PCLK_GPIO_PORT              GPIOG
#define TLI_PCLK_GPIO_PIN               GPIO_PIN_7
#define TLI_PCLK_GPIO_AF                GPIO_AF_14
#define TLI_PCLK_GPIO_CLK               RCU_GPIOG   /* GPIOG时钟使能 */

/* TLI_BL和MCU屏背光引脚共用 */
#define TLI_BL_GPIO_PORT                GPIOG
#define TLI_BL_GPIO_PIN                 GPIO_PIN_3
#define TLI_BL_GPIO_CLK                 RCU_GPIOG   /* GPIOG时钟使能 */

/* TLI_RST和MCU屏复位引脚共用 */
#define TLI_RST_GPIO_PORT               GPIOB
#define TLI_RST_GPIO_PIN                GPIO_PIN_12
#define TLI_RST_GPIO_CLK                RCU_GPIOB   /* GPIOB时钟使能 */


/* 定义颜色像素格式,一般用RGB565 */
#define TLI_PIXFORMAT              TLI_PIXFORMAT_RGB565

/* 定义默认背景层颜色 */
#define TLI_BACKLAYERCOLOR         0X00000000

/* LCD帧缓冲区首地址,这里定义在SDRAM里面 */
#define TLI_FRAME_BUF_ADDR         0XC0000000

/* TLI背光控制 */
#define TLI_BL(x)    do{ x ? \
                         gpio_bit_write(TLI_BL_GPIO_PORT, TLI_BL_GPIO_PIN, SET) : \
                         gpio_bit_write(TLI_BL_GPIO_PORT, TLI_BL_GPIO_PIN, RESET); \
                     }while(0)      

/* TLI复位引脚 */
#define TLI_RST(x)   do{ x ? \
                         gpio_bit_write(TLI_RST_GPIO_PORT, TLI_RST_GPIO_PIN, SET) : \
                         gpio_bit_write(TLI_RST_GPIO_PORT, TLI_RST_GPIO_PIN, RESET); \
                     }while(0)      

/******************************************************************************************/


void tli_select_layer(uint8_t layerx);                                                          /* 选择TLI层 */
void tli_display_dir(uint8_t dir);                                                              /* TLI显示方向设置 */
void tli_draw_point(uint16_t x, uint16_t y, uint32_t color);                                    /* TLI画点函数 */
uint32_t tli_read_point(uint16_t x, uint16_t y);                                                /* TLI读点函数 */
void tli_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);              /* 纯色填充矩形, 使用IPA填充 */
void tli_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);       /* 彩色填充矩形, 使用IPA填充 */
void tli_clear(uint32_t color);                                                                 /* TLI清屏 */
uint8_t tli_clk_set(uint32_t pll2_psc, uint32_t pll2_n, uint32_t pll2_r, uint32_t pll2_r_div);  /* 设置TLI时钟 */
uint16_t tli_panelid_read(void);                                                                /* 读取面板ID */
void tli_config(void);                                                                          /* 初始化TLI */
void tli_gpio_config(void);                                                                     /* 配置TLI相关IO口 */
                     
#endif







