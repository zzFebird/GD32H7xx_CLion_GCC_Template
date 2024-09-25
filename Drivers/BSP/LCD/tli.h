/**
 ****************************************************************************************************
 * @file        tli.h
 * @version     V1.0
 * @brief       TLI ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	

#ifndef __TLI_H
#define __TLI_H

#include "./SYSTEM/sys/sys.h"


/* ѡ���Ƿ�ʹ��8��800X1280RGB��
 * 0: û��ʹ��8��RGB��
 * 1: ʹ��8��RGB��
 */
#define RGB_80_8001280       0     /* Ĭ�ϲ�ʹ��8��800X1280RGB�� */


/* LCD TLI��Ҫ������ */
typedef struct
{
    uint32_t pwidth;        /* LCD���Ŀ��,�̶�����,������ʾ����ı�,���Ϊ0,˵��û���κ�RGB������ */
    uint32_t pheight;       /* LCD���ĸ߶�,�̶�����,������ʾ����ı� */
    uint16_t hsw;           /* ˮƽͬ����� */
    uint16_t vsw;           /* ��ֱͬ����� */
    uint16_t hbp;           /* ˮƽ���� */
    uint16_t vbp;           /* ��ֱ���� */
    uint16_t hfp;           /* ˮƽǰ�� */
    uint16_t vfp;           /* ��ֱǰ�� */
    uint8_t activelayer;    /* ��ǰ����:0/1 */
    uint8_t dir;            /* 0,����;1,����; */
    uint16_t width;         /* LCD��� */
    uint16_t height;        /* LCD�߶� */
    uint32_t pixsize;       /* ÿ��������ռ�ֽ��� */
    uint8_t pixformat;      /* ��ɫ���ظ�ʽ */
} _tli_dev;

extern _tli_dev lcdtli;     /* ����LCD TLI���� */



#define TLI_PIXFORMAT_ARGB8888      0X00    /* ARGB8888��ʽ */
#define TLI_PIXFORMAT_RGB888        0X01    /* RGB888��ʽ */
#define TLI_PIXFORMAT_RGB565        0X02    /* RGB565��ʽ */
#define TLI_PIXFORMAT_ARGB1555      0X03    /* ARGB1555��ʽ */
#define TLI_PIXFORMAT_ARGB4444      0X04    /* ARGB4444��ʽ */
#define TLI_PIXFORMAT_L8            0X05    /* L8��ʽ */
#define TLI_PIXFORMAT_AL44          0X06    /* AL44��ʽ */
#define TLI_PIXFORMAT_AL88          0X07    /* AL88��ʽ */

/******************************************************************************************/
/* TLI_DE/VSYNC/HSYNC/PCLK/BL/RST ���� ���� 
 * TLI_R0~R7, G0~G7, B0~B7,��������̫��,�Ͳ������ﶨ����,ֱ����tli_gpio_config�����޸�.��������ֲ��ʱ��,
 * ���˸���6��IO��, ���ø�tli_gpio_config�����R0~R7, G0~G7, B0~B7���ڵ�IO��.
 */

#define TLI_DE_GPIO_PORT                GPIOF
#define TLI_DE_GPIO_PIN                 GPIO_PIN_10
#define TLI_DE_GPIO_AF                  GPIO_AF_14
#define TLI_DE_GPIO_CLK                 RCU_GPIOF   /* GPIOFʱ��ʹ�� */

#define TLI_VSYNC_GPIO_PORT             GPIOA
#define TLI_VSYNC_GPIO_PIN              GPIO_PIN_4
#define TLI_VSYNC_GPIO_AF               GPIO_AF_14
#define TLI_VSYNC_GPIO_CLK              RCU_GPIOA   /* GPIOAʱ��ʹ�� */

#define TLI_HSYNC_GPIO_PORT             GPIOC
#define TLI_HSYNC_GPIO_PIN              GPIO_PIN_6
#define TLI_HSYNC_GPIO_AF               GPIO_AF_14
#define TLI_HSYNC_GPIO_CLK              RCU_GPIOC   /* GPIOCʱ��ʹ�� */

#define TLI_PCLK_GPIO_PORT              GPIOG
#define TLI_PCLK_GPIO_PIN               GPIO_PIN_7
#define TLI_PCLK_GPIO_AF                GPIO_AF_14
#define TLI_PCLK_GPIO_CLK               RCU_GPIOG   /* GPIOGʱ��ʹ�� */

/* TLI_BL��MCU���������Ź��� */
#define TLI_BL_GPIO_PORT                GPIOG
#define TLI_BL_GPIO_PIN                 GPIO_PIN_3
#define TLI_BL_GPIO_CLK                 RCU_GPIOG   /* GPIOGʱ��ʹ�� */

/* TLI_RST��MCU����λ���Ź��� */
#define TLI_RST_GPIO_PORT               GPIOB
#define TLI_RST_GPIO_PIN                GPIO_PIN_12
#define TLI_RST_GPIO_CLK                RCU_GPIOB   /* GPIOBʱ��ʹ�� */


/* ������ɫ���ظ�ʽ,һ����RGB565 */
#define TLI_PIXFORMAT              TLI_PIXFORMAT_RGB565

/* ����Ĭ�ϱ�������ɫ */
#define TLI_BACKLAYERCOLOR         0X00000000

/* LCD֡�������׵�ַ,���ﶨ����SDRAM���� */
#define TLI_FRAME_BUF_ADDR         0XC0000000

/* TLI������� */
#define TLI_BL(x)    do{ x ? \
                         gpio_bit_write(TLI_BL_GPIO_PORT, TLI_BL_GPIO_PIN, SET) : \
                         gpio_bit_write(TLI_BL_GPIO_PORT, TLI_BL_GPIO_PIN, RESET); \
                     }while(0)      

/* TLI��λ���� */
#define TLI_RST(x)   do{ x ? \
                         gpio_bit_write(TLI_RST_GPIO_PORT, TLI_RST_GPIO_PIN, SET) : \
                         gpio_bit_write(TLI_RST_GPIO_PORT, TLI_RST_GPIO_PIN, RESET); \
                     }while(0)      

/******************************************************************************************/


void tli_select_layer(uint8_t layerx);                                                          /* ѡ��TLI�� */
void tli_display_dir(uint8_t dir);                                                              /* TLI��ʾ�������� */
void tli_draw_point(uint16_t x, uint16_t y, uint32_t color);                                    /* TLI���㺯�� */
uint32_t tli_read_point(uint16_t x, uint16_t y);                                                /* TLI���㺯�� */
void tli_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);              /* ��ɫ������, ʹ��IPA��� */
void tli_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);       /* ��ɫ������, ʹ��IPA��� */
void tli_clear(uint32_t color);                                                                 /* TLI���� */
uint8_t tli_clk_set(uint32_t pll2_psc, uint32_t pll2_n, uint32_t pll2_r, uint32_t pll2_r_div);  /* ����TLIʱ�� */
uint16_t tli_panelid_read(void);                                                                /* ��ȡ���ID */
void tli_config(void);                                                                          /* ��ʼ��TLI */
void tli_gpio_config(void);                                                                     /* ����TLI���IO�� */
                     
#endif







