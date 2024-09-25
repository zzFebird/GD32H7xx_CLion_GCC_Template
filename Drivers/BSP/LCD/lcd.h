/**
 ****************************************************************************************************
 * @file        lcd.h
 * @version     V1.0
 * @brief       2.8��/3.5��/4.3��/ TFTLCD(MCU��) ��������	
 *              ֧������IC�ͺŰ���:NT35310/NT35510/ST7796/ST7789
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 * V1.1
 * ��Ӷ�TLI RGBLCD�ļ���
 *
 ****************************************************************************************************
 */

#ifndef __LCD_H
#define __LCD_H

#include "stdlib.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/LCD/tli.h"


/******************************************************************************************/
/* LCD RST/BL/WR/RD/CS/RS ���� ���� 
 * LCD_D0~D15,��������̫��,�Ͳ������ﶨ����,ֱ����lcd_init�����޸�.��������ֲ��ʱ��,���˸�
 * ��6��IO��, ���ø�lcd_init�����D0~D15���ڵ�IO��.
 */

#define LCD_RST_GPIO_PORT               GPIOB
#define LCD_RST_GPIO_PIN                GPIO_PIN_12
#define LCD_RST_GPIO_CLK                RCU_GPIOB   /* GPIOBʱ��ʹ�� */

#define LCD_BL_GPIO_PORT                GPIOG
#define LCD_BL_GPIO_PIN                 GPIO_PIN_3
#define LCD_BL_GPIO_CLK                 RCU_GPIOG   /* GPIOGʱ��ʹ�� */

#define LCD_WR_GPIO_PORT                GPIOD
#define LCD_WR_GPIO_PIN                 GPIO_PIN_5
#define LCD_WR_GPIO_CLK                 RCU_GPIOD   /* GPIODʱ��ʹ�� */

#define LCD_RD_GPIO_PORT                GPIOD
#define LCD_RD_GPIO_PIN                 GPIO_PIN_4
#define LCD_RD_GPIO_CLK                 RCU_GPIOD   /* GPIODʱ��ʹ�� */

/* LCD_CS(��Ҫ����LCD_EXMC_NEX������ȷ��IO��) �� LCD_RS(��Ҫ����LCD_EXMC_AX������ȷ��IO��) ���� ���� */
#define LCD_CS_GPIO_PORT                GPIOG
#define LCD_CS_GPIO_PIN                 GPIO_PIN_9
#define LCD_CS_GPIO_CLK                 RCU_GPIOG   /* GPIOGʱ��ʹ�� */

#define LCD_RS_GPIO_PORT                GPIOG
#define LCD_RS_GPIO_PIN                 GPIO_PIN_2
#define LCD_RS_GPIO_CLK                 RCU_GPIOG   /* GPIOGʱ��ʹ�� */

/* EXMC��ز��� ���� 
 * ע��: ����Ĭ����ͨ��EXMC��0������LCD, ��0��4��Ƭѡ: EXMC_NE0~3
 *
 * �޸�LCD_EXMC_NEX, ��Ӧ��LCD_CS_GPIO�������Ҳ�ø�
 * �޸�LCD_EXMC_AX , ��Ӧ��LCD_RS_GPIO�������Ҳ�ø�
 */ 
#define LCD_EXMC_NEX             1              /* ʹ��EXMC_NE1��LCD_CS,ȡֵ��Χֻ����: 0 ~ 3 */
#define LCD_EXMC_AX              12             /* ʹ��EXMC_A12��LCD_RS,ȡֵ��Χ��: 0 ~ 25 */

#define LCD_EXMC_SNWTCFGX        EXMC_SNWTCFG(LCD_EXMC_NEX)     /* дʱ��Ĵ���(EXMC_SNWTCFGx,x=0,1,2,3),����LCD_EXMC_NEX�Զ�ȷ�� */

/******************************************************************************************/

/* LCD��Ҫ������ */
typedef struct
{
    uint16_t width;     /* LCD ��� */
    uint16_t height;    /* LCD �߶� */
    uint16_t id;        /* LCD ID */
    uint8_t dir;        /* ���������������ƣ�0��������1�������� */
    uint16_t wramcmd;   /* ��ʼдgramָ�� */
    uint16_t setxcmd;   /* ����x����ָ�� */
    uint16_t setycmd;   /* ����y����ָ�� */
} _lcd_dev;

/* LCD���� */
extern _lcd_dev lcddev; /* ����LCD��Ҫ���� */

/* LCD�Ļ�����ɫ�ͱ���ɫ */
extern uint32_t  g_point_color;     /* Ĭ�Ϻ�ɫ */
extern uint32_t  g_back_color;      /* ������ɫ.Ĭ��Ϊ��ɫ */

/* LCD������� */
#define LCD_BL(x)        do{ x ? \
                             gpio_bit_write(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, SET) : \
                             gpio_bit_write(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, RESET); \
                         }while(0)     

/* LCD��λ���� */
#define LCD_RST(x)       do{ x ? \
                             gpio_bit_write(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN, SET) : \
                             gpio_bit_write(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN, RESET); \
                         }while(0)     
/* LCD��ַ�ṹ�� */
typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
} LCD_TypeDef;


/******************************************************************************************/

/* LCD_BASE����ϸ���㷽��:
 * ����һ��ʹ��EXMC�Ŀ�0(BANK0)������TFTLCDҺ����(MCU��), ��0��ַ��Χ�ܴ�СΪ256MB,���ֳ�4��:
 * �洢��0(EXMC_NE0)��ַ��Χ: 0X6000 0000 ~ 0X63FF FFFF
 * �洢��1(EXMC_NE1)��ַ��Χ: 0X6400 0000 ~ 0X67FF FFFF
 * �洢��2(EXMC_NE2)��ַ��Χ: 0X6800 0000 ~ 0X6BFF FFFF
 * �洢��3(EXMC_NE3)��ַ��Χ: 0X6C00 0000 ~ 0X6FFF FFFF
 *
 * ������Ҫ����Ӳ�����ӷ�ʽѡ����ʵ�Ƭѡ(����LCD_CS)�͵�ַ��(����LCD_RS)
 * GD32H757ZMT6Сϵͳ��ʹ��EXMC_NE1����LCD_CS, EXMC_A12����LCD_RS ,16λ������,���㷽������:
 * ����EXMC_NE1�Ļ���ַΪ: 0X6400 0000;     NEx�Ļ�ַΪ(x=0/1/2/3): 0X6000 0000 + (0X400 0000 * x)
 * EXMC_A12��Ӧ��ֵַ: 2^12 * 2 = 0X2000;       EXMC_Ay��Ӧ�ĵ�ַΪ(y = 0 ~ 25): 2^y * 2
 *
 * LCD->LCD_REG,��ӦLCD_RS = 0(LCD�Ĵ���); LCD->LCD_RAM,��ӦLCD_RS = 1(LCD����)
 * �� LCD->LCD_RAM�ĵ�ַΪ:  0X6400 0000 + 2^12 * 2 = 0X6400 2000
 *    LCD->LCD_REG�ĵ�ַ����Ϊ LCD->LCD_RAM֮��������ַ.
 * ��������ʹ�ýṹ�����LCD_REG �� LCD_RAM(REG��ǰ,RAM�ں�,��Ϊ16λ���ݿ��)
 * ��� �ṹ��Ļ���ַ(LCD_BASE) = LCD_RAM - 2 = 0X6400 2000 -2
 *
 * ����ͨ�õļ��㹫ʽΪ((Ƭѡ��EXMC_NEx)x=0/1/2/3, (RS�ӵ�ַ��EXMC_Ay)y=0~25):
 *          LCD_BASE = (0X6000 0000 + (0X400 0000 * x)) | (2^y * 2 -2)
 *          ��Ч��(ʹ����λ����)
 *          LCD_BASE = (0X6000 0000 + (0X400 0000 * x)) | ((1 << y) * 2 -2)
 */
#define LCD_BASE        (uint32_t)((0X60000000 + (0X4000000 * LCD_EXMC_NEX)) | (((1 << LCD_EXMC_AX) * 2) -2))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

/******************************************************************************************/
/* LCDɨ�跽�����ɫ ���� */

/* ɨ�跽���� */
#define L2R_U2D         0           /* ������,���ϵ��� */
#define L2R_D2U         1           /* ������,���µ��� */
#define R2L_U2D         2           /* ���ҵ���,���ϵ��� */
#define R2L_D2U         3           /* ���ҵ���,���µ��� */

#define U2D_L2R         4           /* ���ϵ���,������ */
#define U2D_R2L         5           /* ���ϵ���,���ҵ��� */
#define D2U_L2R         6           /* ���µ���,������ */
#define D2U_R2L         7           /* ���µ���,���ҵ��� */

#define DFT_SCAN_DIR    L2R_U2D     /* Ĭ�ϵ�ɨ�跽�� */


/* MCU������RGB565��ʽ�洢��ɫ����,����ʹ��MCU��ʱ,
 * ��Ҫ���궨��TLI_PIXFORMAT��ΪTLI_PIXFORMAT_RGB565(��tli.h�ﶨ��)
 */
#if TLI_PIXFORMAT == TLI_PIXFORMAT_RGB565

/* ���û�����ɫ */
#define WHITE           0xFFFF          /* ��ɫ */
#define BLACK           0x0000          /* ��ɫ */
#define RED             0xF800          /* ��ɫ */
#define GREEN           0x07E0          /* ��ɫ */
#define BLUE            0x001F          /* ��ɫ */ 
#define MAGENTA         0XF81F          /* Ʒ��ɫ/�Ϻ�ɫ = BLUE + RED */
#define YELLOW          0XFFE0          /* ��ɫ = GREEN + RED */
#define CYAN            0X07FF          /* ��ɫ = GREEN + BLUE */  

/* �ǳ�����ɫ */
#define BROWN           0XBC40          /* ��ɫ */
#define BRRED           0XFC07          /* �غ�ɫ */
#define GRAY            0X8430          /* ��ɫ */ 
#define DARKBLUE        0X01CF          /* ����ɫ */
#define LIGHTBLUE       0X7D7C          /* ǳ��ɫ */ 
#define GRAYBLUE        0X5458          /* ����ɫ */ 
#define LIGHTGREEN      0X841F          /* ǳ��ɫ */  
#define LGRAY           0XC618          /* ǳ��ɫ(PANNEL),���屳��ɫ */ 
#define LGRAYBLUE       0XA651          /* ǳ����ɫ(�м����ɫ) */ 
#define LBBLUE          0X2B12          /* ǳ����ɫ(ѡ����Ŀ�ķ�ɫ) */ 
  
#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888

/* ���û�����ɫ */
#define WHITE           0xFFFFFF        /* ��ɫ */
#define BLACK           0x000000        /* ��ɫ */
#define RED             0xFF0000        /* ��ɫ */
#define GREEN           0x00FF00        /* ��ɫ */
#define BLUE            0x0000FF        /* ��ɫ */ 
#define MAGENTA         0XFF00FF        /* Ʒ��ɫ/�Ϻ�ɫ = BLUE + RED */
#define YELLOW          0XFFFF00        /* ��ɫ = GREEN + RED */
#define CYAN            0X00FFFF        /* ��ɫ = GREEN + BLUE */  

/* �ǳ�����ɫ */
#define BROWN           0xB88800        /* ��ɫ */
#define BRRED           0XF88038        /* �غ�ɫ */
#define GRAY            0X808480        /* ��ɫ */ 
#define DARKBLUE        0X003878        /* ����ɫ */
#define LIGHTBLUE       0X78ACE0        /* ǳ��ɫ */ 
#define GRAYBLUE        0X5088C0        /* ����ɫ */ 
#define LIGHTGREEN      0X8080F8        /* ǳ��ɫ */  
#define LGRAY           0XC0C0C0        /* ǳ��ɫ(PANNEL),���屳��ɫ */ 
#define LGRAYBLUE       0XA0C888        /* ǳ����ɫ(�м����ɫ) */ 
#define LBBLUE          0x286090        /* ǳ����ɫ(ѡ����Ŀ�ķ�ɫ) */

#else

#define WHITE           0xFFFFFFFF      /* ��ɫ */
#define BLACK           0xFF000000      /* ��ɫ */
#define RED             0xFFFF0000      /* ��ɫ */
#define GREEN           0xFF00FF00      /* ��ɫ */
#define BLUE            0xFF0000FF      /* ��ɫ */ 
#define MAGENTA         0XFFFF00FF      /* Ʒ��ɫ/�Ϻ�ɫ = BLUE + RED */
#define YELLOW          0XFFFFFF00      /* ��ɫ = GREEN + RED */
#define CYAN            0XFF00FFFF      /* ��ɫ = GREEN + BLUE */  

#define BROWN           0xFFB88800      /* ��ɫ */
#define BRRED           0XFFF88038      /* �غ�ɫ */
#define GRAY            0XFF808480      /* ��ɫ */ 
#define DARKBLUE        0XFF003878      /* ����ɫ */
#define LIGHTBLUE       0XFF78ACE0      /* ǳ��ɫ */ 
#define GRAYBLUE        0XFF5088C0      /* ����ɫ */ 
#define LIGHTGREEN      0XFF8080F8      /* ǳ��ɫ */  
#define LGRAY           0XFFC0C0C0      /* ǳ��ɫ(PANNEL),���屳��ɫ */ 
#define LGRAYBLUE       0XFFA0C888      /* ǳ����ɫ(�м����ɫ) */ 
#define LBBLUE          0xFF286090      /* ǳ����ɫ(ѡ����Ŀ�ķ�ɫ) */

#endif

/******************************************************************************************/
/* �������� */
  
void lcd_wr_data(volatile uint16_t data);            /* LCDд���� */
void lcd_wr_regno(volatile uint16_t regno);          /* LCDд�Ĵ������/��ַ */
void lcd_write_reg(uint16_t regno, uint16_t data);   /* LCDд�Ĵ�����ֵ */

void lcd_init(void);                                 /* ��ʼ��LCD */ 
void exmc_sram_init(void);                           /* ��ʼ��EXMC */  
void lcd_display_on(void);                           /* ����ʾ */ 
void lcd_display_off(void);                          /* ����ʾ */
void lcd_scan_dir(uint8_t dir);                      /* ������Ļɨ�跽�� */ 
void lcd_display_dir(uint8_t dir);                   /* ������Ļ��ʾ���� */ 

void lcd_write_ram_prepare(void);                               /* ׼��дGRAM */ 
void lcd_set_cursor(uint16_t x, uint16_t y);                    /* ���ù�� */ 
uint32_t lcd_rgb565torgb888(uint16_t rgb565);                   /* ��RGB565ת��ΪRGB888 */
uint32_t lcd_read_point(uint16_t x, uint16_t y);                /* ����(32λ��ɫ,����TLI) */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color);    /* ����(32λ��ɫ,����TLI) */

void lcd_clear(uint32_t color);                                                                 /* LCD����(32λ��ɫ,����TLI) */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint32_t color);                       /* ���ʵ��Բ */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint32_t color);                      /* ��Բ */
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint32_t color);                      /* ��ˮƽ�� */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);                 /* ���ô��� */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);              /* ��ɫ������(32λ��ɫ,����TLI) */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);       /* ��ɫ������ */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);         /* ��ֱ�� */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);    /* ������ */

void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint32_t color);                       /* ��ʾһ���ַ� */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint32_t color);                     /* ��ʾ���� */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint32_t color);      /* ��չ��ʾ���� */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint32_t color);   /* ��ʾ�ַ��� */


#endif

















