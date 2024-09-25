/**
 ****************************************************************************************************
 * @file        lcd.c
 * @version     V1.1
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
 
#include "stdlib.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/lcdfont.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LCD/tli.h"


/* lcd_ex.c��Ÿ���LCD����IC�ļĴ�����ʼ�����ִ���,�Լ�lcd.c,��.c�ļ�
 * ��ֱ�Ӽ��뵽��������,ֻ��lcd.c���õ�,����ͨ��include����ʽ���.(��Ҫ��
 * �����ļ��ٰ�����.c�ļ�!!����ᱨ��!)
 */
#include "./BSP/LCD/lcd_ex.c"


/* LCD�Ļ�����ɫ�ͱ���ɫ */
uint32_t g_point_color = RED;           /* ������ɫ */
uint32_t g_back_color  = 0XFFFFFFFF;    /* ����ɫ */
         
/* ����LCD��Ҫ���� */
_lcd_dev lcddev;


/**
 * @brief       LCDд����
 * @param       data: Ҫд�������
 * @retval      ��
 */
void lcd_wr_data(volatile uint16_t data)
{
    data = data;            /* ʹ��-O2�Ż���ʱ��,����������ʱ */
    LCD->LCD_RAM = data;
}

/**
 * @brief       LCDд�Ĵ������/��ַ����
 * @param       regno: �Ĵ������/��ַ
 * @retval      ��
 */
void lcd_wr_regno(volatile uint16_t regno)
{
    regno = regno;          /* ʹ��-O2�Ż���ʱ��,����������ʱ */
    LCD->LCD_REG = regno;   /* д��Ҫд�ļĴ������ */
}

/**
 * @brief       LCDд�Ĵ���
 * @param       regno:�Ĵ������/��ַ
 * @param       data:Ҫд�������
 * @retval      ��
 */
void lcd_write_reg(uint16_t regno, uint16_t data)
{
    LCD->LCD_REG = regno;   /* д��Ҫд�ļĴ������ */
    LCD->LCD_RAM = data;    /* д������ */
}

/**
 * @brief       LCD��ʱ����,�����ڲ�����mdk -O1ʱ���Ż�ʱ��Ҫ���õĵط�
 * @param       t:��ʱ����ֵ
 * @retval      ��
 */
static void lcd_opt_delay(uint32_t i)
{
    while (i--);
}

/**
 * @brief       LCD������
 * @param       ��
 * @retval      ��ȡ��������
 */
static uint16_t lcd_rd_data(void)
{
    volatile uint16_t ram;  /* ��ֹ���Ż� */
    lcd_opt_delay(2);
    ram = LCD->LCD_RAM;
    return ram;
}

/**
 * @brief       ׼��дGRAM
 * @param       ��
 * @retval      ��
 */
void lcd_write_ram_prepare(void)
{
    LCD->LCD_REG = lcddev.wramcmd;
}

/**
 * @brief       ��ɫת��
 * @note        ��RGB565��ɫת����RGB888��ɫ
 * @param       rgb565 : RGB565��ɫ
 * @retval      RGB888��ɫֵ
 */
uint32_t lcd_rgb565torgb888(uint16_t rgb565)
{
    uint16_t r, g, b;
    uint32_t rgb888;
  
    r = (rgb565 & 0XF800) >> 8;
    g = (rgb565 & 0X07E0) >> 3;
    b = (rgb565 & 0X001F) << 3;
  
    rgb888 = (r << 16) | (g << 8) | b;
  
    return rgb888;
}

/**
 * @brief       ��ȡĳ�������ɫֵ
 * @param       x,y:����
 * @retval      �˵����ɫ(32λ��ɫ,�������TLI)
 */
uint32_t lcd_read_point(uint16_t x, uint16_t y)
{
    uint16_t r = 0, g = 0, b = 0;

    if (x >= lcddev.width || y >= lcddev.height) return 0;  /* �����˷�Χ,ֱ�ӷ��� */

    if (lcdtli.pwidth != 0)     /* �����RGB�� */
    {
        return tli_read_point(x, y);
    }
    
    lcd_set_cursor(x, y);       /* �������� */

    if (lcddev.id == 0X5510)
    {
        lcd_wr_regno(0X2E00);   /* 5510 ���Ͷ�GRAMָ�� */
    }
    else
    {
        lcd_wr_regno(0X2E);     /* ����IC(7796/5310/7789)���Ͷ�GRAMָ�� */
    }

    r = lcd_rd_data();          /* �ٶ�(dummy read) */

    r = lcd_rd_data();          /* ʵ��������ɫ */
    
    if (lcddev.id == 0X7796) return r;  /* 7796 һ�ζ�ȡһ������ֵ */

    /* 5310/5510/7789 Ҫ��2�ζ��� */
    b = lcd_rd_data();
    g = r & 0XFF;               /* ���� 5310/5510/7789, ��һ�ζ�ȡ����RG��ֵ,R��ǰ,G�ں�,��ռ8λ */
    g <<= 8;
    
    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));  /* 5310/5510/7789 ��Ҫ��ʽת��һ�� */
}

/**
 * @brief       LCD������ʾ
 * @param       ��
 * @retval      ��
 */
void lcd_display_on(void)
{
    if (lcdtli.pwidth != 0)          /* �����RGB�� */
    {
        tli_enable();                /* ʹ��TLI */
    }
    else if (lcddev.id == 0X5510)    /* 5510������ʾָ�� */
    {
        lcd_wr_regno(0X2900);        /* ������ʾ */
    }
    else                             /* 5310/7789/7796 �ȷ��Ϳ�����ʾָ�� */
    {
        lcd_wr_regno(0X29);          /* ������ʾ */
    }
}

/**
 * @brief       LCD�ر���ʾ
 * @param       ��
 * @retval      ��
 */
void lcd_display_off(void)
{
    if (lcdtli.pwidth != 0)          /* �����RGB�� */
    {
        tli_disable();               /* ����TLI */
    }
    else if (lcddev.id == 0X5510)    /* 5510�ر���ʾָ�� */
    {
        lcd_wr_regno(0X2800);        /* �ر���ʾ */
    }
    else                             /* 5310/7789/7796 �ȷ��Ϳ�����ʾָ�� */
    {
        lcd_wr_regno(0X28);          /* �ر���ʾ */
    }
}

/**
 * @brief       ���ù��λ��(��RGB����Ч)
 * @param       x,y: ����
 * @retval      ��
 */
void lcd_set_cursor(uint16_t x, uint16_t y)
{
    if (lcddev.id == 0X5510)  /* 5510�������� */
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(x >> 8);
        lcd_wr_regno(lcddev.setxcmd + 1);
        lcd_wr_data(x & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(y >> 8);
        lcd_wr_regno(lcddev.setycmd + 1);
        lcd_wr_data(y & 0XFF);
    }
    else                      /* 5310/7789/7796�������� */
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(x >> 8);
        lcd_wr_data(x & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(y >> 8);
        lcd_wr_data(y & 0XFF);
    }
}

/**
 * @brief       ����LCD���Զ�ɨ�跽��(��RGB����Ч)
 * @note
 *              ע��:�����������ܻ��ܵ��˺������õ�Ӱ��,
 *              ����,һ������ΪL2R_U2D����,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.
 *
 * @param       dir:0~7,����8������(���嶨���lcd.h)
 * @retval      ��
 */
void lcd_scan_dir(uint8_t dir)
{
    uint16_t regval = 0;
    uint16_t dirreg = 0;
    uint16_t temp;

    /* ����ʱ��IC�ı�ɨ�跽������ʱ, IC���ı�ɨ�跽�� */
    if (lcddev.dir == 1)
    {
        switch (dir)   /* ����ת�� */
        {
            case 0:
                dir = 6;
                break;

            case 1:
                dir = 7;
                break;

            case 2:
                dir = 4;
                break;

            case 3:
                dir = 5;
                break;

            case 4:
                dir = 1;
                break;

            case 5:
                dir = 0;
                break;

            case 6:
                dir = 3;
                break;

            case 7:
                dir = 2;
                break;
        }
    }
 
    /* ����ɨ�跽ʽ ���� 0X36/0X3600 �Ĵ��� bit 5,6,7 λ��ֵ */
    switch (dir)
    {
        case L2R_U2D:   /* ������,���ϵ��� */
            regval |= (0 << 7) | (0 << 6) | (0 << 5);
            break;

        case L2R_D2U:   /* ������,���µ��� */
            regval |= (1 << 7) | (0 << 6) | (0 << 5);
            break;

        case R2L_U2D:   /* ���ҵ���,���ϵ��� */
            regval |= (0 << 7) | (1 << 6) | (0 << 5);
            break;

        case R2L_D2U:   /* ���ҵ���,���µ��� */
            regval |= (1 << 7) | (1 << 6) | (0 << 5);
            break;

        case U2D_L2R:   /* ���ϵ���,������ */
            regval |= (0 << 7) | (0 << 6) | (1 << 5);
            break;

        case U2D_R2L:   /* ���ϵ���,���ҵ��� */
            regval |= (0 << 7) | (1 << 6) | (1 << 5);
            break;

        case D2U_L2R:   /* ���µ���,������ */
            regval |= (1 << 7) | (0 << 6) | (1 << 5);
            break;

        case D2U_R2L:   /* ���µ���,���ҵ��� */
            regval |= (1 << 7) | (1 << 6) | (1 << 5);
            break;
    }

    dirreg = 0X36;  /* �Ծ��󲿷�����IC, ��0X36�Ĵ������� */

    if (lcddev.id == 0X5510)
    {
        dirreg = 0X3600;    /* ����5510, ����������IC�ļĴ����в��� */
    }

    /* 7789 & 7796 Ҫ����BGRλ */
    if (lcddev.id == 0X7789 || lcddev.id == 0X7796)
    {
        regval |= 0X08;
    }

    lcd_write_reg(dirreg, regval);

    if (regval & 0X20)
    {
        if (lcddev.width < lcddev.height)   /* ����X,Y */
        {
            temp = lcddev.width;
            lcddev.width = lcddev.height;
            lcddev.height = temp;
        }
    }
    else
    {
        if (lcddev.width > lcddev.height)   /* ����X,Y */
        {
            temp = lcddev.width;
            lcddev.width = lcddev.height;
            lcddev.height = temp;
        }
    }

    /* ������ʾ����(����)��С */
    if (lcddev.id == 0X5510)
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(0);
        lcd_wr_regno(lcddev.setxcmd + 1);
        lcd_wr_data(0);
        lcd_wr_regno(lcddev.setxcmd + 2);
        lcd_wr_data((lcddev.width - 1) >> 8);
        lcd_wr_regno(lcddev.setxcmd + 3);
        lcd_wr_data((lcddev.width - 1) & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(0);
        lcd_wr_regno(lcddev.setycmd + 1);
        lcd_wr_data(0);
        lcd_wr_regno(lcddev.setycmd + 2);
        lcd_wr_data((lcddev.height - 1) >> 8);
        lcd_wr_regno(lcddev.setycmd + 3);
        lcd_wr_data((lcddev.height - 1) & 0XFF);
    }
    else
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(0);
        lcd_wr_data(0);
        lcd_wr_data((lcddev.width - 1) >> 8);
        lcd_wr_data((lcddev.width - 1) & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(0);
        lcd_wr_data(0);
        lcd_wr_data((lcddev.height - 1) >> 8);
        lcd_wr_data((lcddev.height - 1) & 0XFF);
    }
}

/**
 * @brief       ����
 * @param       x,y: ����
 * @param       color: �����ɫ(32λ��ɫ,�������TLI)
 * @retval      ��
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
    if (lcdtli.pwidth != 0)        /* �����RGB�� */
    {
        tli_draw_point(x, y, color);
    }
    else
    {
        lcd_set_cursor(x, y);       /* ���ù��λ�� */
        lcd_write_ram_prepare();    /* ��ʼд��GRAM */
        LCD->LCD_RAM = color;
    }
}

/**
 * @brief       ����LCD��ʾ����
 * @param       dir:0,����; 1,����
 * @retval      ��
 */
void lcd_display_dir(uint8_t dir)
{
    lcddev.dir = dir;          /* ����/���� */

    if (lcdtli.pwidth != 0)    /* �����RGB�� */
    {
        tli_display_dir(dir);
        lcddev.width = lcdtli.width;
        lcddev.height = lcdtli.height;
        return;
    }
    
    if (dir == 0)       /* ���� */
    {
        lcddev.width = 240;
        lcddev.height = 320;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;  /* ����д��GRAM��ָ�� */
            lcddev.setxcmd = 0X2A00;  /* ����дX����ָ�� */
            lcddev.setycmd = 0X2B00;  /* ����дY����ָ�� */
            lcddev.width = 480;       /* ���ÿ��480 */
            lcddev.height = 800;      /* ���ø߶�800 */
        }
        else   /* ����IC, ����: 5310/7789/7796��IC */
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310 || lcddev.id == 0X7796)    /* �����5310/7796 ���ʾ�� 320*480�ֱ��� */
        {
            lcddev.width = 320;
            lcddev.height = 480;
        }   
    }
    else                /* ���� */
    {
        lcddev.width = 320;         /* Ĭ�Ͽ�� */
        lcddev.height = 240;        /* Ĭ�ϸ߶� */

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;  /* ����д��GRAM��ָ�� */
            lcddev.setxcmd = 0X2A00;  /* ����дX����ָ�� */
            lcddev.setycmd = 0X2B00;  /* ����дY����ָ�� */
            lcddev.width = 800;       /* ���ÿ��800 */
            lcddev.height = 480;      /* ���ø߶�480 */
        }
        else   /* ����IC, ����: 5310/7789/7796��IC */
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310 || lcddev.id == 0X7796)    /* �����5310/7796 ���ʾ�� 320*480�ֱ��� */
        {
            lcddev.width = 480;
            lcddev.height = 320;
        }
    }

    lcd_scan_dir(DFT_SCAN_DIR);     /* Ĭ��ɨ�跽�� */
}

/**
 * @brief       ���ô���,���Զ����û������굽�������Ͻ�(sx,sy).(��RGB����Ч)
 * @param       sx,sy:������ʼ����(���Ͻ�)
 * @param       width,height:���ڿ�Ⱥ͸߶�,�������0!!
 * @note        �����С:width*height.
 *
 * @retval      ��
 */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    uint16_t twidth, theight;
    twidth = sx + width - 1;
    theight = sy + height - 1;

    if (lcdtli.pwidth != 0)      /* �����RGB�� */
    {
        return;                  /* RGB����֧�ָú��� */
    }
    
    if (lcddev.id == 0X5510)     /* 5510���ô��� */
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(sx >> 8);
        lcd_wr_regno(lcddev.setxcmd + 1);
        lcd_wr_data(sx & 0XFF);
        lcd_wr_regno(lcddev.setxcmd + 2);
        lcd_wr_data(twidth >> 8);
        lcd_wr_regno(lcddev.setxcmd + 3);
        lcd_wr_data(twidth & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(sy >> 8);
        lcd_wr_regno(lcddev.setycmd + 1);
        lcd_wr_data(sy & 0XFF);
        lcd_wr_regno(lcddev.setycmd + 2);
        lcd_wr_data(theight >> 8);
        lcd_wr_regno(lcddev.setycmd + 3);
        lcd_wr_data(theight & 0XFF);
    }
    else    /* 5310/7789/7796���ô��� */
    {
        lcd_wr_regno(lcddev.setxcmd);
        lcd_wr_data(sx >> 8);
        lcd_wr_data(sx & 0XFF);
        lcd_wr_data(twidth >> 8);
        lcd_wr_data(twidth & 0XFF);
        lcd_wr_regno(lcddev.setycmd);
        lcd_wr_data(sy >> 8);
        lcd_wr_data(sy & 0XFF);
        lcd_wr_data(theight >> 8);
        lcd_wr_data(theight & 0XFF);
    }
}

/**
 * @brief       ��ʼ��TFTLCDҺ������EXMC����
 *
 * @param       ��
 * @retval      ��
 */
void exmc_sram_init(void)
{
    exmc_norsram_parameter_struct  nor_init_struct;
    exmc_norsram_timing_parameter_struct  sram_timing_read;
    exmc_norsram_timing_parameter_struct  sram_timing_write;
      
    rcu_periph_clock_enable(RCU_EXMC);   /* ʹ��EXMCʱ�� */
	  
  	/* ���ö�ʱ����� */
    sram_timing_read.asyn_access_mode = EXMC_ACCESS_MODE_A;            /* �첽����ģʽA */
    sram_timing_read.syn_data_latency = EXMC_DATALAT_2_CLK;            /* NOR Flash������ʱ���˴�δ�õ� */
    sram_timing_read.syn_clk_division = EXMC_SYN_CLOCK_RATIO_DISABLE;  /* ͬ��ģʽʱ�ӷ�Ƶ�ȣ��˴�δ�õ�  */
    sram_timing_read.bus_latency = 0;                                  /* �����ӳ�ʱ�䣬�˴�δ�õ� */
    /* ��ΪҺ������IC�����ݵ�ʱ���ٶȲ���̫�� */
    sram_timing_read.asyn_data_setuptime = 110;                        /* ���ݽ���ʱ��(DSET)Ϊ110��CK_EXMC 1/300M = 3.3ns * 110 = 363ns */
    sram_timing_read.asyn_address_holdtime = 0;                        /* ��ַ����ʱ��(AHLD)ģʽAδ�õ� */
    sram_timing_read.asyn_address_setuptime = 15;                      /* ��ַ����ʱ��(ASET)Ϊ15��CK_EXMC = 3.3ns * 15 = 49.5ns */
  
	  /* ����дʱ����� */
  	sram_timing_write.asyn_access_mode = EXMC_ACCESS_MODE_A;           /* �첽����ģʽA */
    sram_timing_write.syn_data_latency = EXMC_DATALAT_2_CLK;
    sram_timing_write.syn_clk_division = EXMC_SYN_CLOCK_RATIO_DISABLE;
    sram_timing_write.bus_latency = 0;
    sram_timing_write.asyn_data_setuptime = 15;                        /* ���ݽ���ʱ��(DSET)Ϊ15��CK_EXMC = 3.3ns * 15 = 49.5ns */
    sram_timing_write.asyn_address_holdtime = 0;                  
    sram_timing_write.asyn_address_setuptime = 15;                     /* ��ַ����ʱ��(ASET)Ϊ15��CK_EXMC = 3.3ns * 15 = 49.5ns */

    /* ����EXMC���� */
    exmc_norsram_struct_para_init(&nor_init_struct);                   /* ��ʼ���ṹ��exmc_norsram_parameter_struct����ΪĬ��ֵ */
       
    nor_init_struct.norsram_region = EXMC_BANK0_NORSRAM_REGION1;       /* ѡ��EXMC_BANK0_NORSRAM_REGION1 */   
    nor_init_struct.write_mode = EXMC_ASYN_WRITE;                      /* ѡ��д����ģʽ(ͬ��ģʽ�����첽ģʽ)���˴�δ�õ� */
    nor_init_struct.extended_mode = ENABLE;                            /* ʹ����չģʽ */
    nor_init_struct.asyn_wait = DISABLE;                               /* �����첽�ȴ����� */
    nor_init_struct.nwait_signal = DISABLE;                            /* ��ͬ��ͻ��ģʽ�У�ʹ�ܻ��߽���NWAIT�źţ��˴�δ�õ� */
    nor_init_struct.memory_write = ENABLE;                             /* ʹ��EXMC���ⲿ�洢��д���� */
    nor_init_struct.nwait_config = EXMC_NWAIT_CONFIG_BEFORE;           /* NWAIT�ź����ã�ֻ��ͬ��ģʽ��Ч���˴�δ�õ� */ 
    nor_init_struct.nwait_polarity = EXMC_NWAIT_POLARITY_LOW;          /* NWAIT�źż��ԣ��˴�δ�õ� */
    nor_init_struct.burst_mode = DISABLE;                              /* ����ͻ��ģʽ */
    nor_init_struct.databus_width = EXMC_NOR_DATABUS_WIDTH_16B;        /* �洢���������߿��16λ */
    nor_init_struct.memory_type = EXMC_MEMORY_TYPE_SRAM;               /* �ⲿ�洢��������ΪSRAM */
    nor_init_struct.address_data_mux = DISABLE;                        /* ������/��ַ�߲����� */
    nor_init_struct.cram_page_size = EXMC_CRAM_AUTO_SPLIT;             /* ҳ�߽��Զ�ͻ���ָ� */
    nor_init_struct.read_write_timing = &sram_timing_read;             /* ��ʱ����� */
    nor_init_struct.write_timing = &sram_timing_write;                 /* дʱ����� */

    exmc_norsram_init(&nor_init_struct);                               /* ��ʼ��EXMC NOR/SRAM bank0 region1 */
	
    exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION1);                   /* ʹ��EXMC NOR/SRAM bank0 region1 */
}

/**
 * @brief       ��ʼ��LCD
 *   @note      �ó�ʼ���������Գ�ʼ�������ͺŵ�LCD(�����.c�ļ���ǰ�������)
 *
 * @param       ��
 * @retval      ��
 */
void lcd_init(void)
{
    lcddev.id = tli_panelid_read();   /* ����Ƿ���RGB������ */

#if RGB_80_8001280         
    lcddev.id = 0X8081;
#endif
  
    if (lcddev.id != 0)               /* ID����,˵����RGB������ */
    { 
        tli_config();                 /* ��ʼ��TLI */   
    }
    else
    {
        /* IO �� ʱ������ */
        rcu_periph_clock_enable(LCD_CS_GPIO_CLK);    /* ʹ��LCD_CS����ʱ�� */
        rcu_periph_clock_enable(LCD_RS_GPIO_CLK);    /* ʹ��LCD_RS����ʱ�� */
        rcu_periph_clock_enable(LCD_WR_GPIO_CLK);    /* ʹ��LCD_WR����ʱ�� */
        rcu_periph_clock_enable(LCD_RD_GPIO_CLK);    /* ʹ��LCD_RD����ʱ�� */
        rcu_periph_clock_enable(LCD_BL_GPIO_CLK);    /* ʹ��LCD_BL����ʱ�� */
        rcu_periph_clock_enable(LCD_RST_GPIO_CLK);   /* ʹ��LCD_RST����ʱ�� */
      
        rcu_periph_clock_enable(RCU_GPIOD);          /* ʹ��GPIODʱ�� */
        rcu_periph_clock_enable(RCU_GPIOE);          /* ʹ��GPIOEʱ�� */
        
        /* ���ñ����������LCD_BL ������� */
        gpio_mode_set(LCD_BL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LCD_BL_GPIO_PIN);
        gpio_output_options_set(LCD_BL_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_BL_GPIO_PIN);

        /* ����LCD��λ����LCD_RST ������� */
        gpio_mode_set(LCD_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LCD_RST_GPIO_PIN);
        gpio_output_options_set(LCD_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_RST_GPIO_PIN);

        /* ����LCD_CS����(EXMC_NE1)  ����������� */
        gpio_af_set(LCD_CS_GPIO_PORT, GPIO_AF_12, LCD_CS_GPIO_PIN);
        gpio_mode_set(LCD_CS_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, LCD_CS_GPIO_PIN);
        gpio_output_options_set(LCD_CS_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_CS_GPIO_PIN);
      
        /* ����LCD_RS����(EXMC_A12)  ����������� */
        gpio_af_set(LCD_RS_GPIO_PORT, GPIO_AF_12, LCD_RS_GPIO_PIN);
        gpio_mode_set(LCD_RS_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, LCD_RS_GPIO_PIN);
        gpio_output_options_set(LCD_RS_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_RS_GPIO_PIN); 

        /* ����LCD_WR����(EXMC_NWE)  ����������� */
        gpio_af_set(LCD_WR_GPIO_PORT, GPIO_AF_12, LCD_WR_GPIO_PIN);
        gpio_mode_set(LCD_WR_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, LCD_WR_GPIO_PIN);
        gpio_output_options_set(LCD_WR_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_WR_GPIO_PIN);
      
        /* ����LCD_RD����(EXMC_NOE)  ����������� */
        gpio_af_set(LCD_RD_GPIO_PORT, GPIO_AF_12, LCD_RD_GPIO_PIN);
        gpio_mode_set(LCD_RD_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, LCD_RD_GPIO_PIN);
        gpio_output_options_set(LCD_RD_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, LCD_RD_GPIO_PIN); 
        
        /* ����PD0,1,8,9,10,14,15  ����������� */
        gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |  GPIO_PIN_14 | GPIO_PIN_15);
        gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |  GPIO_PIN_14 | GPIO_PIN_15);
        gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |  GPIO_PIN_14 | GPIO_PIN_15);

        /* ����PE7~15  ����������� */
        gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
            
        exmc_sram_init();           /* ��ʼ��EXMC NOR/SRAM���� */

        lcd_opt_delay(0X1FFFF);     /* ��ʼ��EXMC��,�ȴ�һ��ʱ����ܿ�ʼ��ʼ�� */

        /* LCD��λ */
        LCD_RST(1);
        delay_ms(10);
        LCD_RST(0);
        delay_ms(50);
        LCD_RST(1); 
        delay_ms(200); 

        /* ����7796 ID�Ķ�ȡ */
        lcd_wr_regno(0XD3);
        lcddev.id = lcd_rd_data();  /* dummy read */
        lcddev.id = lcd_rd_data();  /* ����0X00 */
        lcddev.id = lcd_rd_data();  /* ��ȡ0X77 */
        lcddev.id <<= 8;
        lcddev.id |= lcd_rd_data(); /* ��ȡ0X96 */

        if (lcddev.id != 0X7796)    /* ����7796,���Կ����ǲ���ST7789 */
        {
            lcd_wr_regno(0X04);
            lcddev.id = lcd_rd_data();      /* dummy read */
            lcddev.id = lcd_rd_data();      /* ����0X85 */
            lcddev.id = lcd_rd_data();      /* ��ȡ0X85 */
            lcddev.id <<= 8;
            lcddev.id |= lcd_rd_data();     /* ��ȡ0X52 */
            
            if (lcddev.id == 0X8552)        /* ��8552��IDת����7789 */
            {
                lcddev.id = 0x7789;
            }

            if (lcddev.id != 0x7789)        /* Ҳ����ST7789,�����ǲ���NT35310 */
            {
                lcd_wr_regno(0XD4);
                lcddev.id = lcd_rd_data();  /* dummy read */
                lcddev.id = lcd_rd_data();  /* ����0X01 */
                lcddev.id = lcd_rd_data();  /* ����0X53 */
                lcddev.id <<= 8;
                lcddev.id |= lcd_rd_data(); /* �������0X10 */

                if (lcddev.id != 0X5310)    /* Ҳ����NT35310,���Կ����ǲ���NT35510 */
                {
                    /* ������Կ�������ṩ,�հἴ�ɣ� */
                    lcd_write_reg(0xF000, 0x0055);
                    lcd_write_reg(0xF001, 0x00AA);
                    lcd_write_reg(0xF002, 0x0052);
                    lcd_write_reg(0xF003, 0x0008);
                    lcd_write_reg(0xF004, 0x0001);
                    
                    lcd_wr_regno(0xC500);           /* ��ȡID��8λ */
                    lcddev.id = lcd_rd_data();      /* ����0X55 */
                    lcddev.id <<= 8;

                    lcd_wr_regno(0xC501);           /* ��ȡID��8λ */
                    lcddev.id |= lcd_rd_data();     /* ����0X10 */               
                }
            }
        }
    }

    /* �ر�ע��, �����main�����������δ���0��ʼ��, ��Ῠ����printf
     * ����(������f_putc����), ����, �����ʼ������0, �������ε�����
     * ���� printf ��� !!!!!!!
     */
    printf("LCD ID:%x\r\n", lcddev.id); /* ��ӡLCD ID */

    if (lcddev.id == 0X7789)
    {
        lcd_ex_st7789_reginit();    /* ִ��ST7789��ʼ�� */
    }
    else if (lcddev.id == 0x5310)
    {
        lcd_ex_nt35310_reginit();   /* ִ��NT35310��ʼ�� */
    }
    else if (lcddev.id == 0x7796)
    {
        lcd_ex_st7796_reginit();    /* ִ��ST7796��ʼ�� */
    }
    else if (lcddev.id == 0x5510)
    {
        lcd_ex_nt35510_reginit();   /* ִ��NT35510��ʼ�� */
    }
    
    /* ���ڲ�ͬ��Ļ��дʱ��ͬ�������ʱ����Ը����Լ�����Ļ�����޸�
      �������ϳ����߶�ʱ��Ҳ����Ӱ�죬��Ҫ�Լ���������޸ģ� */
    /* ��ʼ������Ժ�,���� */
    if (lcddev.id == 0X7789)
    {
        /* ��������дʱ��Ĵ�����ʱ�� */
        LCD_EXMC_SNWTCFGX &= ~(0XF << 0);   /* �첽��ַ����ʱ��(WASET)���� */
        LCD_EXMC_SNWTCFGX &= ~(0XFF << 8);  /* �첽���ݽ���ʱ��(WDSET)���� */
        LCD_EXMC_SNWTCFGX |= 7 << 0;        /* �첽��ַ����ʱ��(WASET)Ϊ7��CK_EXMC = 23.1ns */
        LCD_EXMC_SNWTCFGX |= 7 << 8;        /* �첽���ݽ���ʱ��(WDSET)Ϊ7��CK_EXMC = 23.1ns */
    }
    else if (lcddev.id == 0X5510)
    { 
        /* ��������дʱ��Ĵ�����ʱ�� */
        LCD_EXMC_SNWTCFGX &= ~(0XF << 0);   /* �첽��ַ����ʱ��(WASET)���� */
        LCD_EXMC_SNWTCFGX &= ~(0XFF << 8);  /* �첽���ݽ���ʱ��(WDSET)���� */
        LCD_EXMC_SNWTCFGX |= 3 << 0;        /* �첽��ַ����ʱ��(WASET)Ϊ3��CK_EXMC = 9.9ns */
        LCD_EXMC_SNWTCFGX |= 3 << 8;        /* �첽���ݽ���ʱ��(WDSET)Ϊ3��CK_EXMC = 9.9ns */
    }
    else if (lcddev.id == 0X5310 || lcddev.id == 0X7796)
    {
        /* ��������дʱ��Ĵ�����ʱ�� */
        LCD_EXMC_SNWTCFGX &= ~(0XF << 0);   /* �첽��ַ����ʱ��(WASET)���� */
        LCD_EXMC_SNWTCFGX &= ~(0XFF << 8);  /* �첽���ݽ���ʱ��(WDSET)���� */
        LCD_EXMC_SNWTCFGX |= 3 << 0;        /* �첽��ַ����ʱ��(WASET)Ϊ3��CK_EXMC = 9.9ns */
        LCD_EXMC_SNWTCFGX |= 3 << 8;        /* �첽���ݽ���ʱ��(WDSET)Ϊ3��CK_EXMC = 9.9ns */
    }

    if (lcdtli.pwidth != 0)      /* �����RGB�� */
    {
        lcd_display_dir(1);      /* Ĭ��Ϊ�M�� */
    }
    else
    {
        lcd_display_dir(0);      /* Ĭ��Ϊ���� */
    }
    
    LCD_BL(1);                   /* �������� */
    lcd_clear(WHITE);            /* ���� */
}

/**
 * @brief       ��������
 * @param       color: Ҫ��������ɫ(32λ��ɫ,�������TLI)
 * @retval      ��
 */
void lcd_clear(uint32_t color)
{
    uint32_t index = 0;
    uint32_t totalpoint = lcddev.width;
  
    if (lcdtli.pwidth != 0)             /* �����RGB�� */
    {
        tli_clear(color);
    }
    else
    {
        totalpoint *= lcddev.height;    /* �õ��ܵ��� */        
        lcd_set_cursor(0x00, 0x0000);   /* ���ù��λ�� */
        lcd_write_ram_prepare();        /* ��ʼд��GRAM */

        for (index = 0; index < totalpoint; index++)
        {
            LCD->LCD_RAM = color;
        }
    }
}

/**
 * @brief       ��ָ����������䵥����ɫ
 * @param       (sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex - sx + 1) * (ey - sy + 1)
 * @param       color: Ҫ������ɫ(32λ��ɫ,�������TLI)
 * @retval      ��
 */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    uint16_t i, j;
    uint16_t xlen = 0;

    if (lcdtli.pwidth != 0)             /* �����RGB�� */
    {
        tli_fill(sx, sy, ex, ey, color);
    }
    else
    {
        xlen = ex - sx + 1;

        for (i = sy; i <= ey; i++)
        {
            lcd_set_cursor(sx, i);      /* ���ù��λ�� */
            lcd_write_ram_prepare();    /* ��ʼд��GRAM */

            for (j = 0; j < xlen; j++)
            {
                LCD->LCD_RAM = color;   /* ��ʾ��ɫ */
            }
        }
    }
}

/**
 * @brief       ��ָ�����������ָ����ɫ��
 * @param       (sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex - sx + 1) * (ey - sy + 1)
 * @param       color: Ҫ������ɫ�����׵�ַ
 * @retval      ��
 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;
 
    if (lcdtli.pwidth != 0)             /* �����RGB�� */
    {
        tli_color_fill(sx, sy, ex, ey, color);
    }
    else
    {
        width = ex - sx + 1;            /* �õ����Ŀ�� */
        height = ey - sy + 1;           /* �߶� */

        for (i = 0; i < height; i++)
        {
            lcd_set_cursor(sx, sy + i); /* ���ù��λ�� */
            lcd_write_ram_prepare();    /* ��ʼд��GRAM */

            for (j = 0; j < width; j++)
            {
                LCD->LCD_RAM = color[i * width + j]; /* д������ */
            }
        }
    }
}

/**
 * @brief       ����
 * @param       x1,y1: �������
 * @param       x2,y2: �յ�����
 * @param       color: �ߵ���ɫ
 * @retval      ��
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    delta_x = x2 - x1;          /* ������������ */
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0) incx = 1;          /* ���õ������� */
    else if (delta_x == 0) incx = 0;    /* ��ֱ�� */
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;    /* ˮƽ�� */
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y) distance = delta_x; /* ѡȡ�������������� */
    else distance = delta_y;

    for (t = 0; t <= distance + 1; t++ )    /* ������� */
    {
        lcd_draw_point(row, col, color);    /* ���� */
        xerr += delta_x ;
        yerr += delta_y ;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief       ��ˮƽ��
 * @param       x,y  : �������
 * @param       len  : �߳���
 * @param       color: ���ε���ɫ
 * @retval      ��
 */
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint32_t color)
{
    if ((len == 0) || (x > lcddev.width) || (y > lcddev.height)) return;

    lcd_fill(x, y, x + len - 1, y, color);
}

/**
 * @brief       ������
 * @param       x1,y1: �������
 * @param       x2,y2: �յ�����
 * @param       color: ���ε���ɫ
 * @retval      ��
 */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
    lcd_draw_line(x1, y1, x2, y1, color);
    lcd_draw_line(x1, y1, x1, y2, color);
    lcd_draw_line(x1, y2, x2, y2, color);
    lcd_draw_line(x2, y1, x2, y2, color);
}

/**
 * @brief       ��Բ
 * @param       x0,y0: Բ��������
 * @param       r    : �뾶
 * @param       color: Բ����ɫ
 * @retval      ��
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint32_t color)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);       /* �ж��¸���λ�õı�־ */

    while (a <= b)
    {
        lcd_draw_point(x0 + a, y0 - b, color);  /* 5 */
        lcd_draw_point(x0 + b, y0 - a, color);  /* 0 */
        lcd_draw_point(x0 + b, y0 + a, color);  /* 4 */
        lcd_draw_point(x0 + a, y0 + b, color);  /* 6 */
        lcd_draw_point(x0 - a, y0 + b, color);  /* 1 */
        lcd_draw_point(x0 - b, y0 + a, color);
        lcd_draw_point(x0 - a, y0 - b, color);  /* 2 */
        lcd_draw_point(x0 - b, y0 - a, color);  /* 7 */
        a++;

        /* ʹ��Bresenham�㷨��Բ */
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

/**
 * @brief       ���ʵ��Բ
 * @param       x,y  : Բ��������
 * @param       r    : �뾶
 * @param       color: Բ����ɫ
 * @retval      ��
 */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint32_t color)
{
    uint32_t i;
    uint32_t imax = ((uint32_t)r * 707) / 1000 + 1;
    uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2;
    uint32_t xr = r;

    lcd_draw_hline(x - r, y, 2 * r, color);

    for (i = 1; i <= imax; i++)
    {
        if ((i * i + xr * xr) > sqmax)
        {
            /* draw lines from outside */
            if (xr > imax)
            {
                lcd_draw_hline (x - i + 1, y + xr, 2 * (i - 1), color);
                lcd_draw_hline (x - i + 1, y - xr, 2 * (i - 1), color);
            }

            xr--;
        }

        /* draw lines from inside (center) */
        lcd_draw_hline(x - xr, y + i, 2 * xr, color);
        lcd_draw_hline(x - xr, y - i, 2 * xr, color);
    }
}

/**
 * @brief       ��ָ��λ����ʾһ���ַ�
 * @param       x,y   : ����
 * @param       chr   : Ҫ��ʾ���ַ�:' '--->'~'
 * @param       size  : �����С 12/16/24/32
 * @param       mode  : ���ӷ�ʽ(1); �ǵ��ӷ�ʽ(0);
 * @param       color : �ַ�����ɫ;
 * @retval      ��
 */
void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint32_t color)
{
    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize = 0;
    uint8_t *pfont = 0;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); /* �õ�����һ���ַ���Ӧ������ռ���ֽ��� */
    chr = chr - ' ';    /* �õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩ */

    switch (size)
    {
        case 12:
            pfont = (uint8_t *)asc2_1206[chr];  /* ����1206���� */
            break;

        case 16:
            pfont = (uint8_t *)asc2_1608[chr];  /* ����1608���� */
            break;

        case 24:
            pfont = (uint8_t *)asc2_2412[chr];  /* ����2412���� */
            break;

        case 32:
            pfont = (uint8_t *)asc2_3216[chr];  /* ����3216���� */
            break;

        default:
            return ;
    }

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t];    /* ��ȡ�ַ��ĵ������� */

        for (t1 = 0; t1 < 8; t1++)  /* һ���ֽ�8���� */
        {
            if (temp & 0x80)        /* ��Ч��,��Ҫ��ʾ */
            {
                lcd_draw_point(x, y, color);        /* �������,Ҫ��ʾ����� */
            }
            else if (mode == 0)     /* ��Ч�㲢��ѡ��ǵ��ӷ�ʽ */
            {
                lcd_draw_point(x, y, g_back_color); /* ������ɫ,�൱������㲻��ʾ(ע�ⱳ��ɫ��ȫ�ֱ�������) */
            }

            temp <<= 1; /* ��λ, �Ա��ȡ��һ��λ��״̬ */
            y++;

            if (y >= lcddev.height) return;     /* �������� */

            if ((y - y0) == size)               /* ��ʾ��һ����? */
            {
                y = y0;                         /* y���긴λ */
                x++;                            /* x������� */

                if (x >= lcddev.width) return;  /* x���곬������ */

                break;
            }
        }
    }
}

/**
 * @brief       ƽ������, m^n
 * @param       m: ����
 * @param       n: ָ��
 * @retval      m��n�η�
 */
static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--) result *= m;

    return result;
}

/**
 * @brief       ��ʾlen������(��λΪ0����ʾ)
 * @param       x,y   : ��ʼ����
 * @param       num   : ��ֵ(0 ~ 2^32)
 * @param       len   : ��ʾ���ֵ�λ��
 * @param       size  : ѡ������ 12/16/24/32
 * @param       color : ���ֵ���ɫ;
 * @retval      ��
 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint32_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)   /* ������ʾλ��ѭ�� */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;   /* ��ȡ��Ӧλ������ */

        if (enshow == 0 && t < (len - 1))   /* û��ʹ����ʾ,�һ���λҪ��ʾ */
        {
            if (temp == 0)
            {
                lcd_show_char(x + (size / 2) * t, y, ' ', size, 0, color);  /* ��ʾ�ո�,ռλ */
                continue;   /* �����¸�һλ */
            }
            else
            {
                enshow = 1; /* ʹ����ʾ */
            }
        }

        lcd_show_char(x + (size / 2) * t, y, temp + '0', size, 0, color);   /* ��ʾ�ַ� */
    }
}

/**
 * @brief       ��չ��ʾlen������(��λ��0Ҳ��ʾ)
 * @param       x,y   : ��ʼ����
 * @param       num   : ��ֵ(0 ~ 2^32)
 * @param       len   : ��ʾ���ֵ�λ��
 * @param       size  : ѡ������ 12/16/24/32
 * @param       mode  : ��ʾģʽ
 *              [7]:0,�����;1,���0.
 *              [6:1]:����
 *              [0]:0,�ǵ�����ʾ;1,������ʾ.
 * @param       color : ���ֵ���ɫ;
 * @retval      ��
 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint32_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)   /* ������ʾλ��ѭ�� */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;    /* ��ȡ��Ӧλ������ */

        if (enshow == 0 && t < (len - 1))   /* û��ʹ����ʾ,�һ���λҪ��ʾ */
        {
            if (temp == 0)
            {
                if (mode & 0X80)   /* ��λ��Ҫ���0 */
                {
                    lcd_show_char(x + (size / 2) * t, y, '0', size, mode & 0X01, color);  /* ��0ռλ */
                }
                else
                {
                    lcd_show_char(x + (size / 2) * t, y, ' ', size, mode & 0X01, color);  /* �ÿո�ռλ */
                }

                continue;
            }
            else
            {
                enshow = 1; /* ʹ����ʾ */
            }
        }

        lcd_show_char(x + (size / 2) * t, y, temp + '0', size, mode & 0X01, color);
    }
}

/**
 * @brief       ��ʾ�ַ���
 * @param       x,y         : ��ʼ����
 * @param       width,height: �����С
 * @param       size        : ѡ������ 12/16/24/32
 * @param       p           : �ַ����׵�ַ
 * @param       color       : �ַ�������ɫ
 * @retval      ��
 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint32_t color)
{
    uint8_t x0 = x;
    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' '))   /* �ж��ǲ��ǷǷ��ַ�! */
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }

        if (y >= height) break; /* �˳� */

        lcd_show_char(x, y, *p, size, 0, color);
        x += size / 2;
        p++;
    }
}








