/**
 ****************************************************************************************************
 * @file        tli.c
 * @version     V1.0
 * @brief       TLI ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/tli.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"


#if !(__ARMCC_VERSION >= 6010050)   /* ����AC6����������ʹ��AC5������ʱ */

/* ���ݲ�ͬ����ɫ��ʽ,����֡�������� */
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    uint32_t tli_lcd_framebuf[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR)));     /* ����������ֱ���ʱ,LCD�����֡���������С */
#else
    uint16_t tli_lcd_framebuf[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR)));     /* ����������ֱ���ʱ,LCD�����֡���������С */
    //uint16_t tli_lcd_framebuf1[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR + 1280 * 800 * 2)));  /* ʹ��TLI��2ʱʹ��(Ĭ��ֻʹ��TLI��1) */
#endif

#else   /* ʹ��AC6������ʱ */

/* ���ݲ�ͬ����ɫ��ʽ,����֡�������� */
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    uint32_t tli_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));     /* ����������ֱ���ʱ,LCD�����֡���������С */
#else
    uint16_t tli_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));     /* ����������ֱ���ʱ,LCD�����֡���������С */
#endif

#endif


uint32_t *g_tli_framebuf[2];       /* TLI LCD֡��������ָ��,����ָ���Ӧ��С���ڴ����� */
_tli_dev lcdtli;                   /* ����LCD TLI����Ҫ���� */


/**
 * @brief       TLIѡ���
 * @param       layerx      : 0,��һ��; 1,�ڶ���;
 * @retval      ��
 */
void tli_select_layer(uint8_t layerx)
{
    lcdtli.activelayer = layerx;
}

/**
 * @brief       TLI��ʾ��������
 * @param       dir         : 0,����; 1,����;
 * @retval      ��
 */
void tli_display_dir(uint8_t dir)
{
    lcdtli.dir = dir;       /* ��ʾ���� */

    if (dir == 0)           /* ���� */
    {
        lcdtli.width = lcdtli.pheight;
        lcdtli.height = lcdtli.pwidth;
    }
    else if (dir == 1)      /* ���� */
    {
        lcdtli.width = lcdtli.pwidth;
        lcdtli.height = lcdtli.pheight;
    }
}

/**
 * @brief       TLI���㺯��
 * @param       x,y         : ����
 * @param       color       : ��ɫֵ
 * @retval      ��
 */
void tli_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888

    if (lcdtli.dir)     /* ���� */
    {
        *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
    }
    else                /* ���� */
    {
        *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
    }

#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888

    if (lcdtli.dir)     /* ���� */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
        *(uint8_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x) + 2) = color >> 16;
    }
    else                /* ���� */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
        *(uint8_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y) + 2) = color >> 16;
    }
    
#else

    if (lcdtli.dir)     /* ���� */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
    }
    else                /* ���� */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
    }

#endif
}

/**
 * @brief       TLI���㺯��
 * @param       x,y         : ����
 * @retval      ��ɫֵ
 */
uint32_t tli_read_point(uint16_t x, uint16_t y)
{
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888

    if (lcdtli.dir)     /* ���� */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x));
    }
    else                /* ���� */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y));
    }

#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888

    if (lcdtli.dir)     /* ���� */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) & 0XFFFFFF;
    }
    else                /* ���� */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) & 0XFFFFFF;
    }
        
#else

    if (lcdtli.dir)     /* ���� */
    {
        return *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x));
    }
    else                /* ���� */
    {
        return *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y));
    }

#endif
}

/**
 * @brief       TLI������, ʹ��IPA���
 * @note       (sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex - sx + 1) * (ey - sy + 1)
 *              ע��:sx,ex,���ܴ���lcddev.width - 1; sy,ey,���ܴ���lcddev.height - 1
 * @param       sx,sy       : ��ʼ����
 * @param       ex,ey       : ��������
 * @param       color       : ������ɫ
 * @retval      ��
 */
void tli_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    ipa_destination_parameter_struct  ipa_destination_init_struct;
  
    uint32_t psx, psy, pex, pey;    /* ��LCD���Ϊ��׼������ϵ,����������仯���仯 */
    uint32_t timeout = 0;
    uint16_t offline;
    uint32_t addr;

    /* ����ϵת�� */
    if (lcdtli.dir)     /* ���� */
    {
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                /* ���� */
    {
        if(ex >= lcdtli.pheight)ex = lcdtli.pheight - 1;  /* ���Ʒ�Χ */
        if(sx >= lcdtli.pheight)sx = lcdtli.pheight - 1;  /* ���Ʒ�Χ */
        
        psx = sy;
        psy = lcdtli.pheight - ex - 1;
        pex = ey;
        pey = lcdtli.pheight - sx - 1;
    }

    offline = lcdtli.pwidth - (pex - psx + 1);     /* ��ƫ��:��ǰ�����һ�����غ���һ�е�һ������֮���������Ŀ */
    addr = ((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * psy + psx));
    
    rcu_periph_clock_enable(RCU_IPA);     /* ʹ��IPAʱ�� */
    
    ipa_deinit();                                                                        /* ���³�ʼ��IPA */                                         

    ipa_pixel_format_convert_mode_set(IPA_FILL_UP_DE);                                   /* ���ظ�ʽת��ģʽ:���ض�����ɫ���Ŀ��洢�� */

    /* ����IPAĿ��� */
    ipa_destination_struct_para_init(&ipa_destination_init_struct);                      /* ��Ĭ��ֵ��ʼ��IPAĿ������ṹ�� */ 

    ipa_destination_init_struct.destination_pf = TLI_PIXFORMAT;                          /* Ŀ��洢�����ظ�ʽ */
    ipa_destination_init_struct.destination_memaddr = (uint32_t)addr;                    /* Ŀ��洢������ַ */

#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    
    ipa_destination_init_struct.destination_preblue = color & 0XFF;                      /* Ŀ���Ԥ������ɫֵ */
    ipa_destination_init_struct.destination_pregreen = (color & 0XFF00) >> 8;            /* Ŀ���Ԥ������ɫֵ */
    ipa_destination_init_struct.destination_prered = (color & 0XFF0000) >> 16;           /* Ŀ���Ԥ�����ɫֵ */
    
#else

    ipa_destination_init_struct.destination_preblue = color & 0X1F;                      /* Ŀ���Ԥ������ɫֵ */
    ipa_destination_init_struct.destination_pregreen = (color & 0X07E0) >> 5;            /* Ŀ���Ԥ������ɫֵ */
    ipa_destination_init_struct.destination_prered = (color & 0XF800) >> 11;             /* Ŀ���Ԥ�����ɫֵ */
    
#endif   

    ipa_destination_init_struct.destination_prealpha = (color & 0XFF000000) >> 24;       /* Ŀ���Ԥ����alphaͨ��ֵ */
    ipa_destination_init_struct.destination_lineoff = offline;                           /* Ŀ��洢����ƫ�� */
    ipa_destination_init_struct.image_height = pey - psy + 1;                            /* ͼ��߶� */
    ipa_destination_init_struct.image_width = pex - psx + 1;                             /* ͼ���� */
    ipa_destination_init_struct.image_rotate = DESTINATION_ROTATE_0;                     /* ͼ����ת�Ƕ� */    
    ipa_destination_init_struct.image_hor_decimation = DESTINATION_HORDECIMATE_DISABLE;  /* ͼ��ˮƽԤ��ȡ�˲������� */
    ipa_destination_init_struct.image_ver_decimation = DESTINATION_VERDECIMATE_DISABLE;  /* ͼ��ֱԤ��ȡ�˲������� */    
    ipa_destination_init(&ipa_destination_init_struct);                                  /* ��ʼ��Ŀ��洢������ */

    ipa_transfer_enable();                                                               /* ʹ��IPA���� */
    
    while(ipa_interrupt_flag_get(IPA_INT_FLAG_FTF) == RESET)                             /* �ȴ�������� */
    {
        timeout++;

        if (timeout > 0X1FFFFF)break;                                                    /* ��ʱ�˳� */
    }
    
    ipa_interrupt_flag_clear(IPA_INT_FLAG_FTF);                                          /* �����������жϱ�־ */
}

/**
 * @brief       ��ָ�����������ָ����ɫ��, ʹ��IPA���
 *              (sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex - sx + 1) * (ey - sy + 1)
 *              ע��:sx,ex,���ܴ���lcddev.width - 1; sy,ey,���ܴ���lcddev.height - 1
 * @param       sx,sy       : ��ʼ����
 * @param       ex,ey       : ��������
 * @param       color       : ������ɫ�����׵�ַ
 * @retval      ��
 */
void tli_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    ipa_destination_parameter_struct  ipa_destination_init_struct;
    ipa_foreground_parameter_struct   ipa_fg_init_struct;
  
    uint32_t psx, psy, pex, pey;    /* ��LCD���Ϊ��׼������ϵ,����������仯���仯 */
    uint32_t timeout = 0;
    uint16_t offline;
    uint32_t addr;

    /* ����ϵת�� */
    if (lcdtli.dir)     /* ���� */
    { 
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                /* ���� */
    {
        psx = sy;
        psy = lcdtli.pheight - ex - 1;
        pex = ey;
        pey = lcdtli.pheight - sx - 1;
    }

    offline = lcdtli.pwidth - (pex - psx + 1);     /* ��ƫ��:��ǰ�����һ�����غ���һ�е�һ������֮���������Ŀ */
    addr = ((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * psy + psx));

    rcu_periph_clock_enable(RCU_IPA);     /* ʹ��IPAʱ�� */
    
    ipa_deinit();                                                          /* ���³�ʼ��IPA */

    ipa_pixel_format_convert_mode_set(IPA_FGTODE);                         /* ���ظ�ʽת��ģʽ:ǰ����洢����Ŀ��洢�������ظ�ʽת�� */

    /* ����IPAĿ��� */
    ipa_destination_struct_para_init(&ipa_destination_init_struct);        /* ��Ĭ��ֵ��ʼ��IPAĿ������ṹ�� */ 
    
    ipa_destination_init_struct.destination_pf = TLI_PIXFORMAT;            /* Ŀ��洢�����ظ�ʽ */
    ipa_destination_init_struct.destination_memaddr = (uint32_t)addr;      /* Ŀ��洢������ַ */
    ipa_destination_init_struct.destination_preblue = 0;                   /* Ŀ���Ԥ������ɫֵ */
    ipa_destination_init_struct.destination_pregreen = 0;                  /* Ŀ���Ԥ������ɫֵ */    
    ipa_destination_init_struct.destination_prered = 0;                    /* Ŀ���Ԥ�����ɫֵ */
    ipa_destination_init_struct.destination_prealpha = 0;                  /* Ŀ���Ԥ����alphaͨ��ֵ */
    ipa_destination_init_struct.destination_lineoff = offline;             /* Ŀ��洢����ƫ�� */
    ipa_destination_init_struct.image_height = pey - psy + 1;              /* ͼ��߶� */
    ipa_destination_init_struct.image_width = pex - psx + 1;               /* ͼ���� */ 
    ipa_destination_init_struct.image_rotate = DESTINATION_ROTATE_0;       /* ͼ����ת�Ƕ� */
    ipa_destination_init_struct.image_hor_decimation = DESTINATION_HORDECIMATE_DISABLE;   /* ͼ��ˮƽԤ��ȡ�˲������� */
    ipa_destination_init_struct.image_ver_decimation = DESTINATION_VERDECIMATE_DISABLE;   /* ͼ��ֱԤ��ȡ�˲������� */    
    ipa_destination_init(&ipa_destination_init_struct);                    /* ��ʼ��Ŀ��洢������ */

    /* ����IPAǰ���� */
    ipa_foreground_struct_para_init(&ipa_fg_init_struct);                  /* ��Ĭ��ֵ��ʼ��IPAǰ��������ṹ�� */

    ipa_fg_init_struct.foreground_memaddr = (uint32_t)color;               /* ǰ����洢������ַ */
    ipa_fg_init_struct.foreground_pf = TLI_PIXFORMAT;                      /* ǰ�������ظ�ʽ */
    ipa_fg_init_struct.foreground_alpha_algorithm = IPA_FG_ALPHA_MODE_0;   /* ǰ����alphaֵ�����㷨 */
    ipa_fg_init_struct.foreground_prealpha = 0x00;                         /* ǰ����Ԥ����alphaͨ��ֵ */
    ipa_fg_init_struct.foreground_lineoff = 0x00;                          /* ǰ������ƫ�� */
    ipa_fg_init_struct.foreground_preblue = 0x00;                          /* ǰ����Ԥ������ɫֵ */
    ipa_fg_init_struct.foreground_pregreen = 0x00;                         /* ǰ����Ԥ������ɫֵ */
    ipa_fg_init_struct.foreground_prered = 0x00;                           /* ǰ����Ԥ�����ɫֵ */
    ipa_fg_init_struct.foreground_efuv_memaddr = 0x00;                     /* ǰ����ż��֡/UV�洢������ַ */
    ipa_fg_init_struct.foreground_interlace_mode = DISABLE;                /* ����ǰ�����������ģʽ */
    ipa_foreground_init(&ipa_fg_init_struct);                              /* ��ʼ��ǰ������� */

    ipa_transfer_enable();                                                 /* ʹ��IPA���� */
    
    while(ipa_interrupt_flag_get(IPA_INT_FLAG_FTF) == RESET)               /* �ȴ�������� */
    {
        timeout++;

        if (timeout > 0X1FFFFF)break;                                      /* ��ʱ�˳� */
    }
    
    ipa_interrupt_flag_clear(IPA_INT_FLAG_FTF);                            /* �����������жϱ�־ */
}

/**
 * @brief       TLI����
 * @param       color       : ��ɫֵ
 * @retval      ��
 */
void tli_clear(uint32_t color)
{
    tli_fill(0, 0, lcdtli.width - 1, lcdtli.height - 1, color);    
}

/**
 * @brief       TLIʱ��(Fdclk)���ú���
 * @param       pll2_psc      : PLL2 VCO Դʱ�ӷ�Ƶ��,   ȡֵ��Χ:1~63.
 * @param       pll2_n        : PLL2 VCO ʱ�ӱ�Ƶ����,   ȡֵ��Χ:9~512
 * @param       pll2_r        : PLL2R ���Ƶ�ʵķ�Ƶϵ����PLL2 VCO ʱ����Ϊ���룩,   ȡֵ��Χ:1~128
 * @param       pll2_r_div    : PLL2R ʱ�ӵķ�Ƶ���ӣ��������� TLI ģ���ʱ��Դ��,   ȡֵ��Χ:0~3(RCU_PLL2R_DIVx(x=2,4,8,16),��ӦPLL2R��x��Ƶ )
 *
 * @note        CK_PLL2VCOSRC = CK_PLL2SRC / pll2_psc;
 *              CK_PLL2VCO = CK_PLL2VCOSRC * pll2_n;
 *              CK_TLI = CK_PLL2VCO / pll2_r / pll2_r_div = CK_PLL2SRC / pll2_psc * pll2_n / pll2_r / pll2_r_div;
 *              ����:
 *              CK_PLL2SRC:PLL2 Դʱ��(ϵͳ��ʼ����ʱ��ѡ��CK_HXTALʱ����ΪPLL��PLL1��PLL2Դʱ��)
 *              CK_PLL2VCOSRC:PLL2 VCO Դʱ��
 *              CK_PLL2VCO:PLL2 VCO ���ʱ��
 *              ����:�ⲿ����(HXTAL_VALUE)Ϊ25M, pll2_psc = 25��ʱ��, CK_PLL2VCOSRC = 1Mhz.
 *              ����:Ҫ�õ�33M��TLIʱ��, ���������: pll2_n = 396, pll2_r = 3, pll2_r_div = 1(RCU_PLL2R_DIV4)
 *              CK_TLI = 25 / 25 * 396 / 3 / 4 = 396 / 12 = 33Mhz
 * @retval      0, �ɹ�;
 *              ����, ʧ��;
 */
uint8_t tli_clk_set(uint32_t pll2_psc, uint32_t pll2_n, uint32_t pll2_r, uint32_t pll2_r_div)
{
    uint8_t status = 0;

    rcu_pll_input_output_clock_range_config(IDX_PLL2, RCU_PLL2RNG_1M_2M, RCU_PLL2VCO_192M_836M); /* ����PLL2����/���ʱ�ӷ�Χ  */
  
    if (ERROR == rcu_pll2_config(pll2_psc, pll2_n, 3, 3, pll2_r))  /* ����PLL2ʱ�� */
    {
        status = 1;                                                /* ������������ */ 
    }

    rcu_pll_clock_output_enable(RCU_PLL2R);                        /* ʹ��PLL2Rʱ����� */
    
    rcu_tli_clock_div_config(pll2_r_div);                          /* ���ô�PLL2Rʱ�ӷ�Ƶ��TLI��Ƶϵ�� */

    rcu_osci_on(RCU_PLL2_CK);                                      /* ��PLL2ʱ�� */

    if (ERROR == rcu_osci_stab_wait(RCU_PLL2_CK))                  /* �ȴ�PLL2ʱ������ */ 
    {
        status = 2;
    }    
    
    return status;
}

/**
 * @brief       TLI��ȡ���ID
 * @note        ����LCD RGB�ߵ����λ(R7,G7,B7)��ʶ�����ID
 *              PG6 = R7(M0); PB15 = G7(M1); PB9 = B7(M2);
 *              M2:M1:M0
 *              0 :0 :0     4.3 ��480*272  RGB��,ID = 0X4342
 *              0 :0 :1     7   ��800*480  RGB��,ID = 0X7084
 *              0 :1 :0     7   ��1024*600 RGB��,ID = 0X7016
 *              0 :1 :1     7   ��1280*800 RGB��,ID = 0X7018
 *              1 :0 :0     4.3 ��800*480  RGB��,ID = 0X4348
 *              1 :0 :1     10.1��1280*800 RGB��,ID = 0X1018
 * @param       ��
 * @retval      0, �Ƿ�; 
 *              ����, LCD ID
 */
uint16_t tli_panelid_read(void)
{
    uint8_t idx = 0;

    rcu_periph_clock_enable(RCU_GPIOB);     /* ʹ��GPIOBʱ�� */
    rcu_periph_clock_enable(RCU_GPIOG);     /* ʹ��GPIOGʱ�� */
    
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);   /* B7����ģʽ����,�������� */

    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);  /* G7����ģʽ����,�������� */  	
  
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);   /* R7����ģʽ����,�������� */  	  

    delay_us(10);
    idx  = gpio_input_bit_get(GPIOG, GPIO_PIN_6);       /* ��ȡM0 */
    idx |= gpio_input_bit_get(GPIOB, GPIO_PIN_15) << 1; /* ��ȡM1 */
    idx |= gpio_input_bit_get(GPIOB, GPIO_PIN_9) << 2;  /* ��ȡM2 */

    switch (idx)
    {
        case 0:
            return 0X4342;      /* 4.3����,480*272�ֱ��� */

        case 1:
            return 0X7084;      /* 7����,800*480�ֱ��� */

        case 2:
            return 0X7016;      /* 7����,1024*600�ֱ��� */

        case 3:
            return 0X7018;      /* 7����,1280*800�ֱ��� */

        case 4:
            return 0X4384;      /* 4.3����,800*480�ֱ��� */

        case 5:
            return 0X1018;      /* 10.1����,1280*800�ֱ��� */

        default:
            return 0;
    }
}

/**
 * @brief       ��ʼ��TLI
 * @param       ��
 * @retval      ��
 */
void tli_config(void)
{
    uint32_t temp;
    tli_parameter_struct               tli_init_struct;
    tli_layer_parameter_struct         tli_layer_init_struct;
  
    uint16_t lcdid = 0;

    lcdid = tli_panelid_read();                  /* ��ȡLCD���ID */
    rcu_periph_clock_enable(RCU_TLI);            /* ʹ��TLIʱ�� */     
    tli_gpio_config();                           /* ����TLI��IO�� */
  
#if RGB_80_8001280         
    lcdid = 0X8081;
#endif
  
    if (lcdid == 0X4342)
    {
        lcdtli.pwidth = 480;                     /* �����,��λ:���� */
        lcdtli.pheight = 272;                    /* ���߶�,��λ:���� */
        lcdtli.hsw = 1;                          /* ˮƽͬ����� */
        lcdtli.hbp = 40;                         /* ˮƽ���� */
        lcdtli.hfp = 5;                          /* ˮƽǰ�� */
        lcdtli.vsw = 1;                          /* ��ֱͬ����� */   
        lcdtli.vbp = 8;                          /* ��ֱ���� */
        lcdtli.vfp = 8;                          /* ��ֱǰ�� */   
        tli_clk_set(25, 288, 4, RCU_PLL2R_DIV8); /* ����TLIʱ��  9Mhz */
    }
    else if (lcdid == 0X7084)
    {
        lcdtli.pwidth = 800;                     /* �����,��λ:���� */
        lcdtli.pheight = 480;                    /* ���߶�,��λ:���� */
        lcdtli.hsw = 1;                          /* ˮƽͬ����� */
        lcdtli.hbp = 46;                         /* ˮƽ���� */
        lcdtli.hfp = 210;                        /* ˮƽǰ�� */
        lcdtli.vsw = 1;                          /* ��ֱͬ����� */  
        lcdtli.vbp = 23;                         /* ��ֱ���� */
        lcdtli.vfp = 22;                         /* ��ֱǰ�� */    
        tli_clk_set(25, 396, 3, RCU_PLL2R_DIV4); /* ����TLIʱ��  33Mhz(�����˫��,��Ҫ����TLIʱ�ӵ�:18.75Mhz,�Ż�ȽϺ�) */
    }
    else if (lcdid == 0X7016)
    {
        lcdtli.pwidth = 1024;                    /* �����,��λ:���� */
        lcdtli.pheight = 600;                    /* ���߶�,��λ:���� */
        lcdtli.hsw = 20;                         /* ˮƽͬ����� */
        lcdtli.hbp = 140;                        /* ˮƽ���� */
        lcdtli.hfp = 160;                        /* ˮƽǰ�� */
        lcdtli.vsw = 3;                          /* ��ֱͬ����� */  
        lcdtli.vbp = 20;                         /* ��ֱ���� */
        lcdtli.vfp = 12;                         /* ��ֱǰ�� */    
        tli_clk_set(25, 360, 2, RCU_PLL2R_DIV4); /* ����TLIʱ��  45Mhz */
    }
    else if (lcdid == 0X7018)
    {
        lcdtli.pwidth = 1280;                    /* �����,��λ:���� */
        lcdtli.pheight = 800;                    /* ���߶�,��λ:���� */
        /* ������������ */
    }
    else if (lcdid == 0X4384)
    {
        lcdtli.pwidth = 800;                     /* �����,��λ:���� */
        lcdtli.pheight = 480;                    /* ���߶�,��λ:���� */
        lcdtli.hsw = 48;                         /* ˮƽͬ����� */
        lcdtli.hbp = 88;                         /* ˮƽ���� */
        lcdtli.hfp = 40;                         /* ˮƽǰ�� */
        lcdtli.vsw = 3;                          /* ��ֱͬ����� */
        lcdtli.vbp = 32;                         /* ��ֱ���� */
        lcdtli.vfp = 13;                         /* ��ֱǰ�� */
        tli_clk_set(25, 396, 3, RCU_PLL2R_DIV4); /* ����TLIʱ��  33M */ 
    }
    else if (lcdid == 0X8081)
    {
        lcdtli.pwidth = 800;                     /* �����,��λ:���� */
        lcdtli.pheight = 1280;                   /* ���߶�,��λ:���� */      
        lcdtli.hsw = 10;                         /* ˮƽͬ����� */
        lcdtli.hbp = 60;                         /* ˮƽ���� */
        lcdtli.hfp = 10;                         /* ˮƽǰ�� */
        lcdtli.vsw = 10;                         /* ��ֱͬ����� */
        lcdtli.vbp = 10;                         /* ��ֱ���� */
        lcdtli.vfp = 10;                         /* ��ֱǰ�� */
        tli_clk_set(25, 300, 3, RCU_PLL2R_DIV2); /* ����TLIʱ��  50M */ 
    }
    else if (lcdid == 0X1018)                    /* 10.1��1280*800 RGB�� */
    {
        lcdtli.pwidth = 1280;                    /* �����,��λ:���� */
        lcdtli.pheight = 800;                    /* ���߶�,��λ:���� */
        lcdtli.hsw = 10;                         /* ˮƽͬ����� */
        lcdtli.hbp = 140;                        /* ˮƽ���� */
        lcdtli.hfp = 10;                         /* ˮƽǰ�� */
        lcdtli.vsw = 3;                          /* ��ֱͬ����� */ 
        lcdtli.vbp = 10;                         /* ��ֱ���� */
        lcdtli.vfp = 10;                         /* ��ֱǰ�� */
        tli_clk_set(25, 360, 2, RCU_PLL2R_DIV4); /* ����TLIʱ��  45Mhz */
    }

    lcddev.width = lcdtli.pwidth;      /* ����lcddev�Ŀ�Ȳ��� */
    lcddev.height = lcdtli.pheight;    /* ����lcddev�ĸ߶Ȳ��� */
    
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    lcdtli.pixsize = 4;       /* ÿ������ռ4���ֽ� */
    lcdtli.pixformat = 0X00;  /* ARGB8888��ʽ */
    temp = 4;
#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    lcdtli.pixsize = 3;       /* ÿ������3���ֽ� */
    lcdtli.pixformat = 0X01;  /* RGB888��ʽ */ 
    temp = 4;
#else
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    //g_tli_framebuf[1]=(uint32_t*)&tli_lcd_framebuf1;
    lcdtli.pixsize = 2;       /* ÿ������ռ2���ֽ� */
    lcdtli.pixformat = 0X02;  /* RGB565��ʽ */
    temp = 2;
#endif
    
    /* TLI initialization */
    tli_init_struct.signalpolarity_hs = TLI_HSYN_ACTLIVE_LOW;                   /* ˮƽͬ������͵�ƽ��Ч */
    tli_init_struct.signalpolarity_vs = TLI_VSYN_ACTLIVE_LOW;                   /* ��ֱͬ������͵�ƽ��Ч */
    tli_init_struct.signalpolarity_de = TLI_DE_ACTLIVE_LOW;                     /* ����ʹ�ܵ͵�ƽ��Ч  */
    
    if (lcdid == 0X1018)
    {
        tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_INVERTEDTLI;   /* ����ʱ����TLIʱ�ӷ�ת */
    }
    else
    {
        tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_TLI;           /* ����ʱ����TLIʱ��  */
    }
    
    /* ����LCDʱ�� */
    tli_init_struct.synpsz_hpsz = lcdtli.hsw - 1;                                               /* ˮƽͬ�������� */
    tli_init_struct.synpsz_vpsz = lcdtli.vsw - 1;                                               /* ��ֱͬ�������� */
    tli_init_struct.backpsz_hbpsz = lcdtli.hsw + lcdtli.hbp - 1;                                /* ˮƽ���ؼ�ͬ������Ŀ�� */
    tli_init_struct.backpsz_vbpsz = lcdtli.vsw + lcdtli.vbp - 1;                                /* ��ֱ���ؼ�ͬ������Ŀ�� */
    tli_init_struct.activesz_hasz = lcdtli.hsw + lcdtli.hbp + lcdtli.pwidth - 1;                /* ˮƽ��Ч��ȼ�ˮƽ�������غ�ˮƽͬ�����ؿ�� */
    tli_init_struct.activesz_vasz = lcdtli.vsw + lcdtli.vbp + lcdtli.pheight - 1;               /* ��ֱ��Ч��ȼӴ�ֱ�������غʹ�ֱͬ�����ؿ�� */
    tli_init_struct.totalsz_htsz = lcdtli.hsw + lcdtli.hbp + lcdtli.pwidth + lcdtli.hfp - 1;    /* ˮƽ�ܿ��,������Ч���,����,ͬ�������ǰ�� */
    tli_init_struct.totalsz_vtsz = lcdtli.vsw + lcdtli.vbp + lcdtli.pheight + lcdtli.vfp - 1;   /* ��ֱ�ܿ��,������Ч���,����,ͬ�������ǰ�� */
    /* ���ñ���ɫ */
    tli_init_struct.backcolor_red = 0xFF;                                                       /* ����ɫ��ɫֵ */
    tli_init_struct.backcolor_green = 0xFF;                                                     /* ����ɫ��ɫֵ */
    tli_init_struct.backcolor_blue = 0xFF;                                                      /* ����ɫ��ɫֵ */
    tli_init(&tli_init_struct);                                                                 /* ��ʼ��TLI */
  
    /* ����TLI layer0 */
    tli_layer_init_struct.layer_window_leftpos = lcdtli.hsw + lcdtli.hbp;                          /* �������λ�� */            
    tli_layer_init_struct.layer_window_rightpos = (lcdtli.pwidth + lcdtli.hsw + lcdtli.hbp - 1);   /* �����Ҳ�λ�� */
    tli_layer_init_struct.layer_window_toppos = lcdtli.vsw + lcdtli.vbp;                           /* ���ڶ���λ�� */
    tli_layer_init_struct.layer_window_bottompos = lcdtli.pheight + lcdtli.vsw + lcdtli.vbp - 1;   /* ���ڵײ�λ�� */
    tli_layer_init_struct.layer_ppf = TLI_PIXFORMAT;                                               /* �����ظ�ʽ */
    tli_layer_init_struct.layer_sa = 255;                                                          /* ��㶨Alphaֵ, 0,ȫ͸��;255,��͸�� */
    tli_layer_init_struct.layer_acf1 = LAYER_ACF1_PASA;                                            /* Alpha��ϼ�������1 */
    tli_layer_init_struct.layer_acf2 = LAYER_ACF2_PASA;                                            /* Alpha��ϼ�������2 */
    tli_layer_init_struct.layer_default_alpha = 0;                                                 /* Ĭ����ɫAlphaֵ */
    tli_layer_init_struct.layer_default_blue = 0;                                                  /* Ĭ����ɫֵ */
    tli_layer_init_struct.layer_default_green = 0;                                                 /* Ĭ����ɫֵ */
    tli_layer_init_struct.layer_default_red = 0;                                                   /* Ĭ�Ϻ�ɫֵ */
    tli_layer_init_struct.layer_frame_bufaddr = (uint32_t)g_tli_framebuf[0];                       /* ֡��������ʼ��ַ */
    tli_layer_init_struct.layer_frame_buf_stride_offset = (lcdtli.pwidth * temp);                  /* ֡����������ƫ�� */
    tli_layer_init_struct.layer_frame_line_length = ((lcdtli.pwidth * temp) + 3);                  /* ֡�г��� */
    tli_layer_init_struct.layer_frame_total_line_number = lcdtli.pheight;                          /* ֡������ */
    tli_layer_init(LAYER0, &tli_layer_init_struct);                                                /* ��ʼ��TLI�� */                                 

    tli_layer_enable(LAYER0);                     /* ʹ��TLI LAYER0�� */                                                                   
    tli_reload_config(TLI_REQUEST_RELOAD_EN);     /* �������������� */
    tli_enable();                                 /* ʹ��TLI */             
    
    tli_display_dir(1);                /* Ĭ�ϙM����Ҳ������lcd_init������������ */
    tli_select_layer(0);               /* ѡ���1�� */
    
    /* TLI LCD��λ */
	  TLI_RST(1);
	  delay_ms(10);
	  TLI_RST(0);
	  delay_ms(50);
	  TLI_RST(1); 
		delay_ms(200); 
  
    TLI_BL(1);                         /* �������� */
    tli_clear(0XFFFFFFFF);             /* ���� */
}

/**
 * @brief       TLI���IO�ڳ�ʼ����ʱ��ʹ��
 * @param       ��
 * @retval      ��
 */
void tli_gpio_config(void)
{
    /* TLI�źſ������� TLI_DE(PF10), TLI_VSYNC(PA4), TLI_HSYNC(PC6), TLI_PCLK(PG7) */
    /* TLI ��������    TLI_R7(PG6), TLI_R6(PB1), TLI_R5(PC0), TLI_R4(PA5), TLI_R3(PB0), TLI_R2(PA1), TLI_R1(PA2), TLI_R0(PG13),
                       TLI_G7(PB15), TLI_G6(PC7), TLI_G5(PB11), TLI_G4(PB10), TLI_G3(PG10), TLI_G2(PA6), TLI_G1(PE6), TLI_G0(PE5),
                       TLI_B7(PB9), TLI_B6(PA15), TLI_B5(PB5), TLI_B4(PG12), TLI_B3(PA8), TLI_B2(PD6), TLI_B1(PA10 �����ã�Ĭ������), TLI_B0(PE4) */
  
    /* ������TLI�źſ������� DE/VSYNC/HSYNC/PCLK�ȵ����� */
    rcu_periph_clock_enable(TLI_DE_GPIO_CLK);       /* ʹ��TLI_DE����ʱ�� */
    rcu_periph_clock_enable(TLI_VSYNC_GPIO_CLK);    /* ʹ��TLI_VSYNC����ʱ�� */
    rcu_periph_clock_enable(TLI_HSYNC_GPIO_CLK);    /* ʹ��TLI_HSYNC����ʱ�� */
    rcu_periph_clock_enable(TLI_PCLK_GPIO_CLK);     /* ʹ��TLI_PCLK����ʱ�� */
    rcu_periph_clock_enable(TLI_BL_GPIO_CLK);       /* ʹ��TLI_BL����ʱ�� */
    rcu_periph_clock_enable(TLI_RST_GPIO_CLK);      /* ʹ��TLI_RST����ʱ�� */

    /* ����TLI_BL���� ������� */
    gpio_mode_set(TLI_BL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TLI_BL_GPIO_PIN);
    gpio_output_options_set(TLI_BL_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, TLI_BL_GPIO_PIN);

    /* ����TLI_RST���� ������� */
    gpio_mode_set(TLI_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TLI_RST_GPIO_PIN);
    gpio_output_options_set(TLI_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, TLI_RST_GPIO_PIN);

    /* ����TLI_DE����  ����������� */
    gpio_af_set(TLI_DE_GPIO_PORT, TLI_DE_GPIO_AF, TLI_DE_GPIO_PIN);
    gpio_mode_set(TLI_DE_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_DE_GPIO_PIN);
    gpio_output_options_set(TLI_DE_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_DE_GPIO_PIN);
  
    /* ����TLI_VSYNC����  ����������� */
    gpio_af_set(TLI_VSYNC_GPIO_PORT, TLI_VSYNC_GPIO_AF, TLI_VSYNC_GPIO_PIN);
    gpio_mode_set(TLI_VSYNC_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_VSYNC_GPIO_PIN);
    gpio_output_options_set(TLI_VSYNC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_VSYNC_GPIO_PIN); 

    /* ����TLI_HSYNC����  ����������� */
    gpio_af_set(TLI_HSYNC_GPIO_PORT, TLI_HSYNC_GPIO_AF, TLI_HSYNC_GPIO_PIN);
    gpio_mode_set(TLI_HSYNC_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_HSYNC_GPIO_PIN);
    gpio_output_options_set(TLI_HSYNC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_HSYNC_GPIO_PIN);
  
    /* ����TLI_PCLK����  ����������� */
    gpio_af_set(TLI_PCLK_GPIO_PORT, TLI_PCLK_GPIO_AF, TLI_PCLK_GPIO_PIN);
    gpio_mode_set(TLI_PCLK_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_PCLK_GPIO_PIN);
    gpio_output_options_set(TLI_PCLK_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_PCLK_GPIO_PIN); 
   
    /* ������TLI �������ŵ����� */   
    rcu_periph_clock_enable(RCU_GPIOA);     /* ʹ��GPIOAʱ�� */
    rcu_periph_clock_enable(RCU_GPIOB);     /* ʹ��GPIOBʱ�� */
    rcu_periph_clock_enable(RCU_GPIOC);     /* ʹ��GPIOCʱ�� */
    rcu_periph_clock_enable(RCU_GPIOD);     /* ʹ��GPIODʱ�� */
    rcu_periph_clock_enable(RCU_GPIOE);     /* ʹ��GPIOEʱ�� */
    rcu_periph_clock_enable(RCU_GPIOG);     /* ʹ��GPIOGʱ�� */
 
    /* ����TLI�������ŵĸ��ù��� */
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_5);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_6);
    gpio_af_set(GPIOA, GPIO_AF_13, GPIO_PIN_8);
    gpio_af_set(GPIOA, GPIO_AF_14, GPIO_PIN_15);
  
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_1);  
    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_9);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_10);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_14, GPIO_PIN_15);

    gpio_af_set(GPIOC, GPIO_AF_14, GPIO_PIN_0);
    gpio_af_set(GPIOC, GPIO_AF_14, GPIO_PIN_7);

    gpio_af_set(GPIOD, GPIO_AF_14, GPIO_PIN_6);
    
    gpio_af_set(GPIOE, GPIO_AF_14, GPIO_PIN_4);
    gpio_af_set(GPIOE, GPIO_AF_14, GPIO_PIN_5);
    gpio_af_set(GPIOE, GPIO_AF_14, GPIO_PIN_6);
        
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_6);
    gpio_af_set(GPIOG, GPIO_AF_9, GPIO_PIN_10);
    gpio_af_set(GPIOG, GPIO_AF_9, GPIO_PIN_12);    
    gpio_af_set(GPIOG, GPIO_AF_14, GPIO_PIN_13);   
        
    /* ����PA1/2/5/6/8/15 ����������� */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_15);
    
    /* ����PB0/1/5/9/10/11/15 ����������� */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15);
    
    /* ����PC0/7 ����������� */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_0 | GPIO_PIN_7);   

    /* ����PD6 ����������� */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6); 
    
    /* ����PE4/5/6 ����������� */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6); 
        
    /* ����PG6/10/12/13 ����������� */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,  GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13);    
}






















