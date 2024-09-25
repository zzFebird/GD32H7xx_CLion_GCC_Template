/**
 ****************************************************************************************************
 * @file        tli.c
 * @version     V1.0
 * @brief       TLI 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "./BSP/LCD/lcd.h"
#include "./BSP/LCD/tli.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"


#if !(__ARMCC_VERSION >= 6010050)   /* 不是AC6编译器，即使用AC5编译器时 */

/* 根据不同的颜色格式,定义帧缓存数组 */
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    uint32_t tli_lcd_framebuf[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR)));     /* 定义最大屏分辨率时,LCD所需的帧缓存数组大小 */
#else
    uint16_t tli_lcd_framebuf[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR)));     /* 定义最大屏分辨率时,LCD所需的帧缓存数组大小 */
    //uint16_t tli_lcd_framebuf1[1280][800] __attribute__((at(TLI_FRAME_BUF_ADDR + 1280 * 800 * 2)));  /* 使能TLI层2时使用(默认只使用TLI层1) */
#endif

#else   /* 使用AC6编译器时 */

/* 根据不同的颜色格式,定义帧缓存数组 */
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    uint32_t tli_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));     /* 定义最大屏分辨率时,LCD所需的帧缓存数组大小 */
#else
    uint16_t tli_lcd_framebuf[1280][800] __attribute__((section(".bss.ARM.__at_0XC0000000")));     /* 定义最大屏分辨率时,LCD所需的帧缓存数组大小 */
#endif

#endif


uint32_t *g_tli_framebuf[2];       /* TLI LCD帧缓存数组指针,必须指向对应大小的内存区域 */
_tli_dev lcdtli;                   /* 管理LCD TLI的重要参数 */


/**
 * @brief       TLI选择层
 * @param       layerx      : 0,第一层; 1,第二层;
 * @retval      无
 */
void tli_select_layer(uint8_t layerx)
{
    lcdtli.activelayer = layerx;
}

/**
 * @brief       TLI显示方向设置
 * @param       dir         : 0,竖屏; 1,横屏;
 * @retval      无
 */
void tli_display_dir(uint8_t dir)
{
    lcdtli.dir = dir;       /* 显示方向 */

    if (dir == 0)           /* 竖屏 */
    {
        lcdtli.width = lcdtli.pheight;
        lcdtli.height = lcdtli.pwidth;
    }
    else if (dir == 1)      /* 横屏 */
    {
        lcdtli.width = lcdtli.pwidth;
        lcdtli.height = lcdtli.pheight;
    }
}

/**
 * @brief       TLI画点函数
 * @param       x,y         : 坐标
 * @param       color       : 颜色值
 * @retval      无
 */
void tli_draw_point(uint16_t x, uint16_t y, uint32_t color)
{
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888

    if (lcdtli.dir)     /* 横屏 */
    {
        *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
    }
    else                /* 竖屏 */
    {
        *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
    }

#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888

    if (lcdtli.dir)     /* 横屏 */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
        *(uint8_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x) + 2) = color >> 16;
    }
    else                /* 竖屏 */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
        *(uint8_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y) + 2) = color >> 16;
    }
    
#else

    if (lcdtli.dir)     /* 横屏 */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) = color;
    }
    else                /* 竖屏 */
    {
        *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) = color;
    }

#endif
}

/**
 * @brief       TLI读点函数
 * @param       x,y         : 坐标
 * @retval      颜色值
 */
uint32_t tli_read_point(uint16_t x, uint16_t y)
{
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888

    if (lcdtli.dir)     /* 横屏 */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x));
    }
    else                /* 竖屏 */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y));
    }

#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888

    if (lcdtli.dir)     /* 横屏 */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x)) & 0XFFFFFF;
    }
    else                /* 竖屏 */
    {
        return *(uint32_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y)) & 0XFFFFFF;
    }
        
#else

    if (lcdtli.dir)     /* 横屏 */
    {
        return *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * y + x));
    }
    else                /* 竖屏 */
    {
        return *(uint16_t *)((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * (lcdtli.pheight - x - 1) + y));
    }

#endif
}

/**
 * @brief       TLI填充矩形, 使用IPA填充
 * @note       (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 *              注意:sx,ex,不能大于lcddev.width - 1; sy,ey,不能大于lcddev.height - 1
 * @param       sx,sy       : 起始坐标
 * @param       ex,ey       : 结束坐标
 * @param       color       : 填充的颜色
 * @retval      无
 */
void tli_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color)
{
    ipa_destination_parameter_struct  ipa_destination_init_struct;
  
    uint32_t psx, psy, pex, pey;    /* 以LCD面板为基准的坐标系,不随横竖屏变化而变化 */
    uint32_t timeout = 0;
    uint16_t offline;
    uint32_t addr;

    /* 坐标系转换 */
    if (lcdtli.dir)     /* 横屏 */
    {
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                /* 竖屏 */
    {
        if(ex >= lcdtli.pheight)ex = lcdtli.pheight - 1;  /* 限制范围 */
        if(sx >= lcdtli.pheight)sx = lcdtli.pheight - 1;  /* 限制范围 */
        
        psx = sy;
        psy = lcdtli.pheight - ex - 1;
        pex = ey;
        pey = lcdtli.pheight - sx - 1;
    }

    offline = lcdtli.pwidth - (pex - psx + 1);     /* 行偏移:当前行最后一个像素和下一行第一个像素之间的像素数目 */
    addr = ((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * psy + psx));
    
    rcu_periph_clock_enable(RCU_IPA);     /* 使能IPA时钟 */
    
    ipa_deinit();                                                                        /* 重新初始化IPA */                                         

    ipa_pixel_format_convert_mode_set(IPA_FILL_UP_DE);                                   /* 像素格式转换模式:用特定的颜色填充目标存储区 */

    /* 配置IPA目标层 */
    ipa_destination_struct_para_init(&ipa_destination_init_struct);                      /* 用默认值初始化IPA目标参数结构体 */ 

    ipa_destination_init_struct.destination_pf = TLI_PIXFORMAT;                          /* 目标存储区像素格式 */
    ipa_destination_init_struct.destination_memaddr = (uint32_t)addr;                    /* 目标存储区基地址 */

#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888 || TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    
    ipa_destination_init_struct.destination_preblue = color & 0XFF;                      /* 目标层预定义蓝色值 */
    ipa_destination_init_struct.destination_pregreen = (color & 0XFF00) >> 8;            /* 目标层预定义绿色值 */
    ipa_destination_init_struct.destination_prered = (color & 0XFF0000) >> 16;           /* 目标层预定义红色值 */
    
#else

    ipa_destination_init_struct.destination_preblue = color & 0X1F;                      /* 目标层预定义蓝色值 */
    ipa_destination_init_struct.destination_pregreen = (color & 0X07E0) >> 5;            /* 目标层预定义绿色值 */
    ipa_destination_init_struct.destination_prered = (color & 0XF800) >> 11;             /* 目标层预定义红色值 */
    
#endif   

    ipa_destination_init_struct.destination_prealpha = (color & 0XFF000000) >> 24;       /* 目标层预定义alpha通道值 */
    ipa_destination_init_struct.destination_lineoff = offline;                           /* 目标存储区行偏移 */
    ipa_destination_init_struct.image_height = pey - psy + 1;                            /* 图像高度 */
    ipa_destination_init_struct.image_width = pex - psx + 1;                             /* 图像宽度 */
    ipa_destination_init_struct.image_rotate = DESTINATION_ROTATE_0;                     /* 图像旋转角度 */    
    ipa_destination_init_struct.image_hor_decimation = DESTINATION_HORDECIMATE_DISABLE;  /* 图像水平预抽取滤波器控制 */
    ipa_destination_init_struct.image_ver_decimation = DESTINATION_VERDECIMATE_DISABLE;  /* 图像垂直预抽取滤波器控制 */    
    ipa_destination_init(&ipa_destination_init_struct);                                  /* 初始化目标存储区参数 */

    ipa_transfer_enable();                                                               /* 使能IPA传输 */
    
    while(ipa_interrupt_flag_get(IPA_INT_FLAG_FTF) == RESET)                             /* 等待传输完成 */
    {
        timeout++;

        if (timeout > 0X1FFFFF)break;                                                    /* 超时退出 */
    }
    
    ipa_interrupt_flag_clear(IPA_INT_FLAG_FTF);                                          /* 清除传输完成中断标志 */
}

/**
 * @brief       在指定区域内填充指定颜色块, 使用IPA填充
 *              (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 *              注意:sx,ex,不能大于lcddev.width - 1; sy,ey,不能大于lcddev.height - 1
 * @param       sx,sy       : 起始坐标
 * @param       ex,ey       : 结束坐标
 * @param       color       : 填充的颜色数组首地址
 * @retval      无
 */
void tli_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    ipa_destination_parameter_struct  ipa_destination_init_struct;
    ipa_foreground_parameter_struct   ipa_fg_init_struct;
  
    uint32_t psx, psy, pex, pey;    /* 以LCD面板为基准的坐标系,不随横竖屏变化而变化 */
    uint32_t timeout = 0;
    uint16_t offline;
    uint32_t addr;

    /* 坐标系转换 */
    if (lcdtli.dir)     /* 横屏 */
    { 
        psx = sx;
        psy = sy;
        pex = ex;
        pey = ey;
    }
    else                /* 竖屏 */
    {
        psx = sy;
        psy = lcdtli.pheight - ex - 1;
        pex = ey;
        pey = lcdtli.pheight - sx - 1;
    }

    offline = lcdtli.pwidth - (pex - psx + 1);     /* 行偏移:当前行最后一个像素和下一行第一个像素之间的像素数目 */
    addr = ((uint32_t)g_tli_framebuf[lcdtli.activelayer] + lcdtli.pixsize * (lcdtli.pwidth * psy + psx));

    rcu_periph_clock_enable(RCU_IPA);     /* 使能IPA时钟 */
    
    ipa_deinit();                                                          /* 重新初始化IPA */

    ipa_pixel_format_convert_mode_set(IPA_FGTODE);                         /* 像素格式转换模式:前景层存储区到目标存储区无像素格式转换 */

    /* 配置IPA目标层 */
    ipa_destination_struct_para_init(&ipa_destination_init_struct);        /* 用默认值初始化IPA目标参数结构体 */ 
    
    ipa_destination_init_struct.destination_pf = TLI_PIXFORMAT;            /* 目标存储区像素格式 */
    ipa_destination_init_struct.destination_memaddr = (uint32_t)addr;      /* 目标存储区基地址 */
    ipa_destination_init_struct.destination_preblue = 0;                   /* 目标层预定义蓝色值 */
    ipa_destination_init_struct.destination_pregreen = 0;                  /* 目标层预定义绿色值 */    
    ipa_destination_init_struct.destination_prered = 0;                    /* 目标层预定义红色值 */
    ipa_destination_init_struct.destination_prealpha = 0;                  /* 目标层预定义alpha通道值 */
    ipa_destination_init_struct.destination_lineoff = offline;             /* 目标存储区行偏移 */
    ipa_destination_init_struct.image_height = pey - psy + 1;              /* 图像高度 */
    ipa_destination_init_struct.image_width = pex - psx + 1;               /* 图像宽度 */ 
    ipa_destination_init_struct.image_rotate = DESTINATION_ROTATE_0;       /* 图像旋转角度 */
    ipa_destination_init_struct.image_hor_decimation = DESTINATION_HORDECIMATE_DISABLE;   /* 图像水平预抽取滤波器控制 */
    ipa_destination_init_struct.image_ver_decimation = DESTINATION_VERDECIMATE_DISABLE;   /* 图像垂直预抽取滤波器控制 */    
    ipa_destination_init(&ipa_destination_init_struct);                    /* 初始化目标存储区参数 */

    /* 配置IPA前景层 */
    ipa_foreground_struct_para_init(&ipa_fg_init_struct);                  /* 用默认值初始化IPA前景层参数结构体 */

    ipa_fg_init_struct.foreground_memaddr = (uint32_t)color;               /* 前景层存储区基地址 */
    ipa_fg_init_struct.foreground_pf = TLI_PIXFORMAT;                      /* 前景层像素格式 */
    ipa_fg_init_struct.foreground_alpha_algorithm = IPA_FG_ALPHA_MODE_0;   /* 前景层alpha值计算算法 */
    ipa_fg_init_struct.foreground_prealpha = 0x00;                         /* 前景层预定义alpha通道值 */
    ipa_fg_init_struct.foreground_lineoff = 0x00;                          /* 前景层行偏移 */
    ipa_fg_init_struct.foreground_preblue = 0x00;                          /* 前景层预定义蓝色值 */
    ipa_fg_init_struct.foreground_pregreen = 0x00;                         /* 前景层预定义绿色值 */
    ipa_fg_init_struct.foreground_prered = 0x00;                           /* 前景层预定义红色值 */
    ipa_fg_init_struct.foreground_efuv_memaddr = 0x00;                     /* 前景层偶数帧/UV存储区基地址 */
    ipa_fg_init_struct.foreground_interlace_mode = DISABLE;                /* 禁能前景层隔行输入模式 */
    ipa_foreground_init(&ipa_fg_init_struct);                              /* 初始化前景层参数 */

    ipa_transfer_enable();                                                 /* 使能IPA传输 */
    
    while(ipa_interrupt_flag_get(IPA_INT_FLAG_FTF) == RESET)               /* 等待传输完成 */
    {
        timeout++;

        if (timeout > 0X1FFFFF)break;                                      /* 超时退出 */
    }
    
    ipa_interrupt_flag_clear(IPA_INT_FLAG_FTF);                            /* 清除传输完成中断标志 */
}

/**
 * @brief       TLI清屏
 * @param       color       : 颜色值
 * @retval      无
 */
void tli_clear(uint32_t color)
{
    tli_fill(0, 0, lcdtli.width - 1, lcdtli.height - 1, color);    
}

/**
 * @brief       TLI时钟(Fdclk)设置函数
 * @param       pll2_psc      : PLL2 VCO 源时钟分频器,   取值范围:1~63.
 * @param       pll2_n        : PLL2 VCO 时钟倍频因子,   取值范围:9~512
 * @param       pll2_r        : PLL2R 输出频率的分频系数（PLL2 VCO 时钟作为输入）,   取值范围:1~128
 * @param       pll2_r_div    : PLL2R 时钟的分频因子（用于生成 TLI 模块的时钟源）,   取值范围:0~3(RCU_PLL2R_DIVx(x=2,4,8,16),对应PLL2R的x分频 )
 *
 * @note        CK_PLL2VCOSRC = CK_PLL2SRC / pll2_psc;
 *              CK_PLL2VCO = CK_PLL2VCOSRC * pll2_n;
 *              CK_TLI = CK_PLL2VCO / pll2_r / pll2_r_div = CK_PLL2SRC / pll2_psc * pll2_n / pll2_r / pll2_r_div;
 *              其中:
 *              CK_PLL2SRC:PLL2 源时钟(系统初始化的时候选择CK_HXTAL时钟作为PLL、PLL1、PLL2源时钟)
 *              CK_PLL2VCOSRC:PLL2 VCO 源时钟
 *              CK_PLL2VCO:PLL2 VCO 输出时钟
 *              假设:外部晶振(HXTAL_VALUE)为25M, pll2_psc = 25的时候, CK_PLL2VCOSRC = 1Mhz.
 *              例如:要得到33M的TLI时钟, 则可以设置: pll2_n = 396, pll2_r = 3, pll2_r_div = 1(RCU_PLL2R_DIV4)
 *              CK_TLI = 25 / 25 * 396 / 3 / 4 = 396 / 12 = 33Mhz
 * @retval      0, 成功;
 *              其他, 失败;
 */
uint8_t tli_clk_set(uint32_t pll2_psc, uint32_t pll2_n, uint32_t pll2_r, uint32_t pll2_r_div)
{
    uint8_t status = 0;

    rcu_pll_input_output_clock_range_config(IDX_PLL2, RCU_PLL2RNG_1M_2M, RCU_PLL2VCO_192M_836M); /* 配置PLL2输入/输出时钟范围  */
  
    if (ERROR == rcu_pll2_config(pll2_psc, pll2_n, 3, 3, pll2_r))  /* 配置PLL2时钟 */
    {
        status = 1;                                                /* 函数参数错误 */ 
    }

    rcu_pll_clock_output_enable(RCU_PLL2R);                        /* 使能PLL2R时钟输出 */
    
    rcu_tli_clock_div_config(pll2_r_div);                          /* 配置从PLL2R时钟分频的TLI分频系数 */

    rcu_osci_on(RCU_PLL2_CK);                                      /* 打开PLL2时钟 */

    if (ERROR == rcu_osci_stab_wait(RCU_PLL2_CK))                  /* 等待PLL2时钟锁定 */ 
    {
        status = 2;
    }    
    
    return status;
}

/**
 * @brief       TLI读取面板ID
 * @note        利用LCD RGB线的最高位(R7,G7,B7)来识别面板ID
 *              PG6 = R7(M0); PB15 = G7(M1); PB9 = B7(M2);
 *              M2:M1:M0
 *              0 :0 :0     4.3 寸480*272  RGB屏,ID = 0X4342
 *              0 :0 :1     7   寸800*480  RGB屏,ID = 0X7084
 *              0 :1 :0     7   寸1024*600 RGB屏,ID = 0X7016
 *              0 :1 :1     7   寸1280*800 RGB屏,ID = 0X7018
 *              1 :0 :0     4.3 寸800*480  RGB屏,ID = 0X4348
 *              1 :0 :1     10.1寸1280*800 RGB屏,ID = 0X1018
 * @param       无
 * @retval      0, 非法; 
 *              其他, LCD ID
 */
uint16_t tli_panelid_read(void)
{
    uint8_t idx = 0;

    rcu_periph_clock_enable(RCU_GPIOB);     /* 使能GPIOB时钟 */
    rcu_periph_clock_enable(RCU_GPIOG);     /* 使能GPIOG时钟 */
    
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);   /* B7引脚模式设置,上拉输入 */

    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);  /* G7引脚模式设置,上拉输入 */  	
  
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);   /* R7引脚模式设置,上拉输入 */  	  

    delay_us(10);
    idx  = gpio_input_bit_get(GPIOG, GPIO_PIN_6);       /* 读取M0 */
    idx |= gpio_input_bit_get(GPIOB, GPIO_PIN_15) << 1; /* 读取M1 */
    idx |= gpio_input_bit_get(GPIOB, GPIO_PIN_9) << 2;  /* 读取M2 */

    switch (idx)
    {
        case 0:
            return 0X4342;      /* 4.3寸屏,480*272分辨率 */

        case 1:
            return 0X7084;      /* 7寸屏,800*480分辨率 */

        case 2:
            return 0X7016;      /* 7寸屏,1024*600分辨率 */

        case 3:
            return 0X7018;      /* 7寸屏,1280*800分辨率 */

        case 4:
            return 0X4384;      /* 4.3寸屏,800*480分辨率 */

        case 5:
            return 0X1018;      /* 10.1寸屏,1280*800分辨率 */

        default:
            return 0;
    }
}

/**
 * @brief       初始化TLI
 * @param       无
 * @retval      无
 */
void tli_config(void)
{
    uint32_t temp;
    tli_parameter_struct               tli_init_struct;
    tli_layer_parameter_struct         tli_layer_init_struct;
  
    uint16_t lcdid = 0;

    lcdid = tli_panelid_read();                  /* 读取LCD面板ID */
    rcu_periph_clock_enable(RCU_TLI);            /* 使能TLI时钟 */     
    tli_gpio_config();                           /* 配置TLI的IO口 */
  
#if RGB_80_8001280         
    lcdid = 0X8081;
#endif
  
    if (lcdid == 0X4342)
    {
        lcdtli.pwidth = 480;                     /* 面板宽度,单位:像素 */
        lcdtli.pheight = 272;                    /* 面板高度,单位:像素 */
        lcdtli.hsw = 1;                          /* 水平同步宽度 */
        lcdtli.hbp = 40;                         /* 水平后廊 */
        lcdtli.hfp = 5;                          /* 水平前廊 */
        lcdtli.vsw = 1;                          /* 垂直同步宽度 */   
        lcdtli.vbp = 8;                          /* 垂直后廊 */
        lcdtli.vfp = 8;                          /* 垂直前廊 */   
        tli_clk_set(25, 288, 4, RCU_PLL2R_DIV8); /* 设置TLI时钟  9Mhz */
    }
    else if (lcdid == 0X7084)
    {
        lcdtli.pwidth = 800;                     /* 面板宽度,单位:像素 */
        lcdtli.pheight = 480;                    /* 面板高度,单位:像素 */
        lcdtli.hsw = 1;                          /* 水平同步宽度 */
        lcdtli.hbp = 46;                         /* 水平后廊 */
        lcdtli.hfp = 210;                        /* 水平前廊 */
        lcdtli.vsw = 1;                          /* 垂直同步宽度 */  
        lcdtli.vbp = 23;                         /* 垂直后廊 */
        lcdtli.vfp = 22;                         /* 垂直前廊 */    
        tli_clk_set(25, 396, 3, RCU_PLL2R_DIV4); /* 设置TLI时钟  33Mhz(如果开双显,需要降低TLI时钟到:18.75Mhz,才会比较好) */
    }
    else if (lcdid == 0X7016)
    {
        lcdtli.pwidth = 1024;                    /* 面板宽度,单位:像素 */
        lcdtli.pheight = 600;                    /* 面板高度,单位:像素 */
        lcdtli.hsw = 20;                         /* 水平同步宽度 */
        lcdtli.hbp = 140;                        /* 水平后廊 */
        lcdtli.hfp = 160;                        /* 水平前廊 */
        lcdtli.vsw = 3;                          /* 垂直同步宽度 */  
        lcdtli.vbp = 20;                         /* 垂直后廊 */
        lcdtli.vfp = 12;                         /* 垂直前廊 */    
        tli_clk_set(25, 360, 2, RCU_PLL2R_DIV4); /* 设置TLI时钟  45Mhz */
    }
    else if (lcdid == 0X7018)
    {
        lcdtli.pwidth = 1280;                    /* 面板宽度,单位:像素 */
        lcdtli.pheight = 800;                    /* 面板高度,单位:像素 */
        /* 其他参数待定 */
    }
    else if (lcdid == 0X4384)
    {
        lcdtli.pwidth = 800;                     /* 面板宽度,单位:像素 */
        lcdtli.pheight = 480;                    /* 面板高度,单位:像素 */
        lcdtli.hsw = 48;                         /* 水平同步宽度 */
        lcdtli.hbp = 88;                         /* 水平后廊 */
        lcdtli.hfp = 40;                         /* 水平前廊 */
        lcdtli.vsw = 3;                          /* 垂直同步宽度 */
        lcdtli.vbp = 32;                         /* 垂直后廊 */
        lcdtli.vfp = 13;                         /* 垂直前廊 */
        tli_clk_set(25, 396, 3, RCU_PLL2R_DIV4); /* 设置TLI时钟  33M */ 
    }
    else if (lcdid == 0X8081)
    {
        lcdtli.pwidth = 800;                     /* 面板宽度,单位:像素 */
        lcdtli.pheight = 1280;                   /* 面板高度,单位:像素 */      
        lcdtli.hsw = 10;                         /* 水平同步宽度 */
        lcdtli.hbp = 60;                         /* 水平后廊 */
        lcdtli.hfp = 10;                         /* 水平前廊 */
        lcdtli.vsw = 10;                         /* 垂直同步宽度 */
        lcdtli.vbp = 10;                         /* 垂直后廊 */
        lcdtli.vfp = 10;                         /* 垂直前廊 */
        tli_clk_set(25, 300, 3, RCU_PLL2R_DIV2); /* 设置TLI时钟  50M */ 
    }
    else if (lcdid == 0X1018)                    /* 10.1寸1280*800 RGB屏 */
    {
        lcdtli.pwidth = 1280;                    /* 面板宽度,单位:像素 */
        lcdtli.pheight = 800;                    /* 面板高度,单位:像素 */
        lcdtli.hsw = 10;                         /* 水平同步宽度 */
        lcdtli.hbp = 140;                        /* 水平后廊 */
        lcdtli.hfp = 10;                         /* 水平前廊 */
        lcdtli.vsw = 3;                          /* 垂直同步宽度 */ 
        lcdtli.vbp = 10;                         /* 垂直后廊 */
        lcdtli.vfp = 10;                         /* 垂直前廊 */
        tli_clk_set(25, 360, 2, RCU_PLL2R_DIV4); /* 设置TLI时钟  45Mhz */
    }

    lcddev.width = lcdtli.pwidth;      /* 设置lcddev的宽度参数 */
    lcddev.height = lcdtli.pheight;    /* 设置lcddev的高度参数 */
    
#if TLI_PIXFORMAT == TLI_PIXFORMAT_ARGB8888
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    lcdtli.pixsize = 4;       /* 每个像素占4个字节 */
    lcdtli.pixformat = 0X00;  /* ARGB8888格式 */
    temp = 4;
#elif TLI_PIXFORMAT == TLI_PIXFORMAT_RGB888
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    lcdtli.pixsize = 3;       /* 每个像素3个字节 */
    lcdtli.pixformat = 0X01;  /* RGB888格式 */ 
    temp = 4;
#else
    g_tli_framebuf[0] = (uint32_t *)&tli_lcd_framebuf;
    //g_tli_framebuf[1]=(uint32_t*)&tli_lcd_framebuf1;
    lcdtli.pixsize = 2;       /* 每个像素占2个字节 */
    lcdtli.pixformat = 0X02;  /* RGB565格式 */
    temp = 2;
#endif
    
    /* TLI initialization */
    tli_init_struct.signalpolarity_hs = TLI_HSYN_ACTLIVE_LOW;                   /* 水平同步脉冲低电平有效 */
    tli_init_struct.signalpolarity_vs = TLI_VSYN_ACTLIVE_LOW;                   /* 垂直同步脉冲低电平有效 */
    tli_init_struct.signalpolarity_de = TLI_DE_ACTLIVE_LOW;                     /* 数据使能低电平有效  */
    
    if (lcdid == 0X1018)
    {
        tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_INVERTEDTLI;   /* 像素时钟是TLI时钟翻转 */
    }
    else
    {
        tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_TLI;           /* 像素时钟是TLI时钟  */
    }
    
    /* 配置LCD时序 */
    tli_init_struct.synpsz_hpsz = lcdtli.hsw - 1;                                               /* 水平同步脉冲宽度 */
    tli_init_struct.synpsz_vpsz = lcdtli.vsw - 1;                                               /* 垂直同步脉冲宽度 */
    tli_init_struct.backpsz_hbpsz = lcdtli.hsw + lcdtli.hbp - 1;                                /* 水平后沿加同步脉冲的宽度 */
    tli_init_struct.backpsz_vbpsz = lcdtli.vsw + lcdtli.vbp - 1;                                /* 垂直后沿加同步脉冲的宽度 */
    tli_init_struct.activesz_hasz = lcdtli.hsw + lcdtli.hbp + lcdtli.pwidth - 1;                /* 水平有效宽度加水平后沿像素和水平同步像素宽度 */
    tli_init_struct.activesz_vasz = lcdtli.vsw + lcdtli.vbp + lcdtli.pheight - 1;               /* 垂直有效宽度加垂直后沿像素和垂直同步像素宽度 */
    tli_init_struct.totalsz_htsz = lcdtli.hsw + lcdtli.hbp + lcdtli.pwidth + lcdtli.hfp - 1;    /* 水平总宽度,包括有效宽度,后沿,同步脉冲和前沿 */
    tli_init_struct.totalsz_vtsz = lcdtli.vsw + lcdtli.vbp + lcdtli.pheight + lcdtli.vfp - 1;   /* 垂直总宽度,包括有效宽度,后沿,同步脉冲和前沿 */
    /* 配置背景色 */
    tli_init_struct.backcolor_red = 0xFF;                                                       /* 背景色红色值 */
    tli_init_struct.backcolor_green = 0xFF;                                                     /* 背景色绿色值 */
    tli_init_struct.backcolor_blue = 0xFF;                                                      /* 背景色蓝色值 */
    tli_init(&tli_init_struct);                                                                 /* 初始化TLI */
  
    /* 配置TLI layer0 */
    tli_layer_init_struct.layer_window_leftpos = lcdtli.hsw + lcdtli.hbp;                          /* 窗口左侧位置 */            
    tli_layer_init_struct.layer_window_rightpos = (lcdtli.pwidth + lcdtli.hsw + lcdtli.hbp - 1);   /* 窗口右侧位置 */
    tli_layer_init_struct.layer_window_toppos = lcdtli.vsw + lcdtli.vbp;                           /* 窗口顶部位置 */
    tli_layer_init_struct.layer_window_bottompos = lcdtli.pheight + lcdtli.vsw + lcdtli.vbp - 1;   /* 窗口底部位置 */
    tli_layer_init_struct.layer_ppf = TLI_PIXFORMAT;                                               /* 层像素格式 */
    tli_layer_init_struct.layer_sa = 255;                                                          /* 层恒定Alpha值, 0,全透明;255,不透明 */
    tli_layer_init_struct.layer_acf1 = LAYER_ACF1_PASA;                                            /* Alpha混合计算因子1 */
    tli_layer_init_struct.layer_acf2 = LAYER_ACF2_PASA;                                            /* Alpha混合计算因子2 */
    tli_layer_init_struct.layer_default_alpha = 0;                                                 /* 默认颜色Alpha值 */
    tli_layer_init_struct.layer_default_blue = 0;                                                  /* 默认蓝色值 */
    tli_layer_init_struct.layer_default_green = 0;                                                 /* 默认绿色值 */
    tli_layer_init_struct.layer_default_red = 0;                                                   /* 默认红色值 */
    tli_layer_init_struct.layer_frame_bufaddr = (uint32_t)g_tli_framebuf[0];                       /* 帧缓存区起始地址 */
    tli_layer_init_struct.layer_frame_buf_stride_offset = (lcdtli.pwidth * temp);                  /* 帧缓存区步幅偏移 */
    tli_layer_init_struct.layer_frame_line_length = ((lcdtli.pwidth * temp) + 3);                  /* 帧行长度 */
    tli_layer_init_struct.layer_frame_total_line_number = lcdtli.pheight;                          /* 帧总行数 */
    tli_layer_init(LAYER0, &tli_layer_init_struct);                                                /* 初始化TLI层 */                                 

    tli_layer_enable(LAYER0);                     /* 使能TLI LAYER0层 */                                                                   
    tli_reload_config(TLI_REQUEST_RELOAD_EN);     /* 层配置立即重载 */
    tli_enable();                                 /* 使能TLI */             
    
    tli_display_dir(1);                /* 默认橫屏，也可以在lcd_init函数里面设置 */
    tli_select_layer(0);               /* 选择第1层 */
    
    /* TLI LCD复位 */
	  TLI_RST(1);
	  delay_ms(10);
	  TLI_RST(0);
	  delay_ms(50);
	  TLI_RST(1); 
		delay_ms(200); 
  
    TLI_BL(1);                         /* 点亮背光 */
    tli_clear(0XFFFFFFFF);             /* 清屏 */
}

/**
 * @brief       TLI相关IO口初始化和时钟使能
 * @param       无
 * @retval      无
 */
void tli_gpio_config(void)
{
    /* TLI信号控制引脚 TLI_DE(PF10), TLI_VSYNC(PA4), TLI_HSYNC(PC6), TLI_PCLK(PG7) */
    /* TLI 数据引脚    TLI_R7(PG6), TLI_R6(PB1), TLI_R5(PC0), TLI_R4(PA5), TLI_R3(PB0), TLI_R2(PA1), TLI_R1(PA2), TLI_R0(PG13),
                       TLI_G7(PB15), TLI_G6(PC7), TLI_G5(PB11), TLI_G4(PB10), TLI_G3(PG10), TLI_G2(PA6), TLI_G1(PE6), TLI_G0(PE5),
                       TLI_B7(PB9), TLI_B6(PA15), TLI_B5(PB5), TLI_B4(PG12), TLI_B3(PA8), TLI_B2(PD6), TLI_B1(PA10 不配置，默认拉高), TLI_B0(PE4) */
  
    /* 以下是TLI信号控制引脚 DE/VSYNC/HSYNC/PCLK等的配置 */
    rcu_periph_clock_enable(TLI_DE_GPIO_CLK);       /* 使能TLI_DE引脚时钟 */
    rcu_periph_clock_enable(TLI_VSYNC_GPIO_CLK);    /* 使能TLI_VSYNC引脚时钟 */
    rcu_periph_clock_enable(TLI_HSYNC_GPIO_CLK);    /* 使能TLI_HSYNC引脚时钟 */
    rcu_periph_clock_enable(TLI_PCLK_GPIO_CLK);     /* 使能TLI_PCLK引脚时钟 */
    rcu_periph_clock_enable(TLI_BL_GPIO_CLK);       /* 使能TLI_BL引脚时钟 */
    rcu_periph_clock_enable(TLI_RST_GPIO_CLK);      /* 使能TLI_RST引脚时钟 */

    /* 设置TLI_BL引脚 推挽输出 */
    gpio_mode_set(TLI_BL_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TLI_BL_GPIO_PIN);
    gpio_output_options_set(TLI_BL_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, TLI_BL_GPIO_PIN);

    /* 设置TLI_RST引脚 推挽输出 */
    gpio_mode_set(TLI_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TLI_RST_GPIO_PIN);
    gpio_output_options_set(TLI_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, TLI_RST_GPIO_PIN);

    /* 设置TLI_DE引脚  复用推挽输出 */
    gpio_af_set(TLI_DE_GPIO_PORT, TLI_DE_GPIO_AF, TLI_DE_GPIO_PIN);
    gpio_mode_set(TLI_DE_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_DE_GPIO_PIN);
    gpio_output_options_set(TLI_DE_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_DE_GPIO_PIN);
  
    /* 设置TLI_VSYNC引脚  复用推挽输出 */
    gpio_af_set(TLI_VSYNC_GPIO_PORT, TLI_VSYNC_GPIO_AF, TLI_VSYNC_GPIO_PIN);
    gpio_mode_set(TLI_VSYNC_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_VSYNC_GPIO_PIN);
    gpio_output_options_set(TLI_VSYNC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_VSYNC_GPIO_PIN); 

    /* 设置TLI_HSYNC引脚  复用推挽输出 */
    gpio_af_set(TLI_HSYNC_GPIO_PORT, TLI_HSYNC_GPIO_AF, TLI_HSYNC_GPIO_PIN);
    gpio_mode_set(TLI_HSYNC_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_HSYNC_GPIO_PIN);
    gpio_output_options_set(TLI_HSYNC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_HSYNC_GPIO_PIN);
  
    /* 设置TLI_PCLK引脚  复用推挽输出 */
    gpio_af_set(TLI_PCLK_GPIO_PORT, TLI_PCLK_GPIO_AF, TLI_PCLK_GPIO_PIN);
    gpio_mode_set(TLI_PCLK_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TLI_PCLK_GPIO_PIN);
    gpio_output_options_set(TLI_PCLK_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, TLI_PCLK_GPIO_PIN); 
   
    /* 以下是TLI 数据引脚的配置 */   
    rcu_periph_clock_enable(RCU_GPIOA);     /* 使能GPIOA时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);     /* 使能GPIOB时钟 */
    rcu_periph_clock_enable(RCU_GPIOC);     /* 使能GPIOC时钟 */
    rcu_periph_clock_enable(RCU_GPIOD);     /* 使能GPIOD时钟 */
    rcu_periph_clock_enable(RCU_GPIOE);     /* 使能GPIOE时钟 */
    rcu_periph_clock_enable(RCU_GPIOG);     /* 使能GPIOG时钟 */
 
    /* 配置TLI数据引脚的复用功能 */
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
        
    /* 设置PA1/2/5/6/8/15 复用推挽输出 */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_15);
    
    /* 设置PB0/1/5/9/10/11/15 复用推挽输出 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15);
    
    /* 设置PC0/7 复用推挽输出 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_0 | GPIO_PIN_7);   

    /* 设置PD6 复用推挽输出 */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6); 
    
    /* 设置PE4/5/6 复用推挽输出 */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6); 
        
    /* 设置PG6/10/12/13 复用推挽输出 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ,  GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13);    
}






















