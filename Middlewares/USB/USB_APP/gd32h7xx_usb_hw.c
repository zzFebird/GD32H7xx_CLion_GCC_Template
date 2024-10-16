/**
 ****************************************************************************************************
 * @file        gd32h7xx_usb_hw.c
 * @version     V1.0
 * @brief       GD32H7xx USB底层硬件配置 代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "drv_usb_regs.h"
#include "drv_usb_hw.h"
#include "./SYSTEM/delay/delay.h"


/**
 * @brief       配置USB时钟
 * @param       无
 * @retval      无
 */
void usb_rcu_config(void)
{
    pmu_usb_regulator_enable();                                  /* 使能USB电压稳压器 */
    pmu_usb_voltage_detector_enable();                           /* 使能VDD33USB电压监控器 */
  
    while (pmu_flag_get(PMU_FLAG_USB33RF) != SET);               /* 等待USB33供电电压就绪 */ 
   
    rcu_periph_clock_enable(RCU_USBHS0);                         /* 使能USBHS0时钟 */

#ifdef USE_USB_FS

#ifndef USE_IRC48M                                               /* 如果没有定义USE_IRC48M，即不使用IRC48M时钟 */

    *(uint32_t*)0x4004000C |= 0x40;                              /* 内部嵌入式全速PHY使能 */

    *(uint32_t*)0x40040038 |= 0x10000;                           /* 内部嵌入式全速PHY上电 */

    rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10); /* 配置PLLUSBHS0时钟 */
  
    RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;                      /* 使能PLLUSBHS0时钟 */
    
    while((RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB) == 0U);       /* 等待PLLUSBHS0时钟稳定 */

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLLUSBHS); /* 选择CK_PLLUSBHS作为USBHS48M时钟源 */

#else                                                            /* 定义了USE_IRC48M */

    rcu_osci_on(RCU_IRC48M);                                     /* 打开IRC48M时钟 */

    while (SUCCESS != rcu_osci_stab_wait(RCU_IRC48M));           /* 等待IRC48M振荡器时钟稳定 */

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_IRC48M);   /* 选择CK_IRC48M作为USBHS48M时钟源 */

#endif /* USE_IRC48M */

#endif /* USE_USB_FS */
}

/**
 * @brief       配置USB中断
 * @param       无
 * @retval      无
 */
void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
  
    nvic_irq_enable((uint8_t)USBHS0_IRQn, 0U, 3U);         /* USBHS0中断配置，抢占优先级0，子优先级3 */

    rcu_periph_clock_enable(RCU_PMU);                      /* 使能PMU时钟 */

    /* USBHS0 唤醒EXTI线配置 */
    exti_interrupt_flag_clear(EXTI_31);                    /* 清除EXTI线31上的中断标志位 */
  
    exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);  /* 初始化EXTI线31，模式配置为上升沿触发中断 */

    nvic_irq_enable((uint8_t)USBHS0_WKUP_IRQn, 1U, 0U);    /* USBHS0 唤醒EXTI线中断配置，抢占优先级1，响应优先级0 */	
}

/**
 * @brief       USB 延时函数(以us为单位)
 * @param       usec   : 延时的us数
 * @retval      无
 */
void usb_udelay (const uint32_t usec)
{
    delay_us(usec);      /* usb_udelay使用delay_us实现 */
}
  
/**
 * @brief       USB 延时函数(以ms为单位)
 * @param       msec   : 延时的ms数
 * @retval      无
 */
void usb_mdelay (const uint32_t msec)
{
    delay_ms(msec);     /* usb_mdelay使用delay_ms实现 */
}



