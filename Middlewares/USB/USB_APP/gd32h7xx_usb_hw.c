/**
 ****************************************************************************************************
 * @file        gd32h7xx_usb_hw.c
 * @version     V1.0
 * @brief       GD32H7xx USB�ײ�Ӳ������ ����
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "drv_usb_regs.h"
#include "drv_usb_hw.h"
#include "./SYSTEM/delay/delay.h"


/**
 * @brief       ����USBʱ��
 * @param       ��
 * @retval      ��
 */
void usb_rcu_config(void)
{
    pmu_usb_regulator_enable();                                  /* ʹ��USB��ѹ��ѹ�� */
    pmu_usb_voltage_detector_enable();                           /* ʹ��VDD33USB��ѹ����� */
  
    while (pmu_flag_get(PMU_FLAG_USB33RF) != SET);               /* �ȴ�USB33�����ѹ���� */ 
   
    rcu_periph_clock_enable(RCU_USBHS0);                         /* ʹ��USBHS0ʱ�� */

#ifdef USE_USB_FS

#ifndef USE_IRC48M                                               /* ���û�ж���USE_IRC48M������ʹ��IRC48Mʱ�� */

    *(uint32_t*)0x4004000C |= 0x40;                              /* �ڲ�Ƕ��ʽȫ��PHYʹ�� */

    *(uint32_t*)0x40040038 |= 0x10000;                           /* �ڲ�Ƕ��ʽȫ��PHY�ϵ� */

    rcu_pllusb0_config(RCU_PLLUSBHSPRE_HXTAL, RCU_PLLUSBHSPRE_DIV5, RCU_PLLUSBHS_MUL96, RCU_USBHS_DIV10); /* ����PLLUSBHS0ʱ�� */
  
    RCU_ADDCTL1 |= RCU_ADDCTL1_PLLUSBHS0EN;                      /* ʹ��PLLUSBHS0ʱ�� */
    
    while((RCU_ADDCTL1 & RCU_ADDCTL1_PLLUSBHS0STB) == 0U);       /* �ȴ�PLLUSBHS0ʱ���ȶ� */

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_PLLUSBHS); /* ѡ��CK_PLLUSBHS��ΪUSBHS48Mʱ��Դ */

#else                                                            /* ������USE_IRC48M */

    rcu_osci_on(RCU_IRC48M);                                     /* ��IRC48Mʱ�� */

    while (SUCCESS != rcu_osci_stab_wait(RCU_IRC48M));           /* �ȴ�IRC48M����ʱ���ȶ� */

    rcu_usb48m_clock_config(IDX_USBHS0, RCU_USB48MSRC_IRC48M);   /* ѡ��CK_IRC48M��ΪUSBHS48Mʱ��Դ */

#endif /* USE_IRC48M */

#endif /* USE_USB_FS */
}

/**
 * @brief       ����USB�ж�
 * @param       ��
 * @retval      ��
 */
void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
  
    nvic_irq_enable((uint8_t)USBHS0_IRQn, 0U, 3U);         /* USBHS0�ж����ã���ռ���ȼ�0�������ȼ�3 */

    rcu_periph_clock_enable(RCU_PMU);                      /* ʹ��PMUʱ�� */

    /* USBHS0 ����EXTI������ */
    exti_interrupt_flag_clear(EXTI_31);                    /* ���EXTI��31�ϵ��жϱ�־λ */
  
    exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);  /* ��ʼ��EXTI��31��ģʽ����Ϊ�����ش����ж� */

    nvic_irq_enable((uint8_t)USBHS0_WKUP_IRQn, 1U, 0U);    /* USBHS0 ����EXTI���ж����ã���ռ���ȼ�1����Ӧ���ȼ�0 */	
}

/**
 * @brief       USB ��ʱ����(��usΪ��λ)
 * @param       usec   : ��ʱ��us��
 * @retval      ��
 */
void usb_udelay (const uint32_t usec)
{
    delay_us(usec);      /* usb_udelayʹ��delay_usʵ�� */
}
  
/**
 * @brief       USB ��ʱ����(��msΪ��λ)
 * @param       msec   : ��ʱ��ms��
 * @retval      ��
 */
void usb_mdelay (const uint32_t msec)
{
    delay_ms(msec);     /* usb_mdelayʹ��delay_msʵ�� */
}



