/**
 ****************************************************************************************************					 
 * @file        usmart_port.c
 * @version     V1.0
 * @brief       USMART ��ֲ�ļ�
 *
 *              ͨ���޸ĸ��ļ�,���Է���Ľ�USMART��ֲ����������
 *              ��:USMART_ENTIMX_SCAN == 0ʱ,����Ҫʵ��: usmart_get_input_string����.
 *              ��:USMART_ENTIMX_SCAN == 1ʱ,��Ҫ��ʵ��4������:
 *              usmart_reset_runtime
 *              usmart_get_runtime
 *              usmart_timerx_init
 *              USMART_TIMERX_IRQHandler
 * 
 ****************************************************************************************************
 */
 
#include "./USMART/usmart.h"
#include "./USMART/usmart_port.h"

/**
 * @brief       ��ȡ����������(�ַ���)
 * @note        USMARTͨ�������ú������ص��ַ����Ի�ȡ����������������Ϣ
 * @param       ��
 * @retval
 *   @arg       0,  û�н��յ�����
 *   @arg       ����,�������׵�ַ(������0)
 */
char *usmart_get_input_string(void)
{
    uint8_t len;
    char *pbuf = 0;
    
    if (g_usart_rx_sta & 0x8000)        /* ���ڽ�����ɣ� */
    {
        len = g_usart_rx_sta & 0x3fff;  /* �õ��˴ν��յ������ݳ��� */
        g_usart_rx_buf[len] = '\0';     /* ��ĩβ���������. */
        pbuf = (char*)g_usart_rx_buf;
        g_usart_rx_sta = 0;             /* ������һ�ν��� */
    }

    return pbuf;
}

/* ���ʹ���˶�ʱ��ɨ��, ����Ҫ�������º��� */
#if USMART_ENTIMX_SCAN == 1

/**
 * ��ֲע��:��������GD32Ϊ��,���Ҫ��ֲ������mcu,������Ӧ�޸�.
 * usmart_reset_runtime,�����������ʱ��,��ͬ��ʱ���ļ����Ĵ����Լ���־λһ������.��������װ��ֵΪ���,������޶ȵ��ӳ���ʱʱ��.
 * usmart_get_runtime,��ȡ��������ʱ��,ͨ����ȡCNTֵ��ȡ,����usmart��ͨ���жϵ��õĺ���,���Զ�ʱ���жϲ�����Ч,��ʱ����޶�
 * ֻ��ͳ��2��CNT��ֵ,Ҳ���������+���һ��,���������2��,û������,���������ʱ,������:2*������CNT*0.1ms.��GD32 TIMERX(16λ)��˵,��:13.1s����
 * ������:USMART_TIMERX_IRQHandler��usmart_timerx_init,��Ҫ����MCU�ص������޸�.ȷ������������Ƶ��Ϊ:10Khz����.����,��ʱ����Ҫ�����Զ���װ�ع���!!
 */

/**
 * @brief       ��λruntime
 *   @note      ��Ҫ��������ֲ����MCU�Ķ�ʱ�����������޸�
 * @param       ��
 * @retval      ��
 */
void usmart_reset_runtime(void)
{
    timer_interrupt_flag_clear(USMART_TIMERX, TIMER_INT_FLAG_UP);   /* �����ʱ�������жϱ�־λ */
    timer_autoreload_value_config(USMART_TIMERX, 0XFFFF);           /* ����װ��ֵ���õ���� */
    timer_counter_value_config(USMART_TIMERX, 0);                   /* ��ն�ʱ����CNT */
    usmart_dev.runtime = 0;
}

/**
 * @brief       ���runtimeʱ��
 *   @note      ��Ҫ��������ֲ����MCU�Ķ�ʱ�����������޸�
 * @param       ��
 * @retval      ִ��ʱ��,��λ:0.1ms,�����ʱʱ��Ϊ��ʱ��CNTֵ��2��*0.1ms
 */
uint32_t usmart_get_runtime(void)
{
    if(timer_interrupt_flag_get(USMART_TIMERX, TIMER_INT_FLAG_UP) == SET)   /* �������ڼ�,�����˶�ʱ����� */
    {
        usmart_dev.runtime += 0XFFFF;
    }

    usmart_dev.runtime += timer_counter_read(USMART_TIMERX);
    return usmart_dev.runtime;      /* ���ؼ���ֵ */
}

/**
 * @brief       ��ʱ����ʼ������
 * @param       tclk: ��ʱ���Ĺ���Ƶ��(��λ:Mhz)
 * @retval      ��
 */
void usmart_timerx_init(uint16_t tclk)
{
    timer_parameter_struct timer_initpara;                 /* timer_initpara���ڴ�Ŷ�ʱ���Ĳ��� */
 
    rcu_periph_clock_enable(USMART_TIMERX_CLK);            /* ʹ��TIMERX��ʱ�� */

    timer_deinit(USMART_TIMERX);                           /* ��λTIMERX */
    timer_struct_para_init(&timer_initpara);               /* ��ʼ��timer_initparaΪĬ��ֵ */

    /* ����tclk��ֵ(��λ:Mhz),����ʱ��������ʱ�ӷ�Ƶ��10KHz ,100ms�ж�һ��
     * ע��,����Ƶ�ʱ���Ϊ10Khz,�Ժ�runtime��λ(0.1ms)ͬ��.
     */
    timer_initpara.prescaler         = (tclk * 100) - 1;   /* ����Ԥ��Ƶֵ */
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE; /* ���ö���ģʽΪ���ض���ģʽ */
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;   /* �������ϼ���ģʽ */
    timer_initpara.period            = 1000;               /* �����Զ���װ��ֵ */
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;   /* ����ʱ�ӷ�Ƶ���� */
    timer_initpara.repetitioncounter = 0;                  /* �����ظ�������ֵ */
    timer_init(USMART_TIMERX, &timer_initpara);            /* ���ݲ�����ʼ����ʱ�� */

    /* ʹ�ܶ�ʱ�������ж� */
    timer_interrupt_flag_clear(USMART_TIMERX, TIMER_INT_FLAG_UP);   /* �����ʱ�������жϱ�־ */
  
    timer_interrupt_enable(USMART_TIMERX, TIMER_INT_UP);            /* ʹ�ܶ�ʱ���ĸ����ж� */
    
    nvic_irq_enable(USMART_TIMERX_IRQn, 3, 3);                      /* ����NVIC�������ȼ�����ռ���ȼ�3����Ӧ���ȼ�3(��2�����ȼ���͵�) */
    
    timer_enable(USMART_TIMERX);                                    /* ʹ�ܶ�ʱ��TIMERX */	
}

/**
 * @brief       USMART��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void USMART_TIMERX_IRQHandler(void)
{
    if (timer_interrupt_flag_get(USMART_TIMERX, TIMER_INT_FLAG_UP) == SET)  /* ����ж� */
    {   
        usmart_dev.scan();                                                  /* ִ��usmartɨ�� */
        timer_counter_value_config(USMART_TIMERX, 0);                       /* ��ն�ʱ����CNT */
        timer_autoreload_value_config(USMART_TIMERX, 1000);                 /* �ָ�ԭ�������� */
    }

    timer_interrupt_flag_clear(USMART_TIMERX, TIMER_INT_FLAG_UP);           /* �����ʱ�������жϱ�־λ */
}

#endif
















