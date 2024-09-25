/**
 ****************************************************************************************************					 
 * @file        sys.c
 * @version     V1.0
 * @brief       ϵͳ��ʼ������(����ʱ������/�жϹ���/GPIO���õ�)            
 ****************************************************************************************************
 * V1.0
 * ��ͷ�ļ�����·���ĳ����·��,�����ظ����ð���·�����鷳
 *
 ****************************************************************************************************
 */ 
 
#include "./SYSTEM/sys/sys.h"


/**
 * @brief       ִ��: WFIָ��(ִ�����ָ�����͹���״̬, �ȴ��жϻ���)
 * @param       ��
 * @retval      ��
 */
void sys_wfi_set(void)
{
    __ASM volatile("wfi");
}

/**
 * @brief       �ر������ж�(���ǲ�����fault��NMI�ж�)
 * @param       ��
 * @retval      ��
 */
void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

/**
 * @brief       ���������ж�
 * @param       ��
 * @retval      ��
 */
void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

/**
 * @brief       ����ջ����ַ
 * @param       addr: ջ����ַ
 * @retval      ��
 */
void sys_msr_msp(uint32_t addr)
{
    __set_MSP(addr);   /* ����ջ����ַ */
}

/**
 * @brief       ϵͳ��λ
 * @param       ��
 * @retval      ��
 */
void sys_soft_reset(void)
{
    NVIC_SystemReset();
}

/**
 * @brief       ʹ��GD32H7��L1-Cache, ͬʱ����D cache��ǿ��͸д
 * @param       ��
 * @retval      ��
 */
void sys_cache_enable(void)
{
    SCB_EnableICache();     /* ʹ��I-Cache */
    SCB_EnableDCache();     /* ʹ��D-Cache */
    SCB->CACR |= 1 << 2;    /* ǿ��D-Cache͸д,�粻����͸д,ʵ��ʹ���п��������������� */
}






