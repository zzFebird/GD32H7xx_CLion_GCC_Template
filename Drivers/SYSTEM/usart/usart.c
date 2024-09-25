/**
 ****************************************************************************************************					 
 * @file        usart.c
 * @version     V1.1
 * @brief       ���ڳ�ʼ�����룬֧��printf            
 ****************************************************************************************************
 * V1.1
 * �޸�SYS_SUPPORT_OS���ִ���, ����ͷ�ļ��ĳ�:"os.h"
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"


/* ���ʹ��os,����������ͷ�ļ�����. */
#if SYS_SUPPORT_OS
#include "os.h" /* os ʹ�� */
#endif

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while (RESET == usart_flag_get(USART_UX, USART_FLAG_TC));       /* �ȴ���һ���ַ�������� */
    
    usart_data_transmit(USART_UX, (uint8_t)ch);                     /* ��Ҫ���͵��ַ� ch д�뵽TDATA�Ĵ��� */  
    return ch;
}
#endif

/******************************************************************************************/

#if USART_EN_RX       /* ���ʹ���˽��� */

/* ���ջ���, ���USART_REC_LEN���ֽ�. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t g_usart_rx_sta = 0;

/**
 * @brief       ����X�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{
    uint8_t rxdata;
#if SYS_SUPPORT_OS  /* ���SYS_SUPPORT_OSΪ�棬����Ҫ֧��OS. */
    OSIntEnter();    
#endif
   
		if (usart_interrupt_flag_get(USART_UX, USART_INT_FLAG_RBNE) != RESET)   /* ���յ����� */
		{
        rxdata = usart_data_receive(USART_UX);	         /* ��ȡ���յ������� */
        
        if ((g_usart_rx_sta & 0x8000) == 0)              /* ����δ���? */
        {
            if (g_usart_rx_sta & 0x4000)                 /* ���յ���0x0d?  */
            { 
                if (rxdata != 0x0a)                      /* ���յ���0x0a? (�����Ƚ��յ���0x0d,�ż��0x0a) */
                { 
                    g_usart_rx_sta = 0;                  /* ���մ���,���¿�ʼ */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;	           /* �յ���0x0a,��ǽ�������� */ 
                }
            }
            else                                         /* ��û�յ�0X0d  */
            {	
                if (rxdata == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;            /* ��ǽ��յ���0x0d  */                            
                } 
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = rxdata;              /* �洢���ݵ� g_usart_rx_buf */
                    g_usart_rx_sta++;
                  
                    if (g_usart_rx_sta > (USART_REC_LEN - 1))g_usart_rx_sta = 0;   /* �����������,���¿�ʼ���� */ 
                }		 
            }
        }   		 
     } 

    usart_flag_clear(USART_UX, USART_FLAG_ORERR);        /* �����������־, ������ܻῨ���ڴ����жϷ��������� */   
     
#if SYS_SUPPORT_OS  /* ���SYS_SUPPORT_OSΪ�棬����Ҫ֧��OS. */
    OSIntExit();  											 
#endif
}
#endif

/**
 * @brief       ����X��ʼ������
 * @param       bound: ������, �����Լ���Ҫ���ò�����ֵ
 * @retval      ��
 */
void usart_init(uint32_t bound)
{
    /* IO �� ʱ������ */
    rcu_periph_clock_enable(USART_TX_GPIO_CLK);     /* ʹ�ܴ���TX��ʱ�� */
    rcu_periph_clock_enable(USART_RX_GPIO_CLK);     /* ʹ�ܴ���RX��ʱ�� */
    rcu_periph_clock_enable(USART_UX_CLK);          /* ʹ�ܴ���ʱ�� */

    /* ����USARTx_TX�ĸ��ù���ѡ�� */
    gpio_af_set(USART_TX_GPIO_PORT, USART_TX_GPIO_AF, USART_TX_GPIO_PIN);

    /* ����USARTx_RX�ĸ��ù���ѡ�� */
    gpio_af_set(USART_RX_GPIO_PORT, USART_RX_GPIO_AF, USART_RX_GPIO_PIN);

    /* USARTx_TX��ģʽ���� */
    gpio_mode_set(USART_TX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART_TX_GPIO_PIN);
    gpio_output_options_set(USART_TX_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, USART_TX_GPIO_PIN);

    /* USARTx_RX��ģʽ���� */
    gpio_mode_set(USART_RX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART_RX_GPIO_PIN);
    gpio_output_options_set(USART_RX_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, USART_RX_GPIO_PIN);

    /* ����USART�Ĳ��� */
    usart_deinit(USART_UX);                                 /* ��λUSARTx */
    usart_baudrate_set(USART_UX, bound);                    /* ���ò����� */
    usart_stop_bit_set(USART_UX, USART_STB_1BIT);           /* һ��ֹͣλ */
    usart_word_length_set(USART_UX, USART_WL_8BIT);         /* �ֳ�Ϊ8λ���ݸ�ʽ */
    usart_parity_config(USART_UX, USART_PM_NONE);           /* ����żУ��λ */
    usart_transmit_config(USART_UX, USART_TRANSMIT_ENABLE); /* ʹ�ܷ��� */
    
#if USART_EN_RX                                             /* ���ʹ���˽��� */
    usart_receive_config(USART_UX, USART_RECEIVE_ENABLE);   /* ʹ�ܽ��� */
    usart_interrupt_enable(USART_UX, USART_INT_RBNE);       /* ʹ�ܽ��ջ������ǿ��ж� */    
    /* ����NVIC���������ж����ȼ� */
    nvic_irq_enable(USART_UX_IRQn, 3, 3);                   /* ��2����ռ���ȼ�3�������ȼ�3 */
#endif

    usart_enable(USART_UX);	                                /* ʹ�ܴ��� */
}




