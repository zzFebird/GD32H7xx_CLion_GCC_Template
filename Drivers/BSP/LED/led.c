/**
 ****************************************************************************************************
 * @file        led.c
 * @version     V1.0
 * @brief       LED ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./BSP/LED/led.h"


/**
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void led_init(void)
{
    rcu_periph_clock_enable(LED0_GPIO_CLK);     /* ʹ��LED0ʱ�� */
    rcu_periph_clock_enable(LED1_GPIO_CLK);     /* ʹ��LED1ʱ�� */
    
    /* LED0����ģʽ���� */
    gpio_mode_set(LED0_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED0_GPIO_PIN);
    gpio_output_options_set(LED0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, LED0_GPIO_PIN);
  
    /* LED1����ģʽ���� */
    gpio_mode_set(LED1_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED1_GPIO_PIN);
    gpio_output_options_set(LED1_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, LED1_GPIO_PIN);
	  
	  LED0(1);                                    /* �ر�LED0 */ 
	  LED1(1);                                    /* �ر�LED1 */
}

