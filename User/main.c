/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       Template����ģ��-�½�����ʹ��
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"


int main(void)
{
    uint8_t t = 0;

    sys_cache_enable();                     /* ʹ��CPU cache */
    delay_init(600);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ���ڳ�ʼ��Ϊ115200 */
    
    rcu_periph_clock_enable(RCU_GPIOF);     /* ʹ��GPIOFʱ�� */
   
    /* ����PF6 �������ģʽ */
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6);
  
    /* ����PF7 �������ģʽ */
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_7);
    
    while(1)
    {        
        gpio_bit_reset(GPIOF, GPIO_PIN_6);  /* PF6��0��LED0�� */
        gpio_bit_set(GPIOF, GPIO_PIN_7);    /* PF7��1��LED1�� */ 
        delay_ms(500);
        gpio_bit_set(GPIOF, GPIO_PIN_6);    /* PF6��1��LED0�� */
        gpio_bit_reset(GPIOF, GPIO_PIN_7);  /* PF7��0��LED1�� */ 
        delay_ms(500);
      
        printf("t:%d\r\n", t);
        t++;
    }
}


