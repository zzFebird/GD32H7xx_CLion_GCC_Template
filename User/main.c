/**
 ****************************************************************************************************
 * @file        main.c
 * @version     V1.0
 * @brief       Template工程模板-新建工程使用
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"


int main(void)
{
    uint8_t t = 0;

    sys_cache_enable();                     /* 使能CPU cache */
    delay_init(600);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    
    rcu_periph_clock_enable(RCU_GPIOF);     /* 使能GPIOF时钟 */
   
    /* 设置PF6 推挽输出模式 */
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_6);
  
    /* 设置PF7 推挽输出模式 */
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_7);
    
    while(1)
    {        
        gpio_bit_reset(GPIOF, GPIO_PIN_6);  /* PF6置0，LED0亮 */
        gpio_bit_set(GPIOF, GPIO_PIN_7);    /* PF7置1，LED1灭 */ 
        delay_ms(500);
        gpio_bit_set(GPIOF, GPIO_PIN_6);    /* PF6置1，LED0灭 */
        gpio_bit_reset(GPIOF, GPIO_PIN_7);  /* PF7置0，LED1亮 */ 
        delay_ms(500);
      
        printf("t:%d\r\n", t);
        t++;
    }
}


