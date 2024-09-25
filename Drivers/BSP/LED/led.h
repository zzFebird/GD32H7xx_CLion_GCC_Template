/**
 ****************************************************************************************************
 * @file        led.h
 * @version     V1.0
 * @brief       LED 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    GD32H757ZMT6小系统板
 *
 ****************************************************************************************************
 */	

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 定义 */

#define LED0_GPIO_PORT                  GPIOF
#define LED0_GPIO_PIN                   GPIO_PIN_6
#define LED0_GPIO_CLK                   RCU_GPIOF   /* GPIOF时钟使能 */

#define LED1_GPIO_PORT                  GPIOF
#define LED1_GPIO_PIN                   GPIO_PIN_7
#define LED1_GPIO_CLK                   RCU_GPIOF   /* GPIOF时钟使能 */

/******************************************************************************************/

/* LED端口定义 */
#define LED0(x)   do{ x ? \
                      gpio_bit_write(LED0_GPIO_PORT, LED0_GPIO_PIN, SET) : \
                      gpio_bit_write(LED0_GPIO_PORT, LED0_GPIO_PIN, RESET); \
                  }while(0)      

#define LED1(x)   do{ x ? \
                      gpio_bit_write(LED1_GPIO_PORT, LED1_GPIO_PIN, SET) : \
                      gpio_bit_write(LED1_GPIO_PORT, LED1_GPIO_PIN, RESET); \
                  }while(0)     
                  
/* LED取反定义 */
#define LED0_TOGGLE()   do{ gpio_bit_toggle(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)       /* 翻转LED0 */
#define LED1_TOGGLE()   do{ gpio_bit_toggle(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)       /* 翻转LED1 */


void led_init(void);             /* 初始化LED */

#endif





