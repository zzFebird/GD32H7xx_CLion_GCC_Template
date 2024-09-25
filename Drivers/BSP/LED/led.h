/**
 ****************************************************************************************************
 * @file        led.h
 * @version     V1.0
 * @brief       LED ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ���� ���� */

#define LED0_GPIO_PORT                  GPIOF
#define LED0_GPIO_PIN                   GPIO_PIN_6
#define LED0_GPIO_CLK                   RCU_GPIOF   /* GPIOFʱ��ʹ�� */

#define LED1_GPIO_PORT                  GPIOF
#define LED1_GPIO_PIN                   GPIO_PIN_7
#define LED1_GPIO_CLK                   RCU_GPIOF   /* GPIOFʱ��ʹ�� */

/******************************************************************************************/

/* LED�˿ڶ��� */
#define LED0(x)   do{ x ? \
                      gpio_bit_write(LED0_GPIO_PORT, LED0_GPIO_PIN, SET) : \
                      gpio_bit_write(LED0_GPIO_PORT, LED0_GPIO_PIN, RESET); \
                  }while(0)      

#define LED1(x)   do{ x ? \
                      gpio_bit_write(LED1_GPIO_PORT, LED1_GPIO_PIN, SET) : \
                      gpio_bit_write(LED1_GPIO_PORT, LED1_GPIO_PIN, RESET); \
                  }while(0)     
                  
/* LEDȡ������ */
#define LED0_TOGGLE()   do{ gpio_bit_toggle(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)       /* ��תLED0 */
#define LED1_TOGGLE()   do{ gpio_bit_toggle(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)       /* ��תLED1 */


void led_init(void);             /* ��ʼ��LED */

#endif





