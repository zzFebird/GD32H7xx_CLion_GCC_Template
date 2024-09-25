/**
 ****************************************************************************************************
 * @file        key.h
 * @version     V1.0
 * @brief       �������� ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	

#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ���� ���� */

#define KEY0_GPIO_PORT                  GPIOC
#define KEY0_GPIO_PIN                   GPIO_PIN_1
#define KEY0_GPIO_CLK                   RCU_GPIOC   /* GPIOCʱ��ʹ�� */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK                   RCU_GPIOA   /* GPIOAʱ��ʹ�� */

/******************************************************************************************/

#define KEY0        gpio_input_bit_get(KEY0_GPIO_PORT, KEY0_GPIO_PIN)   /* ��ȡKEY0���� */
#define WK_UP       gpio_input_bit_get(WKUP_GPIO_PORT, WKUP_GPIO_PIN)   /* ��ȡWKUP���� */

#define KEY0_PRES 	1             /* KEY0���� */
#define WKUP_PRES   2             /* WK_UP���� */

void key_init(void);              /* ������ʼ������ */
uint8_t key_scan(uint8_t mode);   /* ����ɨ�躯�� */

#endif
