/**
 ****************************************************************************************************					 
 * @file        usart.h
 * @version     V1.1
 * @brief       ���ڳ�ʼ�����룬֧��printf            
 ****************************************************************************************************
 * V1.1
 * �޸�SYS_SUPPORT_OS���ִ���, ����ͷ�ļ��ĳ�:"os.h"
 *
 ****************************************************************************************************
 */	

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"


/*******************************************************************************************************/
/* ���źʹ��� ���� */

#define USART_TX_GPIO_PORT              GPIOA
#define USART_TX_GPIO_PIN               GPIO_PIN_9
#define USART_TX_GPIO_AF                GPIO_AF_7
#define USART_TX_GPIO_CLK               RCU_GPIOA   /* GPIOAʱ��ʹ�� */

#define USART_RX_GPIO_PORT              GPIOA
#define USART_RX_GPIO_PIN               GPIO_PIN_10
#define USART_RX_GPIO_AF                GPIO_AF_7
#define USART_RX_GPIO_CLK               RCU_GPIOA   /* GPIOAʱ��ʹ�� */

#define USART_UX                        USART0
#define USART_UX_IRQn                   USART0_IRQn
#define USART_UX_IRQHandler             USART0_IRQHandler
#define USART_UX_CLK                    RCU_USART0  /* USART0 ʱ��ʹ�� */

/*******************************************************************************************************/

#define USART_REC_LEN               200             /* �����������ֽ��� 200 */
#define USART_EN_RX                 1               /* ʹ�ܣ�1��/��ֹ��0�����ڽ��� */


extern uint8_t  g_usart_rx_buf[USART_REC_LEN];      /* ���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
extern uint16_t g_usart_rx_sta;                     /* ����״̬��� */

void usart_init(uint32_t bound);                    /* ���ڳ�ʼ������ */

#endif


