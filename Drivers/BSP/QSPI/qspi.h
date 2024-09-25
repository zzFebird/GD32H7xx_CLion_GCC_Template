/**
 ****************************************************************************************************
 * @file        qspi.h
 * @version     V1.0
 * @brief       QSPI ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */		

#ifndef __QSPI_H
#define __QSPI_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* QSPI ��� ���� ���� */

#define OSPIM_P0_SCK_GPIO_PORT          GPIOB
#define OSPIM_P0_SCK_GPIO_PIN           GPIO_PIN_2
#define OSPIM_P0_SCK_GPIO_AF            GPIO_AF_9
#define OSPIM_P0_SCK_GPIO_CLK           RCU_GPIOB   /* GPIOBʱ��ʹ�� */

#define OSPIM_P0_CSN_GPIO_PORT          GPIOB
#define OSPIM_P0_CSN_GPIO_PIN           GPIO_PIN_6
#define OSPIM_P0_CSN_GPIO_AF            GPIO_AF_10
#define OSPIM_P0_CSN_GPIO_CLK           RCU_GPIOB   /* GPIOBʱ��ʹ�� */

#define OSPIM_P0_IO0_GPIO_PORT          GPIOD
#define OSPIM_P0_IO0_GPIO_PIN           GPIO_PIN_11
#define OSPIM_P0_IO0_GPIO_AF            GPIO_AF_9
#define OSPIM_P0_IO0_GPIO_CLK           RCU_GPIOD   /* GPIODʱ��ʹ�� */

#define OSPIM_P0_IO1_GPIO_PORT          GPIOD
#define OSPIM_P0_IO1_GPIO_PIN           GPIO_PIN_12
#define OSPIM_P0_IO1_GPIO_AF            GPIO_AF_9
#define OSPIM_P0_IO1_GPIO_CLK           RCU_GPIOD   /* GPIODʱ��ʹ�� */

#define OSPIM_P0_IO2_GPIO_PORT          GPIOB
#define OSPIM_P0_IO2_GPIO_PIN           GPIO_PIN_13
#define OSPIM_P0_IO2_GPIO_AF            GPIO_AF_4
#define OSPIM_P0_IO2_GPIO_CLK           RCU_GPIOB   /* GPIOBʱ��ʹ�� */

#define OSPIM_P0_IO3_GPIO_PORT          GPIOD
#define OSPIM_P0_IO3_GPIO_PIN           GPIO_PIN_13
#define OSPIM_P0_IO3_GPIO_AF            GPIO_AF_9
#define OSPIM_P0_IO3_GPIO_CLK           RCU_GPIOD   /* GPIODʱ��ʹ�� */

/******************************************************************************************/

#define OSPI_INTERFACE             OSPI0            /* ʹ�õ�OSPI�ӿ� */

extern ospi_parameter_struct g_ospi_struct;         /* OSPI�����ṹ�� */


void ospi_flash_init(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct);                         /* ��ʼ��OSPI�ӿ� */   
void ospi_send_command(uint32_t ins, uint32_t addr, uint32_t datalen, uint16_t mode, uint8_t dmcycle);  /* OSPI�������� */

#endif
























