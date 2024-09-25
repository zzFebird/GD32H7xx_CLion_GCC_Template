/**
 ****************************************************************************************************					 
 * @file        usmart_port.h
 * @version     V1.0
 * @brief       USMART ��ֲ�ļ�
 *
 *              ͨ���޸ĸ��ļ�,���Է���Ľ�USMART��ֲ����������
 *              runtimeϵͳָ��,��������ͳ�ƺ���ִ��ʱ��.
 *              �÷�:
 *              ����:runtime 1 ,��������ִ��ʱ��ͳ�ƹ���
 *              ����:runtime 0 ,��رպ���ִ��ʱ��ͳ�ƹ���
 *              runtimeͳ�ƹ���,��������:USMART_ENTIMX_SCAN Ϊ1,�ſ���ʹ��!!
 *
 *              USMART��Դռ�������
 *              FLASH:4K~K�ֽ�(ͨ��USMART_USE_HELP��USMART_USE_WRFUNS����)
 *              SRAM:72�ֽ�(���ٵ������)
 *              SRAM���㹫ʽ:   SRAM=PARM_LEN+72-4  ����PARM_LEN������ڵ���4.
 *              Ӧ�ñ�֤��ջ��С��100���ֽ�.
 * 
 ****************************************************************************************************
 */	

#ifndef __USMART_PORT_H
#define __USMART_PORT_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"




/******************************************************************************************/
/* �û����ò��� */

#define MAX_FNAME_LEN           30      /* ��������󳤶ȣ�Ӧ������Ϊ��С����������ĳ��ȡ� */
#define MAX_PARM                10      /* ���Ϊ10������ ,�޸Ĵ˲���,�����޸�usmart_exe��֮��Ӧ. */
#define PARM_LEN                200     /* ���в���֮�͵ĳ��Ȳ�����PARM_LEN���ֽ�,ע�⴮�ڽ��ղ���Ҫ��֮��Ӧ(��С��PARM_LEN) */


#define USMART_ENTIMX_SCAN      1       /* ʹ��TIM�Ķ�ʱ�ж���ɨ��SCAN����,�������Ϊ0,��Ҫ�Լ�ʵ�ָ�һ��ʱ��ɨ��һ��scan����.
                                         * ע��:���Ҫ��runtimeͳ�ƹ���,��������USMART_ENTIMX_SCANΪ1!!!!
                                         */

#define USMART_USE_HELP         1       /* ʹ�ð�������ֵ��Ϊ0�����Խ�ʡ��700���ֽڣ����ǽ������޷���ʾ������Ϣ�� */
#define USMART_USE_WRFUNS       1       /* ʹ�ö�д����,ʹ������,���Զ�ȡ�κε�ַ��ֵ,������д�Ĵ�����ֵ. */

#define USMART_PRINTF           printf  /* ����printf��� */

/******************************************************************************************/
/* USMART��ʱ�� ���� */

# if USMART_ENTIMX_SCAN == 1    /* ������ʹ�ܶ�ʱ��ɨ��,����Ҫ���¶��� */

/* TIMERX �ж϶��� 
 * ���ڶ�ʱ����usmart.scan����ɨ�贮������,��ִ����ز���
 * ע��: ͨ���޸���4���궨��,����֧������һ����ʱ��.
 */
#define USMART_TIMERX                     TIMER3
#define USMART_TIMERX_IRQn                TIMER3_IRQn
#define USMART_TIMERX_IRQHandler          TIMER3_IRQHandler
#define USMART_TIMERX_CLK                 RCU_TIMER3   /* TIMER3 ʱ��ʹ�� */

#endif

/******************************************************************************************/


/* ���û�ж���uint32_t,���� */
#ifndef uint32_t
typedef unsigned           char uint8_t;
typedef unsigned short     int  uint16_t;
typedef unsigned           int  uint32_t;
#endif



char * usmart_get_input_string(void);     /* ��ȡ���������� */
void usmart_reset_runtime(void);          /* ��λ����ʱ�� */
uint32_t usmart_get_runtime(void);        /* ��ȡ����ʱ�� */
void usmart_timerx_init(uint16_t tclk);   /* ��ʼ����ʱ�� */

#endif



























