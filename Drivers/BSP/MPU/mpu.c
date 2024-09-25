/**
 ****************************************************************************************************
 * @file        mpu.c
 * @version     V1.0
 * @brief       MPU�ڴ汣�� ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/MPU/mpu.h"


/**
 * @brief       ����ĳ�������MPU����
 * @param       baseaddr: MPU��������Ļ���ַ(�׵�ַ)
 * @param       size:MPU��������Ĵ�С(��λΪ�ֽڣ�������ڵ���32�ֽ�)
 * @param       rnum:MPU����������,��Χ:0~15,���֧��15����������
 * @param       de  : ��ָֹ�����;MPU_INSTRUCTION_EXEC_PERMIT,����ָ�����;MPU_INSTRUCTION_EXEC_NOT_PERMIT,��ָֹ�����
 * @param       ap:����Ȩ��,���ʹ�ϵ����:
 *   @arg       MPU_AP_NO_ACCESS,�޷��ʣ���Ȩ&�û������ɷ��ʣ�
 *   @arg       MPU_AP_PRIV_RW,��֧����Ȩ��д����
 *   @arg       MPU_AP_PRIV_RW_UNPRIV_RO,��ֹ�û�д���ʣ���Ȩ�ɶ�д���ʣ�
 *   @arg       MPU_AP_FULL_ACCESS,ȫ���ʣ���Ȩ&�û������Է��ʣ�
 *   @arg       MPU_AP_PRIV_RO,��֧����Ȩ������
 *   @arg       MPU_AP_PRIV_UNPRIV_RO,ֻ������Ȩ&�û���������д��
 * @param       sen : �Ƿ�������;MPU_ACCESS_NON_SHAREABLE,������;MPU_ACCESS_SHAREABLE,����
 * @param       cen : �Ƿ�����cache;MPU_ACCESS_NON_CACHEABLE,������;MPU_ACCESS_CACHEABLE,����
 * @param       ben : �Ƿ�������;MPU_ACCESS_NON_BUFFERABLE,������;MPU_ACCESS_BUFFERABLE,����
 * @retval      ��
 */
void mpu_set_protection(uint32_t baseaddr, uint32_t size, uint32_t rnum, uint8_t de, uint8_t ap, uint8_t sen, uint8_t cen, uint8_t ben)
{
    mpu_region_init_struct mpu_init_struct;
    mpu_region_struct_para_init(&mpu_init_struct);

    ARM_MPU_Disable();                                           /* ����MPU֮ǰ�ȹر�MPU,��������Ժ���ʹ��MPU */
  
    mpu_init_struct.region_number       = rnum;                  /* ���ñ��������� */
    mpu_init_struct.region_base_address = baseaddr;              /* ���ñ����������ַ */
    mpu_init_struct.instruction_exec    = de;                    /* �Ƿ�����ָ����� */
    mpu_init_struct.access_permission   = ap;                    /* ���÷���Ȩ�� */
    mpu_init_struct.tex_type            = MPU_TEX_TYPE0;         /* ����TEX����Ϊtype 0 */
    mpu_init_struct.access_shareable    = sen;                   /* �Ƿ���? */
    mpu_init_struct.access_cacheable    = cen;                   /* �Ƿ�cache? */
    mpu_init_struct.access_bufferable   = ben;                   /* �Ƿ񻺳�? */
    mpu_init_struct.subregion_disable   = 0X00;                  /* ��ֹ������ */
    mpu_init_struct.region_size         = size;                  /* ���ñ��������С */    
    mpu_region_config(&mpu_init_struct);                         /* ����MPU���� */
    mpu_region_enable();                                         /* ʹ��MPU���� */
                
    ARM_MPU_Enable(MPU_MODE_PRIV_DEFAULT);                       /* �������,ʹ��MPU���� */
}
 
/**
 * @brief       ������Ҫ�����Ĵ洢��
 * @note        ����Բ��ִ洢�������MPU����,������ܵ��³��������쳣
 *
 * @param       ��
 * @retval      ��
 */
void mpu_memory_protection(void)
{
    /* ��������ITCM,��64K�ֽ� */
    mpu_set_protection( 0x00000000,                 /* �������ַ */
                        MPU_REGION_SIZE_64KB,       /* �����С */
                        MPU_REGION_NUMBER0,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_NON_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */

    /* ��������DTCM,��128K�ֽ� */
    mpu_set_protection( 0x20000000,                 /* �������ַ */
                        MPU_REGION_SIZE_128KB,      /* �����С */
                        MPU_REGION_NUMBER1,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_NON_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */

    /* ��������AXI SRAM,��1024K�ֽ� */
    mpu_set_protection( 0x24000000,                 /* �������ַ */
                        MPU_REGION_SIZE_1MB,        /* �����С */
                        MPU_REGION_NUMBER2,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_SHAREABLE,       /* ������ */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_NON_BUFFERABLE); /* ��ֹ���� */

    /* ��������SRAM0��SRAM1,��32K�ֽ� */
    mpu_set_protection( 0x30000000,                 /* �������ַ */
                        MPU_REGION_SIZE_32KB,       /* �����С */
                        MPU_REGION_NUMBER3,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_NON_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */

    /* ����MCU LCD�����ڵ�EXMC����,��64M�ֽ� */
    mpu_set_protection( 0x64000000,                 /* �������ַ */
                        MPU_REGION_SIZE_64MB,       /* �����С */
                        MPU_REGION_NUMBER4,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_NON_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_NON_CACHEABLE,   /* ��ֹcache */
                        MPU_ACCESS_NON_BUFFERABLE); /* ��ֹ���� */
                        
    /* ����SDRAM����,��32M�ֽ� */
    mpu_set_protection( 0xC0000000,                 /* �������ַ */
                        MPU_REGION_SIZE_32MB,       /* �����С */
                        MPU_REGION_NUMBER5,         /* ������ */
                        MPU_INSTRUCTION_EXEC_PERMIT,/* ����ָ����� */
                        MPU_AP_FULL_ACCESS,         /* ȫ���� */
                        MPU_ACCESS_NON_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */                               
}

/**
 * @brief       MemManage�������ж�
 * @note        ������ж��Ժ�,���޷��ָ���������!!
 *
 * @param       ��
 * @retval      ��
 */
void MemManage_Handler(void)
{
    LED1(0);                            /* ����LED1 */
    printf("Mem Access Error!!\r\n");   /* ���������Ϣ */
    delay_ms(1000);
    printf("Soft Reseting...\r\n");     /* ��ʾ������� */
    delay_ms(1000);
    sys_soft_reset();                   /* ��λ */
}














