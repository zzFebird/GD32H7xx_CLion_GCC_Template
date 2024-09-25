/**
 ****************************************************************************************************
 * @file        ffsystem.c
 * @version     V1.0
 * @brief       FATFS�ײ�(ffsystem) ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./MALLOC/malloc.h"
#include "./SYSTEM/sys/sys.h"
#include "./FATFS/source/ff.h"


/**
 * @brief       ���ʱ��
 * @param       ��
 * @retval      ʱ��
 * @note        ʱ������������:
 *              User defined function to give a current time to fatfs module 
 *              31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31)
 *              15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) 
 */
DWORD get_fattime (void)
{
    return 0;
}

/**
 * @brief       ��̬�����ڴ�
 * @param       size : Ҫ������ڴ��С(�ֽ�)
 * @retval      ���䵽���ڴ��׵�ַ.
 */
void *ff_memalloc (UINT size)
{
    return (void*)mymalloc(SRAMIN, size);
} 

/**
 * @brief       �ͷ��ڴ�
 * @param       mf  : �ڴ��׵�ַ
 * @retval      ��
 */
void ff_memfree (void* mf)
{
    myfree(SRAMIN, mf);
}

















