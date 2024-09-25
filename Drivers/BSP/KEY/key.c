/**
 ****************************************************************************************************
 * @file        key.c
 * @version     V1.0
 * @brief       �������� ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"


/**
 * @brief       ������ʼ������
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
    rcu_periph_clock_enable(KEY0_GPIO_CLK);     /* ʹ��KEY0ʱ�� */
    rcu_periph_clock_enable(WKUP_GPIO_CLK);     /* ʹ��WKUPʱ�� */
    
    gpio_mode_set(KEY0_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, KEY0_GPIO_PIN);  /* KEY0����ģʽ����,�������� */
    
    gpio_mode_set(WKUP_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, WKUP_GPIO_PIN);  /* WKUP����ģʽ����,�������� */  	
}

/**
 * @brief       ����ɨ�躯��
 * @note        �ú�������Ӧ���ȼ�(ͬʱ���¶������): WK_UP > KEY0!!
 * @param       mode:0 / 1, ���庬������:
 *   @arg       0,  ��֧��������(���������²���ʱ, ֻ�е�һ�ε��û᷵�ؼ�ֵ,
 *                  �����ɿ��Ժ�, �ٴΰ��²Ż᷵��������ֵ)
 *   @arg       1,  ֧��������(���������²���ʱ, ÿ�ε��øú������᷵�ؼ�ֵ)
 * @retval      ��ֵ, ��������:
 *              KEY0_PRES, 1, KEY0����
 *              WKUP_PRES, 2, WKUP����
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* �������ɿ���־ */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* ֧������ */
  
    if (key_up && (KEY0 == 1 || WK_UP == 1))     /* �����ɿ���־Ϊ1, ��������һ������������ */
    {
        delay_ms(10);           /* ȥ���� */
        key_up = 0;

        if (KEY0 == 1)  keyval = KEY0_PRES;
      
        if (WK_UP == 1) keyval = WKUP_PRES;
    }
    else if (KEY0 == 0 && WK_UP == 0)            /* û���κΰ�������, ��ǰ����ɿ� */
    {
        key_up = 1;
    }

    return keyval;              /* ���ؼ�ֵ */
}


