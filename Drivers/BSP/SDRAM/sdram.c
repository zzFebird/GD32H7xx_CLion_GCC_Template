/**
 ****************************************************************************************************
 * @file        sdram.c
 * @version     V1.0
 * @brief       SDRAM ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */
 
#include "./BSP/SDRAM/sdram.h"
#include "./SYSTEM/usart/usart.h" 
#include "./SYSTEM/delay/delay.h"


/* ���� SDRAM ģʽ�Ĵ��������� */
/* ͻ������ */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0003)

/* ͻ�����ʵĵ�ַģʽ */
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)

/* �е�ַѡͨ�ӳ�(CAS Latency) */
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)

/* дģʽ */
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)

#define SDRAM_TIMEOUT                            ((uint32_t)0x0000FFFF)

/**
 * @brief       ��ʼ��SDRAM
 * @param       sdram_device: EXMC SDRAM devicex
 * @retval      ��
 */
void sdram_init(uint32_t sdram_device)
{
    exmc_sdram_parameter_struct        sdram_init_struct;
    exmc_sdram_timing_parameter_struct  sdram_timing_init_struct;
    exmc_sdram_command_parameter_struct     sdram_command_init_struct;
   
    uint32_t command_content = 0, bank_select;
    uint32_t timeout = SDRAM_TIMEOUT;
  
    /* IO �� ʱ������ */  
    rcu_periph_clock_enable(RCU_EXMC);      /* ʹ��EXMCʱ�� */
    rcu_periph_clock_enable(RCU_GPIOA);     /* ʹ��GPIOAʱ�� */
    rcu_periph_clock_enable(RCU_GPIOC);     /* ʹ��GPIOCʱ�� */
    rcu_periph_clock_enable(RCU_GPIOD);     /* ʹ��GPIODʱ�� */
    rcu_periph_clock_enable(RCU_GPIOE);     /* ʹ��GPIOEʱ�� */
    rcu_periph_clock_enable(RCU_GPIOF);     /* ʹ��GPIOFʱ�� */
    rcu_periph_clock_enable(RCU_GPIOG);     /* ʹ��GPIOGʱ�� */       

    /* ����PA7  ����������� */
	  gpio_af_set(GPIOA, GPIO_AF_12, GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7);
  
    /* ����PC4/5  ����������� */
	  gpio_af_set(GPIOC, GPIO_AF_12, GPIO_PIN_4 | GPIO_PIN_5);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4 | GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_4 | GPIO_PIN_5);
                            
    /* ����PD0/1/8/9/10/14/15  ����������� */
	  gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                  GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 |GPIO_PIN_9 |
                            GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);

    /* ����PE0/1/7~15  ����������� */
	  gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                GPIO_PIN_11 | GPIO_PIN_12 |GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                  GPIO_PIN_11 | GPIO_PIN_12 |GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 |GPIO_PIN_9 | GPIO_PIN_10 |
                            GPIO_PIN_11 | GPIO_PIN_12 |GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    /* PF0~5/11~15  ����������� */
    gpio_af_set(GPIOF, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                  GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_4 | GPIO_PIN_5 |
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |GPIO_PIN_14 | GPIO_PIN_15);
	
    /* ����PG0~2/4/5/8/15  ����������� */
    gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15);
     
     /* ָ�����������͵�SDRAM device */
    if(EXMC_SDRAM_DEVICE0 == sdram_device) {
        bank_select = EXMC_SDRAM_DEVICE0_SELECT;
    } else {
        bank_select = EXMC_SDRAM_DEVICE1_SELECT;
    }

    exmc_sdram_struct_para_init(&sdram_init_struct);        /* ��ʼ���ṹ��exmc_sdram_parameter_struct */
      
    /* ����SDRAMʱ��Ĵ��� */
    sdram_timing_init_struct.load_mode_register_delay = 2;  /* ����ģʽ�Ĵ����ӳ�Ϊ2��ʱ������ */
    sdram_timing_init_struct.exit_selfrefresh_delay = 11;   /* �˳���ˢ�µ��ӳ�Ϊ11��ʱ������ */
    sdram_timing_init_struct.row_address_select_delay = 10; /* �е�ַѡ���ӳ�Ϊ10��ʱ������ */
    sdram_timing_init_struct.auto_refresh_delay = 10;       /* �Զ�ˢ���ӳ�Ϊ10��ʱ������ */
    sdram_timing_init_struct.write_recovery_delay = 2;      /* д�ָ��ӳ�Ϊ2��ʱ������ */
    sdram_timing_init_struct.row_precharge_delay = 3;       /* ��Ԥ����ӳ�Ϊ3��ʱ������ */
    sdram_timing_init_struct.row_to_column_delay = 3;       /* �����е��ӳ�Ϊ3��ʱ������ */

    /* ����SDRAM���ƼĴ��� */
    sdram_init_struct.sdram_device = sdram_device;                           /* ѡ��EXMC SDRAM devicex */
    sdram_init_struct.column_address_width = EXMC_SDRAM_COW_ADDRESS_9;       /* 9λ�е�ַλ�� */ 
    sdram_init_struct.row_address_width = EXMC_SDRAM_ROW_ADDRESS_13;         /* 13λ�е�ַλ�� */
    sdram_init_struct.data_width = EXMC_SDRAM_DATABUS_WIDTH_16B;             /* 16λSDRAM�������߿�� */
    sdram_init_struct.internal_bank_number = EXMC_SDRAM_4_INTER_BANK;        /* 4���ڲ�Banks */
    sdram_init_struct.cas_latency = EXMC_CAS_LATENCY_3_SDCLK;                /* CAS�ӳ�Ϊ3��SDRAM�洢��ʱ������ */
    sdram_init_struct.write_protection = DISABLE;                            /* ����д����������д���� */
    sdram_init_struct.sdclock_config = EXMC_SDCLK_PERIODS_2_CK_EXMC;         /* SDRAMʱ������Ϊ2��CK_EXMC���� SDCLK=CK_EXMC/2=300M/2=150M=6.7ns */
    sdram_init_struct.burst_read_switch = ENABLE;                            /* ʹ��ͻ���� */
    sdram_init_struct.pipeline_read_delay = EXMC_PIPELINE_DELAY_2_CK_EXMC;   /* ��ˮ�߶������ӳ�2��CK_EXMC����  */
    sdram_init_struct.timing = &sdram_timing_init_struct;                    /* ��дʱ����� */                                 
    exmc_sdram_init(&sdram_init_struct);                                     /* ��ʼ��EXMC SDRAM device */ 

    /* ʱ������ʹ�� */
    sdram_command_init_struct.command = EXMC_SDRAM_CLOCK_ENABLE;                       /* ʱ��ʹ������ */
    sdram_command_init_struct.bank_select = bank_select;                               /* ѡ��SDRAM devicex */
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_1_SDCLK;   /* �������Զ�ˢ�¸���(CMD=011ʱ��Ч) */
    sdram_command_init_struct.mode_register_content = 0;                               /* ģʽ�Ĵ������� */
    /* �ȴ�SDRAM���������� */
    while((exmc_flag_get(sdram_device, EXMC_SDRAM_FLAG_NREADY) != RESET) && (timeout > 0)) 
    {  
        timeout--;
    }

    exmc_sdram_command_config(&sdram_command_init_struct);   /* ��SDRAM�������� */

    delay_ms(10);   /* ��ʱ10ms */

    /* �����д洢��Ԥ��� */
    sdram_command_init_struct.command = EXMC_SDRAM_PRECHARGE_ALL;                      /* ���д洢��Ԥ������� */
    sdram_command_init_struct.bank_select = bank_select;
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_1_SDCLK;   
    sdram_command_init_struct.mode_register_content = 0;
    
    /* �ȴ�SDRAM���������� */
    timeout = SDRAM_TIMEOUT;
    while((exmc_flag_get(sdram_device, EXMC_SDRAM_FLAG_NREADY) != RESET) && (timeout > 0)) 
    {  
        timeout--;
    }

    exmc_sdram_command_config(&sdram_command_init_struct);   /* ��SDRAM�������� */

    /* ������ˢ�´��� */
    sdram_command_init_struct.command = EXMC_SDRAM_AUTO_REFRESH;                       /* �Զ�ˢ������  */
    sdram_command_init_struct.bank_select = bank_select;
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_9_SDCLK;   /* 9�������Զ�ˢ������ */
    sdram_command_init_struct.mode_register_content = 0;
    
    /* �ȴ�SDRAM���������� */
    timeout = SDRAM_TIMEOUT;
    while((exmc_flag_get(sdram_device, EXMC_SDRAM_FLAG_NREADY) != RESET) && (timeout > 0)) 
    {  
        timeout--;
    }

    exmc_sdram_command_config(&sdram_command_init_struct);   /* ��SDRAM�������� */

    /* ����SDRAM��ģʽ�Ĵ��� */
    /* ����ģʽ�Ĵ���,SDRAM��bit0~bit2Ϊָ��ͻ�����ʵĳ��ȣ�
     * bit3Ϊָ��ͻ�����ʵ����ͣ�bit4~bit6ΪCASֵ��bit7��bit8Ϊ����ģʽ
     * bit9Ϊָ����дͻ��ģʽ��bit10��bit11λ����λ */
    command_content = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1    |                      /* ����ͻ������:1(������1/2/4/8) */ 
                      SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      |                       /* ����ͻ������:����(����������/����) */ 
                      SDRAM_MODEREG_CAS_LATENCY_3              |                       /* ����CAS Latencyֵ:3(������2/3) */ 
                      SDRAM_MODEREG_OPERATING_MODE_STANDARD    |                       /* ���ò���ģʽ:0,��׼ģʽ */ 
                      SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;                            /* ����ͻ��дģʽ:1,����д�� */ 

    sdram_command_init_struct.command = EXMC_SDRAM_LOAD_MODE_REGISTER;                 /* ����ģʽ�Ĵ������� */ 
    sdram_command_init_struct.bank_select = bank_select; 
    sdram_command_init_struct.auto_refresh_number = EXMC_SDRAM_AUTO_REFLESH_1_SDCLK;   
    sdram_command_init_struct.mode_register_content = command_content;                 /* ģʽ�Ĵ������� */ 

    /* �ȴ�SDRAM���������� */
    timeout = SDRAM_TIMEOUT;
    while((exmc_flag_get(sdram_device, EXMC_SDRAM_FLAG_NREADY) != RESET) && (timeout > 0)) 
    {  
        timeout--;
    }

    exmc_sdram_command_config(&sdram_command_init_struct);   /* ��SDRAM����ģʽ�Ĵ�����ֵ */

    /**
     * �����Զ�ˢ�¼��(��SDCLKƵ�ʼ���),���㷽��:
     * ARINTV=SDRAMˢ������/����-20=SDRAMˢ������(us)*SDCLKƵ��(Mhz)/����-20
     * ����ʹ�õ�SDRAMˢ������Ϊ64ms,SDCLK=300/2=150Mhz,����Ϊ8192(2^13).
     * ����,ARINTV=64*1000*150/8192-20=1151
     */
    exmc_sdram_refresh_count_set(7.81 * 150 - 20);

    /* �ȴ�SDRAM���������� */
    timeout = SDRAM_TIMEOUT;
    while((exmc_flag_get(sdram_device, EXMC_SDRAM_FLAG_NREADY) != RESET) && (timeout > 0)) 
    {   
        timeout--;
    }
}

/**
 * @brief       ��SDRAMָ����ַ(addr + SDRAM_DEVICE0_ADDR)��ʼ,����д��n���ֽ�
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼд��ĵ�ַ
 * @param       n       : Ҫд����ֽ���
 * @retval      ��
 */
void sdram_writebuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n)
{
    for (; n != 0; n--)
    {
        *(volatile uint8_t *)(SDRAM_DEVICE0_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}

/**
 * @brief       ��SDRAMָ����ַ(addr + SDRAM_DEVICE0_ADDR)��ʼ,������ȡn���ֽ�
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼ��ȡ�ĵ�ַ
 * @param       n       : Ҫ��ȡ���ֽ���
 * @retval      ��
 */
void sdram_readbuffer_8(uint8_t *pbuf, uint32_t addr, uint32_t n)
{
    for (; n != 0; n--)
    {
        *pbuf++ = *(volatile uint8_t *)(SDRAM_DEVICE0_ADDR + addr);
        addr++;
    }
}


