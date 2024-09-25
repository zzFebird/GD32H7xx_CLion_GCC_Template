/**
 ****************************************************************************************************
 * @file        qspi.c
 * @version     V1.0
 * @brief       QSPI ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	
 
#include "./BSP/QSPI/qspi.h"
#include "./SYSTEM/usart/usart.h"


ospi_parameter_struct g_ospi_struct;   /* OSPI�����ṹ�� */

/**
 * @brief       ��ʼ��OSPI�ӿ�
 * @param       ospi_periph : OSPI�ӿڣ�OSPIx(x=0,1)
 * @param       ospi_struct : OSPI�����ṹ��
 * @retval      ��
 */
void ospi_flash_init(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    ospi_deinit(ospi_periph);                          /* ��λOSPI */
    ospim_deinit();                                    /* ��λOSPIM */
  
    rcu_periph_clock_enable(RCU_OSPIM);                /* ʹ��OSPIMʱ�� */  
    rcu_periph_clock_enable(OSPIM_P0_SCK_GPIO_CLK);    /* ʹ��OSPIM_P0_SCK IO��ʱ�� */
    rcu_periph_clock_enable(OSPIM_P0_CSN_GPIO_CLK);    /* ʹ��OSPIM_P0_CSN IO��ʱ�� */
    rcu_periph_clock_enable(OSPIM_P0_IO0_GPIO_CLK);    /* ʹ��OSPIM_P0_IO0 IO��ʱ�� */
    rcu_periph_clock_enable(OSPIM_P0_IO1_GPIO_CLK);    /* ʹ��OSPIM_P0_IO1 IO��ʱ�� */
    rcu_periph_clock_enable(OSPIM_P0_IO2_GPIO_CLK);    /* ʹ��OSPIM_P0_IO2 IO��ʱ�� */
    rcu_periph_clock_enable(OSPIM_P0_IO3_GPIO_CLK);    /* ʹ��OSPIM_P0_IO3 IO��ʱ�� */
  
    /* ����ʹ�õ�OSPIM����:
           OSPIM_P0_SCK(PB2)
           OSPIM_P0_CSN(PB6)
           OSPIM_P0_IO0(PD11)
           OSPIM_P0_IO1(PD12)
           OSPIM_P0_IO2(PB13)
           OSPIM_P0_IO3(PD13) */

    /* ����OSPIM���ŵĸ��ù��� */
    gpio_af_set(OSPIM_P0_SCK_GPIO_PORT, OSPIM_P0_SCK_GPIO_AF, OSPIM_P0_SCK_GPIO_PIN);
    gpio_af_set(OSPIM_P0_CSN_GPIO_PORT, OSPIM_P0_CSN_GPIO_AF, OSPIM_P0_CSN_GPIO_PIN);  
    gpio_af_set(OSPIM_P0_IO0_GPIO_PORT, OSPIM_P0_IO0_GPIO_AF, OSPIM_P0_IO0_GPIO_PIN);
    gpio_af_set(OSPIM_P0_IO1_GPIO_PORT, OSPIM_P0_IO1_GPIO_AF, OSPIM_P0_IO1_GPIO_PIN);
    gpio_af_set(OSPIM_P0_IO2_GPIO_PORT, OSPIM_P0_IO2_GPIO_AF, OSPIM_P0_IO2_GPIO_PIN);
    gpio_af_set(OSPIM_P0_IO3_GPIO_PORT, OSPIM_P0_IO3_GPIO_AF, OSPIM_P0_IO3_GPIO_PIN);
    
    /* OSPIM_P0_SCK����ģʽ���� */
    gpio_mode_set(OSPIM_P0_SCK_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_SCK_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_SCK_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_SCK_GPIO_PIN);

    /* OSPIM_P0_CSN����ģʽ���� */
    gpio_mode_set(OSPIM_P0_CSN_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_CSN_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_CSN_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_CSN_GPIO_PIN);

    /* OSPIM_P0_IO0����ģʽ���� */
    gpio_mode_set(OSPIM_P0_IO0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_IO0_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_IO0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_IO0_GPIO_PIN);

    /* OSPIM_P0_IO1����ģʽ���� */
    gpio_mode_set(OSPIM_P0_IO1_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_IO1_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_IO1_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_IO1_GPIO_PIN);

    /* OSPIM_P0_IO2����ģʽ���� */
    gpio_mode_set(OSPIM_P0_IO2_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_IO2_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_IO2_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_IO2_GPIO_PIN);

    /* OSPIM_P0_IO3����ģʽ���� */
    gpio_mode_set(OSPIM_P0_IO3_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OSPIM_P0_IO3_GPIO_PIN);
    gpio_output_options_set(OSPIM_P0_IO3_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, OSPIM_P0_IO3_GPIO_PIN);
    
    ospim_port_sck_config(OSPIM_PORT0, OSPIM_PORT_SCK_ENABLE);    /* ʹ��OSPIM�˿�0��SCK */
    ospim_port_csn_config(OSPIM_PORT0, OSPIM_PORT_CSN_ENABLE);    /* ʹ��OSPIM�˿�0��CSN */
    ospim_port_io3_0_config(OSPIM_PORT0, OSPIM_IO_LOW_ENABLE);    /* ʹ��OSPIM�˿�0��IO[3:0] */

    switch (ospi_periph) 
    {
        case OSPI0:    
            rcu_periph_clock_enable(RCU_OSPI0);                                        /* ʹ��OSPI0ʱ�� */
            ospim_port_sck_source_select(OSPIM_PORT0, OSPIM_SCK_SOURCE_OSPI0_SCK);     /* ѡ��OSPIM�˿�0��SCKԴΪOSPI0_SCK */
            ospim_port_csn_source_select(OSPIM_PORT0, OSPIM_CSN_SOURCE_OSPI0_CSN);     /* ѡ��OSPIM�˿�0��CSNԴΪOSPI0_CSN */
            ospim_port_io3_0_source_select(OSPIM_PORT0, OSPIM_SRCPLIO_OSPI0_IO_LOW);   /* ѡ��OSPIM�˿�0��IO[3:0]ԴΪOSPI0_IO[3:0] */
            break;
        
        case OSPI1:
            rcu_periph_clock_enable(RCU_OSPI1);                                        /* ʹ��OSPI1ʱ�� */
            ospim_port_sck_source_select(OSPIM_PORT0, OSPIM_SCK_SOURCE_OSPI1_SCK);     /* ѡ��OSPIM�˿�0��SCKԴΪOSPI0_SCK */
            ospim_port_csn_source_select(OSPIM_PORT0, OSPIM_CSN_SOURCE_OSPI1_CSN);     /* ѡ��OSPIM�˿�0��CSNԴΪOSPI0_CSN */
            ospim_port_io3_0_source_select(OSPIM_PORT0, OSPIM_SRCPLIO_OSPI1_IO_LOW);   /* ѡ��OSPIM�˿�0��IO[3:0]ԴΪOSPI0_IO[3:0] */
            break;
        
        default:
            break;
    }

    ospi_struct_init(ospi_struct);                                 /* ��ʼ��OSPI�ṹ�����ΪĬ��ֵ */

    ospi_struct->prescaler = (4 - 1);                              /* ���ں�ʱ�ӷ�Ƶ����OSPIʱ�ӵķ�Ƶ���� 
                                                                    * OSPIʱ������AHB3ʱ�ӣ�����ʹ��4��Ƶ, ��OSPIʱ��Ϊ300M / 4 = 75Mhz      
                                                                    */
    ospi_struct->sample_shift = OSPI_SAMPLE_SHIFTING_HALF_CYCLE;   /* ������λ����֮һ������ */
    ospi_struct->fifo_threshold = OSPI_FIFO_THRESHOLD_4;           /* FIFO�е���ֵΪ4���ֽ� */
    ospi_struct->device_size = OSPI_MESZ_16_MBS;                   /* �ⲿ�洢����СΪ16MB */
    ospi_struct->wrap_size = OSPI_DIRECT;                          /* �ⲿ�洢���豸��֧�ֻؾ��ȡ */
    ospi_struct->cs_hightime = OSPI_CS_HIGH_TIME_4_CYCLE;          /* Ƭѡ�ߵ�ƽʱ��Ϊ4������(4*1000/75=53ns),���ֲ������tSHSL���� */
    ospi_struct->memory_type = OSPI_MICRON_MODE;                   /* �ⲿ�豸����ΪMicronģʽ */
    ospi_struct->delay_hold_cycle = OSPI_DELAY_HOLD_NONE;          /* ���ӳٱ��� */

    ospi_init(ospi_periph, ospi_struct);                           /* ��ʼ��OSPI���� */
    
    ospi_enable(ospi_periph);                                      /* ʹ��OSPI */     
}

/**
 * @brief       OSPI��������
 * @param       ins     : Ҫ���͵�ָ��
 * @param       addr    : ���͵���Ŀ�ĵ�ַ
 * @param       datalen : Ҫ��������ݳ���
 * @param       mode: ģʽ,��ϸλ��������:
 *   @arg       mode[2:0]:  ָ��ģʽ; 000,��ָ��;  001,���ߴ���ָ��; 010,˫�ߴ���ָ��; 011,���ߴ���ָ��; 100,���ߴ���ָ��.
 *   @arg       mode[5:3]:  ��ַģʽ; 000,�޵�ַ;  001,���ߴ����ַ; 010,˫�ߴ����ַ; 011,���ߴ����ַ; 100,���ߴ����ַ.
 *   @arg       mode[7:6]:  ��ַ����; 00,8λ��ַ;   01,16λ��ַ;      10,24λ��ַ;      11,32λ��ַ.
 *   @arg       mode[10:8]: ����ģʽ; 000,������;  001,���ߴ�������; 010,˫�ߴ�������; 011,���ߴ�������; 100,���ߴ�������.
 * @param       dmcycle: ��ָ��������
 * @retval      ��
 */
void ospi_send_command(uint32_t ins, uint32_t addr, uint32_t datalen, uint16_t mode, uint8_t dmcycle)
{
    ospi_regular_cmd_struct cmd_struct = {0};                        /* OSPI��������ṹ�� */
 
    cmd_struct.instruction = ins;                                    /* ����Ҫ���͵�ָ�� */
    cmd_struct.address = addr;                                       /* ����Ҫ���͵ĵ�ַ */
    cmd_struct.dummy_cycles = dmcycle;                               /* ���ÿ�ָ�������� */
    cmd_struct.nbdata = datalen;                                     /* ����Ҫ����������� */
    
    if(((mode >> 0) & 0x07) == 0)
    cmd_struct.ins_mode = OSPI_INSTRUCTION_NONE;                     /* ָ��ģʽ */
    else if(((mode >> 0) & 0x07) == 1)
    cmd_struct.ins_mode = OSPI_INSTRUCTION_1_LINE;                   /* ָ��ģʽ */
    else if(((mode >> 0) & 0x07) == 2)
    cmd_struct.ins_mode = OSPI_INSTRUCTION_2_LINES;                  /* ָ��ģʽ */
    else if(((mode >> 0) & 0x07) == 3)
    cmd_struct.ins_mode = OSPI_INSTRUCTION_4_LINES;                  /* ָ��ģʽ */    
    else if(((mode >> 0) & 0x07) == 4)
    cmd_struct.ins_mode = OSPI_INSTRUCTION_8_LINES;                  /* ָ��ģʽ */    

    if(((mode >> 3) & 0x07) == 0)
    cmd_struct.addr_mode = OSPI_ADDRESS_NONE;                        /* ��ַģʽ */
    else if(((mode >> 3) & 0x07) == 1)
    cmd_struct.addr_mode = OSPI_ADDRESS_1_LINE;                      /* ��ַģʽ */
    else if(((mode >> 3) & 0x07) == 2)
    cmd_struct.addr_mode = OSPI_ADDRESS_2_LINES;                     /* ��ַģʽ */
    else if(((mode >> 3) & 0x07) == 3)
    cmd_struct.addr_mode = OSPI_ADDRESS_4_LINES;                     /* ��ַģʽ */    
    else if(((mode >> 3) & 0x07) == 4)
    cmd_struct.addr_mode = OSPI_ADDRESS_8_LINES;                     /* ��ַģʽ */  

    if(((mode >> 6) & 0x03) == 0)
    cmd_struct.addr_size = OSPI_ADDRESS_8_BITS;                      /* ��ַ��С */
    else if(((mode >> 6) & 0x03) == 1)
    cmd_struct.addr_size = OSPI_ADDRESS_16_BITS;                     /* ��ַ��С */
    else if(((mode >> 6) & 0x03) == 2)
    cmd_struct.addr_size = OSPI_ADDRESS_24_BITS;                     /* ��ַ��С */
    else if(((mode >> 6) & 0x03) == 3) 
    cmd_struct.addr_size = OSPI_ADDRESS_32_BITS;                     /* ��ַ��С */    
 
    if(((mode >> 8) & 0x07) == 0)
    cmd_struct.data_mode = OSPI_DATA_NONE;                           /* ����ģʽ */
    else if(((mode >> 8) & 0x07) == 1)
    cmd_struct.data_mode = OSPI_DATA_1_LINE;                         /* ����ģʽ */
    else if(((mode >> 8) & 0x07) == 2)
    cmd_struct.data_mode = OSPI_DATA_2_LINES;                        /* ����ģʽ */
    else if(((mode >> 8) & 0x07) == 3)
    cmd_struct.data_mode = OSPI_DATA_4_LINES;                         /* ����ģʽ */    
    else if(((mode >> 8) & 0x07) == 4)
    cmd_struct.data_mode = OSPI_DATA_8_LINES;                         /* ����ģʽ */     
 
    cmd_struct.operation_type = OSPI_OPTYPE_COMMON_CFG;               /* ͨ������(���ģʽ��״̬��ѯģʽ��ʹ��)������Ӧ���ڳ���Ĵ��� */
    cmd_struct.ins_size = OSPI_INSTRUCTION_8_BITS;                    /* ����ָ���СΪ8λָ�� */
    cmd_struct.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;             /* ��ַ�׶ν���DTRģʽ */
    cmd_struct.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;          /* �޽����ֽ� */
    cmd_struct.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;               /* ���ݽ׶ν���DTRģʽ */

    ospi_command_config(OSPI_INTERFACE, &g_ospi_struct, &cmd_struct); /* ����OSPI����������� */    
}













