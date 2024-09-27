/**
 ****************************************************************************************************
 * @file        norflash.c
 * @version     V1.0
 * @brief       NOR FLASH(25QXX) ��������
 ****************************************************************************************************
 * @attention   Waiken-Smart ������Զ
 *
 * ʵ��ƽ̨:    GD32H757ZMT6Сϵͳ��
 *
 ****************************************************************************************************
 */	
 
#include "./BSP/QSPI/qspi.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/NORFLASH/norflash.h"


uint16_t g_norflash_type = W25Q128;     /* Ĭ����W25Q128 */

volatile uint8_t g_norflash_addrw = 2;  /* SPI FLASH��ַλ��, ��norflash_read_id�������汻�޸�
                                         * 2, ��ʾ24bit��ַ���
                                         * 3, ��ʾ32bit��ַ���
                                         */
                                         
/**
 * @brief       ��ʼ��SPI NOR FLASH
 * @param       ��
 * @retval      ��
 */
void norflash_init(void)
{
    uint8_t temp;

    ospi_flash_init(OSPI_INTERFACE, &g_ospi_struct);  /* ��ʼ��OSPI */
  
    norflash_qspi_disable();                /* �˳�QPIģʽ(����оƬ֮ǰ�������ģʽ,��������ʧ��) */
    norflash_qe_enable();                   /* ʹ��QEλ */
    
    g_norflash_type = norflash_read_id();   /* ��ȡFLASH ID */
    
    if (g_norflash_type == W25Q256)         /* SPI FLASHΪW25Q256, ����ʹ��4�ֽڵ�ַģʽ */
    {
        temp = norflash_read_sr(3);         /* ��ȡ״̬�Ĵ���3���жϵ�ַģʽ */

        if ((temp & 0X01) == 0)             /* �������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ */
        {
            norflash_write_enable();        /* дʹ�� */
            temp |= 1 << 1;                 /* ADP=1, �ϵ�4λ��ַģʽ */
            norflash_write_sr(3, temp);     /* дSR3 */
            delay_ms(20);                   /* ������ʱ��һ���ϵ����� */            
 
            /* SPI,����ʹ��4�ֽڵ�ַָ��,��ַΪ0,0���ֽ�����,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿����� */
            ospi_send_command(FLASH_Enable4ByteAddr, 0, 0, (0 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);          
        }
    }

    //printf("ID:%x\r\n", g_norflash_type);
}

/**
 * @brief       ʹ��FLASH QEλ��ʹ��IO2/IO3
 * @param       ��
 * @retval      ��
 */
void norflash_qe_enable(void)
{
    uint8_t stareg2;
    stareg2 = norflash_read_sr(2);      /* �ȶ���״̬�Ĵ���2��ԭʼֵ */

    //printf("stareg2:%x\r\n", stareg2);
    if ((stareg2 & 0X02) == 0)          /* QEλδʹ�� */
    {
        norflash_write_enable();        /* дʹ�� */
        stareg2 |= 1 << 1;              /* ʹ��QEλ */
        norflash_write_sr(2, stareg2);  /* д״̬�Ĵ���2 */
    }
}

/**
 * @brief       �˳�QSPIģʽ
 * @param       ��
 * @retval      ��
 */
void norflash_qspi_disable(void)
{  
    /* �����˳�QPIģʽָ��,��ַΪ0,0���ֽ�����,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_ExitQPIMode, 0, 0, (0 << 8) | (0 << 6) | (0 << 3) | (3 << 0), 0);
}

/**
 * @brief       �ȴ�����
 * @param       ��
 * @retval      ��
 */
void norflash_wait_busy(void)
{
    while ((norflash_read_sr(1) & 0x01) == 0x01);   /*  �ȴ�BUSYλ��� */
}

/**
 * @brief       25QXXдʹ��
 * @note        ��S1�Ĵ�����WEL��λ
 * @param       ��
 * @retval      ��
 */
void norflash_write_enable(void)
{
    /* SPI,����дʹ��ָ��,��ַΪ0,0���ֽ�����,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_WriteEnable, 0, 0, (0 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);
}

/**
 * @brief       25QXXд��ֹ
 * @note        ��S1�Ĵ�����WEL����
 * @param       ��
 * @retval      ��
 */
void norflash_write_disable(void)
{
    /* SPI,����д��ָֹ��,��ַΪ0,0���ֽ�����,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_WriteDisable, 0, 0, (0 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);
}

/**
 * @brief       ��ȡ25QXX��״̬�Ĵ�����25QXXһ����3��״̬�Ĵ���
 * @note        ״̬�Ĵ���1��
 *              BIT7  6   5   4   3   2   1   0
 *              SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *              SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
 *              TB,BP2,BP1,BP0:FLASH����д��������
 *              WEL:дʹ������
 *              BUSY:æ���λ(1,æ;0,����)
 *              Ĭ��:0x00
 *
 *              ״̬�Ĵ���2��
 *              BIT7  6   5   4   3   2   1   0
 *              SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
 *
 *              ״̬�Ĵ���3��
 *              BIT7      6    5    4   3   2   1   0
 *              HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
 *
 * @param       regno: ״̬�Ĵ����ţ���Χ:1~3
 * @retval      ״̬�Ĵ���ֵ
 */
uint8_t norflash_read_sr(uint8_t regno)
{
    uint8_t byte = 0, command = 0;

    switch (regno)
    {
        case 1:
            command = FLASH_ReadStatusReg1;  /* ��״̬�Ĵ���1ָ�� */
            break;

        case 2:
            command = FLASH_ReadStatusReg2;  /* ��״̬�Ĵ���2ָ�� */
            break;

        case 3:
            command = FLASH_ReadStatusReg3;  /* ��״̬�Ĵ���3ָ�� */
            break;

        default:
            command = FLASH_ReadStatusReg1;
            break;
    }

    /* SPI,���Ͷ��Ĵ���ָ��,��ַΪ0,1���ֽ�����,���ߴ�����_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿����� */
    ospi_send_command(command, 0, 1, (1 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);
    ospi_receive(OSPI_INTERFACE, &byte);   /* ��ȡһ���ֽ� */
    
    return byte;
}

/**
 * @brief       д25QXX״̬�Ĵ���
 * @note        �Ĵ���˵����norflash_read_sr����˵��
 * @param       regno: ״̬�Ĵ����ţ���Χ:1~3
 * @param       sr   : Ҫд��״̬�Ĵ�����ֵ
 * @retval      ��
 */
void norflash_write_sr(uint8_t regno, uint8_t sr)
{
    uint8_t command = 0;

    switch (regno)
    {
        case 1:
            command = FLASH_WriteStatusReg1;  /* д״̬�Ĵ���1ָ�� */
            break;

        case 2:
            command = FLASH_WriteStatusReg2;  /* д״̬�Ĵ���2ָ�� */
            break;

        case 3:
            command = FLASH_WriteStatusReg3;  /* д״̬�Ĵ���3ָ�� */
            break;

        default:
            command = FLASH_WriteStatusReg1;
            break;
    }

    /* SPI,����д״̬�Ĵ���ָ��,��ַΪ0,1���ֽ�����,���ߴ�����_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿����� */
    ospi_send_command(command, 0, 1, (1 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);
    ospi_transmit(OSPI_INTERFACE, &sr);   /* д��һ���ֽ� */
}

/**
 * @brief       ��ȡоƬID
 * @param       ��
 * @retval      FLASHоƬID
 * @note        оƬID�б��: norflash.h, оƬ�б���
 */
uint16_t norflash_read_id(void)
{
    uint8_t temp[2];
    uint16_t deviceid;

    /* SPI,���Ͷ�IDָ��,��ַΪ0,2���ֽ�����,���ߴ�������_24λ��ַ_���ߴ����ַ_���ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_ManufactDeviceID, 0, 2, (1 << 8) | (2 << 6) | (1 << 3) | (1 << 0), 0);
    ospi_receive(OSPI_INTERFACE, temp);   /* ��ȡ�����ֽ����� */
    deviceid = (temp[0] << 8) | temp[1];
  
    if (deviceid == W25Q256)
    {
        g_norflash_addrw = 3;   /* �����W25Q256, ���32bit��ַ��� */
    }  
    
   return deviceid;
}

/**
 * @brief       ��ȡSPI FLASH,��֧��QSPIģʽ
 * @note        ��ָ����ַ��ʼ��ȡָ�����ȵ�����
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼ��ȡ�ĵ�ַ(���32bit)
 * @param       datalen : Ҫ��ȡ���ֽ���(���65535)
 * @retval      ��
 */
void norflash_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    /* QPI,���Ϳ��ٶ�����ָ��,��ַΪaddr,datalen������,4�ߴ�������_24/32λ��ַ_4�ߴ����ַ_1�ߴ���ָ��,6�������� */
    ospi_send_command(FLASH_FastReadQuad_IO, addr, datalen, (3 << 8) | (g_norflash_addrw << 6) | (3 << 3) | (1 << 0), 6);
    ospi_receive(OSPI_INTERFACE, pbuf);  /* ��ȡ���� */
}

/**
 * @brief       SPI��һҳ(0~65535)��д������256���ֽڵ�����
 * @note        ��ָ����ַ��ʼд�����256�ֽڵ�����
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼд��ĵ�ַ(���32bit)
 * @param       datalen : Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
 * @retval      ��
 */
void norflash_write_page(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    norflash_write_enable();              /* дʹ�� */

    /* QPI,����дҳָ��,��ַΪaddr,datalen������,4�ߴ�������_24/32λ��ַ_1�ߴ����ַ_1�ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_PageProgramQuad, addr, datalen, (3 << 8) | (g_norflash_addrw << 6) | (1 << 3) | (1 << 0), 0);
  
    ospi_transmit(OSPI_INTERFACE, pbuf);  /* �������� */
    norflash_wait_busy();                 /* �ȴ�д����� */
}

/**
 * @brief       �޼���дSPI FLASH
 *   @note      ����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
 *              �����Զ���ҳ����
 *              ��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
 *
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼд��ĵ�ַ(���32bit)
 * @param       datalen : Ҫд����ֽ���(���65535)
 * @retval      ��
 */
void norflash_write_nocheck(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    uint32_t pageremain;
    pageremain = 256 - addr % 256;  /* ��ҳʣ����ֽ��� */

    if (datalen <= pageremain)      /* ������256���ֽ� */
    {
        pageremain = datalen;
    }

    while (1)
    {
        /* ��д���ֽڱ�ҳ��ʣ���ַ���ٵ�ʱ��, һ����д��
         * ��д��ֱ�ӱ�ҳ��ʣ���ַ�����ʱ��, ��д������ҳ��ʣ���ַ, Ȼ�����ʣ�೤�Ƚ��в�ͬ����
         */
        norflash_write_page(pbuf, addr, pageremain);

        if (datalen == pageremain)   /* д������� */
        {
            break;
        }
        else                            /* datalen > pageremain */
        {
            pbuf += pageremain;         /* pbufָ���ַƫ��,ǰ���Ѿ�д��pageremain�ֽ� */
            addr += pageremain;         /* д��ַƫ��,ǰ���Ѿ�д��pageremain�ֽ� */
            datalen -= pageremain;      /* д���ܳ��ȼ�ȥ�Ѿ�д���˵��ֽ��� */

            if (datalen > 256)          /* ʣ�����ݻ�����һҳ,����һ��дһҳ */
            {
                pageremain = 256;       /* һ�ο���д��256���ֽ� */
            }
            else                        /* ʣ������С��һҳ,����һ��д�� */
            {
                pageremain = datalen;   /* ����256���ֽ��� */
            }
        }
    }
}

/**
 * @brief       дSPI FLASH
 *   @note      ��ָ����ַ��ʼд��ָ�����ȵ����� , �ú�������������!
 *              SPI FLASH һ����: 256���ֽ�Ϊһ��Page, 4KbytesΪһ��Sector, 16������Ϊ1��Block
 *              ��������С��λΪSector.
 *
 * @param       pbuf    : ���ݴ洢��
 * @param       addr    : ��ʼд��ĵ�ַ(���32bit)
 * @param       datalen : Ҫд����ֽ���(���65535)
 * @retval      ��
 */
uint8_t g_norflash_buf[4096];   /* �������� */

void norflash_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    uint32_t secpos;
    uint16_t secoff;
    uint32_t secremain;
    uint16_t i;
    uint8_t *norflash_buf;

    norflash_buf = g_norflash_buf;
    secpos = addr / 4096;       /* ������ַ */
    secoff = addr % 4096;       /* �������ڵ�ƫ�� */
    secremain = 4096 - secoff;  /* ����ʣ��ռ��С */

    //printf("ad:%X,nb:%X\r\n", addr, datalen); /* ������ */
    if (datalen <= secremain)
    {
        secremain = datalen;    /* ������4096���ֽ� */
    }

    while (1)
    {
        norflash_read(norflash_buf, secpos * 4096, 4096);   /* ������������������ */

        for (i = 0; i < secremain; i++)   /* У������ */
        {
            if (norflash_buf[secoff + i] != 0XFF)
            {
                break;          /* ��Ҫ����, ֱ���˳�forѭ�� */
            }
        }

        if (i < secremain)      /* ��Ҫ���� */
        {
            norflash_erase_sector(secpos);    /* ����������� */

            for (i = 0; i < secremain; i++)   /* ���� */
            {
                norflash_buf[i + secoff] = pbuf[i];
            }

            norflash_write_nocheck(norflash_buf, secpos * 4096, 4096);  /* д���������� */
        }
        else        /* д�Ѿ������˵�,ֱ��д������ʣ������. */
        {
            norflash_write_nocheck(pbuf, addr, secremain);  /* ֱ��д���� */
        }

        if (datalen == secremain)
        {
            break;  /* д������� */
        }
        else        /* д��δ���� */
        {
            secpos++;               /* ������ַ��1 */
            secoff = 0;             /* ƫ��λ��Ϊ0 */

            pbuf += secremain;      /* ָ��ƫ�� */
            addr += secremain;      /* д��ַƫ�� */
            datalen -= secremain;   /* �ֽ����ݼ� */

            if (datalen > 4096)
            {
                secremain = 4096;   /* ��һ����������д���� */
            }
            else
            {
                secremain = datalen;/* ��һ����������д���� */
            }
        }
    }
}

/**
 * @brief       ��������оƬ
 *   @note      �ȴ�ʱ�䳬��...
 * @param       ��
 * @retval      ��
 */
void norflash_erase_chip(void)
{
    norflash_write_enable();    /* дʹ�� */
    norflash_wait_busy();       /* �ȴ����� */
  
    /* SPI,����ȫƬ����ָ��,��ַΪ0,0���ֽ�����,������_8λ��ַ_�޵�ַ_1�ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_ChipErase, 0, 0, (0 << 8) | (0 << 6) | (0 << 3) | (1 << 0), 0);
    norflash_wait_busy();       /* �ȴ�оƬ�������� */
}

/**
 * @brief       ����һ������
 *   @note      ע��,������������ַ,�����ֽڵ�ַ!!
 *              ����һ������������ʱ��:150ms
 *
 * @param       saddr : ������ַ ����ʵ����������
 * @retval      ��
 */
void norflash_erase_sector(uint32_t saddr)
{
    //printf("fe:%x\r\n", saddr);   /* ����flash�������,������ */
    saddr *= 4096;
    norflash_write_enable();        /* дʹ�� */
    norflash_wait_busy();           /* �ȴ����� */

    /* SPI,������������ָ��,��ַΪ0,0���ֽ�����,������_24/32λ��ַ_1�ߴ����ַ_1�ߴ���ָ��,�޿����� */
    ospi_send_command(FLASH_SectorErase, saddr, 0, (0 << 8) | (g_norflash_addrw << 6) | (1 << 3) | (1 << 0), 0);

    norflash_wait_busy();           /* �ȴ������������ */
}


 
















