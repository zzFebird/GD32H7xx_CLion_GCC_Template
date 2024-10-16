/*!
    \file    cdc_acm_core.c
    \brief   CDC ACM driver

    \version 2024-01-05, V1.2.0, firmware for GD32H7xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "cdc_acm_core.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"

#define USBD_VID                          0x28E9U
#define USBD_PID                          0x018AU

extern usb_core_driver cdc_acm;

/* usb_printf发送缓冲区, 用于vsprintf */
uint8_t g_usb_usart_printf_buffer[USB_USART_REC_LEN];

/* USB接收的数据缓冲区,最大USART_REC_LEN个字节 */
uint8_t g_usb_rx_buffer[USB_USART_REC_LEN];

/* 用类似串口0接收数据的方法,来处理USB虚拟串口接收到的数据 */
uint8_t g_usb_usart_rx_buffer[USB_USART_REC_LEN];       /* 接收缓冲,最大USART_REC_LEN个字节 */


/* 接收状态
 * bit15   , 接收完成标志
 * bit14   , 接收到0x0d
 * bit13~0 , 接收到的有效字节数目
 */
uint16_t g_usb_usart_rx_sta=0;  /* 接收状态标记 */

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
__ALIGN_BEGIN const usb_desc_dev cdc_dev_desc __ALIGN_END = {
    .header =
    {
        .bLength          = USB_DEV_DESC_LEN,
        .bDescriptorType  = USB_DESCTYPE_DEV,
    },
    .bcdUSB                = 0x0200U,
    .bDeviceClass          = USB_CLASS_CDC,
    .bDeviceSubClass       = 0x00U,
    .bDeviceProtocol       = 0x00U,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = USBD_VID,
    .idProduct             = USBD_PID,
    .bcdDevice             = 0x0100U,
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM,
};

/* USB device configuration descriptor */
__ALIGN_BEGIN const usb_cdc_desc_config_set cdc_config_desc __ALIGN_END = {
    .config =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_config),
            .bDescriptorType = USB_DESCTYPE_CONFIG,
        },
        .wTotalLength         = USB_CDC_ACM_CONFIG_DESC_SIZE,
        .bNumInterfaces       = 0x02U,
        .bConfigurationValue  = 0x01U,
        .iConfiguration       = 0x00U,
        .bmAttributes         = 0x80U,
        .bMaxPower            = 0x32U
    },

    .cmd_itf =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = 0x00U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = USB_CLASS_CDC,
        .bInterfaceSubClass   = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol   = USB_CDC_PROTOCOL_AT,
        .iInterface           = 0x00U
    },

    .cdc_header =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_header_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x00U,
        .bcdCDC              = 0x0110U
    },

    .cdc_call_managment =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_call_managment_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x01U,
        .bmCapabilities      = 0x00U,
        .bDataInterface      = 0x01U
    },

    .cdc_acm =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_acm_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x02U,
        .bmCapabilities      = 0x02U,
    },

    .cdc_union =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_union_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x06U,
        .bMasterInterface    = 0x00U,
        .bSlaveInterface0    = 0x01U,
    },

    .cdc_cmd_endpoint =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP,
        },
        .bEndpointAddress    = CDC_CMD_EP,
        .bmAttributes        = USB_EP_ATTR_INT,
        .wMaxPacketSize      = USB_CDC_CMD_PACKET_SIZE,
        .bInterval           = 0x0AU
    },

    .cdc_data_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF,
        },
        .bInterfaceNumber    = 0x01U,
        .bAlternateSetting   = 0x00U,
        .bNumEndpoints       = 0x02U,
        .bInterfaceClass     = USB_CLASS_DATA,
        .bInterfaceSubClass  = 0x00U,
        .bInterfaceProtocol  = USB_CDC_PROTOCOL_NONE,
        .iInterface          = 0x00U
    },

    .cdc_out_endpoint =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP,
        },
        .bEndpointAddress     = CDC_DATA_OUT_EP,
        .bmAttributes         = USB_EP_ATTR_BULK,
        .wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
        .bInterval            = 0x00U
    },

    .cdc_in_endpoint =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress     = CDC_DATA_IN_EP,
        .bmAttributes         = USB_EP_ATTR_BULK,
        .wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
        .bInterval            = 0x00U
    }
};

/* USB language ID Descriptor */
__ALIGN_BEGIN static const usb_desc_LANGID usbd_language_id_desc __ALIGN_END = {
    .header =
    {
        .bLength         = sizeof(usb_desc_LANGID),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .wLANGID              = ENG_LANGID
};

/* USB manufacture string */
__ALIGN_BEGIN static const usb_desc_str manufacturer_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(10U),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'G', 'i', 'g', 'a', 'D', 'e', 'v', 'i', 'c', 'e'}
};

/* USB product string */
__ALIGN_BEGIN static const usb_desc_str product_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(12U),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'G', 'D', '3', '2', '-', 'C', 'D', 'C', '_', 'A', 'C', 'M'}
};

/* USB serial string */
__ALIGN_BEGIN static usb_desc_str serial_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(12U),
        .bDescriptorType = USB_DESCTYPE_STR,
    }
};

/* USB string descriptor set */
void *const usbd_cdc_strings[] = {
    [STR_IDX_LANGID]  = (uint8_t *) &usbd_language_id_desc,
    [STR_IDX_MFC]     = (uint8_t *) &manufacturer_string,
    [STR_IDX_PRODUCT] = (uint8_t *) &product_string,
    [STR_IDX_SERIAL]  = (uint8_t *) &serial_string
};

usb_desc cdc_desc = {
    .dev_desc    = (uint8_t *) &cdc_dev_desc,
    .config_desc = (uint8_t *) &cdc_config_desc,
    .strings     = usbd_cdc_strings
};

/* local function prototypes ('static') */
static uint8_t cdc_acm_init(usb_dev *udev, uint8_t config_index);
static uint8_t cdc_acm_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t cdc_acm_req(usb_dev *udev, usb_req *req);
static uint8_t cdc_ctlx_out(usb_dev *udev);
static uint8_t cdc_acm_in(usb_dev *udev, uint8_t ep_num);
static uint8_t cdc_acm_out(usb_dev *udev, uint8_t ep_num);

/* USB CDC device class callbacks structure */
usb_class_core cdc_class = {
    .command   = NO_CMD,
    .alter_set = 0U,

    .init      = cdc_acm_init,
    .deinit    = cdc_acm_deinit,
    .req_proc  = cdc_acm_req,

    .ctlx_out  = cdc_ctlx_out,
    .data_in   = cdc_acm_in,
    .data_out  = cdc_acm_out
};

/*!
    \brief      check CDC ACM is ready for data transfer
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     0 if CDC is ready, 5 else
*/
uint8_t cdc_acm_check_ready(usb_dev *udev)
{
    if(udev->dev.class_data[CDC_COM_INTERFACE] != NULL) {
        usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

        if((1U == cdc->packet_receive) && (1U == cdc->packet_sent)) {
            return 0U;
        }
    }

    return 1U;
}

/*!
    \brief      send CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_send(usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    if(0U != cdc->receive_length) {
        cdc->packet_sent = 0U;

        usbd_ep_send(udev, CDC_DATA_IN_EP, (uint8_t *)(cdc->data), cdc->receive_length);

        cdc->receive_length = 0U;
    }
}

/*!
    \brief      receive CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_receive(usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    cdc->packet_receive = 0U;
    cdc->packet_sent = 0U;

    usbd_ep_recev(udev, CDC_DATA_OUT_EP, (uint8_t *)(cdc->data), USB_CDC_DATA_PACKET_SIZE);
}

/*!
    \brief      initialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_init(usb_dev *udev, uint8_t config_index)
{
    static usb_cdc_handler cdc_handler;

    /* initialize the data TX endpoint */
    usbd_ep_setup(udev, &(cdc_config_desc.cdc_in_endpoint));

    /* initialize the data RX endpoint */
    usbd_ep_setup(udev, &(cdc_config_desc.cdc_out_endpoint));

    /* initialize the command TX endpoint */
    usbd_ep_setup(udev, &(cdc_config_desc.cdc_cmd_endpoint));

    /* initialize CDC handler structure */
    cdc_handler.packet_receive = 1U;
    cdc_handler.packet_sent = 1U;
    cdc_handler.receive_length = 0U;

    cdc_handler.line_coding = (acm_line) {
        .dwDTERate   = 115200,                /* 波特率 */
        .bCharFormat = 0,                     /* 停止位,默认1位 */
        .bParityType = 0,                     /* 校验位,默认无 */
        .bDataBits   = 0x08                   /* 数据位,默认8位 */
    };

    udev->dev.class_data[CDC_COM_INTERFACE] = (void *)&cdc_handler;

    return USBD_OK;
}

/*!
    \brief      de-initialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_deinit(usb_dev *udev, uint8_t config_index)
{
    /* deinitialize the data TX/RX endpoint */
    usbd_ep_clear(udev, CDC_DATA_IN_EP);
    usbd_ep_clear(udev, CDC_DATA_OUT_EP);

    /* deinitialize the command TX endpoint */
    usbd_ep_clear(udev, CDC_CMD_EP);

    return USBD_OK;
}

/*!
    \brief      handle the CDC ACM class-specific requests
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_req(usb_dev *udev, usb_req *req)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    usb_transc *transc = NULL;

    switch(req->bRequest) {
    case SEND_ENCAPSULATED_COMMAND:
        /* no operation for this driver */
        break;

    case GET_ENCAPSULATED_RESPONSE:
        /* no operation for this driver */
        break;

    case SET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case GET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case CLEAR_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case SET_LINE_CODING:
        transc = &udev->dev.transc_out[0];
        /* set the value of the current command to be processed */
        udev->dev.class_core->alter_set = req->bRequest;

        /* enable EP0 prepare to receive command data packet */
        transc->remain_len = req->wLength;
        transc->xfer_buf = cdc->cmd;
        break;

    case GET_LINE_CODING:
        transc = &udev->dev.transc_in[0];

        cdc->cmd[0] = (uint8_t)(cdc->line_coding.dwDTERate);
        cdc->cmd[1] = (uint8_t)(cdc->line_coding.dwDTERate >> 8);
        cdc->cmd[2] = (uint8_t)(cdc->line_coding.dwDTERate >> 16);
        cdc->cmd[3] = (uint8_t)(cdc->line_coding.dwDTERate >> 24);
        cdc->cmd[4] = cdc->line_coding.bCharFormat;
        cdc->cmd[5] = cdc->line_coding.bParityType;
        cdc->cmd[6] = cdc->line_coding.bDataBits;

        transc->xfer_buf = cdc->cmd;
        transc->remain_len = 7U;
        break;

    case SET_CONTROL_LINE_STATE:
        /* no operation for this driver */
        break;

    case SEND_BREAK:
        /* no operation for this driver */
        break;

    default:
        break;
    }

    return USBD_OK;
}

static uint8_t cdc_ctlx_out(usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    if(NO_CMD != udev->dev.class_core->alter_set) {
        /* process the command data */
        cdc->line_coding.dwDTERate = (uint32_t)((uint32_t)cdc->cmd[0] |
                                                ((uint32_t)cdc->cmd[1] << 8U) |
                                                ((uint32_t)cdc->cmd[2] << 16U) |
                                                ((uint32_t)cdc->cmd[3] << 24U));

        cdc->line_coding.bCharFormat = cdc->cmd[4];
        cdc->line_coding.bParityType = cdc->cmd[5];
        cdc->line_coding.bDataBits = cdc->cmd[6];

        udev->dev.class_core->alter_set = NO_CMD;
    }

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_in(usb_dev *udev, uint8_t ep_num)
{
    usb_transc *transc = &udev->dev.transc_in[EP_ID(ep_num)];

    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    if((0U == transc->xfer_len % transc->max_len) && (0U != transc->xfer_len)) {
        usbd_ep_send(udev, ep_num, NULL, 0U);
    } else {
        cdc->packet_sent = 1U;
    }

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_out(usb_dev *udev, uint8_t ep_num)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    cdc->packet_receive = 1U;
    cdc->receive_length = ((usb_core_driver *)udev)->dev.transc_out[ep_num].xfer_count;

    cdc_vcp_data_rx(g_usb_rx_buffer, cdc->receive_length);

    usbd_ep_recev (udev, CDC_DATA_OUT_EP, g_usb_rx_buffer, USB_USART_REC_LEN);
  
    return USBD_OK;
}

/**
 * @brief       处理从 USB 虚拟串口接收到的数据
 * @param       buf     : 接收数据缓冲区
 * @param       Len     : 接收到的数据长度
 * @retval      无
 */
void cdc_vcp_data_rx (uint8_t *buf, uint32_t Len)
{
    uint8_t i;
    uint8_t res;

    for (i = 0; i < Len; i++)
    {
        res = buf[i];

        if ((g_usb_usart_rx_sta & 0x8000) == 0)     /* 接收未完成 */
        {
            if (g_usb_usart_rx_sta & 0x4000)        /* 接收到了0x0D */
            {
                if (res != 0x0a)
                {
                    g_usb_usart_rx_sta = 0;         /* 接收错误,重新开始 */
                }
                else
                {
                    g_usb_usart_rx_sta |= 0x8000;   /* 接收完成了 */
                }
            }
            else    /* 还没收到0X0D */
            {
                if (res == 0x0d)
                {
                    g_usb_usart_rx_sta |= 0x4000;   /* 标记接收到了0X0D */
                }
                else
                {
                    g_usb_usart_rx_buffer[g_usb_usart_rx_sta & 0X3FFF] = res;
                    g_usb_usart_rx_sta++;

                    if (g_usb_usart_rx_sta > (USB_USART_REC_LEN - 1))
                    {
                        g_usb_usart_rx_sta = 0;     /* 接收数据溢出 重新开始接收 */
                    }
                }
            }
        }
    }
}

/**
 * @brief       通过 USB 发送数据
 * @param       data     : 要发送的数据缓冲区
 * @param       Len      : 数据长度
 * @retval      无
 */
void cdc_vcp_data_tx(uint8_t *data, uint32_t Len)
{
    usbd_ep_send (&cdc_acm, CDC_DATA_IN_EP, data, Len);
    delay_ms(10);   /* 等待一定时间再进行下一次发送 */
}

/**
 * @brief       USB虚拟串口的printf函数
 * @note        通过USB VCP实现printf输出
 *              确保一次发送数据长度不超过USB_USART_REC_LEN字节
 * @param       格式化输出
 * @retval      无
 */
void usb_printf(char *fmt, ...)
{
    uint16_t i;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)g_usb_usart_printf_buffer, fmt, ap);
    va_end(ap);
    i = strlen((const char *)g_usb_usart_printf_buffer);    /* 此次发送数据的长度 */
    cdc_vcp_data_tx(g_usb_usart_printf_buffer, i);          /* 发送数据 */          
}





