
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__
#include "usb_conf.h"
///===============================
#define RNDIS           1
#define CDC_VCP         2
#define CDC_ECM         3
#define CUSTOMHID       4
#define MSC             5
#define MSC_CDC         6

///===============================
///#define USB_CLASS MSC_CDC
#define USB_CLASS CDC_VCP
///#define USB_CLASS RNDIS
/// #define USB_CUSTOMHID
/// #define USB_CLASS MSC
///===============================


#define USBD_CFG_MAX_NUM                1

#define USBD_SELF_POWERED               
#define USB_MAX_STR_DESC_SIZ            255 
///=======================================
  
#if (USB_CLASS==MSC_CDC)
  #define MSC_INTERFACE_IDX 0x0                            	// Index of MSC interface
  #define CDC_INTERFACE_IDX 0x1                            	// Index of CDC interface
 
// endpoints numbers
  #define MSC_EP_IDX                      0x01
  #define CDC_CMD_EP_IDX                  0x02
  #define CDC_EP_IDX                      0x03

  #define IN_EP_DIR			0x80 // Adds a direction bit

  #define MSC_OUT_EP                      MSC_EP_IDX                  /* EP1 for BULK OUT */
  #define MSC_IN_EP                       MSC_EP_IDX | IN_EP_DIR      /* EP1 for BULK IN */
  #define CDC_CMD_EP                      CDC_CMD_EP_IDX| IN_EP_DIR   /* EP2 for CDC commands */
  #define CDC_OUT_EP                      CDC_EP_IDX                  /* EP3 for data OUT */
  #define CDC_IN_EP                       CDC_EP_IDX | IN_EP_DIR      /* EP3 for data IN */
  #define USBD_ITF_MAX_NUM                3
#else
  #define MSC_OUT_EP                      0x01                  /* EP1 for BULK OUT */
  #define MSC_IN_EP                       0x81            /* EP1 for BULK IN */
  #define CDC_CMD_EP                      0x82            /* EP2 for CDC commands */
  #define CDC_OUT_EP                      0x01            /* EP3 for data OUT */
  #define CDC_IN_EP                       0x81            /* EP3 for data IN */
  #define USBD_ITF_MAX_NUM                1

#endif
///=======================================
///=======================================

#define MSC_MAX_PACKET               64
#define MSC_MEDIA_PACKET             4096

///===========================================

#if (USB_CLASS==RNDIS)
#define RNDIS_NOTIFICATION_IN_EP 0x81
#define RNDIS_DATA_IN_EP         0x82
#define RNDIS_DATA_OUT_EP        0x03

#define RNDIS_NOTIFICATION_IN_SZ 0x08
#define RNDIS_DATA_IN_SZ         0x40
#define RNDIS_DATA_OUT_SZ        0x40

#endif

#define CDC_DATA_PACKET_SIZE     64

#endif //__USBD_CONF__H__

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

