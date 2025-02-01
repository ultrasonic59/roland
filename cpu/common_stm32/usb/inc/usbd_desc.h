/**
  ******************************************************************************
  * @file    usbd_desc.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief   header file for the usbd_desc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_req.h"


/** @defgroup USB_DESC_Exported_Defines
  * @{
  */
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_SIZ_DEVICE_DESC                     18
#define USB_SIZ_STRING_LANGID                   4

#define         DEVICE_ID1          (0x1FFF7A10)
#define         DEVICE_ID2          (0x1FFF7A14)
#define         DEVICE_ID3          (0x1FFF7A18)

#define  USB_SIZ_STRING_SERIAL       0x1A

 //// #define DEVIECE_CLASS                         0x02
 //// #define DEVIECE_SUBCLASS                      0x02
///  #define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_MANUFACTURER_STRING      "STM"

///==================================================
///#define USBD_VID                     0x0483
///#define USBD_PID                     0x5750
///==================================================
#define USBD_LANGID_STRING            0x409
#if (USB_CLASS==RNDIS)
  #define USBD_VID                     0x0483
  #define USBD_PID                     0x0125
  #define DEVIECE_CLASS                0x02
  #define DEVIECE_SUBCLASS             0x00
  #define USBD_MANUFACTURER_STRING        "Fetisov Sergey"
  #define USBD_PRODUCT_HS_STRING          "STM32F4 RNDIS"
  #define USBD_SERIALNUMBER_HS_STRING     "00000000123B"
  #define USBD_PRODUCT_FS_STRING          "STM32F4 RNDIS"
  #define USBD_SERIALNUMBER_FS_STRING     "00000000123C"
  #define USBD_CONFIGURATION_HS_STRING    "RNDIS Config"
  #define USBD_INTERFACE_HS_STRING        "RNDIS Interface"
  #define USBD_CONFIGURATION_FS_STRING    "RNDIS Config"
  #define USBD_INTERFACE_FS_STRING        "RNDIS Interface"

#elif (USB_CLASS==CUSTOMHID)
  #define USBD_PRODUCT_HS_STRING        "Custome HID in HS Mode"
  #define USBD_PRODUCT_FS_STRING        "Custome HID in FS Mode"
  #define USBD_CONFIGURATION_HS_STRING  "HID Config"
  #define USBD_INTERFACE_HS_STRING      "HID Interface"
  #define USBD_CONFIGURATION_FS_STRING  "HID Config"
  #define USBD_INTERFACE_FS_STRING      "HID Interface"
#elif (USB_CLASS==MSC)
  #define USBD_VID                     0x0483
  #define USBD_PID                     0x5740
  #define USBD_PRODUCT_HS_STRING        "Mass Storage in HS Mode"
  #define USBD_PRODUCT_FS_STRING        "Mass Storage in FS Mode"
  #define USBD_CONFIGURATION_HS_STRING  "MSC Config"
  #define USBD_INTERFACE_HS_STRING      "MSC Interface"
  #define USBD_CONFIGURATION_FS_STRING  "MSC Config"
  #define USBD_INTERFACE_FS_STRING      "MSC Interface"
#elif (USB_CLASS==MSC_CDC)
  #define USBD_VID                     0x0483
  #define USBD_PID                     0x5742
  #define USBD_PRODUCT_HS_STRING        "Composite MSC CDC"
  #define USBD_PRODUCT_FS_STRING        "Composite MSC CDC"
  #define USBD_CONFIGURATION_HS_STRING  "Config"
  #define USBD_INTERFACE_HS_STRING      "Interface"
  #define USBD_CONFIGURATION_FS_STRING  "Config"
  #define USBD_INTERFACE_FS_STRING      "Interface"
  #ifdef USE_USB_OTG_HS
    #define CDC_IN_FRAME_INTERVAL          40   /* Number of micro-frames between IN transfers */
  #else
    #define CDC_IN_FRAME_INTERVAL          1    ////???5    /* Number of frames between IN transfers */
  #endif /* USE_USB_OTG_HS */

#elif   (USB_CLASS==CDC_VCP)
  #define USBD_VID                     0x03EB
  #define USBD_PID                     0x6124
////  #define USBD_MANUFACTURER_STRING      "STMicroelectronics"
  #define USBD_PRODUCT_HS_STRING          "VCP in HS mode"
  #define USBD_SERIALNUMBER_HS_STRING     "00000000059B"

  #define USBD_PRODUCT_FS_STRING          "STM Virtual COM Port"
  #define USBD_SERIALNUMBER_FS_STRING     "00000000059D"

  #define DEVIECE_CLASS                         0x02
  #define DEVIECE_SUBCLASS                      0x02
  #define USBD_CONFIGURATION_HS_STRING    "VCP Config"
  #define USBD_INTERFACE_HS_STRING        "VCP Interface"

  #define USBD_CONFIGURATION_FS_STRING    "VCP Config"
  #define USBD_INTERFACE_FS_STRING        "VCP Interface"

  #define USB_MAX_STR_DESC_SIZ            255 
  
 /// #define CDC_IN_EP                       0x81  /* EP1 for data IN */
 /// #define CDC_OUT_EP                      0x01  /* EP1 for data OUT */
///  #define CDC_CMD_EP                      0x82  /* EP2 for CDC commands */
  #ifdef USE_USB_OTG_HS
 ///   #define CDC_DATA_MAX_PACKET_SIZE       64      ///512  /* Endpoint IN & OUT Packet size */

    #define CDC_IN_FRAME_INTERVAL          40   /* Number of micro-frames between IN transfers */
  #else
 ///   #define CDC_DATA_MAX_PACKET_SIZE       64   /* Endpoint IN & OUT Packet size */
    #define CDC_IN_FRAME_INTERVAL          1    ////???5    /* Number of frames between IN transfers */
  #endif /* USE_USB_OTG_HS */

////  #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */
////  #define APP_RX_DATA_SIZE               2048 /* Total size of IN buffer: */
                                            /*  APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL */
////  #define APP_FOPS                        VCP_fops
#endif 

  #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */
   #define CDC_DATA_MAX_PACKET_SIZE       64      ///512  /* Endpoint IN & OUT Packet size */
  #define APP_RX_DATA_SIZE               1024           ///2048 ///1024           /// 2048 /* Total size of IN buffer: */
#ifdef USE_USB_OTG_HS
    #define CDC_IN_FRAME_INTERVAL          40   /* Number of micro-frames between IN transfers */
    #define USB_TX_BUFF_SIZE               256  /* Total size of IN buffer */
    #define USB_RX_BUFF_SIZE               256  /* Total size of OUT buffer */

#else
///    #define CDC_IN_FRAME_INTERVAL          1    ////???5    /* Number of frames between IN transfers */
    #define USB_TX_BUFF_SIZE               256  /* Total size of IN buffer */
    #define USB_RX_BUFF_SIZE               256  /* Total size of OUT buffer */

#endif /* USE_USB_OTG_HS */
  #define APP_FOPS                        VCP_fops

extern  uint8_t USBD_DeviceDesc  [USB_SIZ_DEVICE_DESC];
extern  uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];
extern  uint8_t USBD_OtherSpeedCfgDesc[USB_LEN_CFG_DESC]; 
extern  uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];
extern  uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID];
extern  USBD_DEVICE USR_desc; 

uint8_t *     USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ManufacturerStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ProductStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_USR_USRStringDesc (uint8_t speed, uint8_t idx , uint16_t *length);  
#endif /* USB_SUPPORT_USER_STRING_DESC */  
  
#endif /* __USBD_DESC_H */

