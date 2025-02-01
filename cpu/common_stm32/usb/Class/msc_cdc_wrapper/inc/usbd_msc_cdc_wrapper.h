/**
  ******************************************************************************
  * @file    usbd_msc_cdc_wrapper.h
  * @author  MCD Application Team
  * @version V0.0.1
  * @date    11-June-2013
  * @brief   header file for the usbd_msc_cdc_wrapper.c file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#ifndef __USB_MSC_CDC_WRAPPER_H_
#define __USB_MSC_CDC_WRAPPER_H_

#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_MSC_CDC
  * @brief This file is the Header file for usbd_msc_cdc_wrapper.c
  * @{
  */ 


/** @defgroup USBD_CDC_MSC_Exported_Defines
  * @{
  */ 


/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */


/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_Class_cb_TypeDef  USBD_MSC_CDC_cb;
/**
  * @}
  */ 

/**
  * @}
  */ 

#endif  // __USB_MSC_CDC_WRAPPER_H_
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
