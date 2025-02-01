/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Header for usbd_cdc_vcp.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    usbd_cdc_vcp.h
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   Header for usbd_cdc_vcp.c file.
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB and
  *          STM32F4DIS-LCD modules.      
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F2XX
 #include "stm32f2xx.h"
#endif /* STM32F2XX */

#include "usbd_cdc_vcp_core.h"
#include "usbd_conf.h"


/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;


#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1
/*
 * VCP data Rx/Tx API
 */

/* Get the total number of data/space bytes available */
extern unsigned VCP_DataAvail(void);
extern unsigned VCP_SpaceAvail(void);

/* Get the number of contiguous data/space bytes available */
extern unsigned VCP_DataAvailContig(void);
extern unsigned VCP_SpaceAvailContig(void);

/* Get/put data from/to contiguous area */
extern unsigned VCP_GetContig(void* buff, unsigned max_len);
extern unsigned VCP_PutContig(void const* buff, unsigned len);

/* Get/put as much as possible */
extern unsigned VCP_Get(void* buff, unsigned max_len);
extern unsigned VCP_Put(void const* buff, unsigned len);

/* Returns pointer to contiguous input data area */
static inline uint8_t const* VCP_DataPtr(void)
{
	return USB_Rx_Buffer + USB_Rx_buff_tail;
}

/* Returns pointer to contiguous output free space area */
static inline uint8_t* VCP_SpacePtr(void)
{
	return USB_Tx_Buffer + USB_Tx_buff_head;
}

/* Mark data as read */
extern void VCP_MarkRead(unsigned sz);

/* Mark space as written */
extern void VCP_MarkWritten(unsigned sz);



#endif /* __USBD_CDC_VCP_H */

/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
