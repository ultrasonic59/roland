/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/stm32f2xx_conf.h  
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Library configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F2xx_CONF_H
#define __STM32F2xx_CONF_H

#include "stm32f2xx_can.h" 
#include "stm32f2xx_dma.h"
#include "stm32f2xx_exti.h"
#include "stm32f2xx_flash.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_iwdg.h"
#include "stm32f2xx_pwr.h" 
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_rtc.h" 
#include "stm32f2xx_spi.h"
#include "stm32f2xx_syscfg.h"
#include "stm32f2xx_tim.h" 
#include "stm32f2xx_usart.h"
/* #include "stm32f2xx_wwdg.h" */
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */


/* Exported macro ------------------------------------------------------------*/
#ifdef  DEBUG
  extern void assert_failed( unsigned char* pcFile, unsigned long ulLine );
////  extern void assert_param(unsigned long ulLine);
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F2xx_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
