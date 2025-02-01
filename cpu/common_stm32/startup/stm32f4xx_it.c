/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void gpio_irq(void);
extern void dma_irq(void);
extern void usart1_irq(void);
extern void usart2_irq(void);
extern void sdio_irq(void);

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__weak void SVC_Handler(void)
{
    vPortSVCHandler();

}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__weak void PendSV_Handler(void)
{
    xPortPendSVHandler();

}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__weak void SysTick_Handler(void)
{
  xPortSysTickHandler();

}

/**
  * @brief  This function handles EXTI 3 interrupt request.
  * @param  None
  * @retval None
  */
__weak void EXTI9_5_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI 15-10 interrupt request.
  * @param  None
  * @retval None
  */
__weak void EXTI15_10_IRQHandler(void)
{
}

void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
 sdio_irq();
}
extern void sdio_dma_irq(void);
void DMA2_Stream3_IRQHandler(void)
{
sdio_dma_irq();
}
void USART1_IRQHandler(void)
{  
usart1_irq();
}
extern void usart1_tx_dma_irq(void);
void DMA2_Stream7_IRQHandler(void)
{ 
usart1_tx_dma_irq();
}
extern void usart1_rx_dma_irq(void);
void DMA2_Stream2_IRQHandler(void)
{ 
usart1_rx_dma_irq();
}
/*EXTI ISR*/
void EXTI0_IRQHandler(void)
{
gpio_irq();//SDIO OOB interrupt
}

void EXTI1_IRQHandler(void)
{
    gpio_irq();
}

void EXTI2_IRQHandler(void)
{
    gpio_irq();
}

void EXTI3_IRQHandler(void)
{
    gpio_irq();//User defined external interrupt, EMW3162 button 1: PA3
}

void EXTI4_IRQHandler(void)
{
    gpio_irq();
}

#include "usb_core.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern void my_USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
///extern xQueueHandle q_usb_in;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

///USB_OTG_CORE_HANDLE           *pUSB_OTG_dev;
/**
* @brief  This function handles OTG_HS Handler.
* @param  None
* @retval None
*/
void OTG_FS_IRQHandler(void)
{
USBD_OTG_ISR_Handler(&USB_OTG_dev) ; 
  
/// my_USBD_OTG_ISR_Handler (&USB_OTG_dev);
#if 0  
///signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
////pUSB_OTG_dev=&USB_OTG_dev;
////xQueueSendFromISR(q_usb_in, &pUSB_OTG_dev, &xHigherPriorityTaskWoken);
my_USBD_OTG_ISR_Handler (&USB_OTG_dev);
#endif  
}




