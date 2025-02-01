#include <stdint.h>
  #include "stm32f2xx.h"
  #include "stm32f2xx_conf.h"
#include "my_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
////#include "my_misc.h"
#include "printk.h"

#include "usb_dcd_int.h"

extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#if 1
xQueueHandle q_usb_in = 0;
///usb_req_t
void usb_thread( void *arg )
{
////USB_OTG_CORE_HANDLE *pdev;  
 usb_req_t t_usb_req; 
q_usb_in  = xQueueCreate(80,sizeof(usb_req_t));
for(;;)
  {
  xQueueReceive(q_usb_in,&t_usb_req,portMAX_DELAY);
  if(t_usb_req.in_out==0)       ///out
    USBD_DCD_INT_fops->DataOutStage((void*)t_usb_req.addr_dev , t_usb_req.num_point);
  else  ///in
    USBD_DCD_INT_fops->DataInStage((void*)t_usb_req.addr_dev , t_usb_req.num_point);
 
////  USBD_OTG_ISR_Handler(pdev) ; 
///  _printk("U");
  }
}
#endif

void my_USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev)
{
////signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
///xQueueSendFromISR(q_usb_in, pdev, &xHigherPriorityTaskWoken);
 USBD_OTG_ISR_Handler(pdev) ; 
}
///=======================================================
void USB_OTG_EnableSuspend(USB_OTG_CORE_HANDLE *pdev)
{
USB_OTG_GINTMSK_TypeDef  int_mask;
  
int_mask.d32 = 0;
/* Enable the interrupts in the INTMSK */
int_mask.b.usbsuspend = 1; 
USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, int_mask.d32,NULL);
}
void USB_OTG_DisableSuspend(USB_OTG_CORE_HANDLE *pdev)
{
USB_OTG_GINTMSK_TypeDef  int_mask;
int_mask.d32 = 0;
int_mask.b.usbsuspend = 1; 
USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK,NULL, int_mask.d32);
}




