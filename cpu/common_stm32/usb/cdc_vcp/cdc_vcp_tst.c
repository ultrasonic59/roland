#include "usbd_conf.h"

#if(USB_CLASS == CDC_VCP)///================================

#include "rook_v3a.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
   
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;


int main_cdc_vcp_tst(void)
{
  __IO uint32_t i = 0;	  
_printk("\r\n == main_cdc_vcp_tst ==");  

  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */  
 
  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
  
  /* Main loop */
  while (1)
  {
    if (i++ >= 0x100000)
    {
 ////     STM_EVAL_LEDToggle(LED4);
      i = 0;
    }
  }
} 
#endif ///(USB_CLASS == CDC_VCP)