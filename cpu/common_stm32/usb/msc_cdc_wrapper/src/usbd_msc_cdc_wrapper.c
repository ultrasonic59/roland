/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_core.h"
#include "usbd_cdc_vcp_core.h"
#include "usbd_msc_cdc_wrapper.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#if (USB_CLASS==MSC_CDC)

/*********************************************
   CDC Device library callbacks
 *********************************************/
extern uint8_t  usbd_cdc_vcp_Init        (void  *pdev, uint8_t cfgidx);
extern uint8_t  usbd_cdc_vcp_DeInit      (void  *pdev, uint8_t cfgidx);
extern uint8_t  usbd_cdc_vcp_Setup       (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  usbd_cdc_vcp_EP0_RxReady  (void *pdev);
extern uint8_t  usbd_cdc_vcp_DataIn      (void *pdev, uint8_t epnum);
extern uint8_t  usbd_cdc_vcp_DataOut     (void *pdev, uint8_t epnum);
extern uint8_t  usbd_cdc_vcp_SOF         (void *pdev);

extern uint8_t  *USBD_cdc_vcp_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS  
extern uint8_t  *USBD_cdc_vcp_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif

/*********************************************
   MSC Device library callbacks
 *********************************************/
extern uint8_t  USBD_MSC_Init (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_MSC_DeInit (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_MSC_Setup (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  USBD_MSC_DataIn (void  *pdev, uint8_t epnum);
extern uint8_t  USBD_MSC_DataOut (void  *pdev, uint8_t epnum);
extern uint8_t  *USBD_MSC_GetCfgDesc (uint8_t speed, uint16_t *length);

#ifdef USB_OTG_HS_CORE  
extern uint8_t  *USBD_MSC_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif

extern uint8_t USBD_MSC_CfgDesc[USB_MSC_CONFIG_DESC_SIZ];


static uint8_t  USBD_MSC_CDC_Init         (void *pdev , uint8_t cfgidx);
static uint8_t  USBD_MSC_CDC_DeInit       (void *pdev , uint8_t cfgidx);

/* Control Endpoints*/
static uint8_t  USBD_MSC_CDC_Setup        (void *pdev , USB_SETUP_REQ  *req);  
static uint8_t  USBD_MSC_CDC_EP0_RxReady  (void *pdev );  

/* Class Specific Endpoints*/
static uint8_t  USBD_MSC_CDC_DataIn       (void *pdev , uint8_t epnum);   
static uint8_t  USBD_MSC_CDC_DataOut      (void *pdev , uint8_t epnum); 
static uint8_t  USBD_MSC_CDC_SOF          (void *pdev); 



static uint8_t*  USBD_MSC_CDC_GetConfigDescriptor( uint8_t speed , uint16_t *length); 
//#ifdef USB_OTG_HS_CORE 
//static uint8_t*  USBD_MSC_CDC_GetOtherConfigDescriptor( uint8_t speed , uint16_t *length);   
//#endif


///#define USB_MSC_CDC_CONFIG_DESC_SIZ  (USB_MSC_CONFIG_DESC_SIZ -9 + USB_CDC_CONFIG_DESC_SIZ  + 8)
#define USB_MSC_CDC_CONFIG_DESC_SIZ  (USB_MSC_CONFIG_DESC_SIZ -9 + USB_CDC_VCP_CONFIG_DESC_SIZ  + 8)


#define MSC_INTERFACE 0x0
#define CDC_COM_INTERFACE 0x1


USBD_Class_cb_TypeDef  USBD_MSC_CDC_cb = 
{
  USBD_MSC_CDC_Init,
  USBD_MSC_CDC_DeInit,
  USBD_MSC_CDC_Setup,
  NULL, 
  USBD_MSC_CDC_EP0_RxReady, 
  USBD_MSC_CDC_DataIn, 
  USBD_MSC_CDC_DataOut, 
  USBD_MSC_CDC_SOF, 
  NULL,
  NULL,      
  USBD_MSC_CDC_GetConfigDescriptor,
#ifdef USB_OTG_HS_CORE  
  USBD_MSC_CDC_GetConfigDescriptor
#endif  
};


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */ 
/* USB MSC/CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MSC_CDC_CfgDesc[USB_MSC_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{  
  0x09, /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  USB_MSC_CDC_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 3 interfaces (2 for CDC, 1 for MSC)*/
  0x01,         /*bConfigurationValue: Configuration value*/
///  0x00,         /*iConfiguration: Index of string descriptor describing  the configuration*/
  0x02,         /*iConfiguration: Index of string descriptor describing  the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
  /********************  Mass Storage interface ********************/
  0x09,   /* bLength: Interface Descriptor size */
  0x04,   /* bDescriptorType: */
  MSC_INTERFACE,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints*/
  0x08,   /* bInterfaceClass: MSC Class */
  0x06,   /* bInterfaceSubClass : SCSI transparent*/
  0x50,   /* nInterfaceProtocol */
  0X00,          /* iInterface: */
  /********************  Mass Storage Endpoints ********************/
  0x07,   /*Endpoint descriptor length = 7*/
  0x05,   /*Endpoint descriptor type */
  MSC_IN_EP,   /*Endpoint address (IN, address 3) */
  0x02,   /*Bulk endpoint type */
  LOBYTE(MSC_MAX_PACKET),
  HIBYTE(MSC_MAX_PACKET),
  0x00,   /*Polling interval in milliseconds */
  
  0x07,   /*Endpoint descriptor length = 7 */
  0x05,   /*Endpoint descriptor type */
  MSC_OUT_EP,   /*Endpoint address (OUT, address 3) */
  0x02,   /*Bulk endpoint type */
  LOBYTE(MSC_MAX_PACKET),
  HIBYTE(MSC_MAX_PACKET),
  0x00,     /*Polling interval in milliseconds*/
   
   /******** /IAD should be positioned just before the CDC interfaces ******
                IAD to associate the two CDC interfaces */
  0x08, /* bLength */
  0x0B, /* bDescriptorType */
  CDC_INTERFACE_IDX, /* bFirstInterface */
///  0x01, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x00, /* iFunction (Index of string descriptor describing this function) */
  /* 08 bytes */
  
   /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_COM_INTERFACE,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x01,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  CDC_INTERFACE_IDX + 1,   /* bDataInterface: 2 */
/// 0x02,   /* bDataInterface: 2 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  CDC_INTERFACE_IDX,////0x01,   /* bMasterInterface: Communication class interface */
  CDC_INTERFACE_IDX + 1,///0x02,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SZE),
  0x10,///0xFF,                           /* bInterval: */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
  CDC_INTERFACE_IDX + 1,///0x02,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_PACKET_SIZE),///0x40,                             /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_PACKET_SIZE),///0x00,
  0x00,                              /* bInterval */
  
} ;


static uint8_t  USBD_MSC_CDC_Init (void  *pdev, 
                               uint8_t cfgidx)
{
  
  /* MSC initialization */
  USBD_MSC_Init (pdev,cfgidx);
  
  /* CDC initialization */
  usbd_cdc_vcp_Init (pdev,cfgidx);
  
  return USBD_OK;

}

/**
  * @brief  USBD_MSC_CDC_Init
  *         DeInitialize the MSC/CDC interfaces
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MSC_CDC_DeInit (void  *pdev, 
                                 uint8_t cfgidx)
{
  /* MSC De-initialization */
  USBD_MSC_DeInit (pdev,cfgidx);
  
  /* CDC De-initialization */
  usbd_cdc_vcp_DeInit (pdev,cfgidx);
  
  
  return USBD_OK;
}

/**
  * @brief  USBD_MSC_CDC_Setup
  *         Handle the MSC/CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MSC_CDC_Setup (void  *pdev, 
                                USB_SETUP_REQ *req)
{
  
  if (req->wIndex == MSC_INTERFACE)
  {
     return (USBD_MSC_Setup (pdev, req));
  }
  else
  {
    return (usbd_cdc_vcp_Setup(pdev, req));
  }
 
}

/**
  * @brief  USBD_MSC_CDC_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t  *USBD_MSC_CDC_GetConfigDescriptor (uint8_t speed, uint16_t *length)
{
  *length = sizeof (USBD_MSC_CDC_CfgDesc);
  return USBD_MSC_CDC_CfgDesc;
}

/**
  * @brief  USBD_MSC_CDC_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MSC_CDC_DataIn (void  *pdev, 
                                     uint8_t epnum)
{
  /*DataIN can be for CDC or MSC */
  
  if (epnum == (CDC_IN_EP&~0x80) )
  {
    return (usbd_cdc_vcp_DataIn(pdev, epnum));
  }
  else
  {
    return (USBD_MSC_DataIn(pdev, epnum));
  }
}


uint8_t  USBD_MSC_CDC_DataOut(void *pdev , uint8_t epnum)
{
  /*DataOut can be for MSC or CDC */
  
  if (epnum == (CDC_OUT_EP&~0x80) )
  {
    return (usbd_cdc_vcp_DataOut(pdev, epnum));
  }
  else
  {
    return (USBD_MSC_DataOut(pdev, epnum));
  }   
}


uint8_t  USBD_MSC_CDC_SOF (void *pdev)
{
  /*SOF processing needed for CDC */
  return (usbd_cdc_vcp_SOF(pdev));
}


uint8_t  USBD_MSC_CDC_EP0_RxReady  (void *pdev )
{
  /*RxReady processing needed for CDC only*/
  return (usbd_cdc_vcp_EP0_RxReady(pdev));
}

#endif

