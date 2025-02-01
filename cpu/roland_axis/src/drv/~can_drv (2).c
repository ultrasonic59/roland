#include "stdint.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_can.h"
#include "misc.h"
#include "board.h"
#include "printk.h"
////=======================================================
////#define CANx CAN1
#define MAX_CAN_DATA_LEN 8

////=======================================================
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
volatile TestStatus TestRx;
__IO uint32_t ret=0;
uint8_t  CAN_TxRdy = 0;              /* CAN HW ready to transmit message */
uint8_t  CAN_RxRdy = 0;              /* CAN HW received a message        */
CanRxMsg RxMessage;

////=============================================================
void NVIC_can_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;

NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}
void can1_init(void)
{
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  
GPIO_InitTypeDef GPIO_InitStructure;
////============== CAN1_INH ============================
RCC_AHB1PeriphClockCmd(CAN1_INH_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CAN1_INH_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( CAN1_INH_PIN_GPIO, &GPIO_InitStructure );
GPIO_ResetBits(CAN1_INH_PIN_GPIO, CAN1_INH_PIN);

////============== CAN1_RX ============================
RCC_AHB1PeriphClockCmd(CAN1_RX_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init(CAN1_RX_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(CAN1_RX_PIN_GPIO, CAN1_RX_PIN_SOURCE, GPIO_AF_CAN1);
////============== CAN1_TX ============================
RCC_AHB1PeriphClockCmd(CAN1_TX_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init(CAN1_TX_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(CAN1_TX_PIN_GPIO, CAN1_TX_PIN_SOURCE, GPIO_AF_CAN1);
////================================================================
RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
NVIC_can_Config();

  /* CAN register init */
  CAN_DeInit(CAN1);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;///CAN_Mode_LoopBack;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  
  /* Baudrate = 500 Kbps */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 4;
  CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}
////==================================================================
uint8_t can1_send(uint16_t id,uint8_t data_len,uint8_t *data)
{
CanTxMsg TxMessage;
uint8_t ii ;
  /* transmit 1 message */
  TxMessage.StdId = 0;
  TxMessage.ExtId = id;
  TxMessage.IDE = CAN_ID_EXT;
  TxMessage.RTR = CAN_RTR_DATA;
  if(data_len>MAX_CAN_DATA_LEN)
    {
    data_len=MAX_CAN_DATA_LEN ; 
    }
  TxMessage.DLC = data_len;
  for(ii=0;ii<data_len;ii++)
    {
    TxMessage.Data[ii] = data[ii];
    }
  CAN_Transmit(CAN1, &TxMessage);
  return data_len;
}

TestStatus CAN_Interrupt(void)
{
  CanTxMsg TxMessage;
  uint32_t i = 0;


  /* transmit 1 message */
  TxMessage.StdId = 0;
  TxMessage.ExtId = 0x1234;
  TxMessage.IDE = CAN_ID_EXT;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 2;
  TxMessage.Data[0] = 0xDE;
  TxMessage.Data[1] = 0xCA;
  CAN_Transmit(CAN1, &TxMessage);

  /* initialize the value that will be returned */
  ret = 0xFF;
       
  /* receive message with interrupt handling */
  i = 0;
  while((ret ==  0xFF) && (i < 0xFFF))
  {
    i++;
  }
  
  if (i ==  0xFFF)
  {
    ret = 0;  
  }

  /* disable interrupt handling */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);

  return (TestStatus)ret;
}

////==================================================================
int can_main(void)
{
RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
NVIC_can_Config();
////=========================================
__enable_irq();
////========================================
#if 0
TestRx = CAN_Polling();
if (TestRx !=  FAILED)
  { /* OK */
  printk("\r\n CAN_Polling OK");
  }
  else
  { /* KO */
  printk("\r\n CAN_Polling BAD");
  }
#endif
  TestRx = CAN_Interrupt();
if (TestRx !=  FAILED)
  { /* OK */

   printk("\r\n Interrupt OK");
  }
  else
  { /* KO */
   printk("\r\n Interrupt BAD");
  }

  /* Infinite loop */
  while (1)
  {
    
  }

return 0;
}


////==================================================================