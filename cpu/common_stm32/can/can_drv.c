#include "stdint.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_can.h"
#include "misc.h"
#include "board.h"
#include "can.h"
#include "snail_can_cmds.h"
#include "can_cmds.h"
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

can_br_coef_t can_br30[]=
{
  {150,CAN_BS1_15tq,CAN_BS2_5tq},  ////10 kb
  {75,CAN_BS1_15tq,CAN_BS2_5tq},  ////20 kb
  {30,CAN_BS1_15tq,CAN_BS2_5tq},  ////50 kb
  {30,CAN_BS1_8tq,CAN_BS2_2tq},  ////100 kb
  {15,CAN_BS1_14tq,CAN_BS2_2tq},  ////125 kb
  {15,CAN_BS1_7tq,CAN_BS2_1tq},  ////250 kb
  {6,CAN_BS1_7tq,CAN_BS2_2tq},  ////500 kb
  {3,CAN_BS1_11tq,CAN_BS2_2tq},  ////800 kb
  {2,CAN_BS1_13tq,CAN_BS2_2tq}  ////1000 kb
};


CanRxMsg RxMessage;

////=============================================================
void NVIC_can_Config(void)
{
NVIC_InitTypeDef NVIC_InitStructure;

NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;///0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;///0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

}
#if 0
void _CAN_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
 //// CAN_FilterInitTypeDef  CAN_FilterInitStructure;
////=============== CAN1_INH ============================
RCC_AHB1PeriphClockCmd(CAN1_INH_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CAN1_INH_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( CAN1_INH_PIN_GPIO, &GPIO_InitStructure );
GPIO_ResetBits(CAN1_INH_PIN_GPIO, CAN1_INH_PIN);
 /* CAN GPIOs configuration **************************************************/
  /* Enable GPIO clock */
RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT);

  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN | CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
  /* CAN configuration ********************************************************/
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
  NVIC_can_Config();

  /* CAN register init */
  CAN_DeInit(CAN1);

  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = ENABLE;          ////off autoresend
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
  
   /* Baudrate = 500 Kbps */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CAN1, &CAN_InitStructure);
#if 0
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;////CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  //CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
#endif
#if 0
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  //CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
#endif

  /* Enable FIFO 0 message pending Interrupt */
////  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}
#endif

///=============================================

can_msg_t       CAN_RxMsg;                  /* CAN message for receiving        */                        
/*----------------------------------------------------------------------------
  write a message to CAN peripheral and transmit it
 *----------------------------------------------------------------------------*/
void CAN_wrMsg (can_msg_t *msg)  {

  CAN1->sTxMailBox[0].TIR  = 0;           /* Reset TIR register               */
                                          /* Setup identifier information     */
  if (msg->format == STANDARD_FORMAT) {   /*    Standard ID                   */
    CAN1->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21) | CAN_ID_STD;
  } else {                                /* Extended ID                      */
    CAN1->sTxMailBox[0].TIR |= (uint32_t)(msg->id <<  3) | CAN_ID_EXT;
  }
                                          /* Setup type information           */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                       */
    CAN1->sTxMailBox[0].TIR |= CAN_RTR_DATA;
  } else {                                /* REMOTE FRAME                     */
    CAN1->sTxMailBox[0].TIR |= CAN_RTR_REMOTE;
  }
                                          /* Setup data bytes                 */
  CAN1->sTxMailBox[0].TDLR = (((uint32_t)msg->data[3] << 24) | 
                              ((uint32_t)msg->data[2] << 16) |
                              ((uint32_t)msg->data[1] <<  8) | 
                              ((uint32_t)msg->data[0])        );
  CAN1->sTxMailBox[0].TDHR = (((uint32_t)msg->data[7] << 24) | 
                              ((uint32_t)msg->data[6] << 16) |
                              ((uint32_t)msg->data[5] <<  8) |
                              ((uint32_t)msg->data[4])        );
                                          /* Setup length                     */
  CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
  CAN1->sTxMailBox[0].TDTR |=  (msg->len & CAN_TDT0R_DLC);

  CAN1->IER |= CAN_IER_TMEIE;                 /* enable  TME interrupt        */
  CAN1->sTxMailBox[0].TIR |=  CAN_TI0R_TXRQ;  /* transmit message             */
}


/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it
 *----------------------------------------------------------------------------*/
void CAN_rdMsg (can_msg_t *msg)  {
                                              /* Read identifier information  */
  if ((CAN1->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) {
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
  } else {
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
  }
                                              /* Read type information        */
  if ((CAN1->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0) {
    msg->type =   DATA_FRAME;
  } else {
    msg->type = REMOTE_FRAME;
  }
                                              /* Read number of rec. bytes    */
  msg->len     = (CAN1->sFIFOMailBox[0].RDTR      ) & 0x0F;
                                              /* Read data bytes              */
  msg->data[0] = (CAN1->sFIFOMailBox[0].RDLR      ) & 0xFF;
  msg->data[1] = (CAN1->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
  msg->data[2] = (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
  msg->data[3] = (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

  msg->data[4] = (CAN1->sFIFOMailBox[0].RDHR      ) & 0xFF;
  msg->data[5] = (CAN1->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
  msg->data[6] = (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
  msg->data[7] = (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

  CAN1->RF0R |= CAN_RF0R_RFOM0;             /* Release FIFO 0 output mailbox */
}

void CAN1_RX0_IRQHandler (void)
{
if (CAN1->RF0R & CAN_RF0R_FMP0)
  {			/* message pending ?              */
  CAN_rdMsg (&CAN_RxMsg);                 /* read the message               */
  CAN_RxRdy = 1;                          // set receive flag
  }
}
void CAN1_RX1_IRQHandler (void)
{
if (CAN1->RF1R & CAN_RF0R_FMP0)
  {			/* message pending ?              */
  CAN_rdMsg (&CAN_RxMsg);                 /* read the message               */
  CAN_RxRdy = 1;                          // set receive flag
  }
}
////================================================================================
/*----------------------------------------------------------------------------
  CAN transmit interrupt handler
 *----------------------------------------------------------------------------*/
void CAN1_TX_IRQHandler (void) {

  if (CAN1->TSR & CAN_TSR_RQCP0) {          /* request completed mbx 0        */
    CAN1->TSR |= CAN_TSR_RQCP0;             /* reset request complete mbx 0   */
    CAN1->IER &= ~CAN_IER_TMEIE;            /* disable  TME interrupt         */
	
	CAN_TxRdy = 1;
//	sendchar2(0x34);
  }
}
////==============================================================
void CAN_FilterConfig(uint8_t num,uint32_t id,uint32_t mask)
{
 CAN_FilterInitTypeDef  CAN_FilterInitStructure;
 #if 1
  CAN_FilterInitStructure.CAN_FilterNumber = num;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;////CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = (id>>16)&0xffff;
  CAN_FilterInitStructure.CAN_FilterIdLow = id&0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (mask>>16)&0xffff;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow =  mask&0xffff;
  //CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
#endif

}
void canfilter_init(void)
{
uint32_t id;
uint32_t mask= ID_MASK << 21; ///5+16;

id=ID_BRD<<21; ///5+16

CAN_FilterConfig(0,id,mask);	
}
////=============================================================================================
int can_init( FunctionalState ttcm, FunctionalState abom, FunctionalState awum, FunctionalState nart,
	     FunctionalState rflm, FunctionalState txfp, uint32_t sjw, uint32_t ts1, uint32_t ts2,
	     uint32_t brp, uint8_t loopback, uint8_t silent)
{
  uint8_t tmp=0;
CAN_InitTypeDef   CAN_InitStructure;
CAN_DeInit(CAN1);				// CAN register deinit
CAN_StructInit(&CAN_InitStructure);
CAN_InitStructure.CAN_TTCM = ttcm;
CAN_InitStructure.CAN_ABOM = abom;
CAN_InitStructure.CAN_AWUM = awum;
CAN_InitStructure.CAN_NART = nart;
CAN_InitStructure.CAN_RFLM = rflm;
CAN_InitStructure.CAN_TXFP = txfp;
if(loopback)
  tmp|=CAN_Mode_LoopBack;
if(silent)
  tmp|=CAN_Mode_Silent;
CAN_InitStructure.CAN_Mode = tmp;
////CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;

CAN_InitStructure.CAN_SJW = sjw;
CAN_InitStructure.CAN_Prescaler = brp;
CAN_InitStructure.CAN_BS1 = ts1;
CAN_InitStructure.CAN_BS2 = ts2;

if(CAN_Init(CAN1, &CAN_InitStructure) == CAN_InitStatus_Success)
  return 0;
else
  return -1;
}
int _can_init(uint32_t sjw, uint32_t ts1, uint32_t ts2,
	     uint32_t brp)
{
return can_init(DISABLE,DISABLE,DISABLE, ENABLE,
	     DISABLE, DISABLE, sjw, ts1, ts2,
	     brp, _FALSE, _FALSE);
  
}

int can_speed(uint8_t index) {
if(index>=MAX_NUM_BR )
  return -1;
else
  return _can_init(CAN_SJW_1tq, can_br30[index].ts1,can_br30[index].ts2,can_br30[index].pre);
}


void init_can(void)
{
RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);

can_init(DISABLE,DISABLE,DISABLE, ENABLE,
	     DISABLE, DISABLE, CAN_SJW_1tq, CAN_BS1_6tq, CAN_BS2_8tq,
	     4, _FALSE, _FALSE);
can_speed(DEF_CAN_BR);
////nvic_can();
  NVIC_can_Config();

CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);
canfilter_init();
}

////==============================================================