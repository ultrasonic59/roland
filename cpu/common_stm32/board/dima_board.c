#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

#include "can.h"
#include "can_cmds.h"
#include "board.h"
#include "printk.h"

extern void uDelay (const uint32_t usec);
extern void  put_tst_pin(uint8_t idat);

int32_t cur_coord=0;
uint8_t cur_state=0;  /// .5-.4 conc, .3 - .0 status
uint8_t ena_check_conc=0;
void CAN_Config(void);

///=============================
int send_char_dbg (int c) 
{ 
while (!(UART_DBG->SR & 0x0080));
UART_DBG->DR = (c & 0x1FF);
return (c);
}
void _putk(char ch)
{
send_char_dbg(ch);
}

int get_byte_dbg (void) 
{
while (!(UART_DBG->SR & USART_SR_RXNE));
return (UART_DBG->DR);
}
int check_push_key_dbg(void)
{
return  (UART_DBG->SR & USART_SR_RXNE); 
}

void init_gpio(void)
{
////RCC_AHBPeriphClockCmd(XEN_PIN_RCC_AHB,ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
////=============== TST2 ============================
RCC_AHB1PeriphClockCmd(TST2_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST2_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( TST2_PIN_GPIO, &GPIO_InitStructure );
////=============== TST3 ============================
RCC_AHB1PeriphClockCmd(TST3_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST3_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( TST3_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(TST3_PIN_GPIO, TST3_PIN_NPIN, GPIO_AF_TIM1);

////=============== TST7 ============================
RCC_AHB1PeriphClockCmd(TST7_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST7_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( TST7_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(TST7_PIN_GPIO, TST7_PIN_NPIN, GPIO_AF_TIM8);
  
////=========== DBG_UART =================================================== 
RCC_AHB1PeriphClockCmd(UART_DBG_TX_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = UART_DBG_TX_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( UART_DBG_TX_GPIO, &GPIO_InitStructure );

RCC_AHB1PeriphClockCmd(UART_DBG_RX_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = UART_DBG_RX_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( UART_DBG_RX_GPIO, &GPIO_InitStructure );
  
GPIO_PinAFConfig(UART_DBG_TX_GPIO, UART_DBG_TX_PIN_NPIN, UART_DBG_TX_AF);
GPIO_PinAFConfig(UART_DBG_RX_GPIO, UART_DBG_RX_PIN_NPIN, UART_DBG_RX_AF);
////===================================================================
////=============== MOT_FAULT ============================
RCC_AHB1PeriphClockCmd(MOT_FAULT_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_FAULT_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_Init( MOT_FAULT_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_STALLN ============================
RCC_AHB1PeriphClockCmd(MOT_STALLN_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_STALLN_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_Init( MOT_STALLN_PIN_GPIO, &GPIO_InitStructure );
////=============== CONC1 ============================
RCC_AHB1PeriphClockCmd(CONC1_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CONC1_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_Init( CONC1_PIN_GPIO, &GPIO_InitStructure );
////=============== CONC0 ============================
RCC_AHB1PeriphClockCmd(CONC0_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CONC0_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_Init( CONC0_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_RESET ============================
RCC_AHB1PeriphClockCmd(MOT_RESET_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_RESET_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_RESET_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_SLEEP ============================
RCC_AHB1PeriphClockCmd(MOT_SLEEP_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_SLEEP_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_SLEEP_PIN_GPIO, &GPIO_InitStructure );
////#define MOT_STEP_TIM  	        TIM12
////=============== MOT_STEP ============================
#if 0
RCC_AHB1PeriphClockCmd(MOT_STEP_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_STEP_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_STEP_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(MOT_STEP_PIN_GPIO, MOT_STEP_PIN_NPIN, GPIO_AF_TIM1);
#else
RCC_AHB1PeriphClockCmd(MOT_STEP_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_STEP_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
///GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_STEP_PIN_GPIO, &GPIO_InitStructure );
////GPIO_PinAFConfig(MOT_STEP_PIN_GPIO, MOT_STEP_PIN_NPIN, GPIO_AF_TIM1);

#endif
////===================================================================

////=============== MOT_DIR ============================
RCC_AHB1PeriphClockCmd(MOT_DIR_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_DIR_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_DIR_PIN_GPIO, &GPIO_InitStructure );
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

////=============== MOT_SPI_SCS ============================
RCC_AHB1PeriphClockCmd(MOT_SPI_SCS_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_SPI_SCS_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( MOT_SPI_SCS_PIN_GPIO, &GPIO_InitStructure );
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

}
////=============================================
void UART_DBG_Init(void)
{
////GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
UART_DBG_CLK_INIT(UART_DBG_CLK, ENABLE);
USART_DeInit(UART_DBG);
USART_InitStructure.USART_BaudRate = 115200;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1 ;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(UART_DBG, &USART_InitStructure);
UART_DBG->CR1 |= USART_CR1_RXNEIE;
USART_Cmd(UART_DBG, ENABLE);
}

////==================================================


///===========================================================
volatile uint32_t gsr;

////===============================================
uint8_t get_conc0(void)
{
return GPIO_ReadInputDataBit(CONC0_PIN_GPIO, CONC0_PIN);
}
uint8_t get_conc1(void)
{
return GPIO_ReadInputDataBit(CONC1_PIN_GPIO, CONC1_PIN);
}
uint8_t get_conc_n(void)
{
uint8_t rez=0;
if(get_conc0())
  rez|=(0x1<<0);
if(get_conc1())
  rez|=(0x1<<1);
return rez;
}
////=============================================
void hw_board_init(void)
{
NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
init_gpio();
UART_DBG_Init(); 
init_can();

/*
#ifndef BOOTER
   motor_init();
#endif
cur_stat=STATE_READY;
*/
}
////============================================

////============================================
extern uint8_t can1_send(uint16_t id,uint8_t data_len,uint8_t *data);
extern uint8_t  CAN_TxRdy;              /* CAN HW ready to transmit message */
extern uint8_t  CAN_RxRdy;              /* CAN HW received a message        */
///=========================================================
void _CAN1_Init (void)
{
uint32_t id;
 uint32_t mask;
CAN_Config();

mask= ID_MASK << 21; ///5+16

id=ID_BRD<<21; ///5+16

CAN_FilterConfig(0,id,mask);	
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

extern can_msg_t CAN_RxMsg;
void state_task( void *pvParameters )
{
  uint8_t tmp; 
int32_t prev_coord=0xffffffff;  
uint8_t prev_state=0xff; 
printk("\n\r state_task"); 
for(;;)
  {
  tmp=get_conc_n();  
  tmp<<=4;
  cur_state&= ~CONC_MASK;
  cur_state |= tmp;
    if((prev_state!=cur_state)||(prev_coord!=cur_coord))
    {
      prev_state=cur_state;
      prev_coord=cur_coord;
      put_can_cmd_stat(cur_state,cur_coord);
     }
   else
      msleep(50);
  }

}
////========================================================  
void tst1_task( void *pvParameters )
{
////uint8_t btst=0; 
uint32_t t_coord=0;  
uint8_t t_stat=0x8; 

////uint8_t ii=0; 
printk("\n\r tst1_task"); 
///=======================================
#if 0
can_msg_t  send_msg;
go_cmd_t t_go_cmd;
t_go_cmd.cmd=GO_CMD ;
t_go_cmd.dirs=1;
t_go_cmd.step_per=1000;
send_msg.len=CAN_MAX_NUM_BYTES;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
t_go_cmd.steps=10;
memcpy(send_msg.data,&t_go_cmd,sizeof(go_cmd_t));
send_msg.id=ID_BRD; 
#endif
///============================================
#if 0
for(;;)
  {
  if( CAN_RxRdy)
    {
    CAN_RxRdy=0;
    printk("\n\r can_rx"); 
    printk("\n\r ExtId[%x]",CAN_RxMsg.id);
    printk("\n\r DLC[%x]\n\r ",CAN_RxMsg.len);
    for(ii=0;ii<8;ii++)
      {
      printk("[%x] ",CAN_RxMsg.data[ii]);
      }
    }
  else
  {
////   CAN_wrMsg (&send_msg);
    msleep(20);
  }
  }
#endif
for(;;)
  {
  put_can_cmd_stat(t_stat,t_coord);
   
   t_coord++; 
    msleep(20);
  }
}
////=======================================================
void tst_task( void *pvParameters )
{
///uint8_t btst=0; 
char key=0;
printk("\n\r tst_task"); 
 
for(;;)
{
if(check_push_key_dbg())
  {
  key=get_byte_dbg() ;  
  switch(key)
    {
    case 'a':
    case 'A':
      printk("\n\r go to app"); 
      goto_app();
      break;
    case 'b':
    case 'B':
      printk("\n\r go to boot"); 
      goto_booter();
      break;
   }
  } 
msleep(10);
}
}
	
