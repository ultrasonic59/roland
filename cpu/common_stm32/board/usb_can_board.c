#ifndef IS_BOOTER
  #include "FreeRTOS.h"
  #include "queue.h"
  #include "semphr.h"
  #include "my_types.h"
#endif ///IS_BOOTER

///#include "stm32f2xx_gpio.h"
///#include "stm32f2xx_rcc.h"
///#include "my_types.h"
#include "stm32f2xx_conf.h"
#include "board.h"
#include "printk.h"
///=============================
int sendchar2 (int c) 
{ 
while (!(USART2->SR & 0x0080));
USART2->DR = (c & 0x1FF);
return (c);
}

int get_byte2 (void) 
{
while (!(USART2->SR & 0x0020));
return (USART2->DR);
}
int get_byte_dbg (void) 
{
while (!(USART2->SR & 0x0020));
return (USART2->DR);
}

int send_char_dbg(int ch) 
{
//sendchar_hdlc(ch);
sendchar2(ch);
return ch;
}
void _putk(char ch)
{
send_char_dbg(ch);
}

void init_gpio(void)
{
////RCC_AHBPeriphClockCmd(XEN_PIN_RCC_AHB,ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
////=============== TST1 ============================
RCC_AHB1PeriphClockCmd(TST1_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST1_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( TST1_PIN_GPIO, &GPIO_InitStructure );
GPIO_SetBits(TST1_PIN_GPIO, TST1_PIN);
////=============== TST2 ============================
RCC_AHB1PeriphClockCmd(TST2_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST2_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( TST2_PIN_GPIO, &GPIO_InitStructure );
////=============== TST3 ============================
RCC_AHB1PeriphClockCmd(TST3_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = TST3_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
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
  
GPIO_PinAFConfig(UART_DBG_TX_GPIO_PORT, UART_DBG_TX_PIN_NPIN, UART_DBG_TX_AF);
GPIO_PinAFConfig(UART_DBG_RX_GPIO_PORT, UART_DBG_RX_PIN_NPIN, UART_DBG_RX_AF);
////===================================================================
////=============== MOT_FAULT ============================
RCC_AHB1PeriphClockCmd(MOT_FAULT_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_FAULT_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_FAULT_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_CONC ============================
RCC_AHB1PeriphClockCmd(CONC_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CONC_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( CONC_PIN_GPIO, &GPIO_InitStructure );

////=============== MOT_M2 ============================
RCC_AHB1PeriphClockCmd(MOT_M2_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_M2_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_M2_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_M1 ============================
RCC_AHB1PeriphClockCmd(MOT_M1_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_M1_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_M1_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_M0 ============================
RCC_AHB1PeriphClockCmd(MOT_M0_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_M0_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_M0_PIN_GPIO, &GPIO_InitStructure );
////=============== MOT_ENA ============================
RCC_AHB1PeriphClockCmd(MOT_ENA_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = MOT_ENA_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_ENA_PIN_GPIO, &GPIO_InitStructure );
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

////============== LED_PWM ============================
RCC_AHB1PeriphClockCmd(LED_PWM_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = LED_PWM_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( LED_PWM_PIN_GPIO, &GPIO_InitStructure );
GPIO_PinAFConfig(LED_PWM_PIN_GPIO, LED_PWM_PIN_NPIN, GPIO_AF_TIM8);
////=============== ON_LED0 ============================
RCC_AHB1PeriphClockCmd(ON_LED0_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = ON_LED0_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( ON_LED0_PIN_GPIO, &GPIO_InitStructure );
////=============== ON_LED1 ============================
RCC_AHB1PeriphClockCmd(ON_LED1_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = ON_LED1_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( ON_LED1_PIN_GPIO, &GPIO_InitStructure );

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
#ifndef IS_BOOTER

////==================================
void CAN_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;


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
  /* CAN register init */
  CAN_DeInit(CAN1);

  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CAN1, &CAN_InitStructure);


  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  //CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

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

//  CAN2TxMessage.StdId = 0x321;
//  CAN2TxMessage.ExtId = 0x01;
//  CAN2TxMessage.RTR = CAN_RTR_DATA;
//  CAN2TxMessage.IDE = CAN_ID_STD;
//  CAN2TxMessage.DLC = 8;

  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

}
#endif

void TIM_Configuration(TIM_TypeDef* TIMER, u16 Period, u16 Prescaler, u8 PP)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;

TIM_TimeBaseStructure.TIM_Period = Period - 1;
TIM_TimeBaseStructure.TIM_Prescaler = Prescaler - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);

TIM_ClearITPendingBit(TIMER, TIM_IT_Update);
TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);
TIM_Cmd(TIMER, ENABLE);

NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
if (TIMER == TIM2) 
  { 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
  }
else if (TIMER == TIM3) 
  { 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
  }
else if (TIMER == TIM4) 
  { 
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
  }
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PP;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}


////=============================================
void hw_board_init(void)
{
init_gpio();
UART_DBG_Init(); 
#ifndef IS_BOOTER
CAN_Config();
#endif
////led_tim_init();
////mot_tim_init();
}
////============================================
#ifndef IS_BOOTER

////========================================================   
void tst_task( void *pvParameters )
{
printk("\n\r tst_task"); 
for(;;)
  {
//// sendchar2 (0x33) ; 
////  put_tst_pin(btst);
///  btst++;  
  ////delay__ms(1);  
////  uDelay(20000);
    msleep(20); 

  }
}
////============================================
#endif ///IS_BOOTER	
