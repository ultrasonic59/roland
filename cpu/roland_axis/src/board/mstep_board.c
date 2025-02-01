#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "can.h"
#include "can_cmds.h"
#include "board.h"
#include "printk.h"
///=============================

uint8_t cur_state=0;  /// .5-.4 conc, .3 - .0 status
int32_t cur_coord=0;
uint8_t ena_check_conc=0;

int send_char_dbg (int c) 
{ 
while (!(UART_DBG->SR & 0x0080));
UART_DBG->DR = (c & 0x1FF);
return (c);
}

int get_byte_dbg (void) 
{
while (!(UART_DBG->SR & 0x0020));
return (UART_DBG->DR);
}

#if 1
void _putk(char ch)
{
//sendchar_hdlc(ch);
send_char_dbg(ch);
}
#endif
int check_push_key_dbg(void)
{
return  (UART_DBG->SR & USART_SR_RXNE); 
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
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init( MOT_STEP_PIN_GPIO, &GPIO_InitStructure );
////GPIO_PinAFConfig(MOT_STEP_PIN_GPIO, MOT_STEP_PIN_NPIN, GPIO_AF_TIM1);
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

 /* CAN GPIOs configuration **************************************************/
////=============== CAN1_INH ============================
RCC_AHB1PeriphClockCmd(CAN1_INH_PIN_RCC, ENABLE);
GPIO_InitStructure.GPIO_Pin = CAN1_INH_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init( CAN1_INH_PIN_GPIO, &GPIO_InitStructure );
GPIO_ResetBits(CAN1_INH_PIN_GPIO, CAN1_INH_PIN);

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
////==================================================
void  set_m0(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(MOT_M0_PIN_GPIO, MOT_M0_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_M0_PIN_GPIO, MOT_M0_PIN);
  }
}

void  set_sleep_mot(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(MOT_SLEEP_PIN_GPIO, MOT_SLEEP_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_SLEEP_PIN_GPIO, MOT_SLEEP_PIN);
  }
}
void  set_ena_mot(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(MOT_ENA_PIN_GPIO, MOT_ENA_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_ENA_PIN_GPIO, MOT_ENA_PIN);
  }
}
void  set_dir_mot(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(MOT_DIR_PIN_GPIO, MOT_DIR_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_DIR_PIN_GPIO, MOT_DIR_PIN);
  }
}
void  set_reset_mot(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(MOT_RESET_PIN_GPIO, MOT_RESET_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_RESET_PIN_GPIO, MOT_RESET_PIN);
  }
}

void  on_led0(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(ON_LED0_PIN_GPIO, ON_LED0_PIN);
  }
else
  {
   GPIO_ResetBits(ON_LED0_PIN_GPIO, ON_LED0_PIN);
  }
}
////==================================================
void  on_led1(uint8_t idat)
{
if(idat&0x1)
  {
  GPIO_SetBits(ON_LED1_PIN_GPIO, ON_LED1_PIN);
  }
else
  {
   GPIO_ResetBits(ON_LED1_PIN_GPIO, ON_LED1_PIN);
  }
}

////==================================================

void set_led_dutycycle (uint32_t val)
{
if (val > LED_PWM_TIM_PERIOD)
  {
  val = LED_PWM_TIM_PERIOD;
  }
LED_PWM_TIM->CCR3 = val;
}

void led_tim_en (u32 on_off)
{
if (on_off & 0x01)
  {
  TIM_Cmd(LED_PWM_TIM, ENABLE);
  }
else
  {
  TIM_Cmd(LED_PWM_TIM, DISABLE);
  TIM_SetCounter(LED_PWM_TIM, 0);
  }
}

////=======================================================
void led_tim_init(void)
{
RCC->APB2ENR |= LED_PWM_TIM_RCC;
LED_PWM_TIM->PSC = LED_PWM_TIM_PRESC;
////LED_PWM_TIM->ARR = 1000;
LED_PWM_TIM->ARR = LED_PWM_TIM_PERIOD;////(uint16_t) (((SystemCoreClock / APB2_pres) * 2) / TIM_LED_FREQ) - 1;
LED_PWM_TIM->CCR3 = LED_PWM_TIM_PERIOD/2;////30;
LED_PWM_TIM->CCER |= TIM_CCER_CC3NE;////| TIM_CCER_CC3NP;
LED_PWM_TIM->BDTR |= TIM_BDTR_MOE;
LED_PWM_TIM->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; 
LED_PWM_TIM->CR1 &= ~TIM_CR1_DIR;
LED_PWM_TIM->CR1 &= ~TIM_CR1_CMS;
LED_PWM_TIM->CR1 |= TIM_CR1_CEN;
}

extern volatile uint32_t num_step;

uint8_t get_conc(void)
{
return GPIO_ReadInputDataBit(CONC_PIN_GPIO, CONC_PIN);
}
extern void init_can(void);
////=============================================
void hw_board_init(void)
{
init_gpio();
UART_DBG_Init(); 
init_can();

///led_tim_init();
///mot_tim_init();
}
////============================================
extern uint8_t can1_send(uint16_t id,uint8_t data_len,uint8_t *data);
extern uint8_t  CAN_TxRdy;              /* CAN HW ready to transmit message */
extern uint8_t  CAN_RxRdy;              /* CAN HW received a message        */
extern CanRxMsg RxMessage;

extern can_msg_t CAN_RxMsg;
void state_task( void *pvParameters )
{
  uint8_t tmp; 
int32_t prev_coord=0xffffffff;  
uint8_t prev_state=0xff; 
printk("\n\r state_task"); 
for(;;)
  {
  tmp=get_conc();  
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

void tst_task( void *pvParameters )
{
///uint8_t psk=0; 
char key=0;
uint8_t can_send_data[8]; 

printk("\n\r tst_task"); 
for(;;)
  {
  key=dbg_get_byte() ;  
  switch(key)
    {
    case 'a':
      can_send_data[0] ++;
      break;
    case 'd':
      if(can_send_data[0])
        can_send_data[0]--;
        break;
    case 's':
      can1_send(0x13,8,can_send_data);
       break;
   }
  printk("\n\r key[%c]",key); 
  } 

}

////========================================================  
void tst1_task( void *pvParameters )
{
////uint8_t btst=0; 
uint8_t ii=0; 
printk("\n\r tst1_task"); 
for(;;)
  {
  if( CAN_RxRdy)
  {
   CAN_RxRdy=0;
  printk("\n\r can_rx"); 
   printk("\n\r ExtId[%x]",RxMessage.ExtId);
   printk("\n\r DLC[%x]\n\r ",RxMessage.DLC);
   for(ii=0;ii<8;ii++)
    {
    printk("[%x] ",RxMessage.Data[ii]);
    }
  }
  else
  {
    msleep(10);
  }
  }
}

void __tst_task_( void *pvParameters )
{
////uint8_t btst=0; 
uint8_t psk=0; 
char key=0;
int nstep=300;
uint8_t dir=0;
uint8_t mot_rej=0;
printk("\n\r tst_task"); 

set_sleep_mot(1);
set_ena_mot(1);
///set_reset_mot(0);
uDelay(1000);
set_reset_mot(1);
////set_ena_mot(0);

#if 0       
for(;;)
  {
//// sendchar2 (0x33) ; 
  put_tst_pin(btst);
  btst++;  
  ////delay__ms(1);  
  uDelay(20000);
  }
#endif        
 for(;;)
  {
  key=dbg_get_byte() ;  
  switch(key)
    {
    case 'a':
      nstep += 20;
      break;
    case 's':
      if(nstep)
        nstep-= 20;
        break;
    case 'd':
      dir ++;
      dir&=0x1;
      break;
    case 'm':
      mot_rej ++;
      mot_rej&=0x7;
      break;
    case 'p':
     psk=1;
      break;
     
   }
  printk("\n\r nstep[%d] dir[%x] Mot_rej[%x]",nstep,dir,mot_rej); 
  set_dir_mot(dir);
  set_mot_rej(mot_rej);
  if(psk)
    {
    put_mot_nstep(nstep);
    psk=0;
    }
////  set_led_dutycycle (duty);

  } 
}
////============================================
	
