#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
///#include "stm32f2xx_gpio.h"
///#include "stm32f2xx_rcc.h"

#include "mstep_board.h"
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

void _putk(char ch)
{
//sendchar_hdlc(ch);
sendchar2(ch);
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

volatile uint32_t num_step=0;


void mot_tim_init(void)
{
NVIC_InitTypeDef NVIC_InitStructure; 

RCC->APB2ENR |= MOT_STEP_TIM_RCC;
MOT_STEP_TIM ->PSC = MOT_TIM_PRESC;
////LED_PWM_TIM->ARR = 1000;
MOT_STEP_TIM ->ARR = MOT_TIM_PERIOD;////
MOT_STEP_TIM ->CCR2 = MOT_TIM_PERIOD/2;////30;
MOT_STEP_TIM->CCER |= TIM_CCER_CC2NE;////| TIM_CCER_CC3NP;
MOT_STEP_TIM->BDTR |= TIM_BDTR_MOE;
MOT_STEP_TIM->CCMR1 = TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1; 
MOT_STEP_TIM->CR1 &= ~TIM_CR1_DIR;
MOT_STEP_TIM->CR1 &= ~TIM_CR1_CMS;
MOT_STEP_TIM->CR1 |= TIM_CR1_CEN;
MOT_STEP_TIM ->DIER = TIM_DIER_CC2IE|TIM_DIER_COMIE;
TIM_ClearITPendingBit(MOT_STEP_TIM, TIM_IT_COM);
	// Enable interrupt, motor commutation has high piority and has
	// a higher subpriority then the hall sensor

NVIC_InitStructure.NVIC_IRQChannel = MOT_TIM_IRQN;
	// highest priority
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;

NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// highest priority
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
TIM_ITConfig(MOT_STEP_TIM, TIM_IT_CC2, ENABLE);


}
void stop_mot_step_tim(void)
{
TIM_Cmd(MOT_STEP_TIM, DISABLE);
}

void put_mot_nstep(uint32_t nstep)
{
num_step=nstep; 
set_ena_mot(0);
TIM_Cmd(MOT_STEP_TIM, ENABLE);
}
volatile uint32_t gsr;
////=======================================================
void MOT_STEP_TIM_IRQHandler(void)
{ 
if(num_step)
  {
  num_step--;  
  if(num_step==0)
    {
    stop_mot_step_tim(); 
    set_ena_mot(1);
    }
  }
TIM_ClearITPendingBit(MOT_STEP_TIM, TIM_IT_CC2);
}

void set_mot_rej(uint8_t rej)
{
if(rej&0x1)
  {
  GPIO_SetBits(MOT_M0_PIN_GPIO, MOT_M0_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_M0_PIN_GPIO, MOT_M0_PIN);
  }
if(rej&0x2)
  {
  GPIO_SetBits(MOT_M1_PIN_GPIO, MOT_M1_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_M1_PIN_GPIO, MOT_M1_PIN);
  }
if(rej&0x4)
  {
  GPIO_SetBits(MOT_M2_PIN_GPIO, MOT_M2_PIN);
  }
else
  {
   GPIO_ResetBits(MOT_M2_PIN_GPIO, MOT_M2_PIN);
  }
}
uint8_t get_conc(void)
{
return GPIO_ReadInputDataBit(CONC_PIN_GPIO, CONC_PIN);
}
////=============================================
void hw_board_init(void)
{
init_gpio();
UART_DBG_Init(); 

led_tim_init();
mot_tim_init();
}
////============================================
////========================================================   
void tst_task( void *pvParameters )
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
set_reset_mot(0);
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
	
