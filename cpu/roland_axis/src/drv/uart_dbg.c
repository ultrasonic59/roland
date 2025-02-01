
#include "brook.h"
#include "serial_cnf.h"

static void DBG_RCC_Configuration(void)
{
/* Enable BT GPIO clocks */
RCC_AHB1PeriphClockCmd(UART_DBG_TX_GPIO_CLK, ENABLE);
RCC_AHB1PeriphClockCmd(UART_DBG_RX_GPIO_CLK, ENABLE);
UART_DBG_CLK_INIT(UART_DBG_CLK, ENABLE);
}

void USART1_IRQHandler(void)
{
///int cnt;  
#if 0
u8 ch;
uint32_t tmp_sr;
signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
signed portBASE_TYPE xTaskWoken= pdFALSE;

tmp_sr=UART_DBG->SR;  
////put_tst1(1);  
if((tmp_sr&USART_FLAG_RXNE) != 0)
////if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
  UART_DBG->SR &= ~USART_FLAG_RXNE;              ///  USART_ClearITPendingBit(UART_BT, USART_IT_RXNE);
  ch = (u8)(UART_DBG->DR & 0x0FF);
  if (hdlc1_on_bytein(&g_hdlc_bt, ch) > 0)
    {
    xQueueSendFromISR(g_hdlc_bt.ev_rsv_frame, &ch, &xHigherPriorityTaskWoken);
  }
  }
else if((tmp_sr&USART_FLAG_ORE) != 0)
////if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
////  put_tst3(1);  
  ch = (u8)(UART_BT->DR & 0x0FF);
///  ringbuffer_putc(&rx_bt_buffer, ch); 
////  rt_hw_serial_isr(&serial4);
  /* clear interrupt */
  USART_ClearITPendingBit(UART_BT, USART_FLAG_ORE);
  }

#if 1
if ((tmp_sr&USART_FLAG_TXE) != 0)
    {
    if (xQueueReceiveFromISR(g_hdlc_bt.ua_snd_dat, &ch, &xTaskWoken ) == pdPASS)
      {
      UART_BT->DR = ch & 0x1FF;
      }
    if (xQueueIsQueueEmptyFromISR(g_hdlc_bt.ua_snd_dat))
      {
      UART_BT->CR1 &= ~USART_CR1_TXEIE;		      // disable TX interrupt if nothing to send
      }
      UART_BT->SR &= ~USART_IT_TXE;              ///  USART_ClearITPendingBit(UART_BT, USART_IT_TXE);
     
    }
#endif
#endif
}
static void DBG_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    /* Configure BT Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART_DBG_TX_PIN;
    GPIO_Init(UART_DBG_TX_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = UART_DBG_RX_PIN;
    GPIO_Init(UART_DBG_RX_GPIO_PORT, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART_DBG_TX_GPIO_PORT, UART_DBG_TX_SOURCE, UART_DBG_TX_AF);
    GPIO_PinAFConfig(UART_DBG_RX_GPIO_PORT, UART_DBG_RX_SOURCE, UART_DBG_RX_AF);
}
///==========================
int get_byte_dbg(void) 
{
while (!(UART_DBG->SR & USART_SR_RXNE));
return (UART_DBG->DR);
}
int rdy_rx_dbg(void) 
{
if((UART_DBG->SR & USART_SR_RXNE))
  return 1;
else
  return 0;
}
///======================================
int send_char_dbg(int c) 
{ 
while (!(UART_DBG->SR & 0x0080));
UART_DBG->DR = (c & 0x1FF);
return (c);
}
#if 0
void init_irq_bt(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
////NVIC_InitStructure.NVIC_IRQChannel = UART_BT_IRQn;
////NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;///0x6;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;////7;	//1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}
#endif
///===========================================
void uart_dbg_init(void)
{
int i;
USART_InitTypeDef USART_InitStructure;
DBG_RCC_Configuration();
DBG_GPIO_Configuration();
  USART_DeInit(UART_DBG);
  USART_InitStructure.USART_BaudRate = DEF_DBG_BR;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
////  USART_InitStructure.USART_StopBits = USART_StopBits_2 ;
  USART_InitStructure.USART_StopBits = USART_StopBits_1 ;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART_DBG, &USART_InitStructure);
  
  USART_Cmd(UART_DBG, ENABLE);

for (i = 0; i < 0x1000; i++)
  {
  __NOP();   //avoid unwanted output
  }
UART_DBG->CR1 = 0x202C;

UART_DBG->SR = 0;
////init_irq_bt();
}

///================================================
