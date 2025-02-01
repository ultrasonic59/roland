#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

////#include "stm32f2xx_gpio.h"
#include "board.h"

////#define TEST_PIN ZSTP_PIN
////#define TEST_PIN_GPIO ZSTP_PIN_GPIO

void  put_tst_pin(uint8_t idat)
{
#if 1  
if(idat&0x1)
  {
  GPIO_SetBits(TST3_PIN_GPIO, TST3_PIN);
   GPIO_SetBits(TST2_PIN_GPIO, TST2_PIN);
  ////GPIO_SetBits(TST7_PIN_GPIO, TST7_PIN);
  }
else
  {
   GPIO_ResetBits(TST3_PIN_GPIO, TST3_PIN);
   GPIO_ResetBits(TST2_PIN_GPIO, TST2_PIN);
   ////GPIO_ResetBits(TST7_PIN_GPIO, TST7_PIN);
  }
#endif 
}
