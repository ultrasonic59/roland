#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

////#include "board.h"
#include "board.h"
#include "printk.h"

#include "snail_cmds.h"

////cmd_t cur_cmd={0};

void motor_task( void *pvParameters )
{
#if 0
  printk("\n\r tst_task"); 

set_sleep_mot(1);
////set_ena_mot(1);
set_reset_mot(1);
uDelay(1000);
set_reset_mot(0);
uDelay(20000);
init_step_mot();
////set_ena_mot(0);
///tst=mot_spi_rd(0x0);
///tst|=0x1;
///mot_spi_wr(0x0,tst);
#if 0      
for(;;)
  {
//// sendchar2 (0x33) ; 
  put_tst_pin(btst);
  btst++;  
  ////delay__ms(1);  
  uDelay(20000);
///  mot_spi_transfer(0x1234);
tst=mot_spi_rd(0x0);
printk("\n\r rd[%x]",tst); 

  }
#endif 
ena_mot(0) ;
for(;;)
{
if(cur_cmd.cmd==0)
  msleep(10);
else
  {
  switch(cur_cmd.cmd)
    {
    case STOP_X:
///      nstep += 20;
      break;
    case GOTO_X:
////      if(nstep)
////        nstep-= 20;
        break;
    case SET_SPD_X:
////      if(nstep)
////        nstep-= 20;
        break;
    }
  }
}
}
#if 0
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
      if(mot_rej>8)
        mot_rej=0;
 ////     mot_rej&=0x7;
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
#endif
#endif
}

