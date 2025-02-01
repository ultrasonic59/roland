#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
////#include "my_grbl.h"
#include "can.h"
#include "can_cmds.h"
#include "printk.h"
////=======================================
extern xQueueHandle queu_to_send;

int put_can_cmd_reset_all(void)
{
  return 0;
}

void _mc_reset(void)
{
put_can_cmd_reset_all();  
////report_init_message();
}
int put_can_cmd_go(uint8_t dirs,uint16_t per
                   ,uint32_t step_x
                   ,uint32_t step_y
                   ,uint32_t step_z)
{
///uint8_t btst=0;  
can_msg_t  send_msg;
go_cmd_t t_go_cmd;
t_go_cmd.cmd=GO_CMD ;
t_go_cmd.dirs=dirs;
t_go_cmd.step_per=per;
send_msg.len=CAN_MAX_NUM_BYTES;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
if(step_x)
  {
   t_go_cmd.steps=step_x;
   memcpy(send_msg.data,&t_go_cmd,sizeof(go_cmd_t));
   send_msg.id=ID_BRD; 
   xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);
  }
if(step_y)
  {
   t_go_cmd.steps=step_y;
   memcpy(send_msg.data,&t_go_cmd,sizeof(go_cmd_t));
   send_msg.id=ID_BRD; 
   xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);
  }
if(step_z)
  {
   t_go_cmd.steps=step_z;
   memcpy(send_msg.data,&t_go_cmd,sizeof(go_cmd_t));
   send_msg.id=ID_BRD; 
   xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);
  }

////send_msg.data[1]=btst;    
////xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

///go_cmd_t   go_cmd;
#if 0        
        printk("\n\r tst_cnt[%d][%02x][%d][%d][%d]", per, dirs
            , step_x
            , step_y
            , step_z
        );
#endif

  return 0;
}
////================================================
#ifndef _MASTER_
extern uint32_t cur_coord;
extern uint8_t cur_stat;

int put_can_cmd_stat(uint8_t state
                   ,uint32_t coord)
{
///uint8_t btst=0;  
can_msg_t  send_msg;
put_stat_cmd_t t_put_stat_cmd;
t_put_stat_cmd.cmd=PUT_STAT_CMD ;
#if STEP_X
  t_put_stat_cmd.axis= AXIS_X;
#elif STEP_Y
  t_put_stat_cmd.axis= AXIS_Y;
#elif STEP_Z
  t_put_stat_cmd.axis= AXIS_Z;
#else
  t_put_stat_cmd.axis= 0;
#endif
////t_put_stat_cmd.coord=cur_coord;
///t_put_stat_cmd.state=cur_stat;
t_put_stat_cmd.coord=coord;
t_put_stat_cmd.state=state;
send_msg.len=CAN_MAX_NUM_BYTES;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,&t_put_stat_cmd,sizeof(put_stat_cmd_t));
send_msg.id=ID_MASTER_CMD; 
xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

  return 0;
}
int put_can_ack(uint8_t cmd )
{
///uint8_t btst=0;  
can_msg_t  send_msg;

put_ack_t t_put_ack;

t_put_ack.cmd=PUT_ACK ;
#if STEP_X
  t_put_ack.ack.axis= AXIS_X;
#elif STEP_Y
  t_put_ack..ack.axis= AXIS_Y;
#elif STEP_Z
  t_put_ack..ack.axis= AXIS_Z;
#else
  t_put_ack..ack.axis= 0;
#endif
 t_put_ack.ack.ack_cmd=cmd;
send_msg.len= sizeof(put_ack_t);
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,&t_put_ack,sizeof(put_ack_t));
send_msg.id=ID_MASTER_CMD; 
xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

  return 0;
}


#endif
////======================================================
