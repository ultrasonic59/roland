#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"
#include "can.h"
#include "can_cmds.h"
#include "printk.h"
#include "my_misc.h"
///#include "emul_eeprom.h"

////=======================================
///extern uint8_t boot_state;

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

int put_can_cmd_stat(uint8_t state
                   ,uint32_t coord)
{
///uint8_t btst=0;  
can_msg_t  send_msg;
put_stat_cmd_t t_put_stat_cmd;
t_put_stat_cmd.cmd=PUT_STAT_CMD ;
t_put_stat_cmd.axis= AXIS_BRD;

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
int put_can_boot_ans(uint8_t cmd,uint8_t state)
{
can_msg_t  send_msg;
put_boot_stat_cmd_t t_put_stat_cmd;
t_put_stat_cmd.cmd=cmd ;
t_put_stat_cmd.axis= AXIS_BRD;
t_put_stat_cmd.state=state;
send_msg.len=sizeof(put_boot_stat_cmd_t);
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,&t_put_stat_cmd,sizeof(put_boot_stat_cmd_t));
send_msg.id=ID_MASTER_CMD; 
xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

  return 0;
}
int put_can_boot_cmd_stat(uint8_t state)
{
return put_can_boot_ans(PUT_BOOT_STAT,state);  
}

int put_can_rd_eeprom_ans(rd_eeprom_ans_t *t_rd_eeprom_ans)
{
can_msg_t  send_msg;
can_cmd_t t_can_cmd;
t_can_cmd.data[0]=RD_EEPROM_ANS ;
t_can_cmd.data[1]=t_rd_eeprom_ans->num_dates;
t_can_cmd.data[2]=t_rd_eeprom_ans->addr;
memcpy(&t_can_cmd.data[3],t_rd_eeprom_ans->data,sizeof(uint16_t)*t_rd_eeprom_ans->num_dates);
send_msg.len= 3+sizeof(uint16_t)*t_rd_eeprom_ans->num_dates;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,t_can_cmd.data,3+sizeof(uint16_t)*t_rd_eeprom_ans->num_dates);
send_msg.id=ID_MASTER_CMD; 
xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

  return 0;
}
int put_can_wr_eeprom_ans(wr_eeprom_req_t *t_wr_eeprom_ans)
{
can_msg_t  send_msg;
can_cmd_t t_can_cmd;
t_can_cmd.data[0]=WR_EEPROM_ANS ;
t_can_cmd.data[1]=t_wr_eeprom_ans->num_dates;
t_can_cmd.data[2]=t_wr_eeprom_ans->addr;
send_msg.len= 3;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,t_can_cmd.data,3);
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
t_put_ack.ack.axis= AXIS_BRD;

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
int put_can_rd_flash_ans(rd_flash_ans_t *t_rd_flash_ans)
{
can_msg_t  send_msg;
can_cmd_t t_can_cmd;
t_can_cmd.data[0]=RD_FLASH_ANS ;

memcpy(&t_can_cmd.data[1],&t_rd_flash_ans->addr,sizeof(uint32_t));

memcpy(&t_can_cmd.data[5],&t_rd_flash_ans->data,sizeof(uint16_t));
send_msg.len= 7;
send_msg.format=STANDARD_FORMAT;
send_msg.type=DATA_FRAME;
memcpy(send_msg.data,t_can_cmd.data,5+sizeof(uint16_t));
send_msg.id=ID_MASTER_CMD; 
xQueueSend(queu_to_send,&send_msg,CAN_TIMEOUT_SEND);

  return 0;
}


void rd_flash_dat(rd_flash_ans_t *t_rd_flash_ans)
{
uint16_t t_data;  
t_data= *(uint16_t*)(t_rd_flash_ans->addr); 
t_rd_flash_ans->data=t_data;
///printk("\n\rflash [%x:%x] ===",t_rd_flash_ans->addr,t_rd_flash_ans->data); 
}


  

int obr_can_cmd(uint8_t *data)
{
////uint8_t tmp;  
switch(data[0]) {
      case STOP_CMD:
        put_can_ack(STOP_CMD);
        stop_mot_cmd();
        break;
      case GO_CMD:
        {
        put_can_ack(GO_CMD);
         go_cmd((go_cmd_t *)data);
  ////       printk("Go [dir=%x:per=%d:steps=%d] ",p_can_cmd->dirs,p_can_cmd->step_per,p_can_cmd->steps);
        }
        break;
      case GET_STAT_CMD:
        {
        uint8_t tmp=get_conc();  
        tmp<<=4;
        cur_state&= ~CONC_MASK;
        cur_state |= tmp;
        put_can_cmd_stat(cur_state,cur_coord);
         printk("[stat=%x] ",cur_state);
         }
        break;
      case SET_PARAM:
         put_can_ack(SET_PARAM);
         set_param((set_param_cmd_t *)(data));
        break;
     case GET_BOOT_STAT:
///        put_can_boot_cmd_stat(boot_state);
///        printk("[stat=%x] ",boot_state);
        break;
       case GO_TO_BOOTER:
 ////       EE_WriteVariable(ADDR_EEPROM_BOOT_WORK,0);  ///???
 ///       goto_booter();
        break;
      case CHECK_CONN:
         put_can_ack(CHECK_CONN );
 ////   printk("CHECK_CONN[%x] ",cur_stat);
        break;
#if 0
      case RD_EEPROM_REQ:
        {
        rd_eeprom_ans_t t_rd_eeprom_ans;  
        t_rd_eeprom_ans.num_dates=data[1];
        t_rd_eeprom_ans.addr=data[2];
        rd_eeprom_dat(&t_rd_eeprom_ans);
        put_can_rd_eeprom_ans(&t_rd_eeprom_ans);
////     printk("RD_EEPROM_REQ ");
        }
        break;
      case WR_EEPROM_REQ:
        {
        wr_eeprom_req_t t_wr_eeprom_req;
        t_wr_eeprom_req.num_dates=data[1];
        t_wr_eeprom_req.addr=data[2];
        memcpy(t_wr_eeprom_req.data,&data[3],sizeof(uint16_t)*t_wr_eeprom_req.num_dates); 
        wr_eeprom_dat(&t_wr_eeprom_req);
        put_can_wr_eeprom_ans(&t_wr_eeprom_req);
 ////    printk("WR_EEPROM_REQ ");
        }
        break;
#endif       
      case RD_FLASH_REQ:
        {
        rd_flash_ans_t t_rd_flash_ans;  
        memcpy(&t_rd_flash_ans.addr,&data[1],sizeof(uint32_t));
        rd_flash_dat(&t_rd_flash_ans);
        put_can_rd_flash_ans(&t_rd_flash_ans);
////     printk("RD_EEPROM_REQ ");
        }
        break;
     default:
      break;
    }
return 0;  
}
////======================================================
