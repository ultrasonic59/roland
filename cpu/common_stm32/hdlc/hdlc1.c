///#ifdef EMW3165
  #include "stm32f2xx.h"
  #include "stm32f2xx_conf.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "_hdlc.h"
#include "my_types.h"
#include "printk.h"

extern void addfcs16(unsigned char *cp,int len );
///============================
/*
xQueueHandle ev_rsv_ack;
xQueueHandle ev_rsv_dat;
xQueueHandle ev_rsv_frame;
xQueueHandle ev_snd_frame;
///xQueueHandle ua_rsv_dat;
xQueueHandle ua_snd_dat;
*/
///============================
///==================================
////#define HDLC1_OBR_BUF_LENGTH_MAX   8192
///#define hdlc1_obr_buff g_hdlc_st.obr_buff
///#define hdlc1_tbuff1_send g_hdlc_st.t_snd_buff1
///#define hdlc1_tbuff2_send g_hdlc_st.t_snd_buff2

///==============================
void hdlc1_error( char const *err )
{
puts(err);
}
///==============================
void hdlc1_init(hdlc_stat_t *p_hdlc)
{
p_hdlc->frame_in_pos  =0;	
p_hdlc->in_esc_byte		=0;	
p_hdlc->frame_out_len	= 0;
p_hdlc->snd_buff_len= 0;
p_hdlc->snd_buff1_len= 0;
///p_hdlc->snd_buff2_len= 0;
p_hdlc->in_buf = (u8*) pvPortMalloc( HDLC1_MAX_FRAME_LEN );
if( p_hdlc->in_buf == NULL ) 
	hdlc1_error("hdlc_frames_init(): NO MEM\n");

p_hdlc->snd_buff	= (uint8_t*)pvPortMalloc(HDLC1_MAX_FRAME_LEN);
if(p_hdlc->snd_buff == NULL )
	hdlc1_error("hdlc1_init(): NO MEM FOR SND_BUF\n");
p_hdlc->obr_buff	= (uint8_t*)pvPortMalloc(HDLC1_MAX_FRAME_LEN);
if( p_hdlc->obr_buff == NULL )
	hdlc1_error("hdlc1_init(): NO MEM FOR OBR_TMPBUF\n");
	
p_hdlc->t_snd_buff1 = (u8*)pvPortMalloc(HDLC1_MAX_FRAME_LEN);
if(p_hdlc->t_snd_buff1 == NULL)
	{
	hdlc1_error("hdlc1_init(): NO MEM FOR hdlc1_tbuff_send\n");
	return ;	
	}
memset(p_hdlc->t_snd_buff1,0,HDLC1_MAX_FRAME_LEN);	
#if 0
p_hdlc->t_snd_buff2 = (u8*)pvPortMalloc(HDLC1_MAX_FRAME_LEN);
if(p_hdlc->t_snd_buff2 == NULL)
	{
	hdlc1_error("hdlc1_init(): NO MEM FOR hdlc1_tbuff_send\n");
	return ;	
	}
#endif

}

///======================================
///	Обработка ошибки длины входящего кадра
#if 0
void hdlc1_error_frame_len(hdlc_stat_t *p_hdlc)
{
p_hdlc->err_len++;
}
#endif
///volatile uint8_t tst_buf[512];
///int ptr_tst_buf=0;
///========================================================
uint8_t hdlc1_send_iframe(hdlc_stat_t *p_hdlc,u8 *data, int len)
{
////frame_t fr;
uint16_t len_o=0;
u16 ii;
u8 bt;
uint8_t rez=TRUE;
///================================
////p_hdlc->snd_buff2_len=0;
///================================
memcpy(p_hdlc->snd_buff, data, len );
p_hdlc->frame_out_len=len ;
addfcs16(p_hdlc->snd_buff,p_hdlc->frame_out_len);				// Добавить циклический код
p_hdlc->frame_out_len += 2;
p_hdlc->fn_send_byte(PPP_FRAME);
len_o++;
for(ii=0;ii<p_hdlc->frame_out_len;ii++)
	{
	bt = p_hdlc->snd_buff[ii];
	if( bt == PPP_FRAME || bt == PPP_ESCAPE )			// Замена "FLAG" или "ESC" на ESC-последовательность
		{
		p_hdlc->fn_send_byte(PPP_ESCAPE);
		len_o++;
		p_hdlc->fn_send_byte((bt ^ PPP_ESCAPE_BIT));
		len_o++;
		}
	else
		{
		p_hdlc->fn_send_byte(bt);
		len_o++;
		}
		}
p_hdlc->fn_send_byte(PPP_FRAME);
///len_o++;
///put_tst1(0);

////rez = hdlc1_send_frame(send_byte_ua0,&snd_fr);
////p_hdlc->fn_send_frame(p_hdlc->t_snd_buff2,len_o);

return rez;
}

///==============================
int hdlc1_parse_iframe(hdlc_stat_t *p_hdlc,u8 *in_buf,u16 len)
{
int rez =0;
u8 btmp;
uint32_t wtmp=0;
uint16_t h_rez;
////t_frametx frame_tx;

switch(*in_buf)
	{
	case CMD_ACK:
		btmp = *(in_buf+1);
		xQueueSend(p_hdlc->ev_rsv_ack,&btmp,TIMEOUT_SEND);
		break;	
	case CMD_PUT_DAT:
		btmp = *(in_buf+1);
		p_hdlc->fn_set_rec_dat(btmp,(void *)((u32)in_buf+2));
	///	xQueueSend(ev_rsv_dat,&btmp,TIMEOUT_SEND);
		wtmp=SEND_ACK;
		wtmp |= (btmp<<8);
		xQueueSend(p_hdlc->ev_snd_frame,&wtmp,TIMEOUT_SEND);
		break;
	case CMD_PUT_CMD:
		btmp = *(in_buf+1);
		h_rez=p_hdlc->fn_set_rec_dat(btmp,(void *)((u32)in_buf+2));
	///	xQueueSend(ev_rsv_dat,&btmp,TIMEOUT_SEND);
		wtmp=SEND_REZ;
		wtmp |= (btmp<<8);
		wtmp |= (h_rez<<16);
 		xQueueSend(p_hdlc->ev_snd_frame,&wtmp,TIMEOUT_SEND);
		break;
	case CMD_PUT_REQ_DAT:
		btmp = *(in_buf+1);
		p_hdlc->fn_set_rec_dat(btmp,(void *)((u32)in_buf+2));
                xQueueSend(p_hdlc->ev_rsv_dat,&btmp,TIMEOUT_SEND);
///		htmp=SEND_ACK;
///		htmp |= (btmp<<8);
///		xQueueSend(ev_snd_frame,&htmp,TIMEOUT_SEND);
		break;
	case CMD_REQ_DAT:
		btmp = *(in_buf+1);
		p_hdlc->offs_snd_buf = *(in_buf+2);
		p_hdlc->offs_snd_buf |= (*(in_buf+3))<<8;
		wtmp = SEND_REQ_DAT;
		wtmp |= (btmp<<8);
///		set_req_rez((req_rez_t *)((u32)in_buf+1) );
		xQueueSend(p_hdlc->ev_snd_frame,&wtmp,TIMEOUT_SEND);	
		rez =0;
////		printk("r");
		break;
	}
return rez;
}
///==================================
int hdlc1_check_framein(hdlc_stat_t *p_hdlc)
{
int res = 1;
if(p_hdlc->len_obr_dat < 3 )
	{
////	p_hdlc->err_len++;
	res = -1;
	}
else if( !checkfcs16(p_hdlc->obr_buff, p_hdlc->len_obr_dat - 2 ) )
	{
////	p_hdlc->err_fcs++;
	res = -2;
	}
return res;
}
///=====================================
void hdlc1_parse_framein(hdlc_stat_t *p_hdlc)
{
///int flg_parse;
////hdlc_hdr_t *hdr;
p_hdlc->frames_received++;
hdlc1_parse_iframe(p_hdlc, p_hdlc->obr_buff , p_hdlc->len_obr_dat);
}
///================================
int	hdlc1_send_ack_to(hdlc_stat_t *p_hdlc,u8 ipar)
{
int num_send=0;
u8 *p_tmp	 =p_hdlc->t_snd_buff1;
*p_tmp++  = CMD_ACK;
num_send++;
*p_tmp++  = ipar;
num_send++;
return hdlc1_send_iframe(p_hdlc,p_hdlc->t_snd_buff1, num_send);
////return 0;
}
int	hdlc1_send_rez_to(hdlc_stat_t *p_hdlc,u8 ipar,u16 rez)
{
int num_send=0;
u8 *p_tmp	 =p_hdlc->t_snd_buff1;
*p_tmp++  = CMD_REZ;
num_send++;
*p_tmp++  = ipar;
num_send++;
*p_tmp++  = rez&0xff;
num_send++;
*p_tmp++  = (rez>>8)&0xff;
num_send++;
return hdlc1_send_iframe(p_hdlc,p_hdlc->t_snd_buff1, num_send);
////return 0;
}

///====== from uart isr ============================
int hdlc1_on_bytein(hdlc_stat_t *p_hdlc, u8 in_bt )
{
p_hdlc->bytes_received++;
if( in_bt == PPP_FRAME )						// Пришел флаг
	{
	if(p_hdlc->frame_in_pos > 2 )				// Если между двумя флагами были данные:
		{
		memcpy(p_hdlc->obr_buff,p_hdlc->in_buf,p_hdlc->frame_in_pos);
		p_hdlc->len_obr_dat=p_hdlc->frame_in_pos;
		p_hdlc->frame_in_pos = 0;			
		p_hdlc->in_esc_byte	 = 0;
		return 1;
		}
	}
else
	{
	if(p_hdlc->frame_in_pos >= HDLC1_MAX_FRAME_LEN )
		{
////		hdlc1_error_frame_len(p_hdlc);
		printk("\n hdlc1_error_frame_len");
		return -1;
		}
	if(p_hdlc->in_esc_byte )								// Второй байт ESC-последовательности
		{
		p_hdlc->in_buf[p_hdlc->frame_in_pos++] = in_bt ^ PPP_ESCAPE_BIT;
		p_hdlc->in_esc_byte = 0;
		}
	else
		{
		if( in_bt == PPP_ESCAPE)						// Первый байт ESC-последовательности
			{
			p_hdlc->in_esc_byte = TRUE;
			}
		else								// Обычный байт данных
			{
			p_hdlc->in_buf[p_hdlc->frame_in_pos++] = in_bt;	// Запись пришедшего байта в буфер
			}
		}
	return 0;	
	}
return 0;	
}
///==============================
extern u16 pppfcs16(register u16 fcs, register unsigned char *cp, register int len);
///====================================
void obr_hdlc1_cmd(hdlc_stat_t *p_hdlc)
{
int res;
u8 ii;
if((res= hdlc1_check_framein(p_hdlc))>0 )	// Проверить правильность входящего кадра
	{
///==================================
	hdlc1_parse_framein(p_hdlc);
	}
else
	{
	printk("\n\r error framein[%d][%02x]",res,p_hdlc->len_obr_dat);
	if(res==-2)
		{
                  printk("\n\r[%02x]:",p_hdlc->len_obr_dat);
	for(ii=0;ii<p_hdlc->len_obr_dat;ii++)
		{
		printk("%02x ",p_hdlc->obr_buff[ii]);
		}
///	printf("%x\r\n ", 0xffff ^ pppfcs16( 0xffff, p_hdlc->obr_buff, p_hdlc->len_obr_dat-2));	// dbg!
	}
	;
	}
}
///====================================
void hdlc1_obr_frame(void *pdata)
{
///u8 addr= *((u8*)pdata);
hdlc_stat_t *p_hdlc=(hdlc_stat_t *)pdata;
u8 res_byte;
///u8 btmp=0x33;
///u16 cnt=0;
printk("\n\r=== hdlc1_obr_frame===  [%s]",pcTaskGetTaskName(NULL) );
for(;;)
	{
	xQueueReceive(p_hdlc->ev_rsv_frame,&res_byte,portMAX_DELAY);
	obr_hdlc1_cmd(p_hdlc);
	}
}

uint8_t hdlc1_send_dat(hdlc_stat_t *p_hdlc, u8 icmd, u8 *idat, u16 len)
{
uint8_t res_byte;
uint16_t htmp;
uint8_t res=FALSE;
int num_send=0;
u8 *p_tmp=p_hdlc->t_snd_buff1;
if (len == 0)
  return FALSE;
*p_tmp++  = CMD_PUT_DAT;
num_send++;
*p_tmp++  = icmd;
num_send++;
*p_tmp++  = len&0xff;
num_send++;
*p_tmp++  = (len>>8)&0xff;
num_send++;
memcpy(p_tmp, idat, len);
num_send += len;
p_hdlc->snd_buff1_len=num_send;
htmp=SEND_DAT;
htmp |= (icmd<<8);
if (xQueueSend(p_hdlc->ev_snd_frame, &htmp, TIMEOUT_SEND) == pdFAIL)
  {
  return FALSE;
  }
if(xQueuePeek(p_hdlc->ev_rsv_ack,&res_byte,TIMEOUT_RSV)==pdTRUE)
  {
  if(res_byte==icmd)
    {
    xQueueReceive(p_hdlc->ev_rsv_ack,&res_byte,portMAX_DELAY);
    res=TRUE;
    }
  }
else
  {
  res=FALSE;
  }	
return res;
}
int	hdlc1_send_to(hdlc_stat_t *p_hdlc,u8 type_send,u8 type_dat,u16 rez)
{
int res=0;
u16 len_dat;
int num_send=0;
u8 *p_tmp1;
u8 *p_tmp	 = p_hdlc->snd_buff;
if(type_send == SEND_REZ)
	{
	res=hdlc1_send_rez_to(p_hdlc,type_dat,rez);
	}
else if(type_send == SEND_ACK)
	{
	res=hdlc1_send_ack_to(p_hdlc,type_dat);
	}
else if(type_send == SEND_REQ_DAT)
	{
	*p_tmp++  = CMD_PUT_REQ_DAT;
	num_send++;
	*p_tmp++  = type_dat;
	num_send++;
	p_tmp1 = p_tmp+2;  
	len_dat = p_hdlc->fn_get_req_dat(type_dat,p_hdlc->offs_snd_buf, p_tmp1);
	*p_tmp++  = len_dat&0xff;
	num_send++;
	*p_tmp++  = (len_dat>>8)&0xff;
	num_send++;
	if(len_dat > HDLC1_MAX_FRAME_LEN)
		len_dat = HDLC1_MAX_FRAME_LEN;
	num_send += len_dat;
	res=hdlc1_send_iframe(p_hdlc,p_hdlc->snd_buff, num_send);
	}
else if(type_send == SEND_DAT)
	{
	num_send = p_hdlc->snd_buff_len;
	memcpy(p_tmp,p_hdlc->snd_buff,num_send);
	res=hdlc1_send_iframe(p_hdlc,p_hdlc->snd_buff, num_send);
	}
else if(type_send == SEND_REQ)
	{
	num_send = p_hdlc->snd_buff_len;
	memcpy(p_tmp,p_hdlc->snd_buff,num_send);
	res=hdlc1_send_iframe(p_hdlc,p_hdlc->snd_buff, num_send);
	}
return res;
}

void hdlc1_snd_task(void *pdata)
{
////t_frametx frame_tx;
u32 type_send;
hdlc_stat_t *p_hdlc=(hdlc_stat_t *)pdata;
///==================================
#if 0
for(;;)
{
send_char_bt(0x33);
vTaskDelay(5);
send_char_bt(0xcc);
vTaskDelay(5);
}
#endif
///==================================
////u8 addr= *((u8*)pdata);
////_printk("\n\r=== hdlc1_snd_task=== [%s]",pcTaskGetTaskName(NULL));


for(;;)
  {
  if(xQueueReceive(p_hdlc->ev_snd_frame,&type_send,portMAX_DELAY))
    hdlc1_send_to(p_hdlc,type_send&0xff,type_send>>8,type_send>>16);	
  }
}
///====================================


