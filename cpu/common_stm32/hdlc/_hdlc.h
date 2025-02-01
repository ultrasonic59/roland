#ifndef __HDLC_H_
#define __HDLC_H_ 

#include "my_types.h"

//=====================

#ifndef TRUE 
#define	TRUE 0x1
#endif
#ifndef FALSE
#define	FALSE 0
#endif

#ifndef NULL
#define NULL 0                   
#endif
//=====================


///========================================
#define PPP_FRAME	0x7E
#define PPP_ESCAPE	0x7D
#define PPP_ESCAPE_BIT	0x20

///========== put to goat_cpu ====================

#define CMD_PUT_DAT  			0x3	
#define CMD_REQ_DAT  			0x4	
#define CMD_PUT_REQ_DAT  	        0x5	
#define CMD_PUT_CMD  			0x6	

///#define CMD_PUT_BUF  			0x5	
///#define CMD_PRG_FLASH  		0x6	
///#define CMD_VER_FLASH  		0x7	
////#define CMD_PRG_PAR  			0x8	
///#define CMD_PUT_CMD  			0x9	

///#define CMD_REQ_STAT  		0xa		

///========== get from dev ====================
///#define CMD_GET_DAT  			0x84	
#define CMD_ACK  					0x85
#define CMD_REZ  					0x86
///===================================
#define SEND_DAT			(0x1<<0)
#define SEND_ACK			(0x1<<1)
#define SEND_REQ_DAT			(0x1<<2)
#define SEND_REQ			(0x1<<3)
#define SEND_REZ			(0x1<<4)


/*
#ifndef MAX_FRAME_DATA
	#define MAX_FRAME_DATA	1000	////8192		// Макс. длина I-поля кадра
#endif
#ifndef MAX_FRAME_LEN
	#define MAX_FRAME_LEN	(MAX_FRAME_DATA+sizeof(hdlc_hdr_t)+2)// Макс. длина кадра (C+I+FCS)
#endif
*/
////#define HDLC1_MAX_LEN_DATA		512		//	4096				/// 0x2000 Макс. длина I-поля кадра
#define HDLC1_MAX_FRAME_DATA	1024	//	8192				/// 0x2000 Макс. длина I-поля кадра
///#define HDLC1_MASK_FRAME_DATA	HDLC1_MAX_FRAME_DATA-1	/// 0x1fff
#define HDLC1_MAX_FRAME_LEN	HDLC1_MAX_FRAME_DATA         ////   (HDLC1_MAX_FRAME_DATA+2)// Макс. длина кадра (C+I+FCS)

///========================================
#define TIMEOUT_ACK	100
#define TIMEOUT_SEND	10
#define TIMEOUT_RSV	500
///========================================

///==================================
typedef struct hdlc_stat_s_
{
///int		state;
///int		phase;
u16	frame_in_pos  ;				// Позиция для записи следующего принятого байта
u8 	*in_buf;				// Буфер принимаемого в данный момент кадра
u8 	*obr_buff;				// 
u16 len_obr_dat;
u8  in_esc_byte;				// Флаг принятия ESC-байта 
///u8  addr_res;
///u8  addr_snd;
u8 	*snd_buff;				// 
u16	snd_buff_len;
u8 	*t_snd_buff1;				// 
u16	snd_buff1_len;
///u8 	*t_snd_buff2;				// 
///u16	snd_buff2_len;

u16	frame_out_len;
u16	offs_snd_buf;
////void *t_rsv_buff;
xQueueHandle ev_rsv_ack;
xQueueHandle ev_rsv_dat;
xQueueHandle ev_rsv_frame;
xQueueHandle ev_snd_frame;
///xQueueHandle ua_rsv_dat;
xQueueHandle ua_snd_dat;
void_fn_u8 fn_send_byte;
int_fn_u8_pvoid fn_set_rec_dat;
///extern int hdlc1_set_rec_dat(u8 cmd,void *in_buf);
//// int_fn_u8_pvoid fn_get_rez;
///extern int hdlc1_get_rez(u8 type_rez,void *obuf);
u16_fn_u8_u16_pu8 fn_get_req_dat;
////???u8_fn_pu8_u16 fn_send_frame;

///extern u16 hdlc1_get_req_dat(u8 num_req,u8 *buf);

int bytes_sent;
	int bytes_received;
///	int bytes_over_received;
	int	frames_sent;
	int	frames_received;
}hdlc_stat_t;
///================================
////extern int need_send;
extern int checkfcs16(unsigned char *cp, register int len );
extern void hdlc1_init(hdlc_stat_t *p_hdlc);
extern uint8_t hdlc1_send_iframe_s(hdlc_stat_t *p_hdlc,u8 *data, int len);
extern uint8_t hdlc1_send_iframe_ns(hdlc_stat_t *p_hdlc,u8 *data, int len);
										
///extern int req_dat_goat(u8 icmd,u16 offs,void *obuf);
///extern BOOL send_dat_goat(u8 icmd,u8 *idat,u16 len);
////extern int hdlc1_on_bytein(hdlc_stat_t *p_hdlc, u8 in_bt );
extern uint8_t hdlc1_send_dat(hdlc_stat_t *p_hdlc, u8 icmd, u8 *idat, u16 len);
extern void obr_hdlc1_cmd(hdlc_stat_t *p_hdlc);
extern void hdlc1_obr_frame(void *pdata);
extern void hdlc1_snd_task(void *pdata);

typedef struct
{
	u16 type_send;
	u16 len;
	u8 *p_data;
} t_frametx;

#endif ////__HDLC_H_
