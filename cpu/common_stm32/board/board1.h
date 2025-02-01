#ifndef __BOARD_H__
#define __BOARD_H__
////======================================
#ifdef MSTEP_BRD
  #include "mstep_board.h"
#endif
#ifdef STEP_BRD
  #include "step_board.h"
#endif
#define VCP_RX_STACK_SIZE	        1024////( ( unsigned short ) 512 )
#define VCP_TX_STACK_SIZE	      1024//// ( ( unsigned short ) 512 )
#define GRBL_STACK_SIZE                 1024////
#define CAN_SEND_STACK_SIZE                 1024////
#define TST_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

#define         APP_PRIORITY	     ( tskIDLE_PRIORITY + 3 )	
#define APPLICATION_STACK_SIZE         (4000)
#define TST_TASK_PRIORITY		APP_PRIORITY ////		( tskIDLE_PRIORITY + 3 )
#define CAN_TASK_STACK_SIZE			1024            ////( configMINIMAL_STACK_SIZE + 50 )
#define CAN_TASK_PRIORITY		( tskIDLE_PRIORITY + 6 )
#define TIME_WAIT_RDY           200 
#define TIME_WAIT_ACK           100 
////======================================
extern uint32_t curr_x;
extern uint32_t curr_y;
extern uint32_t curr_z;
extern uint8_t stat_ready;
extern uint8_t curr_stat_x;
extern uint8_t curr_stat_y;
extern uint8_t curr_stat_z;

extern void tst_task( void *pvParameters );
extern void board_init(void);

#define msleep vTaskDelay

#endif