#ifndef __SNAIL_CAN_CMDS_H__
#define __SNAIL_CAN_CMDS_H__
////======================================
////======================================
#define CAN_FILTR_ID_X 0x20
#define CAN_FILTR_ID_Y 0x40
#define CAN_FILTR_ID_Z 0x80

#define CAN_FILTR_MASK 0x7F0

#if STEP_X
	#define CAN_FILTR_ID CAN_FILTR_ID_X
#elif STEP_Y
	#define CAN_FILTR_ID CAN_FILTR_ID_Y
#elif STEP_Z
	#define CAN_FILTR_ID CAN_FILTR_ID_Z
#endif

////============= CAN ====================
#define SET_STOP        0x10             ////Avaria
#define SET_START       0x11             ///

#define GET_STATE       0x12             ////
#define SET_HOME        0x13             ////

#define SET_STEP_X     0x20             ////set step,speed,dir X
#define SET_STEP_Y     0x40             ////set step,speed,dir Y
#define SET_STEP_Z     0x80             ////set step,speed,dir Z
#define PUT_STATE_X    0x100             ////
#define PUT_STATE_Y    0x101             ////
#define PUT_STATE_Z    0x102             ////
////========== CAN FILTR =====================
////======================================
typedef struct step_cmd_s_
{
uint16_t steps;
uint16_t b_speed;
uint16_t e_speed;
uint16_t flags;
}step_cmd_t;


#endif