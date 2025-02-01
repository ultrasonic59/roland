#ifndef __CAN_CMDS_H_
#define __CAN_CMDS_H_
///=========================================================
#define CAN_MAX_LEN_QUEU 16
#define ID_MASK         (uint32_t)(0x7e0)

#define AXIS_X           (0x1<<0)
#define AXIS_Y           (0x1<<1)
#define AXIS_Z           (0x1<<2)

#if STEP_X
  #define AXIS_BRD AXIS_X
#elif STEP_Y
  #define AXIS_BRD AXIS_Y
#elif STEP_Z
  #define AXIS_BRD AXIS_Z
#else 
  #error "\n\r=== STEP_... nodefined ==="; 
#endif

#define ID_BRD (AXIS_BRD<<5)

#if 0
#define ID_X_CMD         (0x1<<5)
#define ID_Y_CMD         (0x1<<6)
#define ID_Z_CMD         (0x1<<7)
#endif

#define ID_MASTER_CMD    (0x1<<9)

#define DIR_X           (0x1<<0)
#define DIR_Y           (0x1<<1)
#define DIR_Z           (0x1<<2)

#define READY_X           (0x1<<0)
#define READY_Y           (0x1<<1)
#define READY_Z           (0x1<<2)

#if 0
#define AXIS_X           (0x1<<0)
#define AXIS_Y           (0x1<<1)
#define AXIS_Z           (0x1<<2)
#endif

#define GO_CMD                0x1
#define GET_STAT_CMD          0x2
#define PUT_STAT_CMD          0x3
#define PUT_ACK               0x4
#define PRG_PARAM             0x5
#define CHECK_CONN            0x6
#define ACK_CONN              0x7

#define RD_EEPROM_REQ         0x8
#define RD_EEPROM_ANS         0x9
#define WR_EEPROM_REQ         0xA
#define WR_EEPROM_ANS         0xB
#define RD_FLASH_REQ          0xC
#define RD_FLASH_ANS          0xD

#define CMD_STOP              0xF

///===== booter cmd =============
#define CHECK_ERASE_SECTORS   0xF3
#define SET_ADDR_PRG          0xF4
#define GO_TO_BOOTER          0xF5
#define GO_TO_APP             0xF6
#define ERASE_SECTORS         0xF7
#define ERASE_OK              0xF8
#define PRG_DAT               0xF9
#define PRG_DAT_OK            0xFA
#define GET_BOOT_STAT         0xFB
#define PUT_BOOT_STAT         0xFC

#define BOOTER_STATE_IDLE     0xF0
#define BOOTER_STATE_ERASE    0xF1
#define BOOTER_STATE_PROG     0xF2
#define BOOTER_NO_BOOT        0xF3


#define STATE_IDLE           0
#define STATE_READY          1
#define STATE_MOVE           2
#define STATE_ERROR          3
#define STATE_BOOT          4

#define LEN_ACK_QU      8

#define CAN_REQ_STAT_NUM_BYTES 1

typedef struct  go_cmd_s_{
  uint8_t  cmd;                       /// 
  uint8_t  dirs;                     /// Dirs X,Y,Z ....
  uint16_t  step_per;                 /// period step
  uint32_t  steps;                 /// steps X,Y , Z 
}go_cmd_t;

typedef struct  put_stat_cmd_s_{
  uint8_t  cmd;                         /// 
  uint8_t  axis;                        /// X,Y , Z 
  uint8_t  state;                     /// ready,move,error
  uint8_t  rsv;
  uint32_t  coord;                      /// 
}put_stat_cmd_t;

typedef struct  put_boot_stat_cmd_s_{
  uint8_t  cmd;                         /// 
  uint8_t  axis;                        /// X,Y , Z 
  uint8_t  state;                     /// 
}put_boot_stat_cmd_t;

typedef struct  ack_s_{
  uint8_t  ack_cmd;                         /// 
  uint8_t  axis;                        /// X,Y , Z 
}ack_t;

typedef struct  put_ack_s_{
  uint8_t  cmd;                         /// 
  ack_t   ack;                         /// 
}put_ack_t;

typedef struct  prg_param_cmd_s_{
  uint8_t   cmd;                         /// 
  uint16_t  num_par;                        /// 
  uint8_t   len_par;                     ///bytes 
  uint32_t  par_val;                      /// 
}prg_param_cmd_t;

#define MAX_CAN_NUM_BYTES  8
typedef struct can_cmd_s{
uint32_t id;
uint8_t num_bytes;
uint8_t data[MAX_CAN_NUM_BYTES];
}can_cmd_t;

#define EEPROM_MAX_NUM_DATES 2
typedef struct rd_eeprom_req_s{
uint8_t num_dates;
uint8_t addr;
}rd_eeprom_req_t;

typedef struct rd_eeprom_ans_s{
uint8_t num_dates;
uint8_t addr;
uint16_t data[EEPROM_MAX_NUM_DATES ];
}rd_eeprom_ans_t,wr_eeprom_req_t;

typedef struct wr_eeprom_ans_s{
uint8_t num_dates;
uint8_t addr;
}wr_eeprom_ans_t;

typedef struct rd_flash_req_s{
uint32_t addr;
}rd_flash_req_t;
typedef struct rd_flash_ans_s{
uint32_t addr;
uint16_t data;
}rd_flash_ans_t;

///============ for progr =====================
#define MAX_NUM_WORDS_PRG     3
typedef struct  prg_flash_cmd_s_{
  uint8_t    cmd;                         /// 
  uint8_t    num_bytes;                   ///bytes  
  uint16_t    data[MAX_NUM_WORDS_PRG];                    /// 
}prg_flash_cmd_t;

typedef struct  set_flash_addr_s_{
  uint8_t    cmd;                      /// 
  uint32_t   b_addr;                   ///  
}set_flash_addr_t;

///=========================================================
extern void mc_reset(void);
extern int put_can_cmd_go(uint8_t dirs,uint16_t per
                   ,uint32_t step_x
                   ,uint32_t step_y
                   ,uint32_t step_z);

extern int put_can_cmd_stat(uint8_t state
                   ,uint32_t coord);
extern int put_can_ack(uint8_t cmd );
extern int obr_can_cmd(uint8_t *data);
extern int put_can_boot_ans(uint8_t cmd,uint8_t state);
extern uint8_t check_erase_sectors(uint8_t *data);
extern int go_cmd(go_cmd_t *p_go_cmd);
extern int stop_mot_cmd(void);

///=====================================
#define ERROR_OK            0

#define ERROR_NUM_BYTES_PRG 3
#define ERROR_ADDR_PRG      4
#define ERROR_FLAH_PRG      5
#define ERROR_ERRASE        6

///=====================================
#endif  //// __CAN_CMDS_H_
