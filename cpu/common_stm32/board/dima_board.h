#ifndef __DIMA_BOARD_H__
#define __DIMA_BOARD_H__
////=============================================

#include "stm32f2xx_conf.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_spi.h"
#include "stm32f2xx_flash.h"
#include "misc.h"
#include "flash_if.h"
////=============================================
/*
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGE0_ID               FLASH_Sector_2

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x4000))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
#define PAGE1_ID               FLASH_Sector_3
*/
/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x08008000) /* EEPROM emulation start address:
                                                      after 16KByte of used Flash memory */
#define EEPROM_START_SECTOR    FLASH_Sector_2
#define PAGE0_SECTOR    FLASH_Sector_2                                                     
#define PAGE1_SECTOR    FLASH_Sector_3                                                     

#define EEPROM_PAGE_SIZE   ((uint32_t)0x4000)           ////16 KB
/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + 0x000))
#define PAGE0_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (EEPROM_PAGE_SIZE - 1)))
#define PAGE0_ID               FLASH_Sector_2

#define PAGE1_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + EEPROM_PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (2 * EEPROM_PAGE_SIZE - 1)))
#define PAGE1_ID               FLASH_Sector_3

////=============================================
#define APP_BASE_ADDRESS        ((uint32_t)0x08010000)
#define APP_PAGE_SIZE           ((uint32_t)0x10000)           ////64 KB
#define APP_END_ADDRESS         ((uint32_t)(APP_BASE_ADDRESS + (APP_PAGE_SIZE - 1)))

#define BOOT_BASE_ADDRESS        ((uint32_t)0x08000000)
#define BOOT_PAGE_SIZE           ((uint32_t)0x8000)           ////32 KB

////=============================================
#define	APB1_pres	4
#define APB2_pres	2

#define DEF_MOT_TIM_PERIOD          500
#define DEF_MOT_TIM_PRESC           64  ////8

#define MAX_PER         64000
#define MIN_PER         10        

////============================================
#define ENC_TIM_PERIOD 0Xffff

#define MOT_STEP_TIM_IRQHandler	 TIM1_CC_IRQHandler


#define MOT_TIM_IRQN    TIM1_CC_IRQn
////=============== TST2============================
#define TST2_PIN	    	GPIO_Pin_5
#define TST2_PIN_NPIN	    	5
#define TST2_PIN_GPIO	    	GPIOB
#define TST2_PIN_RCC  	        RCC_AHB1Periph_GPIOB
////=============== TST3 ============================
#define TST3_PIN	    	GPIO_Pin_2
#define TST3_PIN_NPIN	    	2
#define TST3_PIN_GPIO	    	GPIOD
#define TST3_PIN_RCC  	        RCC_AHB1Periph_GPIOD
////=============== TST4 ============================
#define TST4_PIN	    	GPIO_Pin_3
#define TST4_PIN_NPIN	    	3
#define TST4_PIN_GPIO	    	GPIOB
#define TST4_PIN_RCC  	        RCC_AHB1Periph_GPIOB
////=============== TST5 ============================
#define TST5_PIN	    	GPIO_Pin_11
#define TST5_PIN_NPIN	    	11
#define TST5_PIN_GPIO	    	GPIOC
#define TST5_PIN_RCC  	        RCC_AHB1Periph_GPIOC
////=============== TST6 ============================
#define TST6_PIN	    	GPIO_Pin_12
#define TST6_PIN_NPIN	    	12
#define TST6_PIN_GPIO	    	GPIOC
#define TST6_PIN_RCC  	        RCC_AHB1Periph_GPIOC
////=============== TST7 ============================
#define TST7_PIN	    	GPIO_Pin_15
#define TST7_PIN_NPIN	    	15
#define TST7_PIN_GPIO	    	GPIOA
#define TST7_PIN_RCC  	        RCC_AHB1Periph_GPIOA
////=============== TST8 ============================
#define TST8_PIN	    	GPIO_Pin_10
#define TST8_PIN_NPIN	    	10
#define TST8_PIN_GPIO	    	GPIOC
#define TST8_PIN_RCC  	        RCC_AHB1Periph_GPIOC
////=========== DBG_TX ===============================
#define UART_DBG_TX_PIN	    	GPIO_Pin_6
#define UART_DBG_TX_PIN_NPIN	    	6
#define UART_DBG_TX_GPIO        GPIOC
#define UART_DBG_TX_RCC  	RCC_AHB1Periph_GPIOC
////=========== DBG_RX ===============================
#define UART_DBG_RX_PIN	    	GPIO_Pin_7
#define UART_DBG_RX_PIN_NPIN	7
#define UART_DBG_RX_GPIO        GPIOC
#define UART_DBG_RX_RCC  	RCC_AHB1Periph_GPIOC
////============== CAN1 ============================
#define CAN1_CLK                    RCC_APB1Periph_CAN1
#define CAN1_AF_PORT                GPIO_AF_CAN1
#define CAN1_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define CAN1_GPIO_PORT              GPIOB
////============== CAN1_RX ============================
#define CAN1_RX_PIN	    	GPIO_Pin_8
#define CAN1_RX_PIN_SOURCE	GPIO_PinSource8
#define CAN1_RX_SOURCE              GPIO_PinSource8
#define CAN1_RX_PIN_NPIN	8
#define CAN1_RX_PIN_GPIO	GPIOB
#define CAN1_RX_PIN_RCC  	RCC_AHB1Periph_GPIOB
////============== CAN1_TX ============================
#define CAN1_TX_PIN	    	GPIO_Pin_9
#define CAN1_TX_PIN_SOURCE	GPIO_PinSource9
#define CAN1_TX_SOURCE              GPIO_PinSource9
#define CAN1_TX_PIN_NPIN	9
#define CAN1_TX_PIN_GPIO	GPIOB
#define CAN1_TX_PIN_RCC  	RCC_AHB1Periph_GPIOB
////============== CAN1_INH ============================
#define CAN1_INH_PIN	    	GPIO_Pin_0
#define CAN1_INH_PIN_NPIN	    0
#define CAN1_INH_PIN_GPIO	    GPIOC
#define CAN1_INH_PIN_RCC  	RCC_AHB1Periph_GPIOC

////=============== MOT_FAULT ============================
#define MOT_FAULT_PIN	    		GPIO_Pin_11
#define MOT_FAULT_PIN_NPIN	    	11
#define MOT_FAULT_PIN_GPIO	    	GPIOB
#define MOT_FAULT_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_RESET ============================
#define MOT_RESET_PIN	    	GPIO_Pin_9
#define MOT_RESET_PIN_NPIN	9
#define MOT_RESET_PIN_GPIO	GPIOA
#define MOT_RESET_PIN_RCC  	RCC_AHB1Periph_GPIOA
////=============== MOT_SLEEP ============================
#define MOT_SLEEP_PIN	    	GPIO_Pin_10
#define MOT_SLEEP_PIN_NPIN	10
#define MOT_SLEEP_PIN_GPIO	GPIOA
#define MOT_SLEEP_PIN_RCC  	RCC_AHB1Periph_GPIOA
////=============== MOT_STEP ============================
#define MOT_STEP_PIN	    	GPIO_Pin_8
#define MOT_STEP_PIN_NPIN	8
#define MOT_STEP_PIN_GPIO	GPIOA
#define MOT_STEP_PIN_RCC  	RCC_AHB1Periph_GPIOA
#define MOT_STEP_TIM  	        TIM1
#define MOT_STEP_TIM_RCC  	RCC_APB2Periph_TIM1
////=============== MOT_DIR ============================
#define MOT_DIR_PIN	    	GPIO_Pin_9
#define MOT_DIR_PIN_NPIN	    9
#define MOT_DIR_PIN_GPIO	    GPIOC
#define MOT_DIR_PIN_RCC  	RCC_AHB1Periph_GPIOC
////=============== MOT_BIN1 ============================
#define MOT_BIN1_PIN	    	GPIO_Pin_8
#define MOT_BIN1_PIN_NPIN	    8
#define MOT_BIN1_PIN_GPIO	    GPIOC
#define MOT_BIN1_PIN_RCC  	RCC_AHB1Periph_GPIOC
////=============== MOT_BIN2 ============================
#define MOT_BIN2_PIN	    	GPIO_Pin_1
#define MOT_BIN2_PIN_NPIN	    1
#define MOT_BIN2_PIN_GPIO	    GPIOB
#define MOT_BIN2_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_STALLN ============================
#define MOT_STALLN_PIN	    	GPIO_Pin_10
#define MOT_STALLN_PIN_NPIN	    10
#define MOT_STALLN_PIN_GPIO	    GPIOB
#define MOT_STALLN_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_SPI_SCK ============================
#define MOT_SPI_SCK_PIN	    	GPIO_Pin_13
#define MOT_SPI_SCK_PIN_NPIN	    13
#define MOT_SPI_SCK_PIN_GPIO	    GPIOB
#define MOT_SPI_SCK_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_SPI_MOSI ============================
#define MOT_SPI_MOSI_PIN	    	GPIO_Pin_15
#define MOT_SPI_MOSI_PIN_NPIN	    15
#define MOT_SPI_MOSI_PIN_GPIO	    GPIOB
#define MOT_SPI_MOSI_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_SPI_SCS ============================
#define MOT_SPI_SCS_PIN	    	GPIO_Pin_12
#define MOT_SPI_SCS_PIN_NPIN	    12
#define MOT_SPI_SCS_PIN_GPIO	    GPIOB
#define MOT_SPI_SCS_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_SPI_MISO ============================
#define MOT_SPI_MISO_PIN	    	GPIO_Pin_14
#define MOT_SPI_MISO_PIN_NPIN	    14
#define MOT_SPI_MISO_PIN_GPIO	    GPIOB
#define MOT_SPI_MISO_PIN_RCC  	RCC_AHB1Periph_GPIOB

#define MOT_SPI                 SPI2
#define MOT_SPI_PeriphClockCmd 	RCC_APB1PeriphClockCmd
#define MOT_SPI_RCC 	RCC_APB1Periph_SPI2
#define MOT_SPI_AF      GPIO_AF_SPI2
////=============== CONC1 ============================
#define CONC1_PIN	    GPIO_Pin_2
#define CONC1_PIN_NPIN	    2
#define CONC1_PIN_GPIO	    GPIOA
#define CONC1_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== CONC0 ============================
#define CONC0_PIN	    GPIO_Pin_1
#define CONC0_PIN_NPIN	    1
#define CONC0_PIN_GPIO	    GPIOA
#define CONC0_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== ENC_A ============================
////#define ENC_A_PIN	    GPIO_Pin_0
////#define ENC_A_PIN_NPIN	    0
////#define ENC_A_PIN_GPIO	    GPIOA
////#define ENC_A_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////#define ENC_TIM  	    TIM5
////#define ENC_TIM_RCC  	    RCC_APB1Periph_TIM5
////=============== ENC_B ============================
////#define ENC_B_PIN	    GPIO_Pin_1
////#define ENC_B_PIN_NPIN	    1
////#define ENC_B_PIN_GPIO	    GPIOA
////#define ENC_B_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== ENC_C ============================
/////#define ENC_C_PIN	    GPIO_Pin_2
////#define ENC_C_PIN_NPIN	    2
////#define ENC_C_PIN_GPIO	    GPIOA
////#define ENC_C_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== UART_RX1 ============================
#define UART_RX1_PIN	    	    GPIO_Pin_7
#define UART_RX1_PIN_NPIN	    7
#define UART_RX1_PIN_GPIO	    GPIOB
#define UART_RX1_PIN_RCC  	    RCC_AHB1Periph_GPIOB
////=============== UART_TX1 ============================
#define UART_TX1_PIN	    	    GPIO_Pin_6
#define UART_TX1_PIN_NPIN	    6
#define UART_TX1_PIN_GPIO	    GPIOB
#define UART_TX1_PIN_RCC  	    RCC_AHB1Periph_GPIOB
////============== CAN1_RX ============================
#define CAN1_RX_PIN	    	GPIO_Pin_8
#define CAN1_RX_PIN_NPIN	8
#define CAN1_RX_PIN_GPIO	GPIOB
#define CAN1_RX_PIN_RCC  	RCC_AHB1Periph_GPIOB
////============== CAN1_TX ============================
#define CAN1_TX_PIN	    	GPIO_Pin_9
#define CAN1_TX_PIN_NPIN	9
#define CAN1_TX_PIN_GPIO	GPIOB
#define CAN1_TX_PIN_RCC  	RCC_AHB1Periph_GPIOB
////============== CAN1_INH ============================
#define CAN1_INH_PIN	    	GPIO_Pin_0
#define CAN1_INH_PIN_NPIN	    0
#define CAN1_INH_PIN_GPIO	    GPIOC
#define CAN1_INH_PIN_RCC  	RCC_AHB1Periph_GPIOC
////================= USB ======================
#define USB_DP_PIN	    	GPIO_Pin_12
#define USB_DP_PIN_NPIN	    12
#define USB_DP_PIN_GPIO	    GPIOA
#define USB_DP_PIN_RCC  RCC_APB2Periph_GPIOA

#define USB_DM_PIN	    	GPIO_Pin_11
#define USB_DM_PIN_NPIN	    11
#define USB_DM_PIN_GPIO	    GPIOA
#define USB_DM_PIN_RCC  RCC_APB2Periph_GPIOA
////=================================================================
////#define UART_DBG_TX_GPIO_PORT   GPIOC
////#define UART_DBG_RX_GPIO_PORT   GPIOC
////=================================================================
#define GPIO_CLK_INIT           RCC_AHB1PeriphClockCmd
#define UART_DBG_CLK            RCC_APB2Periph_USART6
#define UART_DBG_CLK_INIT       RCC_APB2PeriphClockCmd

#define UART_DBG_RX_GPIO_CLK    RCC_AHB1Periph_GPIOC
#define UART_DBG_RX_AF          GPIO_AF_USART6
#define UART_DBG_TX_AF          GPIO_AF_USART6
#define UART_DBG                USART6
///#define UART_DBG_TX_SOURCE      GPIO_Pin_2
///#define UART_DBG_RX_SOURCE      GPIO_Pin_3
////=================================================================

#define ADDR_MOT_CTRL   0
#define ADDR_MOT_TORQUE 1
#define ADDR_MOT_OFF    2
#define ADDR_MOT_BLANK  3
#define ADDR_MOT_DECAY  4
#define ADDR_MOT_STALL  5
#define ADDR_MOT_DRIVE  6
#define ADDR_MOT_STATUS 7

#define DEF_MOT_REJ   2
////======================================
// CTRL Register
typedef struct CTRL_Register
{
uint16_t ENBL:1;		// bit 0
uint16_t RDIR:1;		// bit 1
uint16_t RSTEP:1;		// bit 2
uint16_t MODE:4;		// bits 6-3
uint16_t EXSTALL:1;	// bit 7
uint16_t ISGAIN:2;	// bits 9-8
uint16_t DTIME:2;		// bits 11-10
}CTRL_Register_t;

// TORQUE Register
typedef struct TORQUE_Register
{
uint16_t TORQUE:8;	// bits 7-0
uint16_t SIMPLTH:3;  	// bits 10-8
}TORQUE_Register_t;

// OFF Register
typedef struct OFF_Register
{
uint16_t TOFF:8;		// bits 7-0
uint16_t PWMMODE:1;  	// bit 8
}OFF_Register_t;

// BLANK Register
typedef struct BLANK_Register
{
uint16_t TBLANK:8;	// bits 7-0
uint16_t ABT:1;  		// bit 8
}BLANK_Register_t;

// DECAY Register
typedef struct DECAY_Register
{
uint16_t TDECAY:8;	// bits 7-0
  uint16_t DECMOD:3;  	// bits 10-8
}DECAY_Register_t;

// STALL Register
typedef struct STALL_Register
{
uint16_t SDTHR:8;		// bits 7-0
uint16_t SDCNT:2;		// bits 9-8
  uint16_t VDIV:2;  	// bits 11-10
}STALL_Register_t;

// DRIVE Register
typedef struct DRIVE_Register
{
uint16_t OCPTH:2;	// bits 1-0
uint16_t OCPDEG:2;	// bits 3-2
uint16_t TDRIVEN:2;	// bits 5-4
uint16_t TDRIVEP:2;	// bits 7-6
uint16_t IDRIVEN:2;	// bits 9-8
uint16_t IDRIVEP:2;  	// bits 11-10
}DRIVE_Register_t;

// STATUS Register
typedef struct STATUS_Register
{
uint16_t OTS:1;		// bit 0
uint16_t AOCP:1;	// bit 1
uint16_t BOCP:1;	// bit 2
uint16_t APDF:1;	// bit 3
uint16_t BPDF:1;	// bit 4
uint16_t UVLO:1;	// bit 5
uint16_t STD:1;		// bit 6
uint16_t STDLAT:1;  	// bit 7
}STATUS_Register_t;

typedef union cmd_param_u
{
uint8_t bpar[4];
uint16_t hpar[2];
int32_t wpar;
}cmd_param_t; 

typedef struct cmd_s_
{
uint8_t cmd;
cmd_param_t par;
}cmd_t;
extern cmd_t cur_cmd;
////========================================================================
////#define EXEC_CYCLE_STOP     (0x1<<2) // bitmask 00000100
////#define STATE_HOMING        (0x1<<2) // Performing homing cycle


#ifndef SEGMENT_BUFFER_SIZE
 #define SEGMENT_BUFFER_SIZE 10
#endif
#define N_AXIS 3 // Number of axes

#define X_AXIS 0 // Axis indexing value.
#define Y_AXIS 1
#define Z_AXIS 2

#define X_STEP_BIT 0
#define Y_STEP_BIT 1
#define Z_STEP_BIT 2
#define  X_DIRECTION_BIT 0
#define  Y_DIRECTION_BIT 1
#define  Z_DIRECTION_BIT 2
typedef struct system_s_{
  uint8_t state;               // Tracks the current system state of Grbl.
  uint8_t abort;               // System abort flag. Forces exit back to main loop for reset.             
  uint8_t suspend;             // System suspend bitflag variable that manages holds, cancels, and safety door.
  uint8_t soft_limit;          // Tracks soft limit errors for the state machine. (boolean)
  uint8_t step_control;        // Governs the step segment generator depending on system state.
  uint8_t probe_succeeded;     // Tracks if last probing cycle was successful.
  uint8_t homing_axis_lock;    // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
  uint8_t f_override;          // Feed rate override value in percent
  uint8_t r_override;          // Rapids override value in percent
  uint8_t spindle_speed_ovr;   // Spindle speed value in percent
  uint8_t spindle_stop_ovr;    // Tracks spindle stop override states
  uint8_t report_ovr_counter;  // Tracks when to add override data to status reports.
  uint8_t report_wco_counter;  // Tracks when to add work coordinate offset data to status reports.
	#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
		uint8_t override_ctrl;     // Tracks override control states.
	#endif
	#ifdef VARIABLE_SPINDLE
    float spindle_speed;
  #endif
} system_t;
extern system_t sys;

typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  #ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
  #endif
} st_block_t;

////========================================================================
// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;           // Number of step events to be executed for this segment
  uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
  uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
  #endif
} segment_t;


// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct stepper_s_{
#if 0  
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif

  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise

  PORTPINDEF step_outbits;         // The next stepping-bits to be output
  PORTPINDEF dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

#endif 
  // Used by the bresenham line algorithm
  uint32_t counter_x;        // Counter variables for the bresenham line tracer
  uint32_t counter_y;
  uint32_t counter_z;
    
  uint16_t step_count;       // Steps remaining in line segment motion
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
segment_t *exec_segment;  // Pointer to the segment being executed
uint8_t dir_outbits;
uint8_t step_outbits;         // The next stepping-bits to be output
} stepper_t;


////======================================
#define MOTOR_TASK_STACK_SIZE			1024            ////( configMINIMAL_STACK_SIZE + 50 )
#define MOTOR_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define CAN_SEND_STACK_SIZE                     1024////
#define CAN_TASK_STACK_SIZE			1024            ////( configMINIMAL_STACK_SIZE + 50 )
#define CAN_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define APP_PRIORITY	                       (6)	

#define TST_TASK_STACK_SIZE			1024            ////( configMINIMAL_STACK_SIZE + 50 )
#define TST_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
////=============Addr eeprom ==========================================
#define ADDR_EEPROM_BOOT_WORK   0x0
#define VAL_EEPROM_WORK  0xAA55
#define ADDR_KS_APP             0x1
#define ADDR_EEPROM_SIZEL_APP  0x2
#define ADDR_EEPROM_SIZEH_APP  0x3
#define ADDR_EEPROM_MOT_REJ    0x4

#define MAX_MOT_REJ    0x8

////============================================
#if STEP_Z
  #define MASK_CON0 0x1
  #define MASK_CON1 0x2
#else
///  #define MASK_CON 0x3
  #define MASK_CON0  0x1
  #define MASK_CON1  0x2
  
#endif

////============================================
////=================================================================
extern int32_t cur_coord;
extern uint8_t cur_stat;

extern void mot_spi_init(void);
extern void init_step_mot(void);
extern void ena_mot(uint8_t ena_dis);
extern void motor_init(void);

extern void init_gpio(void);
extern void hw_board_init(void);
extern void put_mot_nstep(uint32_t nstep);
///extern void set_step_per(uint16_t step_per);
extern void set_mot_per(uint16_t per);

extern void  set_sleep_mot(uint8_t idat);
extern void  set_ena_mot(uint8_t idat);
extern void  set_dir_mot(uint8_t idat);
extern void  set_reset_mot(uint8_t idat);
extern void set_mot_rej(uint8_t rej);
extern int send_char_dbg (int c) ;
extern int get_byte_dbg(void) ;
extern void can1_init(void);
extern void motor_task( void *pvParameters );
extern int check_push_key_dbg(void);
extern void print_mot_reg(void);

////#define dbg_sendchar  sendchar6 
////#define dbg_get_byte get_byte6
////#define check_push_key get_byte6

////=============================================
#endif ////__DIMA_BOARD_H__



	
