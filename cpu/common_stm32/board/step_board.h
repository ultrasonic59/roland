#ifndef __STEP_BOARD_H__
#define __STEP_BOARD_H__
////=============================================

#include "stm32f2xx_conf.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_spi.h"
#include "misc.h"

////=============================================
#define	APB1_pres	4
#define APB2_pres	2

#define MOT_TIM_PERIOD     8000
#define MOT_TIM_PRESC     8

#define MOT_STEP_TIM_IRQHandler	 TIM1_CC_IRQHandler


#define MOT_TIM_IRQN    TIM1_CC_IRQn
#if 0
////=============== TST1 ============================
#define TST1_PIN	    	GPIO_Pin_4
#define TST1_PIN_NPIN	    	4
#define TST1_PIN_GPIO	    	GPIOB
#define TST1_PIN_RCC  	        RCC_AHB1Periph_GPIOB
#endif
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
////=============== MOT_FAULT ============================
#define MOT_FAULT_PIN	    		GPIO_Pin_11
#define MOT_FAULT_PIN_NPIN	    	11
#define MOT_FAULT_PIN_GPIO	    	GPIOB
#define MOT_FAULT_PIN_RCC  	RCC_AHB1Periph_GPIOB
#if 0
////=============== MOT_M2 ============================
#define MOT_M2_PIN	    	GPIO_Pin_1
#define MOT_M2_PIN_NPIN	        1
#define MOT_M2_PIN_GPIO	        GPIOB
#define MOT_M2_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_M1 ============================
#define MOT_M1_PIN	    	GPIO_Pin_2
#define MOT_M1_PIN_NPIN	        2
#define MOT_M1_PIN_GPIO	        GPIOB
#define MOT_M1_PIN_RCC  	RCC_AHB1Periph_GPIOB
////=============== MOT_M0 ============================
#define MOT_M0_PIN	    	GPIO_Pin_10
#define MOT_M0_PIN_NPIN	        10
#define MOT_M0_PIN_GPIO	        GPIOB
#define MOT_M0_PIN_RCC  	RCC_AHB1Periph_GPIOB

////=============== MOT_ENA ============================
#define MOT_ENA_PIN	    	GPIO_Pin_11
#define MOT_ENA_PIN_NPIN	11
#define MOT_ENA_PIN_GPIO	GPIOB
#define MOT_ENA_PIN_RCC  	RCC_AHB1Periph_GPIOB
#endif

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
////=============== CONC ============================
#define CONC_PIN	    GPIO_Pin_4
#define CONC_PIN_NPIN	    4
#define CONC_PIN_GPIO	    GPIOB
#define CONC_PIN_RCC  	    RCC_AHB1Periph_GPIOB
////=============== ENC_A ============================
#define ENC_A_PIN	    GPIO_Pin_0
#define ENC_A_PIN_NPIN	    0
#define ENC_A_PIN_GPIO	    GPIOA
#define ENC_A_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== ENC_B ============================
#define ENC_B_PIN	    GPIO_Pin_1
#define ENC_B_PIN_NPIN	    1
#define ENC_B_PIN_GPIO	    GPIOA
#define ENC_B_PIN_RCC  	    RCC_AHB1Periph_GPIOA
////=============== ENC_C ============================
#define ENC_C_PIN	    GPIO_Pin_2
#define ENC_C_PIN_NPIN	    2
#define ENC_C_PIN_GPIO	    GPIOA
#define ENC_C_PIN_RCC  	    RCC_AHB1Periph_GPIOA
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
#define GPIO_CLK_INIT     RCC_AHB1PeriphClockCmd
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

////======================================
extern void mot_spi_init(void);

extern void init_gpio(void);
extern void hw_board_init(void);
extern void put_mot_nstep(uint32_t nstep);
extern void  set_sleep_mot(uint8_t idat);
extern void  set_ena_mot(uint8_t idat);
extern void  set_dir_mot(uint8_t idat);
extern void  set_reset_mot(uint8_t idat);
extern void set_mot_rej(uint8_t rej);
extern int sendchar6 (int c) ;
extern int get_byte6(void) ;

#define dbg_sendchar  sendchar6 
#define dbg_get_byte get_byte6

////=============================================
#endif ////__STEP_BOARD_H__



	
