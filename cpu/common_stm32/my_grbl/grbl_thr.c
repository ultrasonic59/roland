#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "printk.h"
#include "my_grbl.h"
#include "grbl_sys.h"
#include "my_stepper.h"
extern void serial_reset_read_buffer(void);
#if 0
extern uint8_t serial_read_rx(void);
extern uint8_t serial_get_rx_buffer_available();
extern uint8_t system_execute_line(char *line);
extern void system_clear_exec_alarm() ;
extern void system_clear_exec_motion_overrides() ;
extern void system_clear_exec_accessory_overrides() ;
static char buf_line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
#endif
TaskHandle_t  grbl_thread_handle;
void _delay_ms(uint32_t x)
{
#if 0
	u32 temp;
	SysTick->LOAD = (u32)72000000 / 8000;                     // Loading time
	SysTick->VAL = 0x00;                                            // Empty the counter
	SysTick->CTRL = 0x01;                                           // Start from bottom
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)));                             // Wait time arrive
	SysTick->CTRL = 0x00;                                            // Close the counter
	SysTick->VAL = 0X00;                                            // Empty the counter
#endif
vTaskDelay(x);        
}



void grbl_thread(void* pp)
{
printk("\n\r grbl_thread\n\r"); 
////  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
////  system_init();   // Configure pinout pins and pin-change interrupt
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.


for (;;) { 
  uint8_t prior_state = sys.state;
  memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
  sys.state = prior_state;
  sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
  sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
  sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
  memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
  sys_probe_state = 0;
  sys_rt_exec_state = 0;
  sys_rt_exec_alarm = 0;
  sys_rt_exec_motion_override = 0;
  sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
  serial_reset_read_buffer(); // Clear serial read buffer
  gc_init(); // Set g-code parser to default state
  spindle_init();
  limits_init();
  probe_init();
  plan_reset(); // Clear block buffer and planner variables
  st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
  plan_sync_position();
  gc_sync_position();

    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
  report_init_message();

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();
} 

}





