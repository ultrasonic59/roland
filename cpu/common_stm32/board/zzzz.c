#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

////#include "stm32f2xx_gpio.h"
#include "board.h"
////#include "my_grbl.h"
#include "printk.h"

////#define TEST_PIN ZSTP_PIN
////#define TEST_PIN_GPIO ZSTP_PIN_GPIO

void  put_tst_pin(uint8_t idat)
{
#if 1  
if(idat&0x1)
  {
  GPIO_SetBits(TST3_PIN_GPIO, TST3_PIN);
   GPIO_SetBits(TST2_PIN_GPIO, TST2_PIN);
  ////GPIO_SetBits(TST7_PIN_GPIO, TST7_PIN);
  }
else
  {
   GPIO_ResetBits(TST3_PIN_GPIO, TST3_PIN);
   GPIO_ResetBits(TST2_PIN_GPIO, TST2_PIN);
   ////GPIO_ResetBits(TST7_PIN_GPIO, TST7_PIN);
  }
#endif 
}
////===============================================================
// Method to store coord data parameters into EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{
}
// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
  
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
#if 0
  uint32_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  if (!(memcpy_from_eeprom_with_checksum((char*)coord_data, addr, sizeof(float)*N_AXIS))) {
    // Reset with default zero vector
		coord_data[X_AXIS] = 0.0f;
		coord_data[Y_AXIS] = 0.0f;
		coord_data[Z_AXIS] = 0.0f;
		settings_write_coord_data(coord_select,coord_data);
    return(false);
  }
#endif
  return(1);
}
// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_build_info(char *line)
{
#if 0
  if (!(memcpy_from_eeprom_with_checksum((char*)line, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    line[0] = 0; // Empty line
    settings_store_build_info(line);
    return(false);
  }
#endif
  return(1);
}
// Method to store build info into EEPROM
// NOTE: This function can only be called in IDLE state.
void settings_store_build_info(char *line)
{
  // Build info can only be stored when state is IDLE.
////  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO,(char*)line, LINE_BUFFER_SIZE);
}

void settings_restore(uint8_t restore_flag) {
}
 // Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
#if 0
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  if (!(memcpy_from_eeprom_with_checksum((char*)line, addr, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    line[0] = 0; // Empty line
    settings_store_startup_line(n, line);
    return(false);
  }
#endif
  return(1);
}
uint8_t settings_store_global_setting(uint8_t parameter, float value) {
  return(1);
}
  // Method to store startup lines into EEPROM
void settings_store_startup_line(uint8_t n, char *line)
{
#if 0
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing. 
  #endif
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  memcpy_to_eeprom_with_checksum(addr,(char*)line, LINE_BUFFER_SIZE);
#endif
}


////========================================================================
void coolant_stop(void)
{
}
void coolant_init(void)
{
}
void coolant_sync(uint8_t mode)
{
}
void coolant_set_state(uint8_t mode)
{
}
uint8_t coolant_get_state()
{
return 0;
}
  
void  SetSpindleEnablebit(void)
{
}
void spindle_init(void)
{
}
void  ResetSpindleEnablebit(void)
{
}
void __spindle_set_state(uint8_t state)
  {
  }
void spindle_set_state(uint8_t state, float rpm)
  {
  }

void spindle_sync(uint8_t state, float rpm)
{
}

void spindle_stop()
{
}
uint8_t spindle_get_state()
{
return 0;
}
uint16_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
{
  return 0;

}
void spindle_set_speed(uint16_t pwm_value)
  {
  }
void  put_steps(uint8_t steps)////???
{
if(steps&(1 << X_STEP_BIT))
  {
  printk("x");  
  }
if(steps&(1 << Y_STEP_BIT))
  {
  printk("y");  
  }
if(steps&(1 << Z_STEP_BIT))
  {
  printk("z");  
  }
         
}
void steps_reset(void)
{
  
}

void SetStepperDisableBit(void) ////GPIO_SetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)
{
}

void ResetStepperDisableBit(void) ////GPIO_ResetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)
{
}
