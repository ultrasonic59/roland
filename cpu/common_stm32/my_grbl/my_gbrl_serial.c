#include <stdint.h>
#include "my_grbl.h"
#include "printk.h"
#ifndef USEUSB
#include "stm32f2xx_usart.h"
#endif
#define RX_RING_BUFFER (RX_BUFFER_SIZE)
#define TX_RING_BUFFER (TX_BUFFER_SIZE)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count(void)
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
#ifndef USEUSB
  USART_SendData(USART1, data);
  while (!(USART1->SR & USART_FLAG_TXE));
    return;
#else
  // Calculate next head
uint8_t next_head = serial_tx_buffer_head + 1;
////printk("%c",data);
if (next_head == TX_RING_BUFFER) { 
    next_head = 0; 
  }
  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { 
      return; 
      } // Only check for abort to avoid an endless loop.
    ;
  }
  // Store data and advance head
serial_tx_buffer[serial_tx_buffer_head] = data;
serial_tx_buffer_head = next_head;
#endif
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read_rx(void)
{
 uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { 
      tail = 0; 
      }
    serial_rx_buffer_tail = tail;

    return data;
  }
}
int serial_read_tx(void)
{
uint8_t tail = serial_tx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_tx_buffer_head == tail) {
    return -1;
  } else {
    uint8_t data = serial_tx_buffer[tail];
    tail++;
    if (tail == TX_RING_BUFFER) { 
      tail = 0; 
      }
    serial_tx_buffer_tail = tail;
    return data;
  }
}
extern int send_char_dbg(int ch); 

////===================================================================
#ifdef USEUSB
void OnUsbDataRx(uint8_t* data_in, uint16_t length)
{
	//lcd_write_char(*dataIn);
uint8_t next_head;
uint8_t data;
uint16_t ii=0;
//// Write data to buffer unless it is full.
while (length != 0){
  data = *(data_in+ii);
#else 
void USART1_IRQHandler (void) 
{
    volatile unsigned int IIR;
    uint8_t data;
    uint8_t next_head;

    IIR = USART1->SR;
    if (IIR & USART_FLAG_RXNE) 
    {                  // read interrupt
        data = USART1->DR & 0x1FF;
#endif
  ////====================================
 ////       serial_write(data+1);
  ////====================================
 ///// ii++;
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:     // Call motion control reset routine.    
      mc_reset(); 
      break; 
    case CMD_STATUS_REPORT: 
      system_set_exec_state_flag(EXEC_STATUS_REPORT); 
      break; // 
    case CMD_CYCLE_START:   
      system_set_exec_state_flag(EXEC_CYCLE_START); 
      break; // 
    case CMD_FEED_HOLD:     
      system_set_exec_state_flag(EXEC_FEED_HOLD); 
      break; // 
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   
            system_set_exec_state_flag(EXEC_SAFETY_DOOR); 
            break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
           case CMD_DEBUG_REPORT: 
             {
 ////???              uint8_t sreg = SREG; 
 ////???                cli(); 
  ////???               bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); 
  ////???               SREG = sreg;
             } break;
          #endif
          case CMD_FEED_OVR_RESET: 
            system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); 
            break;
          case CMD_FEED_OVR_COARSE_PLUS: 
            system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); 
            break;
          case CMD_FEED_OVR_COARSE_MINUS: 
            system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); 
            break;
          case CMD_FEED_OVR_FINE_PLUS: 
            system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); 
            break;
          case CMD_FEED_OVR_FINE_MINUS: 
            system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); 
            break;
          case CMD_RAPID_OVR_RESET: 
            system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); 
            break;
          case CMD_RAPID_OVR_MEDIUM: 
            system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); 
            break;
          case CMD_RAPID_OVR_LOW: 
            system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); 
            break;
          case CMD_SPINDLE_OVR_RESET: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); 
            break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); 
            break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); 
            break;
          case CMD_SPINDLE_OVR_FINE_PLUS: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); 
            break;
          case CMD_SPINDLE_OVR_FINE_MINUS: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); 
            break;
          case CMD_SPINDLE_OVR_STOP: 
            system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); 
            break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: 
            system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); 
            break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: 
              system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); 
              break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
///==================================
////printk("[%c]",data);
///========================================

        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
#ifndef USEUSB
        USART1->SR &= ~USART_FLAG_RXNE;	          // clear interrupt
#else
    length--;
    ii++;
#endif
   }
}
///=========================================================================
void serial_reset_read_buffer(void)
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
