/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "my_grbl.h"

#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "queue.h"

#include "my_stepper.h"
#include "printk.h"

extern xQueueHandle rdy_to_send;
extern void obr_segment(void);



void TIM_Configuration(TIM_TypeDef* TIMER, u16 Period, u16 Prescaler, u8 PP);
extern void  put_steps(uint8_t steps);

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;
uint8_t curr_dir=0;

////static 
st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];
////static 
segment_t segment_buffer[SEGMENT_BUFFER_SIZE];
////static 
stepper_t st;
// Step segment ring buffer indices
///static 
volatile uint8_t segment_buffer_tail;
///static 
uint8_t segment_buffer_head;
static uint8_t segment_next_head;
///static 
uint16_t step_port_invert_mask;
///static 
////uint16_t dir_port_invert_mask;
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped
static st_prep_t prep;
////static uint8_t nTimer1Out =0;
////===============================================
void  set_curr_dir(uint8_t dirs)
{
curr_dir=dirs;  
}

////===================================================================

/*    BLOCK VELOCITY PROFILE DEFINITION
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity

  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters
  are shown and defined in the above illustration.
*/

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up(void)
{
/////uint8_t  btmp=0;
#if 0
  // Enable stepper drivers.
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) 
  { 
  SetStepperDisableBit();
  }
  else 
  { 
  ResetStepperDisableBit(); 
  }
#endif
///#if 0
st.step_pulse_time = (settings.pulse_microseconds)*TICKS_PER_MICROSECOND;
///==== ena steps ========
 ////xQueueSend(rdy_to_send,&btmp,TIMEOUT_SEND);
obr_segment();
//// rdy_blk=1; 
#ifdef WIN32
  nTimer1Out = 1;
#endif

}
// Stepper shutdown
void st_go_idle(void)
{
#ifdef WIN32
  nTimer1Out = 0;
#endif

  busy = false;

  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
  bool pin_state = false; // Keep enabled.
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay__ms(settings.stepper_idle_lock_time);
    pin_state = true; // Override. Disable steppers.
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) {
    pin_state = !pin_state; 
    } // Apply pin invert.
  if (pin_state) 
  { 
	  SetStepperDisableBit();
  }
  else 
  { 
	  ResetStepperDisableBit();
  }
}
////==================================================================


// Initialize and start the stepper motor subsystem
void stepper_init(void)
{
}
#if 0
// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= step_pin_mask[idx]; }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= direction_pin_mask[idx]; }
  }
}
#endif


// Reset and clear stepper subsystem variables
void st_reset(void)
{
  // Initialize stepper driver idle state.
  st_go_idle();

  // Initialize stepper algorithm variables.
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

///  st_generate_step_dir_invert_masks();
////  st.dir_outbits = dir_port_invert_mask; // Initialize direction bits to default.

  // Initialize step and direction port pins.
  steps_reset();
//// GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
////  GPIO_Write(DIRECTION_PORT, (GPIO_ReadOutputData(DIRECTION_PORT) & ~DIRECTION_MASK) | (dir_port_invert_mask & DIRECTION_MASK));
}
// Stepper shutdown


// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load and check active velocity profile.
  }
}


// Increments the step segment buffer block data ring buffer.
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}

extern settings_t settings;
#if 1
void obr_segment(void)
{
uint32_t tTimer;
uint32_t tst_cnt[3] = { 0 };
do
  {
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
     // Initialize new step segment and load number of steps to execute
    st.exec_segment = &segment_buffer[segment_buffer_tail];
    // Initialize step segment timing per step and load number of steps to execute.
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                nTimer1Out = st.exec_segment->cycles_per_tick * (st.exec_segment->prescaler + 1);
#else
 ////          nTimer1Out = st.exec_segment->cycles_per_tick;
                tTimer = st.exec_segment->cycles_per_tick;
 ////               printf("\n nTimer1Out [%d]", st.exec_segment->cycles_per_tick);
 ////               nTimer1Out = 0;

#endif
    st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
    // If the new segment starts a new planner block, initialize stepper variables and counters.
    // NOTE: When the segment data index changes, this indicates a new planner block.
    if (st.exec_block_index != st.exec_segment->st_block_index) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
      // Initialize Bresenham line and distance counters
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
       }
      st.dir_outbits = st.exec_block->direction_bits;

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
                st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
                st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
                st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
#endif

            }
            else {
                // Segment buffer empty. Shutdown.
                st_go_idle();
                // Ensure pwm is set properly upon completion of rate-controlled motion.
                system_set_exec_state_flag(EXEC_CYCLE_STOP); // Flag main program for cycle end
                return; // Nothing to do but exit.
            }
        }
        ///====================================================
            // Check probing state.
        if (sys_probe_state == PROBE_ACTIVE) {
            probe_state_monitor();
        }
      do
        {
        st.step_outbits = 0;

       // Execute step displacement profile by Bresenham line algorithm
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
            st.counter_x += st.steps[X_AXIS];
#else
            st.counter_x += st.exec_block->steps[X_AXIS];
#endif
            if (st.counter_x > st.exec_block->step_event_count) {
                st.step_outbits |= (1 << X_STEP_BIT);
                tst_cnt[X_AXIS] ++;
                st.counter_x -= st.exec_block->step_event_count;
                if (st.exec_block->direction_bits & (1 << X_DIRECTION_BIT)) { 
                    sys_position[X_AXIS]--; 
                }
                else { 
                    sys_position[X_AXIS]++; 
                }
            }
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
            st.counter_y += st.steps[Y_AXIS];
#else
            st.counter_y += st.exec_block->steps[Y_AXIS];
#endif
            if (st.counter_y > st.exec_block->step_event_count) {
                st.step_outbits |= (1 << Y_STEP_BIT);
                tst_cnt[Y_AXIS] ++;
                st.counter_y -= st.exec_block->step_event_count;
                if (st.exec_block->direction_bits & (1 << Y_DIRECTION_BIT)) {
                    sys_position[Y_AXIS]--; 
                }
                else {
                    sys_position[Y_AXIS]++; 
                }
            }
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
            st.counter_z += st.steps[Z_AXIS];
#else
            st.counter_z += st.exec_block->steps[Z_AXIS];
#endif
            if (st.counter_z > st.exec_block->step_event_count) {
                st.step_outbits |= (1 << Z_STEP_BIT);
                tst_cnt[Z_AXIS] ++;
                st.counter_z -= st.exec_block->step_event_count;
                if (st.exec_block->direction_bits & (1 << Z_DIRECTION_BIT)) { 
                    sys_position[Z_AXIS]--; 
                }
                else { 
                    sys_position[Z_AXIS]++; 
                }
            }
            // During a homing cycle, lock out and prevent desired axes from moving.
            if (sys.state == STATE_HOMING) {
                st.step_outbits &= sys.homing_axis_lock;
            }

            st.step_count--; // Decrement step events count

        } while (st.step_count);
    put_can_cmd_go(st.exec_block->direction_bits,st.exec_segment->cycles_per_tick
                   ,tst_cnt[X_AXIS]
                   ,tst_cnt[Y_AXIS]
                   ,tst_cnt[Z_AXIS]);

#if 0        
        printf("\n tst_cnt[%d][%02x][%d][%d][%d]", st.exec_segment->cycles_per_tick, st.exec_block->direction_bits
            , tst_cnt[X_AXIS]
            , tst_cnt[Y_AXIS]
            , tst_cnt[Y_AXIS]
        );
#endif
        
        ////   if (st.step_count == 0) 
        {
            // Segment is complete. Discard current segment and advance segment indexing.
            st.exec_segment = NULL;

            if (++segment_buffer_tail == SEGMENT_BUFFER_SIZE)
            {
                segment_buffer_tail = 0;
            }
        }
    }while (tTimer);
}
#endif

/* Prepares step segment buffer. Continuously called from main program.

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it.
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer()
{
  // Block step prep buffer, while in a suspend state and there is no suspend motion to execute.
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { 
    return; 
    }

  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

    // Determine if we need to load a new planner block or if the block needs to be recomputed.
    if (pl_block == NULL) {

      // Query planner for a queued block
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { 
        pl_block = plan_get_system_motion_block(); 
        }
      else { 
        pl_block = plan_get_current_block(); 
        }
      if (pl_block == NULL) { // No planner blocks. Exit.
        return; 
        } 

      // Check if we need to only recompute the velocity profile or load a new block.
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        // Load the Bresenham stepping data for the block.
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { 
            st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); 
          }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          for (idx=0; idx<N_AXIS; idx++) { 
            st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; 
            }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0f; // Reset for new segment block

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // New block loaded mid-hold. Override planner block entry speed to enforce deceleration.
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
        }
      }

			/* ---------------------------------------------------------------------------------
			 Compute the velocity profile of a new planner block based on its entry and exit
			 speeds, or recompute the profile of a partially-completed planner block if the
			 planner has updated it. For a commanded forced-deceleration, such as from a feed
			 hold, override the planner velocities and decelerate to the target exit speed.
			*/
			prep.mm_complete = 0.0f; // Default velocity profile complete at 0.0mm from end of block.
			float inv_2_accel = 0.5f/pl_block->acceleration;
			if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [Forced Deceleration to Zero Velocity]
				// Compute velocity profile parameters for a feed hold in-progress. This profile overrides
				// the planner block profile, enforcing a deceleration to zero speed.
				prep.ramp_type = RAMP_DECEL;
				// Compute decelerate distance relative to end of block.
				float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
				if (decel_dist < 0.0f) {
					// Deceleration through entire planner block. End of feed hold is not in this block.
					prep.exit_speed = sqrtf(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
				} else {
					prep.mm_complete = decel_dist; // End of feed hold.
					prep.exit_speed = 0.0f;
				}
			} else { // [Normal Operation]
				// Compute or recompute velocity profile parameters of the prepped planner block.
				prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
				prep.accelerate_until = pl_block->millimeters;

				float exit_speed_sqr;
				float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0f; // Enforce stop at end of system motion.
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrtf(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
				float nominal_speed_sqr = nominal_speed*nominal_speed;
				float intersect_distance =
								0.5f*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0f) { // Deceleration-only.
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // Compute override block exit speed since it doesn't match the planner exit speed.
            prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // Flag to load next block as deceleration override.

            // TODO: Determine correct handling of parameters in deceleration-only.
            // Can be tricky since entry speed will be current speed, as in feed holds.
            // Also, look into near-zero speed handling issues with this.

          } else {
            // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
						prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
				} else if (intersect_distance > 0.0f) {
					if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
						// NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
						prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
						if (prep.decelerate_after < intersect_distance) { // Trapezoid type
							prep.maximum_speed = nominal_speed;
							if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
								// Cruise-deceleration or cruise-only type.
								prep.ramp_type = RAMP_CRUISE;
							} else {
								// Full-trapezoid or acceleration-cruise types
								prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
							}
						} else { // Triangle type
							prep.accelerate_until = intersect_distance;
							prep.decelerate_after = intersect_distance;
							prep.maximum_speed = sqrtf(2.0f*pl_block->acceleration*intersect_distance+exit_speed_sqr);
						}
					} else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
					}
				} else { // Acceleration-only type
					prep.accelerate_until = 0.0f;
					// prep.decelerate_after = 0.0f;
					prep.maximum_speed = prep.exit_speed;
				}
			}
      
     }
    
    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create
      a full segment based on the current ramp conditions. If the segment time is incomplete
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if
      an incomplete segment terminates at the end of the velocity profile, the segment is
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at
      the end of planner block (typical) or mid-block at the end of a forced deceleration,
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0f; // Initialize segment time
    float time_var = dt_max; // Time worker variable
    float mm_var; // mm-Distance worker variable
    float speed_var; // Speed worker variable
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    if (minimum_mm < 0.0f) { minimum_mm = 0.0f; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
					if (prep.current_speed-prep.maximum_speed <= speed_var) {
            // Cruise or cruise-deceleration types only for deceleration override.
						mm_remaining = prep.accelerate_until;
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // Mid-deceleration override ramp.
						mm_remaining -= time_var*(prep.current_speed - 0.5f*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5f*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise.
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5f*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          }
          // Otherwise, at end of block or end of forced-deceleration.
          time_var = 2.0f*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions.
       However, since floats have only 7.2 significant digits, long moves with extremely
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    float step_dist_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
    float n_steps_remaining = ceilf(step_dist_remaining); // Round-up current steps remaining
    float last_n_steps_remaining = ceilf(prep.steps_remaining); // Round-up last steps remaining
	prep_segment->n_step = (uint16_t)(last_n_steps_remaining - n_steps_remaining); // Compute number of steps to execute.

    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS
        // requires full steps to execute. So, just bail.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse

    // Compute CPU cycles per step for the prepped segment.
	uint32_t cycles = (uint32_t)ceilf((TICKS_PER_MICROSECOND * 1000000) *inv_rate * 60); // (cycles/step)
////uint32_t cycles = 1000;////????
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) 
      { 
        prep_segment->amass_level = 0; 
      }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else
      // Compute step timing and timer prescalar for normal step generation.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.)
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { 
      segment_next_head = 0; 
    }

    // Update the appropriate planner and segment data.
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    // Check for exit conditions and flag to load next planner block.
    if (mm_remaining == prep.mm_complete) {
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0f) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // Set pointer to indicate check and load next planner block.
        plan_discard_current_block();
      }
    }

  }
}

// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds.
float st_get_realtime_rate()
{
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
    return prep.current_speed;
  }
  return 0.0f;
}


////===================================================================



