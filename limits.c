/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

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
  
#include <util/delay.h>
#include <avr/io.h>
#include "stepper.h"
#include "limits.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"
#include "motion_control.h"
#include "planner.h"

void limits_init() {
	LIMIT_DDR &= ~(LIMIT_MASK);
#ifdef LIMIT_PULLUP
	LIMIT_PORT |= LIMIT_MASK;
#endif
}

static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, bool c_axis, bool reverse_direction, uint32_t microseconds_per_pulse) {
  uint32_t step_delay = microseconds_per_pulse - settings.pulse_microseconds;
  uint8_t out_bits = DIRECTION_MASK;
  uint8_t limit_bits;

  if (x_axis) { out_bits |= (1<<X_STEP_BIT); }
  if (y_axis) { out_bits |= (1<<Y_STEP_BIT); }
  if (z_axis) { out_bits |= (1<<Z_STEP_BIT); }
  if (c_axis) { out_bits |= (1<<C_STEP_BIT); }

  // Invert direction bits if this is a reverse homing_cycle
  if (reverse_direction) {
    out_bits ^= DIRECTION_MASK;
  }

  // Apply the global invert mask
  out_bits ^= settings.invert_mask_stepdir;

  // Set direction pins, can't use |= because we may have 1 -> 0 transitions,
  // e.g. when reverse_direction is true
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);

  uint8_t x_limit_count = 0, y_limit_count = 0, z_limit_count = 0, c_limit_count = 0;

  for(;;) {
    limit_bits = LIMIT_PIN;

    if (reverse_direction) {
      // Invert limit_bits if this is a reverse homing_cycle
      limit_bits ^= LIMIT_MASK;
    }

    // Apply the global invert mask
    limit_bits ^= settings.invert_mask_limit;

    if (x_axis) {
      if (!(limit_bits & (1<<X_LIMIT_BIT))) {
        x_limit_count++;
      }
      else {
    	x_limit_count = 0;
      }
      if (x_limit_count >= 10) {
        x_axis = false;
        out_bits ^= (1<<X_STEP_BIT);
      }
    }
    if (y_axis) {
      if (!(limit_bits & (1<<Y_LIMIT_BIT))) {
        y_limit_count++;
      }
      else {
    	y_limit_count = 0;
      }
      if (y_limit_count >= 10) {
        y_axis = false;
        out_bits ^= (1<<Y_STEP_BIT);
      }
    }
    if (z_axis) {
      if (!(limit_bits & (1<<Z_LIMIT_BIT))) {
        z_limit_count++;
      }
      else {
    	z_limit_count = 0;
      }
      if (z_limit_count >= 10) {
        z_axis = false;
        out_bits ^= (1<<Z_STEP_BIT);
      }
    }
    if (c_axis) {
      if (!(limit_bits & (1<<C_LIMIT_BIT))) {
        c_limit_count++;
      }
      else {
    	c_limit_count = 0;
      }
      if (c_limit_count >= 10) {
        c_axis = false;
        out_bits ^= (1<<C_STEP_BIT);
      }
    }

    // Check if we are done
    if(!(x_axis || y_axis || z_axis || c_axis)) { return; }
    // Send stepping pulse, can't use |= because we may have 1 -> 0 transitions,
    // e.g. when the STEP lines are inverted
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (out_bits & STEP_MASK);
    delay_us(settings.pulse_microseconds);
    STEPPING_PIN = (out_bits & STEP_MASK); // End pulse via toggle, saves one port access
    delay_us(step_delay);
  }
  return;
}

// Usually all axes have the same resolution and when that's not the case, X and
// Y have identical resolutions and Z has more -- we're looking for the slowest
// going one, i.e. the one with the least resolution, thus X makes a good
// candidate
#define FEEDRATE_TO_PERIOD_US(f) \
  ((60.0 / ((f) * settings.steps_per_mm[X_AXIS])) * 1000000.0)

static void approach_limit_switch(bool x, bool y, bool z, bool c) {
  homing_cycle(x, y, z, c, false, FEEDRATE_TO_PERIOD_US(settings.default_seek_rate));
}

static void leave_limit_switch(bool x, bool y, bool z, bool c) {
  homing_cycle(x, y, z, c, true, FEEDRATE_TO_PERIOD_US(settings.default_feed_rate));
}

void limits_go_home() {
  plan_synchronize();

  st_enable();

  bool home_x = false, home_y = false, home_z = false, home_c = false;
#ifdef HOME_X
  home_x = true;
#endif
#ifdef HOME_Y
  home_y = true;
#endif
#ifdef HOME_Z
  home_z = true;
#endif
#ifdef HOME_C
  home_c = true;
#endif

  approach_limit_switch(false, false, home_z, false); // First home the z axis
  approach_limit_switch(home_x, home_y, false, home_c);  // Then home the x, y, and c axis
  // Now carefully leave the limit switches
  leave_limit_switch(home_x, home_y, home_z, home_c);

  // Conclude that this is machine zero
  sys.position[X_AXIS] = sys.position[Y_AXIS] = sys.position[Z_AXIS] = sys.position[C_AXIS] = 0;
}
