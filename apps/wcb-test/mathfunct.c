/*
 * Copyright (c) 2021, University of Trento, Italy and 
 * Fondazione Bruno Kessler, Trento, Italy.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its 
 *    contributors may be used to endorse or promote products derived from this 
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Matteo Trobinger <matteo.trobinger@gmail.com>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mathfunct.h"
#include "wcb-conf.h"

#define SCALE_FACTOR_HEIGHT pow(2, 16)
#define SCALE_FACTOR_FLOW_STATE pow(2, 19)
#define SCALE_FACTOR_INTEGRAL pow(2, 9)
#define SCALE_FACTOR_ACT pow(2, 9)
#define NUM_SOURCES 10
#define T_EPOCH_MIN 1 /* NB: NEEDS TO BE CHANGED IN FUNCTION OF THE CONTROL PERIOD ADOPTED! */

#define BZERO_SARR() bzero(sback_state, sizeof(sback_state))
#define BZERO_CTRLARR() bzero(sback_actcmd, sizeof(sback_actcmd))


static const double De[15][15] = {
  {16593.982746852318, 0, 0, 0, 0, 0, 0, 0, 0, 0, 103.04397308739921, 0, 0, 0, 0},
  {0, 3885.3125831187926, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 9585.75762004085, 0, 0, 0, 0, 0, 0, 0, 0, 71.49165323371092, 0, 0, 0},
    {0, 0, 0, 2173.0636551864495, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 39488.77290408321, 0, 0, 0, 0, 0, 0, 0, 234.68522052932096 , 0, 0},
  {0, 0, 0, 0, 0, 9438.650647830156, 0, 0, 0, 0, 0, 0, 0 ,0, 0},
  {0, 0, 0, 0, 0, 0, 47199.07795261111, 0, 0, 0, 0, 0, 0, 327.1526144120279, 0},
  {0, 0, 0, 0, 0, 0, 0, 11092.757672677713, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 85989.70401981143, 0, 0, 0, 0, 0, 573.988981438914},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 18103.014720086223, 0, 0, 0, 0, 0},
  {103.04397308739921, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.7167127490045153, 0, 0, 0, 0},
  {0, 0, 71.49165323371092, 0, 0, 0, 0, 0, 0, 0, 0, 0.6661136650832599, 0, 0, 0},
  {0, 0, 0, 0, 234.68522052932096, 0, 0, 0, 0, 0, 0, 0, 1.6037670422088819, 0, 0},
  {0, 0, 0, 0, 0, 0, 327.1526144120279 , 0, 0, 0, 0, 0, 2.814873351823985, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 573.988981438914, 0, 0, 0, 0, 0, 4.349685987611367}
};

static const double Dx[10] = {0.7497682392622664, 0, 0.7497294072036311, 0, 1.4997101098342682, 0, 2.999680592874938, 0, 4.499779297708301, 0};

static const double epsilon_vector[10] = {0.6440736172716024, 3.0, 0.4895152676534626, 3.0, 0.9935702440438394, 3.0, 1.0862329201421101, 3.0, 1.4661616916332398, 3.0};

static const double K[5][15] = {
  {120.929591, 59.4507121, 37.5628692, 18.6629077, 82.2959704,
  40.7164100, 74.3749477, 36.8267327, 101.163056, 49.8818667,
  1.01544082, 0.238006701, 0.432084570, 0.361236819, 0.466440972},
  {-26.2092994, -12.4929307, 66.1584213, 32.6582249, 98.2244373,
  48.4359682, 83.4431726, 41.2385776, 111.272914, 54.7518618, 
  -0.430997649, 0.847398865, 0.677631399, 0.484150588, 0.589730038},
  {-13.1451911, -6.39542193, -34.6132583, -16.7210869, 115.560052,
  56.6830062, 86.5935104, 42.7341090, 112.786504, 55.4314418,
  -0.173278307, -0.683035752, 1.10023373, 0.564465421, 0.641206802},
  {-4.33918773, -2.11793084, -6.14326719, -3.02679959, -44.6084146,
  -21.1395525, 144.496045, 70.5585291, 148.183173, 72.4013656,
  -0.0510191031, -0.0876830777, -0.780277692, 1.70138032, 1.12681411e+00},
  {-1.95298910, -0.954051803, -2.45862624, -1.21291131, -13.2779604,
  -6.44722872, -60.2990730, -28.1365813, 203.091677, 98.1076647,
  -0.0221955058, -0.0322771390, -0.186470058, -1.19238834, 2.29211586}
};


bool triggering_condition(int16_t s_trig_ms, int16_t s_held_ms, int16_t s_integral_hgt, 
  int16_t s_held_integral_hgt, uint8_t my_position, uint8_t is_hgtsensor) {
  if (is_hgtsensor) { /* Height sensors and flow sensors have *different* triggering conditions */
    double trig_ms, held_ms, integral_hgt, held_integral_hgt, leftside_eq;
    /* Scale back sensor readings and held values */
    integral_hgt = s_integral_hgt / SCALE_FACTOR_INTEGRAL;
    held_integral_hgt = s_held_integral_hgt / SCALE_FACTOR_INTEGRAL;
    trig_ms = s_trig_ms / SCALE_FACTOR_HEIGHT;
    held_ms = s_held_ms / SCALE_FACTOR_HEIGHT;
    /* Compute the left side of the triggering equation */
    leftside_eq = De[my_position][my_position] * pow((trig_ms-held_ms), 2)\
            + 2 * De[my_position][NUM_SOURCES+(my_position/2)]\
            * (trig_ms-held_ms) * (integral_hgt-held_integral_hgt)\
            + De[NUM_SOURCES+(my_position/2)][NUM_SOURCES+(my_position/2)]\
            * pow((integral_hgt-held_integral_hgt), 2) - Dx[my_position] * pow(trig_ms, 2);
    /* Evaluate the triggering condition */
    if (isgreaterequal(leftside_eq, pow(epsilon_vector[my_position], 2)))
      return true;
    else
      return false;
  }
  else { /* The node is a flow sensor */
    double trig_ms, held_ms;
    /* Scale back the sensor reading and the held value */
    trig_ms = s_trig_ms / SCALE_FACTOR_FLOW_STATE;
    held_ms = s_held_ms / SCALE_FACTOR_FLOW_STATE;
    /* Evaluate the triggering condition */
    if (isgreaterequal(2 * De[my_position][my_position] * pow((trig_ms-held_ms), 2), pow(epsilon_vector[my_position], 2)))
      return true;
    else
      return false;
  }
}


void compute_actcommands(int16_t state_array[], app_control_payload *ctrl_payload) {
  static double sback_state[NUM_SOURCES+NUM_HEIGHT_SENSORS]; // Temporal array to store scaled back sensor data
  static double sback_actcmd[NUM_ACTUATORS];                 // Temporal array to store scaled back actuation commands
  int r, c;
  double sum = 0; // Store intermediate values for matrix multiplication
  /* Scale back the values stored in state_array in function of the type of sensor they refer to.
   * Remember: state_array[h1, f1, h2, f2, h3, f3, h4, f4, h5, f5, Ih1, Ih2, Ih3, Ih4, Ih5], with 
   * h* = height measure, f* = flow measure, Ih* = integral height measure */
  for (r = 0; r < (NUM_SOURCES + NUM_HEIGHT_SENSORS); r++) {
    if (r < NUM_SOURCES) { /* Flow or height sensor reading */
      if (r & 1) /* Odd position -> flow sensor reading */
        sback_state[r] = state_array[r] / SCALE_FACTOR_FLOW_STATE;
      else /* Even position -> height sensor reading */
        sback_state[r] = state_array[r] / SCALE_FACTOR_HEIGHT;
    }
    else /* Integral height sensor reading */
      sback_state[r] = state_array[r] / SCALE_FACTOR_INTEGRAL;
  }
  /* Compute actuation commands: act_vector = -K@x */
  for (r = 0; r < NUM_ACTUATORS; r++) {
    for (c = 0; c < (NUM_SOURCES+NUM_HEIGHT_SENSORS); c++) {
      sum += K[r][c] * sback_state[c];
    }
    sback_actcmd[r] = -sum;
    sum = 0;
  }
  /* Scale actuation commands, and convert them to int16_t 
   * TBD: check for overflow and in case rise an error! */
  for (r = 0; r < NUM_ACTUATORS; r++) {
    ctrl_payload->upd_ctrl[r] = (int16_t)(sback_actcmd[r] * SCALE_FACTOR_ACT);
    /* [Side note] Using round (round(sback_actcmd[r] * SCALE_FACTOR_ACT)) might be preferable, but:
     *  1. Could slow down processing;
     *  2. Might rise linker issues. 
     */
  }
  BZERO_SARR();
  BZERO_CTRLARR();
}
