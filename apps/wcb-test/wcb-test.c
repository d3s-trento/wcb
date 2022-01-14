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
 * Authors: Matteo Trobinger <matteo.trobinger@gmail.com>
 *          Timofei Istomin  <tim.ist@gmail.com>
 *
 */

#include <stdio.h>
#include "contiki.h"
#include "etimer.h"
#include "mathfunct.h"
#include "wcb-test.h"
#include "wcb.h"
#include "wcb-private.h"
#include "deployment.h"
#include "serial-line.h"


/* -------------------- EXPERIMENT SETUP AND NODES ROLES, DEFINED IN FUNCTION OF THE TESBED ADOPTED -------------------- */
#if TESTBED_TOPOLOGY == 0
/**********************
 HALL TOPOLOGY
  - CONTROLLER: NODE 73
  - REMOVED NODES: NONE
***********************/
static uint8_t source_id[NUM_SOURCES] = {75, 74, 63, 62, 53, 56, 65, 51, 72, 76};
static uint8_t height_sensor_id[NUM_HEIGHT_SENSORS] = {75, 63, 53, 65, 72};
static uint8_t actuator_id[NUM_ACTUATORS] = {77, 64, 57, 52, 70};

#elif TESTBED_TOPOLOGY == 1
/*************************
  DEPT TOPOLOGY
  - CONTROLLER: NODE 20;
  - REMOVED NODES: 21, 22;
*************************/
static uint8_t source_id[NUM_SOURCES] = {14, 18, 7, 11, 35, 4, 29, 32, 23, 26};
static uint8_t height_sensor_id[NUM_HEIGHT_SENSORS] = {14, 7, 35, 29, 23};
static uint8_t actuator_id[NUM_ACTUATORS] = {19, 12, 5, 34, 27};

#endif /* TESTBED_TOPOLOGY */
/* ----------------------------------------------------------------------------------------------------- */

PROCESS(wcb_test, "WCB test");

static app_t_payload       t_payload;
static app_a_payload       a_payload;
static app_control_payload ctrl_payload;

static rtimer_clock_t act_ts;        // Actuation timestamp in clock ticks
static uint16_t act_lat;             // Actuation latency (in ms) relative to the beginning of the epoch

static wcb_config_t conf;            // WCB configuration structure

static uint8_t is_sink;              // The node is the sink (aka the controller)
static uint8_t is_source;            // The node is a source
static uint8_t is_hgtsensor;         // The node is an height sensor
static uint8_t is_actuator;          // The node is an actuator
static uint8_t my_position;          // Node position in the source_id or actuator_id arrays, exploited to identify the node itself
static uint8_t ev_detected;          // An event has been generated
static uint8_t i_detected;           // The sensor node detected an event
static uint8_t is_ack;               // The sensor node has been ACKed
static int16_t cont_state_array[NUM_SOURCES + NUM_HEIGHT_SENSORS]; /* Array used by the controller to store
                                                                    * the sensor states and the integral height values
                                                                    * collected during the Collection / Recovery phases */
static uint8_t  all_data_rcvd;       // The controller received all messages
static uint8_t  new_s_act_command;   // For actuators, to notify the reception of an updated actuation command
static uint8_t  update_held_measure; // The source updated its held measure(s) with the reading(s) acquired in the current epoch

/* For analysis purposes */
static uint8_t  new_ctrpkt_recv;     // For sources, to notify the reception of a control packet
static uint16_t collection_bitmap;   // For the controller, to store the reception bitmap after the Collection phase
static uint8_t  n_pkt_recv;          // Number of packets received by the node during the current epoch
static uint16_t reception_bitmap;    /* Reception bitmap (Collection / Recovery) stored by common nodes to 
                                      * compute the number of packets received during the epoch */
static uint8_t evdtc_sl1;            // An event has been detected in the first EV slot
static uint8_t evdtc_sl2;            // An event has been detected in the second EV slot

/* --- Interacting with the testbed / Simulink --- */
static uint8_t correct_epoch;        // The epoch reported by the testbed is correct
static uint16_t ev_epoch;            // Epoch number retrieved by the testbed (together with a new reading). Used to check if the reading has arrived in the correct epoch or was delayed
static int16_t s_trig_ms;            // Measurement (SCALED) used to evaluate the triggering condition
static int16_t s_data_ms;            // Measurement (SCALED) acquired after the Event phase, when an event is detected (the measurement that will be TX to the controller)
static int16_t s_held_ms;            // Held measurement, i.e., the value currently available at the controller
static uint8_t new_command;          // New actuation commands have been computed 
static uint8_t new_readings;         // New sensor readings have been correctly retrieved from the testbed / Simulink
static uint16_t s_integral_hgt_trig; /* Integral height value (SCALED) computed with the measurements acquired after the S phase (triggering values). 
                                      * It is exploited by sensor nodes to evaluate their triggering condition */
static uint16_t s_integral_hgt_data; /* Integral height value (SCALED) computed with the measurements acquired after the Event phase (data values).
                                      * This value is communicated by sensor nodes to the controlled and stored locally */
static uint16_t s_held_integral_hgt; // Held integral height value (SCALED): the last s_integral_hgt_data value sent to the controller
static int16_t s_act_command;        // Actuation command (SCALED). Acquired from the control packet broadcasted by the controller during the Dissemination phase

/* --- Useful macro functions ---- */
#define BZERO_STATE_ARRAY() bzero(cont_state_array, sizeof(cont_state_array))
#define BZER0_CTRL_PAYLOAD() bzero(ctrl_payload.upd_ctrl, sizeof(ctrl_payload.upd_ctrl))
#define MS_TO_TICKS(v) ((uint32_t)RTIMER_SECOND*(v)/1000)
#define TICKS_TO_MS(v) (1000*(v)/RTIMER_SECOND)
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define LOGGING 1 // Enable / disable the application-level logging
#define RTIMER_CLOCK_MAX ((rtimer_clock_t) - 1)

/* --- Process events exploited to interact with the testbed server and the main thread --- */
static process_event_t EPOCH_PRE_START_EV;
static process_event_t EPOCH_CTRL_EV;
static process_event_t EPOCH_END_EV;

/* --- App function prototypes --- */
static inline int get_src_idx(uint8_t src_id);
static inline int get_hgtsensor_idx(uint8_t src_id);
static inline void init_sink_epoch_state();
static inline void init_node_epoch_state();
static inline void compute_act_latency();


/* ---------------------------------------- WCB callbacks ----------------------------------------

 * Note that app_* WCB callbacks are called in the interrupt context
 * between WCB slots, so they should return ASAP.
 *
 * The callbacks are described in wcb.h.

-------------------------------------------------------------------------------------------------- */

/* Post S phase WCB callback.
 * - All nodes initialise epoch related variables. 
 * - Sources evaluate their triggering conditions. */
void app_post_S() {
  if (is_sink) 
    init_sink_epoch_state();
  else 
    init_node_epoch_state();    
  if (is_source) { /* Sources (i) evaluate their triggering conditions, which depend on the sensor's type, and 
                    * (ii) set the i_detected and ev_detected flags accordingly */
    if (is_hgtsensor) {
      if (triggering_condition(s_trig_ms, s_held_ms, s_integral_hgt_trig, s_held_integral_hgt, my_position, is_hgtsensor)) {
        i_detected = 1;
        ev_detected = 1;
      }
    }
    else { /* Flow sensors */
      if (triggering_condition(s_trig_ms, s_held_ms, 0, 0, my_position, is_hgtsensor)) {
        i_detected = 1;
        ev_detected = 1;
      }
    }
  }
}


/* Pre Event phase WCB callback */
bool app_pre_EV() {
  /* Sensor node(s) detecting and event TX an event message during the Event phase */
  if (is_source && i_detected) 
    return true;
  else
    return false;
}


/* Post Event phase WCB callback */
void app_post_EV(int received, uint8_t corrupted_EV, uint8_t n_ea) {
  if (received || corrupted_EV) { /* During the event phase WCB interprets the reception of *any* 
                                   * IEEE 802.15.04 frame (even corrupted ones) as an event */
    ev_detected = 1; 
    /* [Analysis purposes] Logs to analyse the reliability of the first and second EV slot */
    if (n_ea == 0)
      evdtc_sl1 = 1; 
    else if (n_ea == 1)
      evdtc_sl2 = 1; 
    /* End of the logging procedure */
  }
}


/* Pre T dedicated slot (Collection phase) WCB callback exploited by sensor nodes
 * to know if it is their turn to TX */
uint8_t* app_pre_Tded(uint8_t tx_round) {
  if (is_source) {
    if (my_position == tx_round) /* Node's turn to TX */
      return (uint8_t*)&t_payload;
  }
  return NULL;
}


/* WCB callback called in between T dedicated slots (Collection phase) and after T shared slots (Recovery phase) */
uint8_t* app_post_T(int received, uint8_t* payload) {
  if (is_sink) {
    if (received) {
      int src_idx;
      t_payload = *(app_t_payload*)payload;
      src_idx = get_src_idx(t_payload.src);
      if (src_idx >= 0 && src_idx < NUM_SOURCES) {
        if (!CHECK_BIT(a_payload.ack_bitmap, src_idx)) { // First time in the epoch the sink receives a message from that sensor
          uint16_t bit = (1 << src_idx);
          a_payload.ack_bitmap |= bit; // Update the ACK bitmap 
          memcpy(&cont_state_array[src_idx], &t_payload.measurement, sizeof(t_payload.measurement)); // Store the fresh measurement for processing

          /* Height sensor TX 2 measurements, we need to store them both */
          int hgt_idx;
          hgt_idx = get_hgtsensor_idx(t_payload.src); // Check whether the sender is an height sensor, and in case which is its position in height_sensor_id
          if (hgt_idx >= 0 && hgt_idx < NUM_HEIGHT_SENSORS) { // The sender is a height sensor
            memcpy(&cont_state_array[NUM_SOURCES + hgt_idx], &t_payload.integral_hgt, sizeof(t_payload.integral_hgt)); // Store the fresh integral height measure
          }
          n_pkt_recv ++;
        }
      }
    }
    return (uint8_t*)&a_payload;
  }
  /* [For analysis purposes, only] */
  if (!is_source) {
    if (received) {
      int src_idx;
      t_payload = *(app_t_payload*)payload;
      src_idx = get_src_idx(t_payload.src);
      if (src_idx >= 0 && src_idx < NUM_SOURCES) {
        if (!CHECK_BIT(reception_bitmap, src_idx)) { // First time in the epoch the node receives something from that source
          uint16_t bit = (1 << src_idx);
          reception_bitmap |= bit; // Update the reception bitmap 
          n_pkt_recv ++;
        }
      }
    }
  }
  return NULL;
}


/* WCB callback called before each TX slot in the Recovery phase. */
uint8_t* app_pre_T() {
  /* Unacknowledged sources prepare themselves to TX */
  if (is_source && !is_ack) {
    return (uint8_t*)&t_payload;
  }
  return NULL;
}


/* Post ACK slots WCB callback */
void app_post_A(int received, uint8_t* payload) {
  /* Sources not yet ACKed check if they managed to communicate with the controller */
  if (is_source && received && !is_ack) {
    a_payload = *(app_a_payload*)payload;
    if (CHECK_BIT(a_payload.ack_bitmap, my_position)) { /* The source related bit in the ACK bitmap is set to 1: the controller received the
                                                         * message from the source! The source can thus stop sending its reading(s) and update 
                                                         * its held value(s) */
      is_ack = 1;
      s_held_ms = s_data_ms;
      s_held_integral_hgt = s_integral_hgt_data;
      update_held_measure = 1;
    }
  }
}


/* Pre Dissemination phase WCB callback */
uint8_t* app_pre_ctrl() {
  return (uint8_t*)&ctrl_payload;
}


/* Post Dissemination phase WCB callback */
void app_post_CTRL(int received, uint8_t* payload, uint8_t complete_reception_flag) {
  if (is_source && received && !new_ctrpkt_recv) {
    new_ctrpkt_recv = 1; // [For analysis purposes, only]
    if (!is_ack && complete_reception_flag) { /* As the complete_reception_flag is set to 1 the controller 
                                               * received all sensor readings. The source can thus safely update
                                               * its held values (it probably missed the cumulative ACK) */
      s_held_ms = s_data_ms;
      s_held_integral_hgt = s_integral_hgt_data;
      update_held_measure = 1;
    }
  }
  else if (is_actuator && received && !new_s_act_command) { // First CTRL command received by the actuator
    ctrl_payload = *(app_control_payload*)payload;
    s_act_command = ctrl_payload.upd_ctrl[my_position];     // Store the new CTRL commands
    new_s_act_command = 1;
    compute_act_latency();
  }
}


/* ---------------------------------------- Utility functions ---------------------------------------- */

/* Preparation for the new epoch */
void app_pre_epoch() {
  if (is_source) {
    /* - Clear the readings acquired during the last epoch (just to be sure)
     * - Request new readings from Simulink / the testbed */
    s_trig_ms = 0;
    s_data_ms = 0;
    process_post(&wcb_test, EPOCH_PRE_START_EV, NULL);
  }
  else if (is_sink) {
    /* - Clear the control payload (just to be sure) */
    BZER0_CTRL_PAYLOAD();
  }
}


/* Source-only: check if legitimate new readings have been
 * retrieved from Simulink / the testbed in time */
bool app_have_new_readings() {
  if (is_source) {
    if (new_readings && correct_epoch)
      return true;
    else 
      return false;
  }
  else /* Non-source nodes */
    return true;
}


/* Nodes decide whether to enter in the Collection Phase or go to sleep until next epoch */
bool app_enter_collection() {
  if (ev_detected) // An event has been detected, enter the Collection Phase
    return true;
  else // No event detected, sleep until the next epoch
    return false;
}


/* Before starting the Collection phase sensor nodes synchronously acquire new measurements 
 * and update the payloads of their data packets */
void app_acquire_measurement() {
  if (is_source) {
    t_payload.measurement = s_data_ms;
    t_payload.integral_hgt = s_integral_hgt_data;
    
    /* NB: Held measurement(s), instead, are updated *only upon* receiving an ACK
     * from the controller for the specific data packet(s) */
  }
}


/* The sink decides whether to put the network to sleep (all packets have been collected) or not */
uint8_t app_complete_reception() {
  int i;
  for (i=0; i<NUM_SOURCES; i++) {
    if (CHECK_BIT(a_payload.ack_bitmap, i))
      continue;
    else
      return 0;
  }
  all_data_rcvd = 1;
  return 1;
}


/* 
  [For debugging and analysis purposes] Store the ACK bitmap at the end of the Collection Phase
  */
void app_store_collection_bitmap() {
  collection_bitmap = a_payload.ack_bitmap;
}


/* The system computes new actuation commands */
void app_control_functions() {
#if ON_BOARD // The controller computes the new actuation commands onboard
  compute_actcommands(cont_state_array, &ctrl_payload);
  new_command = 1;
  correct_epoch = 1;
#else // The controller interacts with the testbed to compute new control commands
  process_post(&wcb_test, EPOCH_CTRL_EV, NULL);
#endif
}


/* The controller checks if it manages to compute/retrieve new actuation commands on time */
bool app_have_new_ctrlcommands() {
  if (new_command && correct_epoch)
    return true;
  else
    return false;
}


/* Called when WCB goes to sleep (inactive portion of the epoch) to "wake up" the main process */
void app_epoch_end() {
  process_post(&wcb_test, EPOCH_END_EV, NULL);
}


static inline int get_src_idx(uint8_t src_id) {
  int i;
  for (i=0; i<NUM_SOURCES; i++) {
    if (src_id == source_id[i]) {
      return i;
    }
  }
  return -1;
}


static inline int get_hgtsensor_idx(uint8_t src_id) {
  int i;
  for (i=0; i<NUM_HEIGHT_SENSORS; i++) {
    if (src_id == height_sensor_id[i]) {
      return i;
    }
  }
  return -1;
}


/* Zero out epoch-related variables for the controller */
static inline void init_sink_epoch_state() {
  ev_detected = 0;
  evdtc_sl1   = 0;
  evdtc_sl2   = 0;
  n_pkt_recv  = 0;
  all_data_rcvd = 0;
  new_command = 0;
  correct_epoch = 0;
  collection_bitmap = 0;    // Reset the reception bitmap for the collection phase
  a_payload.ack_bitmap = 0; // Reset the ACK bitmap
}


/* Zero out epoch-related variables for non-sink nodes */
static inline void init_node_epoch_state() {
  ev_detected = 0;
  i_detected  = 0;
  evdtc_sl1   = 0;
  evdtc_sl2   = 0;
  is_ack = 0;
  reception_bitmap = 0;
  n_pkt_recv    = 0;
  new_readings  = 0;
  correct_epoch = 0;
  new_s_act_command = 0;
  new_ctrpkt_recv = 0;
  update_held_measure = 0;
  act_ts = 0;
  act_lat = 61; /* Default "actuation timestamp" (in ms), communicated to Simulink when NO 
                * event is detected. Upon receiving an actuation command, actuators update
                * this value with the *real* actuation timestamp.
                * NB: Setting this value to 0 could cause malfunctions in the Simulink 
                * model emulating the water irrigation system. */
}


/* Compute the end-to-end actuation latency
 * NB: If the processing to compute the new actuation commands is NOT carried out onboard, CTRL_COMPUTATION_GUARD
 * is removed from the actuation latency as it is a fake delay, introduced by the need to interact with the 
 * testbed infrastructure, only */
static inline void compute_act_latency() {
  act_ts = RTIMER_NOW();

#if ON_BOARD /* The control related processing is carried out onboard */
  if (RTIMER_CLOCK_LT(wcb_info.t_ref, act_ts))  // No timer overflow occurred
    act_lat = TICKS_TO_MS(act_ts - wcb_info.t_ref);
  else // A timer overflow occurred, we should take this into account while computing the actuation command latency
    act_lat = TICKS_TO_MS(RTIMER_CLOCK_MAX - (wcb_info.t_ref - act_ts));
#else /* The control related processing is carried out in the testbed server, CTRL_COMPUTATION_GUARD should be removed
       * as it is a fake delay introduced by the need to interact with the testbed server */
  if (RTIMER_CLOCK_LT(wcb_info.t_ref, act_ts))  // No timer overflow occurred
    act_lat = TICKS_TO_MS(act_ts - wcb_info.t_ref) - TICKS_TO_MS(CTRL_COMPUTATION_GUARD);
  else // A timer overflow occurred, we should take this into account while computing the actuation command latency
    act_lat = TICKS_TO_MS(RTIMER_CLOCK_MAX - (wcb_info.t_ref - act_ts)) - TICKS_TO_MS(CTRL_COMPUTATION_GUARD);
#endif
}

/* -------------------------------------------------------------------------------------------------- */

/* ---------------------------------------- Main process -------------------------------------------- */
AUTOSTART_PROCESSES(&wcb_test);
PROCESS_THREAD(wcb_test, ev, data) {
  PROCESS_BEGIN();

  static struct etimer et;
  static bool ret;
  static int i, j;
  static char dist; 

  EPOCH_PRE_START_EV = process_alloc_event(); /* Used by the sensors to request new readings */
  EPOCH_END_EV       = process_alloc_event(); /* Wake up the main process and proceed with the logging */
  EPOCH_CTRL_EV      = process_alloc_event(); /* Used by the controller to compute actuation commands 
                                               * when such computation is not carried out onboard */

  deployment_load_ieee_addr();
  deployment_set_node_id_ieee_addr();

  is_sink = node_id == SINK_ID;

  if (!is_sink) {
    for (i=0; i<NUM_SOURCES; i++) {
      if (node_id == source_id[i]) { // Check if the node is a (flow) sensor
        is_source = 1;
        my_position = i; 
        for (j=0; j<NUM_HEIGHT_SENSORS; j++){ // Check if the node is a height sensor
          if (node_id == height_sensor_id[j]){
            is_hgtsensor = 1;
            break;
          }
        }
        break;
      }
    }
    if (!is_source) {
      for (i=0; i<NUM_ACTUATORS; i++) { // Check if the node is an actuator 
        if (node_id == actuator_id[i]) {
          is_actuator = 1;
          my_position = i;
          break;
        }
      }
    }
  }

  t_payload.src = node_id;

  if (is_sink)
    etimer_set(&et, START_DELAY_SINK * CLOCK_SECOND);
  else
    etimer_set(&et, START_DELAY_NONSINK * CLOCK_SECOND);

  PROCESS_YIELD_UNTIL(etimer_expired(&et));

  wcb_init();
  printf("I am alive! Node ID:%d\n", node_id);
  deployment_print_id_info();

  conf = wcb_get_config();
  conf.plds_T     = sizeof(app_t_payload);
  conf.plds_A     = sizeof(app_a_payload);
  conf.plds_CTRL  = sizeof(app_control_payload);
  conf.is_sink = is_sink;
  PRINT_WCB_CONFIG(conf);

  ret = wcb_start(&conf); /* Check if the configuration parameters are set properly and start WCB */
  if (!ret)
    printf("[ERROR] EPOCH 0 - WCB failed to start\n");

  while(1) {
    PROCESS_WAIT_EVENT();
    if (ev == EPOCH_END_EV) { /* The active portion of the epoch ended; it is time to print */
#if LOGGING
      if (is_actuator)
        printf("ACTUATOR %u:%u %u %u %u %u %u %u %hd %hd\n", wcb_info.epoch, ev_detected, evdtc_sl1, evdtc_sl2,\
              reception_bitmap, n_pkt_recv, my_position, new_s_act_command, s_act_command, act_lat);
      else if (is_source)
        printf("SENSOR %u:%u %u %u %u %u %u %u %u\n", wcb_info.epoch, i_detected, ev_detected, evdtc_sl1, evdtc_sl2,\
              is_ack, my_position, new_ctrpkt_recv, update_held_measure);
      else if (is_sink)
        printf("SINK %u:%u %u %u %u %u %u %u\n", wcb_info.epoch, ev_detected, evdtc_sl1, evdtc_sl2,\
              collection_bitmap , all_data_rcvd, a_payload.ack_bitmap, n_pkt_recv);
      else
        printf("NODE %u:%u %u %u %u %u\n", wcb_info.epoch, ev_detected, evdtc_sl1, evdtc_sl2,\
              reception_bitmap, n_pkt_recv);
      wcb_print_epoch_logs();
#endif /* LOGGING */
    }
    else if (ev == EPOCH_PRE_START_EV) { /* A new epoch is going to start soon, sensors request the readings for the new epoch */
      printf("GetReadings %u:%u %u\n", (wcb_info.epoch + 1), my_position, is_hgtsensor);
    }
    else if (ev == EPOCH_CTRL_EV) { /* If the processing is not held onboard the controller sends the measurements and the integral
                                     * height values acquired within the epoch to the server to compute the new actuation commands */
      printf("SensorStates %u:%hd %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd\n", wcb_info.epoch,\
            cont_state_array[0], cont_state_array[1], cont_state_array[2], cont_state_array[3], cont_state_array[4],\
            cont_state_array[5], cont_state_array[6], cont_state_array[7], cont_state_array[8], cont_state_array[9],\
            cont_state_array[10], cont_state_array[11], cont_state_array[12], cont_state_array[13], cont_state_array[14]);
    }
    else if (ev == serial_line_event_message) { /* New incoming info from the testbed / Simulink model */
      sscanf((char*)data, "%c", &dist);
      if ((dist == 'D') && is_source) { /* New readings for sensor nodes. NB: Flow and height sensors expect different incoming readings */
        if (is_hgtsensor) { /* The node is an height sensor */
          if (sscanf((char*)data,"%*c %hu %hd %hd %hd %hd", &ev_epoch, &s_trig_ms, &s_data_ms, &s_integral_hgt_trig,\
              &s_integral_hgt_data)) { // The incoming serial line message is formatted correctly
            new_readings = 1;
            if (wcb_info.epoch == (ev_epoch - 1)) // The readings arrived on time (namely in the correct epoch)
              correct_epoch = 1;
          }
        }
        else { // The node is a flow sensor, no integral height value is provided by Simulink
          if (sscanf((char*)data, "%*c %hu %hd %hd", &ev_epoch, &s_trig_ms, &s_data_ms)) { // The incoming serial line message is formatted correctly
            new_readings = 1;
            if (wcb_info.epoch == (ev_epoch - 1)) // The readings arrived on time (namely in the correct epoch)
              correct_epoch = 1;
          }
        }
      }
      else if ((dist == 'C') && is_sink) { /* The server provided new actuation commands */
        if (sscanf((char*)data, "%*c %hu %hd  %hd  %hd  %hd  %hd", &ev_epoch, &(ctrl_payload.upd_ctrl[0]),\
            &(ctrl_payload.upd_ctrl[1]), &(ctrl_payload.upd_ctrl[2]), &(ctrl_payload.upd_ctrl[3]), &(ctrl_payload.upd_ctrl[4]))) { // Serial line message correctly formatted
           new_command = 1;
           if (wcb_info.epoch == ev_epoch)  // Control commands received in the correct epoch
            correct_epoch = 1;
        }
      }
    }
  }
  PROCESS_END();
}
