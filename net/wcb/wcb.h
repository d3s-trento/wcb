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

#ifndef WCB_H_
#define WCB_H_

#include "glossy.h"
#include <stdbool.h>

#define WCB_MAX_SCAN_EPOCHS  200

typedef uint8_t wcb_addr_t;
typedef uint16_t wcb_epoch_t;

typedef struct {
  uint32_t period;       // WCB period in rtimer clock ticks
  uint8_t is_sink;       // Is this node the sink (aka the controller)
  uint8_t ntx_S;         // Number of Glossy TX in synchronization (S) slots
  uint16_t w_S;          // Max duration of S slots in rtimer ticks
  uint8_t plds_S;        // App. payload size for S slots
  uint8_t ntx_EV;        // Number of Glossy TX in EV slots (Event phase)
  uint16_t w_EV;         // Max duration of EV slots in rtimer ticks
  uint8_t plds_EV;       // App. payload size for EV slots
  uint8_t ntx_T;         // Number of Glossy TX in T slots (Collection and Recovery phases)
  uint16_t w_T;          // Max duration of T slots in rtimer ticks
  uint8_t plds_T;        // App. payload size for T slots
  uint8_t ntx_A;         // Number of Glossy TX in A slots (Collection and Recovery phases)
  uint16_t w_A;          // Max duration of A slots in rtimer ticks
  uint8_t plds_A;        // App. payload size for A slots
  uint8_t ntx_CTRL;      // Number of Glossy TX in control (CTRL) slots (Dissemination phase)
  uint16_t w_CTRL;       // Max duration of control slots in rtimer ticks
  uint8_t plds_CTRL;     // App. payload size for control slots
  uint8_t y;             /* Number of empty TA pairs in a row causing a node without data to leave 
                          * the Recovery phase and enter to sleep until the beginning of the Dissemination phase */
  uint8_t z;             /* Number of missing ACKs in a row causing a node with data to leave the Recovery 
                          * phase and enter to sleep until the beginning of the Dissemination phase */
  uint8_t enc_enable;    // Glossy-level encryption enabled (currently not supported in WCB)
  uint8_t scan_duration; // Scan duration in number of epochs
} wcb_config_t;

typedef struct {
  wcb_epoch_t    epoch;
  rtimer_clock_t t_ref;
} wcb_info_t;

extern wcb_info_t wcb_info; /* A variable holding the current state of WCB */


/* -------------------- WCB application interface (callbacks) -------------------- */

/* An interrupt-context callback called by WBC at the end of the Synchronization Phase */
void app_post_S();

/* An interrupt-context callback called by WCB non-controller nodes before the beginning of the Event phase.
 * - returned value: 
 *   - sensor nodes: true if the node detected an event, false otherwise.
 *   - non-sensor nodes: false, always.
 */
bool app_pre_EV();

/* An interrupt-context callback called by WCB after each EV slot in the Event phase.
 * - received: whether a correct packet was received in the EV slot;
 * - corrupted_EV: whether a corrupted packet was received during the EV flood. 
 *   (The reception of a corrupted packet might signal that someone was transmitting 
 *   during the EV phase but the event message was lost due to collisions/noise. 
 *   Therefore, upon receiving a corrupted packet, nodes still enter the Collection phase).
 * - n_ev: EV slot number, starting from 0.
 */
void app_post_EV(int received, uint8_t corrupted_EV, uint8_t n_ev);

/* An interrupt-context callback called by WCB nodes to decide whether to enter in the Collection phase
 * or not. If during the Event phase an event has been detected nodes enter the Collection phase,
 * if not the network immediately goes to sleep until next epoch. 
 * - returned value: true if the node detected an event, false otherwise.
 */
bool app_enter_collection();

/* An interrupt-context callback called by WCB at the beginning of the Collection Phase.
 * As event has been detected, sources acquire fresh measurements and update the payloads of
 * their data packets (T messages).
 */
void app_acquire_measurement();

/* An interrupt-context callback called by WCB non-controller nodes before each T dedicated slot
 * (Collection Phase) to check if it is their turn to TX.
 * - tx_round: slot counter for the Collection phase. 
 * - returned value: pointer to the application payload for T (sensor data) messages
 */
uint8_t* app_pre_Tded(uint8_t tx_round);

/* An interrupt-context callback called by WCB after each T slot (Collection and Recovery phases).
 * - received: whether a correct packet was received in the slot
 * - payload: pointer to the application payload of the received T massage
 * - returned value:
 *   - controller: pointer to the application payload for A (cumulative ACK) messages
 *   - non-controller nodes: NULL (this callback is exploited for analysis purposes, only) 
 */
uint8_t* app_post_T(int received, uint8_t* payload);

/* An interrupt-context callback called by the controller before transmitting the cumulative ACK that 
 * ends the Collection Phase. 
 * - returned value: 1 if all sensor readings have been collected at the controller during the 
 *   Collection phase, 0 otherwise.
 */
uint8_t app_complete_reception();

/* An interrupt-context callback called by the controller to store the ACK bitmap at the end of the 
 * Collection Phase. [For analysis purposes]
 */
void app_store_collection_bitmap();

/* An interrupt-context callback called by WCB non-controller nodes before each T slot in the 
 * Recovery Phase.
 * - returned value: pointer to the application payload for T (sensor data) messages
 */
uint8_t* app_pre_T();

/* An interrupt-context callback called by WCB non-controller nodes after each A slot.
 * - received: whether a correct packet was received in the slot
 * - payload: pointer to the application payload of the received A massage
 */
void app_post_A(int received, uint8_t* payload);

/* An interrupt-context callback called by the WCB controller before each control slot.
 * - returned value: pointer to the application payload for CTRL (control) messages (Dissemination 
 *   phase)
 */
uint8_t* app_pre_ctrl();

/* An interrupt-context callback called by WCB non-controller nodes after each control slot.
 * - received: whether a correct packet was received in the last CTRL slot
 * - payload: pointer to the application payload of the received control message
 * - complete_reception_flag: flag used by the controller to notify whether all sensor readings 
 *   have been received during the current epoch
 */
void app_post_CTRL(int received, uint8_t* payload, uint8_t complete_reception_flag);

/* An interrupt-context callback that signals the end of the active part of the epoch */
void app_epoch_end();

/* An interrupt-context callback that pings the app WCB_CONF_APP_PRE_EPOCH_CB_TIME ms before a 
 * new epoch starts
 */
void app_pre_epoch();

/* An interrupt-context callback called by WCB non-controller nodes before the beginning of the epoch
 * to check whether new sensor readings have been retrieved in time from Simulink. 
 * - returned value:
 *   - sensor nodes: true if new sensor readings have been received from Simulink in time, 
       false otherwise 
 *   - non-sensor nodes: false, always.
 */
bool app_have_new_readings();

/* An interrupt-context callback called by the controller to compute new control commands */
void app_control_functions();

/* An interrupt-context callback called by the controller to check whether new control
 * commands have been computed in time.
 * - returned value: true if the controller managed to compute / retrieve new control commands in
 *   time, false otherwise.
 */
bool app_have_new_ctrlcommands();


/* -------------------- WCB application interface (requests) -------------------- */

/* Initialise WCB (to be called once at boot) */
void wcb_init();

/* Read the current configuration.
 * - returned value: the current protocol configuration
 */
wcb_config_t wcb_get_config();

/* Print the current WCB configuration (per node) */
#define PRINT_WCB_CONFIG(conf) do {\
 printf("WCB config. Node ID %x\n", node_id);\
 printf("Period: %lu\n", (conf).period);\
 printf("Sink: %u\n", (conf).is_sink);\
 printf("S: %u %u %u\n", (conf).ntx_S, (conf).w_S, (conf).plds_S);\
 printf("T: %u %u %u\n", (conf).ntx_T, (conf).w_T, (conf).plds_T);\
 printf("A: %u %u %u\n", (conf).ntx_A, (conf).w_A, (conf).plds_A);\
 printf("EV: %u %u %u\n", (conf).ntx_EV, (conf).w_EV, (conf).plds_EV);\
 printf("CONTROL: %u %u %u\n", (conf).ntx_CTRL, (conf).w_CTRL, (conf).plds_CTRL);\
 printf("Term: %u %u\n", (conf).y, (conf).z);\
 printf("Enc: %u, Scan: %u\n", (conf).enc_enable, (conf).scan_duration);\
} while (0)

/* Start WCB with the given configuration.
 * - returned value: true if WCB started correctly, false otherwise.
 */
bool wcb_start(wcb_config_t* conf);

/* Print logs for the current epoch.
 * If worth, the application can call this function from its process at every epoch.
 */
void wcb_print_epoch_logs();


#endif /* WCB_H_ */
