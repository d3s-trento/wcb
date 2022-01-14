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
 *          Timofei Istomin <tim.ist@gmail.com>
 *
 */

#include "wcb.h"
#include "wcb-conf.h"
#include "wcb-private.h"
#include <stdio.h>
#include "cc2538-rf.h"


static union {
  uint8_t raw[WCB_PKTBUF_LEN];
  struct  __attribute__((packed, aligned(1))) {
    uint8_t type;
    union  __attribute__((packed, aligned(1))){
      wcb_sync_hdr_t    sync_hdr;
      wcb_ack_hdr_t     ack_hdr;
      wcb_control_hdr_t ctrl_hdr;
    };
  };
} buf;

#define WCB_S_HDR_LEN (sizeof(wcb_sync_hdr_t) + sizeof(buf.type))
#define WCB_EV_HDR_LEN (sizeof(buf.type))
#define WCB_T_HDR_LEN (sizeof(buf.type))
#define WCB_A_HDR_LEN (sizeof(wcb_ack_hdr_t) + sizeof(buf.type))
#define WCB_CTRL_HDR_LEN (sizeof(wcb_control_hdr_t) + sizeof(buf.type))
#define BZERO_BUF() bzero(buf.raw, WCB_PKTBUF_LEN)

static wcb_config_t conf = {
  .period      = WCB_CONF_PERIOD,
  .is_sink     = WCB_CONF_IS_SINK,
  .ntx_S       = WCB_CONF_NTX_S,
  .w_S         = WCB_CONF_DUR_S,
  .plds_S      = 0,
  .ntx_T       = WCB_CONF_NTX_T,
  .w_T         = WCB_CONF_DUR_T,
  .plds_T      = 0,
  .ntx_A       = WCB_CONF_NTX_A,
  .w_A         = WCB_CONF_DUR_A,
  .plds_A      = 0,
  .ntx_EV      = WCB_CONF_NTX_EV,
  .w_EV        = WCB_CONF_DUR_EV,
  .plds_EV     = 0,
  .ntx_CTRL    = WCB_CONF_NTX_CTRL,
  .w_CTRL      = WCB_CONF_DUR_CTRL,
  .plds_CTRL   = 0,
  .y           = WCB_CONF_MAX_SILENT_TAS,
  .z           = WCB_CONF_MAX_MISSING_ACKS,
  
  .enc_enable    = 0,
  .scan_duration = WCB_MAX_SCAN_EPOCHS,
};

#define WCB_S_TOTAL_LEN    (WCB_S_HDR_LEN + conf.plds_S)
#define WCB_T_TOTAL_LEN    (WCB_T_HDR_LEN + conf.plds_T)
#define WCB_A_TOTAL_LEN    (WCB_A_HDR_LEN + conf.plds_A)
#define WCB_EV_TOTAL_LEN   (WCB_EV_HDR_LEN + conf.plds_EV)
#define WCB_CTRL_TOTAL_LEN (WCB_CTRL_HDR_LEN + conf.plds_CTRL)

wcb_info_t wcb_info;                                // Public read-only status information about WCB

static struct rtimer rt;                            // Rtimer used to schedule WCB
static rtimer_callback_t timer_handler;             // Pointer to the main thread function (either root or node)
static rtimer_clock_t t_ref_root;                   // Epoch reference time (only for root)
static rtimer_clock_t t_ref_estimated;              // Estimated reference time for the current epoch
static rtimer_clock_t t_ref_corrected_s;            // Reference time acquired during the S slot of the current epoch (used to schedule the next S phase)
static rtimer_clock_t t_ref_corrected;              // Reference time acquired during the S, A or CTRL slot of the current epoch (used to schedule the next slots within the epoch)
static rtimer_clock_t t_ref_skewed;                 // Reference time in the local time frame
static rtimer_clock_t t_wakeup;                     // Time to wake up to prepare for the next epoch
static rtimer_clock_t t_s_start, t_s_stop;          // Start/stop times for S slots
static rtimer_clock_t t_slot_start, t_slot_stop;    // Start/stop times for EV, T, A, and CTRL slots
static rtimer_clock_t t_ctrl_command;               // Time at which the controller finishes to compute CTRL commands 
static rtimer_clock_t t_ctrl_start;                 // Time at which the controller should be ready to start the Dissemination phase

static struct pt pt;                                // Main protothread of WCB

/* --- ROOT'S PROTOTHREADS --- */
static struct pt pt_s_root;                         // Protothread for S phase (root)
static struct pt pt_evphase_evonly_root;            // Protothread for the Event Phase (root), slots: EV..EV
static struct pt pt_collection_root;                // Protothread for the Collection Phase (root), slots: TTT..TA
static struct pt pt_recovery_root;                  // Protothread for the Recovery Phase (root), slots: TA..TA
static struct pt pt_ctrl_root;                      // Protothread for the Dissemination Phase (root), slots: CTRL..CTRL

/* --- NODE'S PROTOTHREADS --- */
static struct pt pt_scan;                           // Protothread for scanning the channel (only once, at bootstrap)
static struct pt pt_s_node;                         // Protothread for S phase (non-root)
static struct pt pt_evphase_evonly_node;            // Protothread for the Event Phase (non-root), slots: EV..EV
static struct pt pt_collection_node;                // Protothread for the Collection Phase (non-root), slots: TTT..TA
static struct pt pt_recovery_node;                  // Protothread for the Recovery Phase (non-root), slots: TA..TA
static struct pt pt_ctrl_node;                      // Protothread for the Dissemination Phase (non-root), slots: CTRL..CTRL

static uint8_t* payload;                            // Pointer to the application payload for the current slot
static wcb_epoch_t epoch;                           // Epoch number received from the sink (or extrapolated)
static uint8_t  channel;                            // Current channel
static uint16_t synced_with_ack;                    // Synchronized with an acknowledgement (A, CTRL slots)
static uint16_t n_noack_epochs;                     // Current number of consecutive epochs the node did not synchronize with *any* acknowledgement
static uint16_t sync_missed;                        // Current number of consecutive S phases without resynchronization
static uint16_t sink_id = GLOSSY_UNKNOWN_INITIATOR; // The node ID of the sink
static uint16_t skew_estimated;                     // Whether the clock skew over WCB_CONF_PERIOD has already been estimated
static uint16_t long_skew_estimated;                // Whether the clock skew over WCB_CONF_PERIOD has been estimated during the "bootstrap"
static int      period_skew;                        // Current estimation of clock skew over a period of length WCB_CONF_PERIOD
static int      period_skew_tmp;
static uint8_t  successful_scan;                    // Whether the node managed to join the network (synchronise with the sink)
static uint16_t correct_packet;                     // Whether the received packet is correct
static uint16_t sleep_order;                        // The sink sent the sleep command
static uint16_t n_ta;                               // The current TA index in the Recovery Phase
static uint16_t n_t_tx;                             // How many times the node tried to send data in the epoch [for analysis purposes]
static uint16_t n_empty_ts;                         // Number of consecutive "T" slots without data in the Recovery phase
static uint16_t n_noacks;                           // Number of consecutive "A" phases without any acks
static uint16_t n_bad_acks;                         // Number of consecutive "A" phases with bad acks
static uint16_t n_all_acks;                         // Number of negative and positive acks
static uint16_t n_badtype_A;                        // Number of packets of wrong type received in A slots
static uint16_t n_badlen_A;                         // Number of packets of wrong length received in A slots
static uint16_t n_badcrc_A;                         // Number of packets with wrong CRC received in A slots
static uint16_t recvtype_S;                         // Type of packet received in the S phase
static uint16_t recvlen_S;                          // Length of packet received in the S phase
static uint16_t recvsrc_S;                          // Source address of packet received in the S phase
static uint16_t hopcount;                           // Node hop count inferred from Glossy (during S floods)
static uint16_t rx_count_S, tx_count_S;             // Tx and rx counters for the S slot as reported by Glossy
static uint16_t rx_count_T;                         // Rx counters for T slots as reported by Glossy
static uint16_t rx_count_A;                         // Rx counters for A slots as reported by Glossy
static uint16_t rx_count_EV;                        // Rx counters for EV slots as reported by Glossy
static uint16_t rx_count_CTRL;                      // Rx counters for CTRL slots as reported by Glossy
static uint8_t  recv_pkt_type;                      // Holds the received packet type after a Glossy session
static uint8_t  all_states_rcvd;                    // Flag to notify whether the controller received all sensor states in the current epoch
static unsigned long ton_S, ton_T, ton_A, ton_EV, ton_CTRL;                // Total duration of the phases in the current epoch
static unsigned long tf_S, tf_T, tf_A, tf_EV, tf_CTRL;                     // Total duration of the phases when all N packets are received
static uint16_t n_short_S, n_short_T, n_short_A, n_short_EV, n_short_CTRL; // Number of "complete" S/T/A/EV/CTRL phases (those counted in tf_S/tf_T/tf_A/tf_EV/tf_CTRL)

static uint8_t  log_recv_type;
static uint16_t log_recv_length;
static uint16_t log_ta_status;
static uint16_t log_ack_skew_err;                   // ACK skew estimation outlier

static uint8_t n_ev;                                // Current (relative) slot index in the Event phase
static uint8_t n_t;                                 // Current (relative) slot index in the Collection Phase
static uint8_t n_ctrls;                             // Current (relative) slot index in the Dissemination phase

/* For analysis purposes */
static uint8_t corrupted_EV;                        // Whether a corrupted packet has been received during the last EV flood
static uint8_t enter_coll_phase;                    // Whether a node enters in the Collection Phase
static uint8_t enter_recv_phase;                    // Whether a node enters in the Recovery Phase
static uint8_t enter_control_phase;                 // Whether a node enters in the Dissemination Phase
static int tx_power;


/* ---------- Offsets of the various phases ---------- */
/* S Phase offset */
#define PHASE_S_END_OFFS       (WCB_INIT_GUARD*2 + conf.w_S)

/* Event Phase offset & ref definition */
#define EV_START_OFFS          (PHASE_S_END_OFFS + WCB_INTER_SLOT_GAP)
#define EV_DURATION            (conf.w_EV + WCB_INTER_SLOT_GAP)
#define PHASE_EV_OFFS(n)       (EV_START_OFFS + (n)*EV_DURATION)

/* Collection Phase offset & ref definition */
#define TDED_START_OFFS        (PHASE_EV_OFFS(WCB_EV_SLOTS))
#define TDED_DURATION          (conf.w_T + WCB_INTER_SLOT_GAP)
#define PHASE_TDED_OFFS(n)     (TDED_START_OFFS + (n)*TDED_DURATION)
#define N_CACK_TO_REF(tref, n) (tref-PHASE_TDED_OFFS(n))

/* Recovery Phase offset & ref definition */
#define TAS_START_OFFS        (PHASE_TDED_OFFS(NUM_SOURCES) + conf.w_A + WCB_INTER_SLOT_GAP)
#define TA_DURATION           (conf.w_T + conf.w_A + 2*WCB_INTER_SLOT_GAP)
#define PHASE_T_OFFS(n)       (TAS_START_OFFS + (n)*TA_DURATION)
#define PHASE_A_OFFS(n)       (PHASE_T_OFFS(n) + (conf.w_T + WCB_INTER_SLOT_GAP))
#define N_TA_FROM_OFFS(offs)  ((offs - TAS_START_OFFS)/TA_DURATION)
#define N_TA_TO_REF(tref, n)  (tref-PHASE_A_OFFS(n))

/* Dissemination (CTRL) Phase offset and ref definition */
#if RECOVERY_ON
  #define CTRL_START_OFFS    (PHASE_T_OFFS(WCB_MAX_TAS_RECV) + CTRL_COMPUTATION_GUARD)
#else
  #define CTRL_START_OFFS    (PHASE_TDED_OFFS(NUM_SOURCES) + conf.w_A + WCB_INTER_SLOT_GAP + CTRL_COMPUTATION_GUARD)
#endif
#define CTRL_PHASE_DURATION  (conf.w_CTRL + WCB_INTER_SLOT_GAP)
#define CTRL_PHASE_OFFS(n)   (CTRL_START_OFFS + (n)*CTRL_PHASE_DURATION)
#define CTRL_TO_REF(tref, n) (tref - CTRL_PHASE_OFFS(n))
/* --------------------------------------------------- */

/* --- Bootstrap configuration --- */
#if DEP_BOOTSTRAP
  static uint16_t fast_bstrap_epochs;
  #define FAST_BS_DURATION 30*(uint32_t)RTIMER_SECOND
  #define BS_PERIOD (conf.w_S + 90) /* 66 doesn't work well with WCB_REF_SHIFT 17. TBD: investigate why */
  #define BS_N 5
  #define BS_EPOCHS_TO_STABLE_CLOCK 500
#endif
/* ------------------------------- */

/* --- Log info about the current epoch --- */
#define UPDATE_SLOT_STATS(phase, transmitting) do { \
  unsigned long trx_on = glossy_get_trx_on(); \
  ton_##phase += trx_on; \
  if (glossy_get_n_tx() >= ((transmitting) ? ((conf.ntx_##phase) - 1) : (conf.ntx_##phase))) { \
    tf_##phase += trx_on; \
    n_short_##phase ++; \
  } \
} while(0)
/* ---------------------------------------- */

#include "wcb-chseq.c"

/* --------------------------------------------------- */
#define IS_SYNCED()          (glossy_is_t_ref_updated())
#define PRE_TIME 10          /* This variable could be further shrink */

#define WAIT_UNTIL(time, cl_pt) \
{\
  rtimer_set(t, (time), 0, timer_handler, ptr); \
  PT_YIELD(cl_pt); \
}

#define GLOSSY(init_id, length, type_, ntx, channel, is_sync, pt) \
buf.type = type_;\
WAIT_UNTIL(t_slot_start, pt);\
glossy_start(init_id, buf.raw, length, ntx, channel, is_sync);

#define GLOSSY_WAIT(pt) WAIT_UNTIL(t_slot_stop, pt); recv_pkt_type = buf.type; glossy_stop();
/* --------------------------------------------------- */

/* ---------- Workarounds for wrong ref times reported by Glossy (which happens rarely) ---------- */

/* Sometimes it happens due to a wrong hop count (CRC collision?) */
static inline int correct_hops() {
#if (MAX_CORRECT_HOPS > 0)
  return (glossy_get_relay_cnt_first_rx() <= MAX_CORRECT_HOPS);
#else
  return 1;
#endif
}

static inline int correct_ack_skew(rtimer_clock_t new_ref) {
#if WCB_ACK_SKEW_ERROR_DETECTION
  static int new_skew;
#if (MAX_CORRECT_HOPS > 0) /* Check if the hop count looks legitimate */
  if (!correct_hops())
    return 0;
#endif /* MAX_CORRECT_HOPS */
  /* The number of hops looks reasonable, let's check if the same holds for the skew */
  new_skew = new_ref - t_ref_corrected;
  if (new_skew < 60 && new_skew > -60)
    return 1;  /* The skew looks legitimate */
  else if (sync_missed && !synced_with_ack) 
    return 1;  /* The skew is big but we did not synchronise during the current epoch... So it might be fine */
  else { /* The skew looks suspicious, don't trust this value! */
    log_ack_skew_err = new_skew;
    return 0;
  }
#else /* !WCB_ACK_SKEW_ERROR_DETECTION */ 
  return 1;
#endif
}
/* ---------- End of workarounds for wrong ref time reported by Glossy ---------- */

/* Zero out epoch-related variables */
static inline void init_epoch_state() {
  tf_S = 0; tf_T = 0; tf_A = 0; tf_EV = 0; tf_CTRL = 0;
  n_short_S = 0; n_short_T = 0; n_short_A = 0; n_short_EV = 0; n_short_CTRL = 0;
  ton_S = 0; ton_T = 0; ton_A = 0; ton_EV = 0; ton_CTRL = 0;
  n_badlen_A = 0; n_badtype_A = 0; n_badcrc_A = 0;
  log_ack_skew_err = 0;

  n_empty_ts = 0;
  n_noacks = 0;
  n_bad_acks = 0;
  n_ta = 0;
  n_t_tx = 0;
  n_all_acks = 0;
  sleep_order = 0;
  synced_with_ack = 0;

  recvlen_S = 0;
  recvtype_S = 0;
  recvsrc_S = 0;

  n_ev = 0;    // Slot counter for the Event Phase
  n_t = 0;     // Slot counter for the Collection Phase
  n_ctrls = 0; // Slot counter for the Dissemination Phase
  enter_coll_phase = 0;
  enter_recv_phase = 0;
  all_states_rcvd = 0;
  enter_control_phase = 0;
}

/* ---------------------------------------------- Synchronization Phase thread (root) ---------------------------------------------- */
PT_THREAD(s_root_thread(struct rtimer *t, void* ptr)) 
{
  PT_BEGIN(&pt_s_root);

  // Prepare the header of the packet
  buf.sync_hdr.epoch = epoch;
  buf.sync_hdr.src   = node_id;

  // Set the transmission channel
  channel = get_channel_epoch(epoch);

  t_slot_start = t_s_start;
  t_slot_stop  = t_s_stop;

  GLOSSY(
    node_id,
    WCB_S_TOTAL_LEN, 
    WCB_TYPE_SYNC, 
    conf.ntx_S,
    channel, 
    GLOSSY_WITH_SYNC,
    &pt_s_root);
  GLOSSY_WAIT(&pt_s_root);

  UPDATE_SLOT_STATS(S, 1);

  // Store some stats about the S flood
  tx_count_S = glossy_get_n_tx();
  rx_count_S = glossy_get_n_rx();

  app_post_S();
  BZERO_BUF();
  PT_END(&pt_s_root);
}


/* ---------------------------------------------- Event Phase Thread (root) ---------------------------------------------- */
PT_THREAD(evphase_evonly_root_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_evphase_evonly_root);

  /* --- Event loop --- */
  while (n_ev < WCB_EV_SLOTS) {
    t_slot_start = t_ref_root - WCB_SHORT_GUARD + PHASE_EV_OFFS(n_ev);
    t_slot_stop = t_slot_start + conf.w_EV + WCB_SHORT_GUARD + WCB_SINK_END_GUARD;

    channel = get_channel_epoch_ta(epoch, n_ev, 1);

    GLOSSY(
      GLOSSY_UNKNOWN_INITIATOR, 
      WCB_EV_TOTAL_LEN, 
      WCB_TYPE_EV,
      conf.ntx_EV, 
      channel, 
      GLOSSY_WITHOUT_SYNC,
      &pt_evphase_evonly_root);
    GLOSSY_WAIT(&pt_evphase_evonly_root);

    UPDATE_SLOT_STATS(EV, 0);

    correct_packet = 0;

    rx_count_EV = glossy_get_n_rx();
    corrupted_EV = glossy_is_corrupted();

    if (rx_count_EV) { // A packet has been received, check if it is a correct one
      log_recv_type = recv_pkt_type;
      log_recv_length = glossy_get_payload_len();
      correct_packet = (log_recv_length == WCB_EV_TOTAL_LEN && log_recv_type == WCB_TYPE_EV);
    }
    app_post_EV(correct_packet, corrupted_EV, n_ev);
    BZERO_BUF();
    n_ev ++;
  } /* --- End of Event loop --- */

  PT_END(&pt_evphase_evonly_root);
}


/* ---------------------------------------------- Collection Phase thread (root) ---------------------------------------------- */
PT_THREAD(collection_root_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_collection_root);

  /* ----------- (Dedicated) T slot sequence loop ----------- */
  while (n_t < NUM_SOURCES) {
    t_slot_start = t_ref_root - WCB_SHORT_GUARD + PHASE_TDED_OFFS(n_t);  
    t_slot_stop = t_slot_start + conf.w_T + WCB_SHORT_GUARD + WCB_SINK_END_GUARD;
    
    channel = get_channel_epoch_ta(epoch, n_t, 2);

    GLOSSY(
      GLOSSY_UNKNOWN_INITIATOR, 
      WCB_T_TOTAL_LEN, 
      WCB_TYPE_DATA,
      conf.ntx_T, 
      channel, 
      GLOSSY_WITHOUT_SYNC,
      &pt_collection_root);
    GLOSSY_WAIT(&pt_collection_root);

    UPDATE_SLOT_STATS(T, 0);

    correct_packet = 0;
    rx_count_T = glossy_get_n_rx();

    if (rx_count_T) { // A packet has been received, check if it is a correct one
      log_recv_type = recv_pkt_type;
      log_recv_length = glossy_get_payload_len();
      correct_packet = (log_recv_length == WCB_T_TOTAL_LEN && log_recv_type == WCB_TYPE_DATA);
    }

    payload = app_post_T(correct_packet, buf.raw + WCB_T_HDR_LEN);
    BZERO_BUF();

    n_t ++;
  } 
  /* ----------- End of the (dedicated) T slot sequence loop ----------- */

  /* ----------- Global acknowledgment slot ----------- */
  /* Prepare the header of the cumulative ACK */
  buf.ack_hdr.epoch = epoch;
  all_states_rcvd = app_complete_reception();           // Check if all sensor readings have been collected
  /* Set the command field accordingly */
  if (all_states_rcvd)                                  // The controller collected all updated readings; no need to enter in the Recovery Phase
    WCB_SET_ACK_ALL_READINGS_RECEIVED(buf.ack_hdr);
  else                                                  // The controller didn't receive all updated readings. Recovery is needed.
#if RECOVERY_ON                                        
    WCB_SET_ACK_AWAKE(buf.ack_hdr);                     // The recovery feature is enabled, nodes should enter in the Recovery Phase
#else
    WCB_SET_ACK_SLEEP_UNTIL_CTRLPHASE(buf.ack_hdr);     /* The TA chain is disabled, the controller puts the network 
                                                         * to sleep until the Dissemination phase and computes new actuation
                                                         * commands with the data in its possession */
#endif

  app_store_collection_bitmap(); /* [For debugging and analysis purposes] 
                                  * Store the set of sensors measurements received in the Collection Phase */

  memcpy(buf.raw + WCB_A_HDR_LEN, payload, conf.plds_A); // Prepare the payload of the ACK message

  t_slot_start = t_ref_root + PHASE_TDED_OFFS(n_t);
  t_slot_stop = t_slot_start + conf.w_A;

  channel = get_channel_epoch_ta(epoch, n_t, 2);

  GLOSSY(
    node_id, 
    WCB_A_TOTAL_LEN,
    WCB_TYPE_ACK,
    conf.ntx_A, 
    channel,
    WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
    &pt_collection_root);
  GLOSSY_WAIT(&pt_collection_root);

  UPDATE_SLOT_STATS(A, 1);
  BZERO_BUF();
  /* ----------- End of the global acknowledgment  slot ----------- */

  PT_END(&pt_collection_root);
}


/* ---------------------------------------------- Recovery Phase thread (root) ---------------------------------------------- */
PT_THREAD(recovery_root_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_recovery_root);

  /* --- TA chain ---*/
  while (!all_states_rcvd && !sleep_order) { 

    /* --- T slot ---*/
    t_slot_start = t_ref_root - WCB_SHORT_GUARD + PHASE_T_OFFS(n_ta);
    t_slot_stop = t_slot_start + conf.w_T + WCB_SHORT_GUARD + WCB_SINK_END_GUARD;

    channel = get_channel_epoch_ta(epoch, n_ta, 3);

    GLOSSY(
      GLOSSY_UNKNOWN_INITIATOR, 
      WCB_T_TOTAL_LEN, 
      WCB_TYPE_DATA,
      conf.ntx_T, 
      channel, 
      GLOSSY_WITHOUT_SYNC,
      &pt_recovery_root);
    GLOSSY_WAIT(&pt_recovery_root);

    UPDATE_SLOT_STATS(T, 0);

    correct_packet = 0;
    rx_count_T = glossy_get_n_rx();

    if (rx_count_T) { // A packet has been received, check if it is a correct/legitimate one
      n_empty_ts = 0;
      log_recv_type = recv_pkt_type;
      log_recv_length = glossy_get_payload_len();
      correct_packet = (log_recv_length == WCB_T_TOTAL_LEN && log_recv_type == WCB_TYPE_DATA);
    }
    else if (glossy_is_corrupted()) { // The received packet was corrupted
      n_empty_ts = 0;
    }
    else { // Just silence
      n_empty_ts ++;
    }

    payload = app_post_T(correct_packet, buf.raw + WCB_T_HDR_LEN);

    BZERO_BUF();

    /* --- End of the T shared slot, begin of the A slot ---*/
    // Prepare the header of the ACK message
    buf.ack_hdr.epoch = epoch;
    all_states_rcvd = app_complete_reception();
    sleep_order = (n_ta >= WCB_MAX_TAS_RECV-1);
    
    if (all_states_rcvd) // The controller collected all sensor states, set the network to sleep until the Dissemination Phase
      WCB_SET_ACK_ALL_READINGS_RECEIVED(buf.ack_hdr);
    else {
      if (sleep_order) // The controller didn't receive all sensor states, but there is no more time to schedule an additional TA
        WCB_SET_ACK_SLEEP_UNTIL_CTRLPHASE(buf.ack_hdr);
      else // The controller didn't receive all sensor states, schedule a new TA 
        WCB_SET_ACK_AWAKE(buf.ack_hdr);
    }

    memcpy(buf.raw + WCB_A_HDR_LEN, payload, conf.plds_A); // Prepare the payload of the ACK message

    t_slot_start = t_ref_root + PHASE_A_OFFS(n_ta);
    t_slot_stop = t_slot_start + conf.w_A;

    GLOSSY(
      node_id, 
      WCB_A_TOTAL_LEN,
      WCB_TYPE_ACK,
      conf.ntx_A, 
      channel,
      WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
      &pt_recovery_root);
    GLOSSY_WAIT(&pt_recovery_root);

    UPDATE_SLOT_STATS(A, 1);
    BZERO_BUF();
    /* --- End of the A slot ---*/
    n_ta ++;
  } /* --- End of TA chain --- */
  PT_END(&pt_recovery_root);
}


/* ---------------------------------------------- Control Phase thread (root) ---------------------------------------------- */
PT_THREAD(ctrl_root_thread(struct rtimer *t, void* ptr)) 
{
  PT_BEGIN(&pt_ctrl_root);

  /* The same CTRL packet is transmitted WCB_CONTROL_SLOTS times in a raw, thus it is possible to prepare the header and the payload 
   * of the control packet once (outside the while loop), and clear the buffer ONLY at the end of the loop */
  buf.ctrl_hdr.epoch = epoch; // Prepare the header
  payload = app_pre_ctrl();   // Prepare the payload
  if (payload) {
    memcpy(buf.raw + WCB_CTRL_HDR_LEN, payload, conf.plds_CTRL);
  }

  /* --- Start of the Control loop --- */
  while (n_ctrls < WCB_CONTROL_SLOTS) {

    channel = get_channel_epoch_ta(epoch, n_ctrls, 4);

    t_slot_start = t_ref_root + CTRL_PHASE_OFFS(n_ctrls);
    t_slot_stop  = t_slot_start + conf.w_CTRL;

    GLOSSY(
      node_id,
      WCB_CTRL_TOTAL_LEN, 
      all_states_rcvd ? SET_RECEPTION_FLAG(WCB_TYPE_CONTROL) : WCB_TYPE_CONTROL, /* The controller informs the network about the correct 
                                                                                  * reception of all measurements by setting the most
                                                                                  * significant bit of the type field to 1. */
      conf.ntx_CTRL,
      channel, 
      WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
      &pt_ctrl_root);
    GLOSSY_WAIT(&pt_ctrl_root);

    UPDATE_SLOT_STATS(CTRL, 1);
    n_ctrls ++;
  }  /* --- End of Control loop --- */

  BZERO_BUF(); 

  PT_END(&pt_ctrl_root);
}

/* ---------------------------------------------- Main thread (root) ---------------------------------------------- */
static char root_main_thread(struct rtimer *t, void *ptr) {
  PT_BEGIN(&pt);

  t_ref_root = RTIMER_NOW() + PRE_TIME;

#if DEP_BOOTSTRAP 
  t_s_start = t_ref_root;
  t_s_stop = t_s_start + conf.w_S;

  while (epoch < fast_bstrap_epochs) {
    t_ref_root = t_s_start;
    epoch ++;
    buf.sync_hdr.epoch = epoch;
    buf.sync_hdr.src   = node_id;

    t_slot_start = t_s_start;
    t_slot_stop  = t_s_stop;

    GLOSSY(
      node_id,
      WCB_S_TOTAL_LEN, 
      WCB_TYPE_SYNC, 
      BS_N,
      WCB_DEF_CHANNEL, 
      GLOSSY_WITH_SYNC,
      &pt);
    GLOSSY_WAIT(&pt);

    t_s_start += BS_PERIOD;
    t_s_stop = t_s_start + conf.w_S;
  }

  t_ref_root += conf.period;
  t_wakeup = t_ref_root - WCB_INTER_SLOT_GAP;

  WAIT_UNTIL(t_wakeup - WCB_APP_PRE_EPOCH_CB_TIME, &pt);
  app_pre_epoch();

  WAIT_UNTIL(t_wakeup, &pt);
#endif

  while (1) {
    init_epoch_state(); /* Zero out epoch-related variables */

    epoch ++;
    wcb_info.epoch = epoch;
    wcb_info.t_ref = t_ref_root;

    t_s_start = t_ref_root;
    t_s_stop = t_s_start + conf.w_S;

    WAIT_UNTIL(t_s_start - PRE_TIME, &pt);

    PT_SPAWN(&pt, &pt_s_root, s_root_thread(t, ptr)); /* Synchronization thread */

    PT_SPAWN(&pt, &pt_evphase_evonly_root, evphase_evonly_root_thread(t, ptr)); /* Event thread */

    if (app_enter_collection()) { /* An event has been detected, enter in the Collection Phase */
      enter_coll_phase = 1; /* [For analysis purposes, only] */
      PT_SPAWN(&pt, &pt_collection_root, collection_root_thread(t, ptr)); /* Collection thread */

#if RECOVERY_ON /* If the Recovery phase is enabled, check whether a recovery action is needed */
      if (!all_states_rcvd) { /* At least one message was lost, enter the Recovery Phase */
        enter_recv_phase = 1; /* [For analysis purposes, only]  */
        PT_SPAWN(&pt, &pt_recovery_root, recovery_root_thread(t, ptr)); /* Recovery thread */
      }
#endif

      app_control_functions(); /* Compute new actuation commands */
#if ON_BOARD
      /* Check if we managed to compute the new actuation commands in time; 
       * if not, rise an error and exit from the protothread */
      t_ctrl_command = RTIMER_NOW();
      t_ctrl_start = t_ref_root + CTRL_START_OFFS - WCB_CTRL_GUARD;
      /* Check for timer overflow */
      if ((RTIMER_CLOCK_LT(t_ref_root, t_ctrl_command) && RTIMER_CLOCK_LT(t_ref_root, t_ctrl_start)) || 
        (RTIMER_CLOCK_LT(t_ctrl_command, t_ref_root) && RTIMER_CLOCK_LT(t_ctrl_start, t_ref_root))) { /* No overflow occurred or both timers overflowed,
                                                                                                       * we can safely compare the two timers */
        if (RTIMER_CLOCK_LT(t_ctrl_start, t_ctrl_command)) { /* We are late, the control computations took too long */
          printf("[ERROR] EPOCH: %d - THE COMPUTATION OF ACTUATION COMMANDS TOOK TOO LONG!\n", epoch);
          PT_EXIT(&pt);
        }
      }
      else if (RTIMER_CLOCK_LT(t_ref_root, t_ctrl_start) && RTIMER_CLOCK_LT(t_ctrl_command, t_ref_root)) { /* Only the control timer overflowed, 
                                                                                                            * t_ctrl_command > t_ctrl_start */
        printf("[ERROR] EPOCH: %d - THE COMPUTATION OF ACTUATION COMMANDS TOOK TOO LONG!\n", epoch);
        PT_EXIT(&pt);
      }
#endif      
      WAIT_UNTIL(t_ref_root + CTRL_START_OFFS - WCB_CTRL_GUARD, &pt);
      if (!app_have_new_ctrlcommands()) {
        printf("[ERROR] EPOCH: %d - NO DATA AVAILABLE\n", epoch);
        PT_EXIT(&pt);
      }
      enter_control_phase = 1;
      WAIT_UNTIL(t_ref_root + CTRL_START_OFFS - PRE_TIME, &pt);
      PT_SPAWN(&pt, &pt_ctrl_root, ctrl_root_thread(t, ptr)); /* Control Dissemination thread */
    }

    t_ref_root += conf.period;
    t_wakeup = t_ref_root - WCB_INTER_SLOT_GAP; /* Time to wake up to prepare for the next epoch */
    app_epoch_end();

    WAIT_UNTIL(t_wakeup - WCB_APP_PRE_EPOCH_CB_TIME, &pt);

    app_pre_epoch();
    WAIT_UNTIL(t_wakeup, &pt);
  }
  PT_END(&pt);
}

/* ---------------------------------------------- Scan thread (node) ---------------------------------------------- */
PT_THREAD(scan_thread(struct rtimer *t, void* ptr))
{
  static uint32_t max_scan_duration, scan_duration;
  PT_BEGIN(&pt_scan);

  channel = get_channel_node_bootstrap(SCAN_RX_NOTHING);

  max_scan_duration = conf.period * conf.scan_duration; // To avoid overflows
  scan_duration = 0;

  /* --- Scanning loop --- */
  while (1) {
    t_slot_start = RTIMER_NOW() + 6; // + 6 is just to be sure
    t_slot_stop = t_slot_start + WCB_SCAN_SLOT_DURATION;

    GLOSSY(
      GLOSSY_UNKNOWN_INITIATOR,
      GLOSSY_UNKNOWN_PAYLOAD_LEN,
      GLOSSY_IGNORE_TYPE,
      GLOSSY_UNKNOWN_N_TX_MAX,
      channel,
      GLOSSY_WITH_SYNC,
      &pt_scan);
    GLOSSY_WAIT(&pt_scan);

    if (glossy_get_n_rx() > 0) { /* Something has been received */
      recvtype_S = recv_pkt_type;
      recvlen_S = glossy_get_payload_len();

      /* Rely only on S packets to synchronize and exit from the scanning loop */
      if (recvtype_S == WCB_TYPE_SYNC && 
          recvlen_S  == WCB_S_TOTAL_LEN) { // Synch packet received
        sink_id = glossy_get_initiator_id();
        epoch = buf.sync_hdr.epoch;
        wcb_info.epoch = epoch;
        if (IS_SYNCED()) {
          t_ref_corrected = glossy_get_t_ref();
          successful_scan = 1;
          break; // Exit the scanning loop
        }
        channel = get_channel_node_bootstrap(SCAN_RX_S);
        continue;
      }
      /* A non S packet was received */
      else if (recvtype_S == WCB_TYPE_EV || recvtype_S == WCB_TYPE_DATA ||\
               recvtype_S == WCB_TYPE_ACK || recvtype_S == WCB_TYPE_CONTROL)
      {
        /* Introduce some gaps in the scanning to reduce the probability to overhear again this 
           flood, which might still be propagated by other desynchronized nodes */
        WAIT_UNTIL(RTIMER_NOW() + WCB_SCAN_SLOT_DURATION, &pt_scan); 
        /*
        TODO: This scanning approach is still breakable if we are unlucky... Needs to be improve.
        */
        continue;
      }
    }
    /* No packet was received */
    channel = get_channel_node_bootstrap(SCAN_RX_NOTHING);

    scan_duration += WCB_SCAN_SLOT_DURATION;
    /* If the scanning procedure takes too long something wrong is probably happening ...
     * It is better to just exit the scanning phase */
    if (scan_duration > max_scan_duration) {
      successful_scan = 0;
      break; 
    }
  }
  PT_END(&pt_scan);
}

/* ---------------------------------------------- Synchronization Phase thread (node) ---------------------------------------------- */
PT_THREAD(s_node_thread(struct rtimer *t, void* ptr))
{
  static uint16_t ever_synced_with_s; // Synchronized at least once with an S packet
  PT_BEGIN(&pt_s_node);
  
  channel = get_channel_epoch(epoch);

  t_slot_start = t_s_start;
  t_slot_stop  = t_s_stop;

  GLOSSY(sink_id, 
    WCB_S_TOTAL_LEN,
    WCB_TYPE_SYNC, 
    conf.ntx_S,
    channel,
    GLOSSY_WITH_SYNC,
    &pt_s_node);
  GLOSSY_WAIT(&pt_s_node);

  UPDATE_SLOT_STATS(S, 0);

  rx_count_S = glossy_get_n_rx();
  tx_count_S = glossy_get_n_tx();

  correct_packet = 0;

  if (rx_count_S > 0) { /* A packet has been received, check if it is a correct / legitimate 
    `                    * S message */
    recvlen_S = glossy_get_payload_len();
    recvtype_S = recv_pkt_type;
    recvsrc_S = buf.sync_hdr.src;

    correct_packet = (recvtype_S == WCB_TYPE_SYNC && recvlen_S == WCB_S_TOTAL_LEN
      && recvsrc_S == sink_id);
    if (correct_packet) {
      epoch = buf.sync_hdr.epoch;
      wcb_info.epoch = epoch;
      hopcount = glossy_get_relay_cnt_first_rx();
    }
  }
  if (IS_SYNCED() && correct_packet && correct_hops()) {
    t_ref_corrected_s = glossy_get_t_ref();
    t_ref_corrected = t_ref_corrected_s;
    /* If we have already estimated the skew over a long period, we trust that skew value.
     * Otherwise, let's try to compute it */
    if (!long_skew_estimated && ever_synced_with_s) {
      /* Rarely, with the CC2538 implementation of Glossy a wrong reference time is retrieved 
       * when calling glossy_get_t_ref(). If this happens, we filter that reference time value 
       * and avoid computing the skew.
       * TBD: Further investigation would be worth. */
      period_skew_tmp = (int16_t)(t_ref_corrected_s - (t_ref_skewed + conf.period)) / ((int)sync_missed + 1);
      if (!sync_missed && (period_skew_tmp < -30 || period_skew_tmp > 30)) { /* The skew looks wrong, forget 
                                                                              * about the last acquired t_ref */
        t_ref_corrected = t_ref_estimated;
        t_ref_corrected_s = t_ref_estimated;
      }
      else { /* The skew looks legitimate or we might be desynchronized, so we trust the new period skew estimate */
        period_skew = period_skew_tmp;
        skew_estimated = 1;
      }
    }
    t_ref_skewed = t_ref_corrected_s;
    ever_synced_with_s = 1;
    sync_missed = 0;
  }
  else {
    sync_missed++;
    t_ref_skewed += conf.period;
    t_ref_corrected = t_ref_estimated;
    t_ref_corrected_s = t_ref_estimated;
  }

  wcb_info.t_ref = t_ref_corrected;

  app_post_S();
  BZERO_BUF();

  PT_END(&pt_s_node);
}

/* ---------------------------------------------- Event Phase thread (node) ---------------------------------------------- */
PT_THREAD(evphase_evonly_node_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_evphase_evonly_node);

  static int  guard;
  static bool have_event;
  static int  i_tx;

  have_event = app_pre_EV();
  i_tx = (have_event && 
            (sync_missed < N_SILENT_EPOCHS_TO_STOP_SENDING || n_noack_epochs < N_SILENT_EPOCHS_TO_STOP_SENDING));

  /* ---------- Event loop ---------- */
  while (n_ev < WCB_EV_SLOTS) {
    if (i_tx)
      guard = 0; // No guards when transmitting
    else 
      guard = (sync_missed && !synced_with_ack) ? WCB_SHORT_GUARD_NOSYNC : WCB_SHORT_GUARD; // Set a guard for receiving

    t_slot_start = t_ref_corrected + PHASE_EV_OFFS(n_ev) - WCB_REF_SHIFT - guard;
    t_slot_stop = t_slot_start + conf.w_EV + guard;

    channel = get_channel_epoch_ta(epoch, n_ev, 1); // Set the channel

    correct_packet = 0;
    corrupted_EV = 0; // Probably not needed

    GLOSSY(
      i_tx ? node_id : GLOSSY_UNKNOWN_INITIATOR, 
      WCB_EV_TOTAL_LEN, 
      WCB_TYPE_EV, 
      conf.ntx_EV,
      channel,
      GLOSSY_WITHOUT_SYNC,
      &pt_evphase_evonly_node);
    GLOSSY_WAIT(&pt_evphase_evonly_node);

    UPDATE_SLOT_STATS(EV, i_tx);

    rx_count_EV = glossy_get_n_rx();

    if (!i_tx) { /* If the node didn't TX, let's check if it received something */
      if (rx_count_EV) { // The node received something
        log_recv_type = recv_pkt_type;
        log_recv_length = glossy_get_payload_len();
        correct_packet = (log_recv_length == WCB_EV_TOTAL_LEN && log_recv_type == WCB_TYPE_EV);
      }
      corrupted_EV = glossy_is_corrupted();
    }

    app_post_EV(correct_packet, corrupted_EV, n_ev);
    BZERO_BUF();
    n_ev ++;
  } /* End of Event loop */
  PT_END(&pt_evphase_evonly_node);
}

/* ---------------------------------------------- Collection Phase thread (node) ---------------------------------------------- */
PT_THREAD(collection_node_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_collection_node);

  app_acquire_measurement();

  static int guard;
  static int i_tx;
  static uint16_t have_packet;

  /* ---------- Dedicated T slot sequence loop ----------*/
  while (n_t < NUM_SOURCES) {
    correct_packet = 0;
    payload = app_pre_Tded(n_t);
    have_packet = payload != NULL;

    i_tx = (have_packet && 
        (sync_missed < N_SILENT_EPOCHS_TO_STOP_SENDING || n_noack_epochs < N_SILENT_EPOCHS_TO_STOP_SENDING));

    if (i_tx) {
      n_t_tx ++;
      guard = 0; // No guards when transmitting
      memcpy(buf.raw + WCB_T_HDR_LEN, payload, conf.plds_T);
    }
    else { /* Non transmitting nodes */ 
      guard = (sync_missed && !synced_with_ack) ? WCB_SHORT_GUARD_NOSYNC : WCB_SHORT_GUARD; // Guards for receiving
    }

    t_slot_start = t_ref_corrected + PHASE_TDED_OFFS(n_t) - WCB_REF_SHIFT - guard;
    t_slot_stop = t_slot_start + conf.w_T + guard;

    channel = get_channel_epoch_ta(epoch, n_t, 2);

    GLOSSY(
      i_tx ? node_id : GLOSSY_UNKNOWN_INITIATOR, 
      WCB_T_TOTAL_LEN, 
      WCB_TYPE_DATA, 
      conf.ntx_T,
      channel,
      GLOSSY_WITHOUT_SYNC,
      &pt_collection_node);
    GLOSSY_WAIT(&pt_collection_node);

    UPDATE_SLOT_STATS(T, i_tx);

    rx_count_T = glossy_get_n_rx();
    if (!i_tx) {
      if (rx_count_T) { // Received data
        log_recv_type = recv_pkt_type;
        log_recv_length = glossy_get_payload_len();
        correct_packet = (log_recv_length == WCB_T_TOTAL_LEN && log_recv_type == WCB_TYPE_DATA);
      }
    }

    app_post_T(correct_packet, buf.raw + WCB_T_HDR_LEN);
    BZERO_BUF();

    n_t ++;
  } /* All sources had a chance to TX their data packet via a dedicated T slot within the current epoch */
  /* ---------- End of the dedicated T slot sequence loop ----------*/

  /* ---------- Cumulative acknowledgment slot ---------- */
  correct_packet = 0;
  guard = (sync_missed && !synced_with_ack) ? WCB_SHORT_GUARD_NOSYNC : WCB_SHORT_GUARD;

  t_slot_start = t_ref_corrected + PHASE_TDED_OFFS(n_t) - WCB_REF_SHIFT - guard;
  t_slot_stop = t_slot_start + conf.w_A + guard;

  channel = get_channel_epoch_ta(epoch, n_t, 2);

  GLOSSY(
    sink_id, 
    WCB_A_TOTAL_LEN,
    WCB_TYPE_ACK, 
    conf.ntx_A,
    channel,
    WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
    &pt_collection_node);
  GLOSSY_WAIT(&pt_collection_node);

  UPDATE_SLOT_STATS(A, 0);

  rx_count_A = glossy_get_n_rx();
  if (rx_count_A) { /* The node received something, let's check if it is a legitimate ACK message */
    if (glossy_get_payload_len() == WCB_A_TOTAL_LEN && recv_pkt_type == WCB_TYPE_ACK 
        && WCB_ACK_CMD_CORRECT(buf.ack_hdr)) { /* The received packet seems a legitimate ACK message */
      correct_packet = 1;
      n_all_acks ++;
      /* Update the epoch value in case we "skipped" some epochs (e.g., right after the initial scan phase) but got an ACK.
       * We can "skip" epochs if we are too late for the next T/A/EV/CTRL and set the timer to the past */
      epoch = buf.ack_hdr.epoch; 
      wcb_info.epoch = epoch;

#if (WCB_SYNC_ACKS) /* ACK messages are exploited to (re)synchronize the network */
      if (IS_SYNCED() && correct_ack_skew(N_CACK_TO_REF(glossy_get_t_ref(), n_t))) {
        t_ref_corrected = N_CACK_TO_REF(glossy_get_t_ref(), n_t);
        synced_with_ack ++;
        n_noack_epochs = 0; /* NB: It's important to reset it here to re-enable source nodes to TX data packets right away (if this was suppressed) */
      }
#endif

      /* Check the ACK message to see if the Collection phase was successful */
      if (WCB_ACK_ALL_READINGS_RECEIVED(buf.ack_hdr))
        all_states_rcvd = 1; /* The controller received all readings. This is a key information to decide what to do next. */
    }
  }
  /* ---------- End of the cumulative ACK slot ---------- */

  app_post_A(correct_packet, buf.raw + WCB_A_HDR_LEN);
  BZERO_BUF();

  PT_END(&pt_collection_node);
}

/* ---------------------------------------------- Recovery Phase thread (node) ---------------------------------------------- */
PT_THREAD(recovery_node_thread(struct rtimer *t, void* ptr))
{
  PT_BEGIN(&pt_recovery_node);

  /* ---------- TA chain ---------- */
  while (1) { 
    /* ---------- (Shared) T slot ---------- */
    static int guard;
    static uint16_t have_packet;
    static int i_tx;

    correct_packet = 0;
    payload = app_pre_T();
    have_packet = payload != NULL;

    i_tx = (have_packet && 
        (sync_missed < N_SILENT_EPOCHS_TO_STOP_SENDING || n_noack_epochs < N_SILENT_EPOCHS_TO_STOP_SENDING));

    if (i_tx) {
      n_t_tx ++;
      guard = 0;
      memcpy(buf.raw + WCB_T_HDR_LEN, payload, conf.plds_T);
    }
    else { /* Non transmitting nodes */
      guard = (sync_missed && !synced_with_ack) ? WCB_SHORT_GUARD_NOSYNC : WCB_SHORT_GUARD;
    }

    t_slot_start = t_ref_corrected + PHASE_T_OFFS(n_ta) - WCB_REF_SHIFT - guard;
    t_slot_stop = t_slot_start + conf.w_T + guard;

    channel = get_channel_epoch_ta(epoch, n_ta, 3);

    GLOSSY(
      i_tx ? node_id : GLOSSY_UNKNOWN_INITIATOR, 
      WCB_T_TOTAL_LEN, 
      WCB_TYPE_DATA, 
      conf.ntx_T,
      channel,
      GLOSSY_WITHOUT_SYNC,
      &pt_recovery_node);
    GLOSSY_WAIT(&pt_recovery_node);

    UPDATE_SLOT_STATS(T, i_tx);

    rx_count_T = glossy_get_n_rx();
    if (!i_tx) {
      if (rx_count_T) { // The node received something
        log_recv_type = recv_pkt_type;
        log_recv_length = glossy_get_payload_len();
        correct_packet = (log_recv_length == WCB_T_TOTAL_LEN && log_recv_type == WCB_TYPE_DATA);
        log_ta_status = correct_packet ? WCB_RECV_OK : WCB_BAD_DATA;
        n_empty_ts = 0;
      }
      else if (glossy_is_corrupted()) {
        log_ta_status = WCB_BAD_CRC;
        n_empty_ts = 0;
      }
      else {
        log_ta_status = WCB_SILENCE;
        n_empty_ts ++;
      }
    }

    app_post_T(correct_packet, buf.raw + WCB_T_HDR_LEN);
    BZERO_BUF();
    /* ---------- End of the (shared) T slot ---------- */

    /* ---------- ACK slot ---------- */
    correct_packet = 0;
    guard = (sync_missed && !synced_with_ack) ? WCB_SHORT_GUARD_NOSYNC : WCB_SHORT_GUARD;
    t_slot_start = t_ref_corrected + PHASE_A_OFFS(n_ta) - WCB_REF_SHIFT - guard;
    t_slot_stop = t_slot_start + conf.w_A + guard;

    GLOSSY(
      sink_id, 
      WCB_A_TOTAL_LEN,
      WCB_TYPE_ACK, 
      conf.ntx_A,
      channel,
      WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
      &pt_recovery_node);
    GLOSSY_WAIT(&pt_recovery_node);

    UPDATE_SLOT_STATS(A, 0);

    rx_count_A = glossy_get_n_rx();
    if (rx_count_A) { /* The node received something */
      if (glossy_get_payload_len() == WCB_A_TOTAL_LEN 
          && recv_pkt_type == WCB_TYPE_ACK 
          && WCB_ACK_CMD_CORRECT(buf.ack_hdr)) { /* The received packet is a legitimate A packet */
        correct_packet = 1;
        n_noacks = 0;
        n_bad_acks = 0;
        n_all_acks ++;
        /* Update the epoch value in case we "skipped" some epochs (e.g., right after the initial scan phase) but got an ACK.
         * We can "skip" epochs if we are too late for the next T/A/EV/CTRL and set the timer to the past */
        epoch = buf.ack_hdr.epoch; 
        wcb_info.epoch = epoch;

#if (WCB_SYNC_ACKS) /* ACK messages are exploited to (re)synchronize the network */
        if (IS_SYNCED() && correct_ack_skew(N_TA_TO_REF(glossy_get_t_ref(), n_ta))) {
          t_ref_corrected = N_TA_TO_REF(glossy_get_t_ref(), n_ta);
          synced_with_ack ++;
          n_noack_epochs = 0; /* NB: It's important to reset it here to re-enable source nodes to TX data packets right away (if this was suppressed) */
        }
#endif  
        /* Check the ACK message to see if the controller managed to collect all readings */
        if (WCB_ACK_ALL_READINGS_RECEIVED(buf.ack_hdr))
          all_states_rcvd = 1; /* The controller received all source readings! Nodes can immediately exit from the Recovery Phase and enter sleep */
        else {
          if (WCB_ACK_SLEEP_UNTIL_CTRLPHASE(buf.ack_hdr)) 
            sleep_order = 1; /* Some source readings are still missing but there is not enough time for an additional TA pair,
                              * nodes need to immediately exit from the Recovery Phase */
        }
      }
      else { /* The node received something but not an ACK (we might be out of sync) */
        n_bad_acks ++;
        /* Logging info about bad packets */
        if (recv_pkt_type != WCB_TYPE_ACK)
          n_badtype_A ++;
        if (glossy_get_payload_len() != WCB_A_TOTAL_LEN)
          n_badlen_A ++;
      }
    }
    else if (glossy_is_corrupted()) { /* The node received a corrupted packet */ 
      n_bad_acks ++;
      n_badcrc_A ++;
    }
    else /* The node didn't received anything */
      n_noacks ++;

    app_post_A(correct_packet, buf.raw + WCB_A_HDR_LEN);

    BZERO_BUF();
    /* ---------- End of A slot ---------- */
    n_ta ++;
    
    /* Decide whether to remain in the recovery phase or exit */
    if (all_states_rcvd || sleep_order || (n_ta >= WCB_MAX_TAS_RECV) ||
         (( have_packet  && (n_noacks >= conf.z)) ||
          ((!have_packet) && (n_noacks >= conf.y) && n_empty_ts >= conf.y)
         )
       ) { 
      break; /* Time to exit for the Recovery phase */
    }
  } 
  /* ---------- End of the TA chain ---------- */
  PT_END(&pt_recovery_node);
}


/* ---------------------------------------------- Control Dissemination Phase thread (node) ---------------------------------------------- */
PT_THREAD(ctrl_node_thread(struct rtimer *t, void* ptr)) 
{
  PT_BEGIN(&pt_ctrl_node);

  /* ---------- Control loop ---------- */
  while (n_ctrls < WCB_CONTROL_SLOTS) {
    static int guard;
    correct_packet = 0;
    uint8_t complete_reception_flag = 0;
    
    /* Guards for receiving */
    guard = ((!long_skew_estimated && !skew_estimated) || 
            (sync_missed >= N_MISSED_FOR_INIT_GUARD && !synced_with_ack)) ? WCB_CTRL_GUARD : WCB_SHORT_GUARD;

    t_slot_start = t_ref_corrected + CTRL_PHASE_OFFS(n_ctrls) - WCB_REF_SHIFT - guard;
    t_slot_stop = t_slot_start + conf.w_CTRL + guard;

    channel = get_channel_epoch_ta(epoch, n_ctrls, 4);

    GLOSSY(
      sink_id, 
      WCB_CTRL_TOTAL_LEN, 
      WCB_TYPE_CONTROL, 
      conf.ntx_CTRL,
      channel,
      WCB_SYNC_ACKS ? GLOSSY_WITH_SYNC : GLOSSY_WITHOUT_SYNC,
      &pt_ctrl_node);
    GLOSSY_WAIT(&pt_ctrl_node);

    UPDATE_SLOT_STATS(CTRL, 0);

    rx_count_CTRL = glossy_get_n_rx();
    if (rx_count_CTRL) { /* The node received something, check if it is legitimate CTRL packet */
      if (glossy_get_payload_len() == WCB_CTRL_TOTAL_LEN &&
          MASK_RECEPTION_FLAG(recv_pkt_type) == WCB_TYPE_CONTROL) { /* Before to check the packet type mask the MSB of the 
                                                                     * control type as it is used by the controller
                                                                     * to notify the reception of all measurements */
        correct_packet = 1;
        n_all_acks ++;
        if (CHECK_RECEPTION_FLAG(recv_pkt_type))
          complete_reception_flag = 1;
        else
          complete_reception_flag = 0;
        /* Update the epoch value in case we "skipped" some epochs (e.g., right after the initial scan phase) but got an ACK.
         * We can "skip" epochs if we are too late for the next T/A/EV/CTRL and set the timer to the past */
        epoch = buf.ctrl_hdr.epoch;
        wcb_info.epoch = epoch;

#if (WCB_SYNC_ACKS)
        if (IS_SYNCED() && correct_ack_skew(CTRL_TO_REF(glossy_get_t_ref(), n_ctrls))) {
          t_ref_corrected = CTRL_TO_REF(glossy_get_t_ref(), n_ctrls);
          synced_with_ack ++;
          n_noack_epochs = 0; /* NB: It's important to reset it here to re-enable source nodes to TX data packets right away (if this was suppressed) */
        }
#endif
      }
    }

    app_post_CTRL(correct_packet, buf.raw + WCB_CTRL_HDR_LEN, complete_reception_flag);
    BZERO_BUF();
    n_ctrls++;
  }
  /* ---------- End of the Control loop ---------- */
  PT_END(&pt_ctrl_node);
}


/* ---------------------------------------------- Main thread (node) ---------------------------------------------- */
static char node_main_thread(struct rtimer *t, void *ptr) {
  static rtimer_clock_t s_guard;
  PT_BEGIN(&pt);

  successful_scan = 0;
  PT_SPAWN(&pt, &pt_scan, scan_thread(t, ptr));

  if (!successful_scan) {
    printf("[ERROR] EPOCH 0 - The scan procedure is taking too long! "
      "The node is currently unable to join the network!\n");
    PT_EXIT(&pt);
  }

  wcb_info.t_ref = t_ref_corrected;

  BZERO_BUF();

#if DEP_BOOTSTRAP
  if (recvtype_S == WCB_TYPE_SYNC && recvlen_S == WCB_S_TOTAL_LEN && epoch < fast_bstrap_epochs) {
    static rtimer_clock_t time1, time2; // Timestamps of the first and last S packet received
    static wcb_epoch_t epoch1, epoch2;  // Epoch numbers of the first and last S packet received

    time1 = t_ref_corrected;
    time2 = t_ref_corrected;
    epoch1 = epoch;
    epoch2 = epoch;

    t_s_start = t_ref_corrected + BS_PERIOD - WCB_SHORT_GUARD - WCB_REF_SHIFT;
    t_s_stop = t_s_start + conf.w_S + WCB_SHORT_GUARD;

    while (epoch < fast_bstrap_epochs) {
      t_slot_start = t_s_start;
      t_slot_stop = t_s_stop;

      GLOSSY(
        sink_id, 
        WCB_S_TOTAL_LEN,
        WCB_TYPE_SYNC, 
        BS_N,
        WCB_DEF_CHANNEL,
        GLOSSY_WITH_SYNC,
        &pt);
      GLOSSY_WAIT(&pt);

      recvtype_S = recv_pkt_type;
      recvlen_S = glossy_get_payload_len();

      /* Check if a new S message has been received (just to be sure, during the dep_bootstrap phase
       * S packets only should circulate in the network). If yes and it provides synchronization,
       * update the stored info (time2, epoch2) */
      if (recvtype_S == WCB_TYPE_SYNC && recvlen_S == WCB_S_TOTAL_LEN && IS_SYNCED()) {
        t_ref_corrected = glossy_get_t_ref();
        time2 = t_ref_corrected;
        epoch = buf.sync_hdr.epoch;
        epoch2 = epoch;
      }
      else { /* Nothing good has been received */
        t_ref_corrected += BS_PERIOD;
        epoch ++;
      }

      t_s_start = t_ref_corrected + BS_PERIOD - WCB_SHORT_GUARD - WCB_REF_SHIFT;
      t_s_stop = t_s_start + conf.w_S + WCB_SHORT_GUARD;
    }

    if (epoch2 - epoch1 > BS_EPOCHS_TO_STABLE_CLOCK) { /* Enough time has elapsed to compute a (hopefully)
                                                        * stable clock drift */ 
      int16_t  delta;
      int32_t  scaled_delta;
      uint32_t expected_time; 

      expected_time = (uint32_t)(BS_PERIOD) * (epoch2 - epoch1);
      delta = time2 - (time1 + (uint16_t)(expected_time));
      scaled_delta = ((int32_t)delta * (uint32_t)conf.period);
      period_skew = scaled_delta / (int32_t)expected_time;
      long_skew_estimated = 1;

      /* Compensate the drift accumulated during the bootstrap itself */
      t_ref_corrected += ((uint32_t)(BS_PERIOD) * (fast_bstrap_epochs - epoch2) / conf.period) * period_skew;
    }
  }
#endif /* End of DEP_BOOTSTRAP */

  /* Here we have the ref time pointing at the previous epoch */
  t_ref_corrected_s = t_ref_corrected;
  wcb_info.epoch = epoch;

  /* For S if we are not skipping it */
  if (long_skew_estimated == 1) { /* We have already estimated the clock skew over a
                                   * long period of time: we can rely on this value */
    t_ref_estimated = t_ref_corrected + conf.period + period_skew;
    t_ref_skewed = t_ref_estimated;
    t_s_start = t_ref_estimated - WCB_REF_SHIFT - 2*WCB_SHORT_GUARD_NOSYNC;
    t_s_stop = t_s_start + conf.w_S + 4*WCB_SHORT_GUARD_NOSYNC; 
  }
  else { /* The clock skew has not been estimated yet; let's use a "relatively big" guard */
    t_ref_estimated = t_ref_corrected + conf.period;
    t_ref_skewed = t_ref_estimated;
    t_s_start = t_ref_estimated - WCB_REF_SHIFT - WCB_INIT_GUARD;
    t_s_stop = t_s_start + conf.w_S + 2*WCB_INIT_GUARD; 
  }

  t_wakeup = t_s_start - WCB_INTER_SLOT_GAP;
  WAIT_UNTIL(t_wakeup - WCB_APP_PRE_EPOCH_CB_TIME, &pt);

  app_pre_epoch(); // Let nodes prepare for the next epoch 
  WAIT_UNTIL(t_wakeup, &pt);
  
  /* Check if updated readings have been received from Simulink; 
   * if not, rise an error and exit the protothread */
  if (!app_have_new_readings()) {
    printf("[ERROR] EPOCH %d - NO DATA AVAILABLE\n", epoch);
    PT_EXIT(&pt);
  }

  /* ---------- Infinite epoch loop ---------- */
  while (1) {
    init_epoch_state(); /* Zero out epoch-related variables */

    epoch ++;
    wcb_info.epoch = epoch;

    WAIT_UNTIL(t_s_start - PRE_TIME, &pt);

    PT_SPAWN(&pt, &pt_s_node, s_node_thread(t, ptr)); /* Synchronization thread */
  
    PT_SPAWN(&pt, &pt_evphase_evonly_node, evphase_evonly_node_thread(t, ptr)); /* Event thread */

    if (app_enter_collection()) { /* An event has been detected, enter in the Collection phase */
      
      enter_coll_phase = 1;
      PT_SPAWN(&pt, &pt_collection_node, collection_node_thread(t, ptr)); /* Collection thread */

#if RECOVERY_ON /* If the recovery feature is enabled, check whether a recovery action is needed  */
      if (!all_states_rcvd) { /* Some messages got lost, enter the recovery phase */
        enter_recv_phase = 1;
        PT_SPAWN(&pt, &pt_recovery_node, recovery_node_thread(t, ptr)); /* Recovery thread */
      }
#endif

      enter_control_phase = 1;
      WAIT_UNTIL(t_ref_corrected + CTRL_START_OFFS - WCB_INTER_SLOT_GAP, &pt);
      PT_SPAWN(&pt, &pt_ctrl_node, ctrl_node_thread(t, ptr)); /* Control Dissemination thread */
    }

    /* ---------- End of the active part of an epoch ---------- */

    if (!synced_with_ack && app_enter_collection()) {
      n_noack_epochs ++;
    }

    /* Define a reasonable guard for the next synchronization phase (next epoch) */
    s_guard = ((!long_skew_estimated && !skew_estimated) || sync_missed >= N_MISSED_FOR_INIT_GUARD) ? WCB_INIT_GUARD : WCB_SHORT_GUARD;

    /* Schedule the start and stop times for the next S Glossy flood */
    t_ref_estimated = t_ref_corrected_s + conf.period + period_skew;
    t_s_start = t_ref_estimated - WCB_REF_SHIFT - s_guard;
    t_s_stop = t_s_start + conf.w_S + 2*s_guard;

    /* Schedule the nodes wake up time (nodes need to wake up a bit in advance to prepare for the next epoch) */
    t_wakeup = t_s_start - WCB_INTER_SLOT_GAP;

    app_epoch_end(); /* The active part of the epoch ended, it is a good time to print per-node epoch related stats */
    WAIT_UNTIL(t_wakeup - WCB_APP_PRE_EPOCH_CB_TIME, &pt);

    app_pre_epoch(); /* Sources wake up a bit in advance to receive the new sensor readings from Simulink */
    WAIT_UNTIL(t_wakeup, &pt);

    /* If something went wrong and no sensor reading is available rise an error and exit the protothread */
    if (!app_have_new_readings()) {
      printf("[ERROR] EPOCH: %d - NO DATA AVAILABLE\n", epoch + 1);
      PT_EXIT(&pt);
    }
  }
  PT_END(&pt);
}

/* ---------- End of protocol logic ----------  */
void wcb_init() {
  glossy_init();
  cc2538_rf_set_channel(WCB_DEF_CHANNEL);
}


bool wcb_start(wcb_config_t* conf_)
{
  printf("WCB S PACKET SIZE: %u\n", (WCB_S_HDR_LEN + conf_->plds_S));
  printf("WCB T PACKET SIZE: %u\n", (WCB_T_HDR_LEN + conf_->plds_T));
  printf("WCB A PACKET SIZE: %u\n", (WCB_A_HDR_LEN + conf_->plds_A));
  printf("WCB EV PACKET SIZE: %u\n", (WCB_EV_HDR_LEN + conf_->plds_EV));
  printf("WCB CTRL PACKET SIZE: %u\n", (WCB_CTRL_HDR_LEN + conf_->plds_CTRL));
  
  /* Check the current configuration */
  if (WCB_S_HDR_LEN + conf_->plds_S > WCB_PKTBUF_LEN       ||
      WCB_T_HDR_LEN + conf_->plds_T > WCB_PKTBUF_LEN       ||
      WCB_A_HDR_LEN + conf_->plds_A > WCB_PKTBUF_LEN       ||
      WCB_EV_HDR_LEN + conf_->plds_EV > WCB_PKTBUF_LEN     ||
      WCB_CTRL_HDR_LEN + conf_->plds_CTRL > WCB_PKTBUF_LEN ||
      conf_->period == 0                                   ||
      conf_->scan_duration == 0                            ||
      conf_->scan_duration > WCB_MAX_SCAN_EPOCHS
     )
    return false;
  
  conf = *conf_;

  /* In case we start after being stopped, zero-out WCB state */
  bzero(&wcb_info, sizeof(wcb_info));
  epoch = 0;
  skew_estimated = 0;
  synced_with_ack = 0;
  n_noack_epochs = 0;
  sync_missed = 0;
  period_skew = 0;

  if (conf.is_sink)
    timer_handler = root_main_thread;
  else
    timer_handler = node_main_thread;

#if DEP_BOOTSTRAP
  fast_bstrap_epochs = FAST_BS_DURATION/(BS_PERIOD);
  printf("Fast bootstrap epochs: %d\n", fast_bstrap_epochs);
#endif

  /* Set the working channel and TX power */
  channel = WCB_DEF_CHANNEL;
  tx_power = cc2538_rf_get_tx_power();
  printf("Default Tx Power: %d\n", tx_power);
  cc2538_rf_set_tx_power(TX_POWER);
  tx_power = cc2538_rf_get_tx_power();
  printf("Set Tx Power: %d\n", tx_power);

  /* Start WCB */
  rtimer_set(&rt, RTIMER_NOW() + 10, 0, timer_handler, NULL);
  return true;
}


/* ---------- Log output ---------- */
void wcb_print_epoch_logs() {
  static int first_time = 1;
  unsigned long radio_on_time;

#if WCB_LOGLEVEL
  if (!conf.is_sink) {
    printf("S %u:%u %u %u:%u %u %d %u\n", epoch, n_t_tx, n_all_acks, synced_with_ack, sync_missed, n_noack_epochs, period_skew, hopcount);
    printf("P %u:%u %u %u:%u %u %u %d\n", 
        epoch, recvsrc_S, recvtype_S, recvlen_S, n_badtype_A, n_badlen_A, n_badcrc_A, log_ack_skew_err);
  }
  printf("R %u:%u %u\n", epoch, tx_count_S, rx_count_S);
  printf("M %u:%u %u:%u %u %u\n", epoch, enter_coll_phase, enter_recv_phase, n_ta, enter_control_phase, sleep_order);
#endif

  glossy_debug_print();

#if ENERGEST_CONF_ON && WCB_LOGLEVEL
  /* REMINDER: the first radio-on time printed is the one of the SECOND epoch after the bootstrap!
   * This needs to be taken into account when computing the DC */
  if (!first_time) {
    /* Compute average radio-on time per epoch */
    radio_on_time = energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT);
    /* Print radio on time in ticks */
    printf("E %u:%lu %lu %lu %lu %lu %lu\n", epoch, radio_on_time, ton_S,
        ton_EV, ton_T, ton_A, ton_CTRL);
    printf("F %u:%lu %lu %lu %lu %lu:%u %u %u %u %u\n", epoch, tf_S, tf_EV,
        tf_T, tf_A, tf_CTRL, n_short_S, n_short_EV, n_short_T, n_short_A, n_short_CTRL);
  }
  energest_init(); /* Initialize Energest values */ 
  first_time = 0;
#endif /* ENERGEST_CONF_ON && WCB_LOGLEVEL */
}


wcb_config_t wcb_get_config() {
  return conf;
}
