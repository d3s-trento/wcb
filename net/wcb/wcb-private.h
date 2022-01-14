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

#ifndef WCB_PRIVATE_H
#define WCB_PRIVATE_H

#include "wcb.h"
#include "wcb-conf.h"

#define WCB_INTER_SLOT_GAP (RTIMER_SECOND / 500) // 2 ms [TBD: It could be optimised!]

#define WCB_SHORT_GUARD           5 // Inherited from Crystal (~150 us)
#define WCB_SHORT_GUARD_NOSYNC    6

#define WCB_SINK_END_GUARD  8       // End guard for non sink-initiated floods (give a bit more time to receive packets)

#define WCB_REF_SHIFT 17            // Measured with oscilloscope for Firefly

/* Guard-time when the clock skew is not yet estimated */
#define WCB_INIT_GUARD  (RTIMER_SECOND / 100) // 10 ms

/* Slot duration during bootstrap at non-controller nodes */
#define WCB_SCAN_SLOT_DURATION    (RTIMER_SECOND / 20) // 50 ms

/* Number of S packets missed in a row to enlarge the reception guard */
#define N_MISSED_FOR_INIT_GUARD 3

#define N_SILENT_EPOCHS_TO_RESET 100
#define N_SILENT_EPOCHS_TO_STOP_SENDING 3

#if ON_BOARD 
  #define WCB_CTRL_GUARD (RTIMER_SECOND / 2000)
  #define CTRL_COMPUTATION_GUARD 0 /* The controller can exploit the INTER_SLOT_GAP to carry out its computations.
                                    * If this gap is not enough, an additional guard (e.g., (RTIMER_SECOND / 1000)) can be used */
#else
  #define WCB_CTRL_GUARD (RTIMER_SECOND / 1000) 
  #define CTRL_COMPUTATION_GUARD (RTIMER_SECOND / 20) /* For sure it can be reduced when the Recovery phase is enabled; 
                                                       * anyway in app_post_CTRL() we compensate for this fake delay 
                                                       * caused by the need to interact with the testbed server */
#endif


/* ---------- WCB packets headers ---------- */
/* NB: Event and data packets have no header (just the type field, added in wcb.c) */ 

typedef struct {
  wcb_addr_t src;
  wcb_epoch_t epoch;
} 
__attribute__((packed, aligned(1)))
wcb_sync_hdr_t;


typedef struct {
  wcb_epoch_t epoch;
  uint8_t cmd; /* [Possible optimization]
                * This field is NOT strictly needed; it could be removed to make the ACK header shorter. 
                * Nodes can indeed decide which action to perform next by looking at the ACK bitmap. */
}
__attribute__((packed, aligned(1)))
wcb_ack_hdr_t;


typedef struct {
  wcb_epoch_t epoch; /* Only needed to exploit control packets to synchronize; if not it can be removed */
}
__attribute__((packed, aligned(1)))
wcb_control_hdr_t;
/* ---------------------------------------- */


/* ---------- WCB packets types ---------- */
#define WCB_TYPE_SYNC 	   0x01
#define WCB_TYPE_DATA	     0x02
#define WCB_TYPE_ACK 	     0x03
#define WCB_TYPE_EV   	   0x04
#define WCB_TYPE_CONTROL   0x05
/* ---------------------------------------- */


/* ---------- WCB commands ---------- */
/* [Possible optimization]:
 * When an event is detected ALL nodes need to participate in the CTRL Dissemination phase.
 * This makes the schedule very deterministic; in turn, it is now possible to get rid of the following commands 
 * exploited by the controller in the Collection / Recovery phases, sparing one byte. Nodes can indeed decide 
 * whether to enter (and/or remain) in the Recovery phase by simply checking the ACK bitmap. 
 */

#define WCB_ACK_AWAKE(ack) ((ack).cmd == 0x11)
#define WCB_ACK_SLEEP_UNTIL_CTRLPHASE(ack) ((ack).cmd == 0x22)
#define WCB_ACK_ALL_READINGS_RECEIVED(ack) ((ack).cmd == 0x33)

#define WCB_SET_ACK_AWAKE(ack) ((ack).cmd = 0x11)
#define WCB_SET_ACK_SLEEP_UNTIL_CTRLPHASE(ack) ((ack).cmd = 0x22)
#define WCB_SET_ACK_ALL_READINGS_RECEIVED(ack) ((ack).cmd = 0x33)

#define WCB_ACK_CMD_CORRECT(ack) (WCB_ACK_AWAKE(ack) || WCB_ACK_SLEEP_UNTIL_CTRLPHASE(ack) || \
 WCB_ACK_ALL_READINGS_RECEIVED(ack))

/* Use the most significant bit of the CTRL TYPE to inform the network whether
 * the controller successfully received all readings or not.
 */
#define SET_RECEPTION_FLAG(var) ((var) | (1<<(7)))
#define CHECK_RECEPTION_FLAG(var) ((var) & (1<<(7)))
#define MASK 0x7f
#define MASK_RECEPTION_FLAG(var) ((var) & (MASK))
/* ---------------------------------------- */


/* Store some potentially interesting information about the Recovery Phase */
#define WCB_RECV_OK    0
#define WCB_BAD_DATA   1
#define WCB_BAD_CRC    2
#define WCB_SILENCE    3


/* Platform adaptation stubs */
#define GLOSSY_IGNORE_TYPE 0  // This version of glossy does not carry the packet type

#endif //WCB_PRIVATE_H
