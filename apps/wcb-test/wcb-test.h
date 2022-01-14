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

#ifndef WCB_TEST_H
#define WCB_TEST_H

#include "wcb.h"
#include "wcb-conf.h"

#define CONTROL_PAYLOAD_SIZE NUM_ACTUATORS

/* NB: Synchronization and event packets have NO (application) payload! */

typedef struct {
  uint8_t src; 
  int16_t measurement;
  int16_t integral_hgt;
} 
__attribute__((packed))
app_t_payload;


typedef struct {
  uint16_t ack_bitmap; /* The packets collected at the controller are ACKed by means of a 1-bit per measurement bitmap */
} 
__attribute__((packed))
app_a_payload;


typedef struct {
  int16_t upd_ctrl[CONTROL_PAYLOAD_SIZE];
} 
__attribute__((packed))
app_control_payload;

#endif
