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

#define CHMAP_nohop        1
#define CHMAP_nomap        2
#define CHMAP_testchop     3
#define CHMAP_no11         4
#define CHMAP_blacklist    5
#define CHMAP_tmp32        6

#define BSTRAP_nohop       1
#define BSTRAP_hop3        3

#if WCB_CHHOP_MAPPING == CHMAP_nohop
int channel_array[16] = {[0 ... 15] = WCB_DEF_CHANNEL}; /* No channel hopping */
#elif WCB_CHHOP_MAPPING == CHMAP_nomap 
int channel_array[16] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26}; /* No mapping */
#elif WCB_CHHOP_MAPPING == CHMAP_testchop
int channel_array[16] = {25, 25, 15, 25, 25, 15, 25, 26, 25, 26, 15, 26, 26, 26, 26, 15}; /* CHMAP_test */
#elif WCB_CHHOP_MAPPING == CHMAP_no11
int channel_array[16] = {26, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26}; /* Remove channel 11 as highly jammed (could bias results) */
#elif WCB_CHHOP_MAPPING == CHMAP_blacklist
int channel_array[16] = {15, 20, 25, 26, 15, 20, 25, 26, 15, 20, 25, 26, 15, 20, 25, 26}; /* Consider only low interfered channels 15, 20, 25 and 26 */
#elif WCB_CHHOP_MAPPING == CHMAP_tmp32
int channel_array[] = {110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 
  160, 165, 170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 
  230, 235, 240, 245, 250, 255, 260};
#else
#error "Unsupported channel mapping name"
#endif

static inline int get_num_channels() {
  return (sizeof(channel_array) / sizeof(channel_array[0]));
}

static inline int get_channel_epoch(wcb_epoch_t epoch) {
  return channel_array[(epoch * 7) % get_num_channels()];
}

static inline int get_channel_epoch_ta(wcb_epoch_t epoch, uint8_t ta, uint8_t phase) {
  if (phase == 1 || phase == 2) { // Event or Collection Phases
    return channel_array[((7 * epoch + ta + 1) * 7) % get_num_channels()];
  }
  else {
    if (phase == 3) { // Recovery Phase
      return channel_array[((7 * epoch + ta + NUM_SOURCES + 1) * 7) % get_num_channels()];
    }
    else { // Control Dissemination Phase
      return channel_array[((7 * epoch + ta + NUM_SOURCES + 2) * 7) % get_num_channels()];
    }
  }

}

enum scan_rx {SCAN_RX_NOTHING, SCAN_RX_S, SCAN_RX_A};

#if WCB_BSTRAP_CHHOPPING == BSTRAP_nohop
static inline int get_channel_node_bootstrap() {
  return WCB_DEF_CHANNEL;
}
#elif WCB_BSTRAP_CHHOPPING == BSTRAP_hop3

enum scan_state {SCAN_ST_CHASE_S, SCAN_ST_SEARCH} scan_state;

static uint32_t        scan_last_hop_ago;
static rtimer_clock_t  scan_last_check_time;
static wcb_epoch_t     scan_last_epoch;
static uint8_t         scan_epoch_misses;
static uint8_t         scan_last_ch_index;

#define SCAN_MAX_S_MISSES    5
#define SCAN_START_CHASING_S 0

static inline int get_channel_node_bootstrap(enum scan_rx rx) {
  static int first_time = 1;

  if (first_time) { // First time we are scanning during the bootstrap
    first_time = 0;
    scan_last_check_time = RTIMER_NOW();
#if SCAN_START_CHASING_S
    /* The sink starts with epoch 1, if all nodes start roughly at the same time, 
     * they will use the same channel. */
    scan_last_epoch = 1;
    scan_state = SCAN_ST_CHASE_S;
    /* We might be late for the first S, try it but change to the next one in 
    half-epoch time */
    scan_last_hop_ago = conf.period / 2;
    return get_channel_epoch(scan_last_epoch);
#else // !SCAN_START_CHASING_S
    scan_last_ch_index = 0;
    scan_state = SCAN_ST_SEARCH;
    scan_last_hop_ago = 0;
    return channel_array[scan_last_ch_index];
#endif // SCAN_START_CHASING_S
  }

  scan_last_hop_ago += RTIMER_NOW() - scan_last_check_time;
  scan_last_check_time = RTIMER_NOW();

  if (rx == SCAN_RX_S) { // Caught an S, try to catch the next one
    scan_state = SCAN_ST_CHASE_S;
    scan_epoch_misses = 0;
    scan_last_hop_ago = conf.period / 2;
    scan_last_epoch = epoch; // The current one but will change in half an epoch
    return get_channel_epoch(epoch);
  }
  if (rx == SCAN_RX_A) { // Caught an A
    scan_state = SCAN_ST_CHASE_S;
    scan_epoch_misses = 0;
    if (n_ta < WCB_MAX_TAS_RECV - 1) { // Enough time to catch the next S
      scan_last_hop_ago = 0;
      scan_last_epoch = epoch + 1; // The next one
      return get_channel_epoch(scan_last_epoch);
    }
    else { // Might be late for the next S, try it but change to +2 in half-epoch time
      scan_last_hop_ago = conf.period / 2;
      scan_last_epoch = epoch + 1; // The next one
      return get_channel_epoch(epoch);
    }
  }
  else { // Not received anything
    if (scan_state == SCAN_ST_CHASE_S) {
      if (scan_epoch_misses > SCAN_MAX_S_MISSES) {
        scan_state = SCAN_ST_SEARCH;
        scan_last_ch_index = 0;
        scan_last_hop_ago = 0;
        return channel_array[scan_last_ch_index];
      }
      // Is it time to change the epoch?
      if (scan_last_hop_ago > conf.period) {
        scan_epoch_misses ++;
        scan_last_hop_ago = scan_last_hop_ago % (uint16_t)conf.period;
        scan_last_epoch ++;
      }
      return get_channel_epoch(scan_last_epoch);
    }
    else { /* Just searching
            * Is it time to change the channel? */
      if (scan_last_hop_ago > conf.period*1.3) {
        scan_last_hop_ago = 0;
        scan_last_ch_index = (scan_last_ch_index + 1) % 16;
      }
      return channel_array[scan_last_ch_index];
    }
  }
}
#else
#error "Unsupported bootstrap channel hopping setting"
#endif // WCB_BSTRAP_CHHOPPING
