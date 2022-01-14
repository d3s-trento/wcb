#!/usr/bin/env python3

# Authors: Matteo Trobinger <matteo.trobinger@gmail.com>
#          Timofei Istomin <tim.ist@gmail.com>

import sys
import re
import argparse

ap = argparse.ArgumentParser(description='WCB log parser')
ap.add_argument('--input', required=False, default=("test.log",),
                nargs="+",
                help='File(s) to parse')
ap.add_argument('--format', required=False, default="unitn",
                help='File to parse')

args = ap.parse_args()

input_files = args.input
record_format = args.format

record_pattern = {
    "unitn": "^\[(?P<time>.{23})\] INFO:firefly\.\d+: (?P<node_id>\d+)\.firefly < b'%s'$"
}.get(record_format, None)


if record_pattern is None:
    sys.stderr.write("Unknown record format: %s\n" % record_format)
    sys.exit(1)


S = re.compile(record_pattern %
    "S (?P<epoch>\d+):(?P<n_tx>\d+) (?P<n_acks>\d+) (?P<sync_acks>\d+):(?P<sync_missed>\d+) (?P<n_noack_epochs>\d+) (?P<skew>-?\d+) (?P<hops>\d+)")
P = re.compile(record_pattern %
    "P (?P<epoch>\d+):(?P<recvsrc_s>\d+) (?P<recvtype_s>\d+) (?P<recvlen_s>\d+):(?P<n_badtype_a>\d+) (?P<n_badlen_a>\d+) "
    "(?P<n_badcrc_a>\d+) (?P<ack_skew_err>-?\d+)")
M = re.compile(record_pattern %
    "M (?P<epoch>\d+):(?P<enter_ph2>\d+) (?P<enter_ph3>\d+):(?P<n_ta>\d+) (?P<enter_ctrlph>\d+) (?P<sleep_cmd>\d+)")
E = re.compile(record_pattern %
    "E (?P<epoch>\d+):(?P<radioon_time_ticks>[\d\.]+) (?P<ton_s>[\d\.]+) (?P<ton_ev>[\d\.]+) (?P<ton_t>[\d\.]+) "
    "(?P<ton_a>[\d\.]+) (?P<ton_ctrl>[\d\.]+)")
F = re.compile(record_pattern %
    "F (?P<epoch>\d+):(?P<tf_s>\d+) (?P<tf_ev>\d+) (?P<tf_t>\d+) (?P<tf_a>\d+) (?P<tf_ctrl>\d+):(?P<n_short_s>\d+) "
    "(?P<n_short_ev>\d+) (?P<n_short_t>\d+) (?P<n_short_a>\d+) (?P<n_short_ctrl>\d+)")
Ctrl_reception = re.compile(record_pattern %
    "SensorStates (?P<epoch>\d+):(?P<state_s0>-?\d+) (?P<state_s1>-?\d+) (?P<state_s2>-?\d+) "
    "(?P<state_s3>-?\d+) (?P<state_s4>-?\d+) (?P<state_s5>-?\d+) (?P<state_s6>-?\d+) (?P<state_s7>-?\d+) "
    "(?P<state_s8>-?\d+) (?P<state_s9>-?\d+) (?P<inthgt_0>-?\d+) (?P<inthgt_1>-?\d+) (?P<inthgt_2>-?\d+) "
    "(?P<inthgt_3>-?\d+) (?P<inthgt_4>-?\d+)")
Node = re.compile(record_pattern %
    "NODE (?P<epoch>\d+):(?P<ev_detected>\d+) (?P<pair1_detection>\d+) (?P<pair2_detection>\d+) (?P<reception_bitmap>\d+) "
    "(?P<n_pkt_recv_node>\d+)")
Actuator = re.compile(record_pattern %
    "ACTUATOR (?P<epoch>\d+):(?P<ev_detected>\d+) (?P<pair1_detection>\d+) (?P<pair2_detection>\d+) (?P<reception_bitmap>\d+) "
    "(?P<n_pkt_recv_node>\d+) (?P<my_position>\d+) (?P<command_recv>\d+) (?P<actuation_command>-?\d+) (?P<actuation_ts>\d+)")
Sensor = re.compile(record_pattern %
    "SENSOR (?P<epoch>\d+):(?P<i_detected>\d+) (?P<ev_detected>\d+) (?P<pair1_detection>\d+) (?P<pair2_detection>\d+) "
    "(?P<is_ack>\d+) (?P<my_position>\d+) (?P<ctrlpkt_recv>\d+) (?P<held_measures_updated>\d+)")
Sink = re.compile(record_pattern %
    "SINK (?P<epoch>\d+):(?P<ev_detected>\d+) (?P<pair1_detection>\d+) (?P<pair2_detection>\d+) (?P<collection_bitmap>\d+) "
    "(?P<compl_recp>\d+) (?P<ack_bitmap>\d+) (?P<n_pkt_recv_sink>\d+)")
Alive = re.compile(record_pattern %
    "I am alive! Node ID: (?P<ID>\d+)")
BS_epoch = re.compile(record_pattern %
    "fast bstrap epochs: (?P<BS_epochs>\d+)")
WCB_error_message = re.compile(record_pattern %
    "\[ERROR\] EPOCH: (?P<epoch>\d+) - (?P<message>[A-Z !]*)")
Testbed_ev = re.compile(
    "^\[(?P<time>.{23})\] INFO:pFile: Epoch (?P<epoch>\d+): sending Trig:(?P<trig_measure>-?\d+) - "
    "Data:(?P<data_measure>-?\d+) - Int Height S1:(?P<hgt_measure_s1>-?\d+) - Int Height S2:"
    "(?P<hgt_measure_s2>-?\d+) to node (?P<node_id>\d+)")
Ctrl_overflow = re.compile(
    "^\[(?P<time>.{23})\] INFO:pFile: [ERROR] EPOCH (?P<epoch>\d+): THE (?P<cmd_idx>\d+) ACTUATION "
    "COMMAND OVERFLOWED (?P<cmd>-?\d+)")
Simulink_error = re.compile(
    "^\[(?P<time>.{23})\] INFO:pFile: (?P<message>CLOSING CONNECTION TO SIMULINK|DEBUG: NO UPDATED DATA RETREIVED FROM SIMULINK)")


class T:
    def __init__(self, re, fname):
        self.re = re
        self.fname = fname
        self.file = None


tables = [
    T(S,                    "send_summary.txt"),
    T(P,                    "dbg.txt"),
    T(M,                    "phase.txt"),
    T(E,                    "energy.txt"),
    T(F,                    "energy_tf.txt"),
    T(Ctrl_reception,       "controller_reception.txt"),
    T(Node,                 "app_commonnode.txt"),
    T(Actuator,             "app_actuator.txt"),
    T(Sensor,               "app_sensor.txt"),
    T(Sink,                 "app_sink.txt"),
    T(BS_epoch,             "bootstrap.txt"),
    T(WCB_error_message,    "wcb_error_message.txt"),
    T(Testbed_ev,           "testbed_ev.txt"),
    T(Ctrl_overflow,        "ctrl_overflow.txt"),
    T(Simulink_error,       "simulink_error.txt"),
]


def log(t, gdict):
    if t.file is None:
        t.file = open(t.fname, "w")
        t.file.write("\t".join(gdict.keys()))
        t.file.write("\n")
    t.file.write("\t".join(gdict.values()))
    t.file.write("\n")


for fname in input_files:
    with open(fname, "r") as f:
        for line in f:
            for t in tables:
                m = t.re.match(line)
                if m is not None:
                    log(t, m.groupdict())
                    break
