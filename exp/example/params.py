powers = [-1]     # Possible values: 7, 5, 3, 1, 0, -1, -3, -5, -7, -9, -11, -13, -15, -24
channels = [25]

testbed = "unitn"
topology = 1      # Decide in which topology to run the experiment. 0 = HALL, 1 = DEPT.

synch_bs = 0      # Enable or disable the EWSN2019 Dependability Competition bootstrap
on_board = 1      # Control related processing carried out onboard (1) or not (0)
recovery_on = 1   # Enable or disable the Recovery Phase
max_tas_recv = 3  # Maximum number of TA pairs during the Recovery

period = 3        # Duration of a WCB epoch in seconds

n_tx_s = 3
dur_s = 10

n_tx_t = 2
dur_t = 9
 
n_tx_a = 3
dur_a = 11

n_tx_ev = 2
dur_ev = 6
n_ev_slot = 2

n_tx_ctrl = 2
dur_ctrl = 11
n_ctrl_slot = 2

n_emptys = [(2, 4)]
sync_ack = 1       # Exploit S, A, and CTRL messages to synchronise the network

if topology == 0:  # HALL
    sinks = [73]
    nodes = [50, 51, 52, 53, 54, 56, 57, 62, 63,
             64, 65, 70, 71, 72, 73, 74, 75, 76, 77]

if topology == 1:  # DEPT
    sinks = [20]
    nodes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
             18, 19, 20, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36]


boot_chop = "nohop"   # Hopping strategy during the bootstrap
chmap = "nohop"       # Hopping strategy during normal protocol operations

logging = True

start_time = "asap"   # Time to schedule the test
duration = 4500       # Duration of the test in seconds 

test_version = "std"  # Exploit the Simulink version *with* measurement noise ("noise") or *without* ("std")
seed_num = 100        # Seed value
