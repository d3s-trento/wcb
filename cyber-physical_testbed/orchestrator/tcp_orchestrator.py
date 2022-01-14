#!/usr/bin/env python3

# Authors: Matteo Trobinger <matteo.trobinger@gmail.com>
#          Davide Molteni
#          Timofei Istomin <tim.ist@gmail.com>

import testbed.Testbed
import logging
import socket
from selectors import DefaultSelector, EVENT_READ
import struct
import re
import numpy as np
import scipy.io as sio

HOST = ''  # all addresses
PORT_OUT = 10020
PORT_IN  = 10021


# Scale factors to compute the control related functions
SCALE_FACTOR_HEIGHT   = np.power(2, 16)
SCALE_FACTOR_FLOW     = np.power(2, 19)
SCALE_FACTOR_INTEGRAL = np.power(2, 9)
SCALE_FACTOR_ACT      = np.power(2, 9)


NUM_SURCES = 10
NUM_ACTUATORS = 5


# To parse data either coming from Simulink or from the controller
Sensor_Request    = re.compile("^GetReadings (?P<epoch>\d+):(?P<position>\d+) (?P<is_hgtsensor>\d+)$")
Control_Function  = re.compile("^SensorStates (?P<epoch>\d+):(?P<state_s0>-?\d+) (?P<state_s1>-?\d+) (?P<state_s2>-?\d+) (?P<state_s3>-?\d+) "
                                "(?P<state_s4>-?\d+) (?P<state_s5>-?\d+) (?P<state_s6>-?\d+) (?P<state_s7>-?\d+) (?P<state_s8>-?\d+) (?P<state_s9>-?\d+) "
                                "(?P<inthgt_0>-?\d+) (?P<inthgt_1>-?\d+) (?P<inthgt_2>-?\d+) (?P<inthgt_3>-?\d+) (?P<inthgt_4>-?\d+)$")
Actuation_Command = re.compile("^ACTUATOR (?P<epoch>\d+):(?P<ev_detected>\d+) (?P<pair1_detection>\d+) (?P<pair2_detection>\d+) (?P<reception_bitmap>\d+) "
                                "(?P<n_pkt_recv_node>\d+) (?P<my_position>\d+) (?P<command_recv>\d+) (?P<actuation_command>-?\d+) (?P<actuation_ts>\d+)$")


logger = logging.getLogger(__name__)


class Main:
    def __init__(self, workspace):
        # Initialization variables:
        self.workspace = workspace
        self.K = self.workspace['K']
        self.nT = int(self.workspace['nT'])
        self.Qt = self.workspace['Qt']
        self.Minv = self.workspace['Minv']
        self.Phit = self.workspace['Phit']
        self.n = int(self.workspace['n']) + 5  # with integrators
        self.epsilon = self.workspace['epsilon']
        self.ep_vector = self.workspace['epvector']
        self.De = self.workspace['De']
        self.Dx = self.workspace['Dx']

        # Sensor readings received from Simulink to pass to the sensors
        self.trig_scaled = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.data_scaled = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        # Values provided by the controller to compute new actuation commands (sensor readings + discrete time integral height)
        self.sensor_state_vector_scaled = np.array([0] * 15)

        # Actuation commands to send back to the controller
        self.actuators_scaled = np.array([0, 0, 0, 0, 0])

        # Actuation commands and timestamps from the actuators to be communicated to Simulink
        self.actuation_command = np.array([0, 0, 0, 0, 0])
        self.actuation_timestamps = np.array([61, 61, 61, 61, 61])  # NB: To set a default actuation timestamp vector equal to [0, 0, 0, 0, 0]
                                                                    # leads to negative time delays in Simulink and needs to be avoided
        self.rcvd_actuation_command = set()

        # Simulink and WCB do not start working simultaneously. Until Simulink is ready to provide real readings to the sensor nodes
        # this python script generates and communicates fake data, preventing WCB to stop working prematurely.
        self.n_fakedata = 0  # This flag is exploited to notify that fake data have already been generated in the current epoch: the 
                             # related logs should not be considered in the experiment analysis
        self.have_new_data = False  # It might happen that the new sensor readings from Simulink arrive too late. If / when this happen
                                    # we need to log it and quit the experiment
        self.initializing = True
        self.first_after_initialization = False  # First epoch after the connection with Simulink has been successfully established

    # Format the actuation commands and the relative timestamps to be sent to Simulink
    def pack_actuator_state(self):
        array = [x for x in self.actuation_command] + [y for y in self.actuation_timestamps]
        return struct.pack(">hhhhhhhhhh", *array)

    def unpack_sensor_states(self, data):
        values = struct.unpack(">" + "h" * 30, data)
        return values

    def on_accept_in(self, key):
        new_conn, addr = self.listen_socket_in.accept()
        logger.info('accepted "IN" connection from {0}'.format(addr))
        new_conn.setblocking(False)

        key = self.selector.register(fileobj=new_conn, events=EVENT_READ,
                               data=self.on_simulink_data)
        self.simulink_socket_in = new_conn

    def on_accept_out(self, key):
        new_conn, addr = self.listen_socket_out.accept()
        logger.info('accepted "OUT" connection from {0}'.format(addr))
        new_conn.setblocking(False)

        self.simulink_socket_out = new_conn

    def on_simulink_data(self, key):
        # This is a handler for peer sockets - it's called when there's new data.
        try:
            data = key.fileobj.recv(30 * 2)  # Simulink sends 15 * 2 values (20 readings and 10 integral heights)
            if data:
                peername = key.fileobj.getpeername()
                logging.debug('got data from {}: {!r}'.format(peername, data))
                values = self.unpack_sensor_states(data)

                self.trig_scaled = values[:15]  # Readings [0:10] and integral height [10:15] at the first timestamp
                                                # (exploited by sensor nodes to evaluate their triggering conditions)
                self.data_scaled = values[15:]  # Readings [15:25] and integral height [25:30] at the second timestamp
                                                # (acquired by sensor nodes upon an event detection and communicated to the controller)
                logger.info("Sensors at ts1: {}".format(self.trig_scaled))
                logger.info("Sensors at ts2: {}".format(self.data_scaled))

                self.have_new_data = True  # New sensor readings have been received

                if self.initializing:  # We enter here the first time we receive correct data from Simulink, only
                    self.initializing = False
                    self.first_after_initialization = True
            else:
                if not self.initializing:
                    logger.info("DEBUG: Data not valid. Closing all connections")
                    self.selector.unregister(self.listen_socket_in)
                    self.selector.unregister(self.listen_socket_out)
                    self.listen_socket_in.close()
                    self.listen_socket_out.close()
                    self.close_connection()
                else:
                    logger.info("DEBUG: Data not valid")
                    self.close_connection()
        except ConnectionResetError:
            logger.info("DEBUG: Connection Reset")
            self.close_connection()


    def on_node_data(self, key):
        node = self.fd2node[key.fd]
        data = node.read().decode("ASCII")

        lines = data.split("\n")
        for line in lines:
            m = Sensor_Request.match(line)
            if m:
                g = m.groupdict()
                epoch = str(g["epoch"])
                position = int(g["position"])
                is_hgtsensor = int(g["is_hgtsensor"])  # Check whether the node is an height sensor or not

                if (self.initializing or self.n_fakedata != 0):  # The communication protocol is running, BUT either the connection with Simulink has not been established 
                                                                 # yet (self.initializing = True) or fake data have been already generated in the current epoch (n_fakedata != 0). 
                                                                 # The epoch should not be considered in the analysis.
                    # Generate fake sensor readings for the node
                    trig_ms = str(0)
                    data_ms = str(0)
                    inthgt_ms_trg = str(0)
                    inthgt_ms_dt = str(0)
                    self.n_fakedata = self.n_fakedata + 1
                    # Communicate the fake readings to the sensor node via serial line
                    if is_hgtsensor:
                        logger.info("Epoch {}: sending Trig:{} - Data:{} - Int Height S1:{} - Int Height S2:{} to node {}".format(epoch,
                            trig_ms, data_ms, inthgt_ms_trg, inthgt_ms_dt, node.getId()))  # Debug print
                        node.write(b"D "+epoch.encode()+b" "+trig_ms.encode()+b" "+data_ms.encode()+b" "+inthgt_ms_trg.encode()+b" "+inthgt_ms_dt.encode()+b"\n")
                    else:
                        logger.info("Epoch {}: sending Trig:{} - Data:{} - Int Height S1:0 - Int Height S2:0 to node {}".format(epoch, trig_ms, data_ms, node.getId()))
                        node.write(b"D "+epoch.encode()+b" "+trig_ms.encode()+b" "+data_ms.encode()+b"\n")

                elif self.have_new_data:  # (Real) updated readings have been received from Simulink
                    if self.first_after_initialization:  # Keep trace of the first epoch after the connection with Simulink has
                                                         # been established: we are going to analyse the logs starting from that epoch
                        logger.info("ANALYSIS: FIRST VALID EPOCH: {}".format(epoch))
                        self.first_after_initialization = False
                    if is_hgtsensor:  # If the node requesting new readings is an height sensor, write on serial: epoch, reading at ts1, reading at ts2, 
                                      # integral height computed with the reading collected at ts1, integral height computed with the reading collected at ts2
                        trig_ms = str(self.trig_scaled[position])
                        data_ms = str(self.data_scaled[position])
                        inthgt_ms_trg = str(self.trig_scaled[int(NUM_SURCES + (position / 2))])  # Height measures are stored in locations [10:15]; Height sensors 
                                                                                                 # are located in position 0, 2, 4, 6, 8 in the source_id array (wbc-test.c)
                        inthgt_ms_dt = str(self.data_scaled[int(NUM_SURCES + (position / 2))])
                        logger.info("Epoch {}: sending Trig:{} - Data:{} - Int Height S1:{} - Int Height S2:{} to node {}".format(epoch, trig_ms, data_ms,
                            inthgt_ms_trg, inthgt_ms_dt, node.getId()))
                        node.write(b"D "+epoch.encode()+b" "+trig_ms.encode()+b" "+data_ms.encode()+b" "+inthgt_ms_trg.encode()+b" "+inthgt_ms_dt.encode()+b"\n")
                    else:  # Flow sensor, no integral height values. Return the epoch, the triggering and the data readings, only
                        trig_ms = str(self.trig_scaled[position])
                        data_ms = str(self.data_scaled[position])
                        logger.info("Epoch {}: sending Trig:{} - Data:{} - Int Height S1:0 - Int Height S2:0 to node {}".format(epoch, trig_ms, data_ms, node.getId()))
                        node.write(b"D "+epoch.encode()+b" "+trig_ms.encode()+b" "+data_ms.encode()+b"\n")
                else:  # No updated readings have been received from Simulink. 
                    logger.info("DEBUG: NO UPDATED DATA RETREIVED FROM SIMULINK")
                continue

            m = Control_Function.match(line)
            if m:
                g = m.groupdict()
                epoch = str(g["epoch"])

                if self.initializing or self.n_fakedata != 0:  # The communication protocol is running, BUT either the connection with Simulink has not been established
                                                               # yet (self.initializing = True) or fake data have been already generated in the current epoch (n_fakedata != 0).
                                                               # Fake actuation commands will be communicated. Clearly, the epoch should not be considered in the analysis.
                    node.write(b"C "+epoch.encode()+b" "+str(self.actuators_scaled[0]).encode()+b" "+str(self.actuators_scaled[1]).encode()+b" "+
                        str(self.actuators_scaled[2]).encode()+b" "+str(self.actuators_scaled[3]).encode()+b" "+str(self.actuators_scaled[4]).encode()+b"\n")

                else:  # Both the communication protocol and Simulink are running; we need to compute and communicate real actuation commands
                    for i in range(0, 15):
                        if i < 10:
                            self.sensor_state_vector_scaled[i] = int(g["state_s" + str(i)])
                        else:
                            self.sensor_state_vector_scaled[i] = int(g["inthgt_" + str(i - 10)])

                    # Scale back the sensor state vector before computing the control related functions
                    # sensor_state[h1, f1, h2, f2, h3, f3, h4, f4, h5, f5, Ih, Ih, Ih, Ih, Ih]
                    sensor_state_vector = np.zeros(15)
                    sensor_state_vector[0:10:2] = np.divide(self.sensor_state_vector_scaled[0:10:2], SCALE_FACTOR_HEIGHT)
                    sensor_state_vector[1:10:2] = np.divide(self.sensor_state_vector_scaled[1:10:2], SCALE_FACTOR_FLOW)
                    sensor_state_vector[10:15]  = np.divide(self.sensor_state_vector_scaled[10:15], SCALE_FACTOR_INTEGRAL)

                    logger.info("SCALED BACK SENSOR STATE VECTOR: {}".format(sensor_state_vector))  # Debug print

                    self.actuators_scaled = self.act_commands(sensor_state_vector, self.K, epoch)  # Compute new actuation commands

                    logger.info("SCALED ACTUATION COMMANDS: {}".format(self.actuators_scaled))  # Debug print
                    logger.info("Sending actuation commands to the controller")  # Debug print
                    # Communicate the new actuation commands to the controller
                    node.write(b"C "+epoch.encode()+b" "+str(self.actuators_scaled[0]).encode()+b" "+str(self.actuators_scaled[1]).encode()+b" "+
                        str(self.actuators_scaled[2]).encode()+b" "+str(self.actuators_scaled[3]).encode()+b" "+str(self.actuators_scaled[4]).encode()+b"\n")
                continue

            m = Actuation_Command.match(line)
            if m:
                g = m.groupdict()
                epoch = str(g["epoch"])
                position = int(g["my_position"])

                if self.initializing or self.n_fakedata != 0:  # The communication protocol is running, BUT either the connection with Simulink has not been established
                                                               # yet (self.initializing = True) or fake data have been generated in the current epoch (n_fakedata != 0).
                                                               # The received actuation commands and timestamps can be ignored.
                    self.rcvd_actuation_command.add(position)
                    if (len(self.rcvd_actuation_command) == NUM_ACTUATORS):  # All actuators have communicated their current actuation command and the related timestamp, the epoch is finished
                        self.n_fakedata = 0  # Reset the fake data counter
                        self.rcvd_actuation_command.clear()  # Reset the "counter" of actuation commands received in the epoch
                        logger.info("DEBUG: INITIALIZATION EPOCH COMPLETED")
                elif self.have_new_data:  # Both the communication protocol and Simulink are running; we need to forward the actuation commands and timestamps to Simulink
                    logger.info("RECEIVED ACTUATION COMMAND {}:{}".format(epoch, position))

                    self.actuation_command[position] = int(g["actuation_command"])
                    self.actuation_timestamps[position] = int(g["actuation_ts"])
                    self.rcvd_actuation_command.add(position)

                    if (len(self.rcvd_actuation_command) == NUM_ACTUATORS):  # All actuators have communicated their current actuation command and the related timestamp, the epoch is finished
                        self.have_new_data = False  # End of the "epoch", reset the new data flag
                        self.rcvd_actuation_command.clear()  # Reset the "counter" of the received actuation commands in the current epoch
                        self.simulink_socket_out.send(self.pack_actuator_state())  # Send actuator states and actuation timestamps to Simulink
                        logger.info("Sending actuator commands and timestamps to Simulink")
                continue


    def close_connection(self):
        if self.simulink_socket_in is not None:
            peername = self.simulink_socket_in.getpeername()
            logger.info('CLOSING "IN" CONNECTION TO SIMULINK {0}'.format(peername))
            self.selector.unregister(self.simulink_socket_in)
            self.simulink_socket_in.close()

        if self.simulink_socket_out is not None:
            peername = self.simulink_socket_out.getpeername()
            logger.info('CLOSING "OUT" CONNECTION TO SIMULINK {0}'.format(peername))
            self.simulink_socket_out.close()


    def act_commands(self, sensor_state_vector, K, epoch):
        act_vector = -K @ sensor_state_vector
        # Scale actuation commands and round them to int16
        for idx, command in enumerate(act_vector):
            if command > 32767 or command < -32768:
                logger.info(f"[ERROR] EPOCH {epoch}: THE {idx} ACTUATION COMMAND OVERFLOWED {command}")
        u = np.array([max(-32768, min(32767, int(round(x)))) for x in SCALE_FACTOR_ACT * act_vector])
        return u


    def run(self, testbed):
        try:
            chattyNodes = [n for n in testbed.activeNodes]

            self.simulink_socket_in = None
            self.listen_socket_in = socket.socket()
            self.listen_socket_in.bind((HOST, PORT_IN))
            self.listen_socket_in.listen(2)
            self.listen_socket_in.setblocking(False)

            self.simulink_socket_out = None
            self.listen_socket_out = socket.socket()
            self.listen_socket_out.bind((HOST, PORT_OUT))
            self.listen_socket_out.listen(2)
            self.listen_socket_out.setblocking(False)

            self.selector = DefaultSelector()

            self.selector.register(fileobj=self.listen_socket_in,
                    events=EVENT_READ, 
                    data=self.on_accept_in)

            self.selector.register(fileobj=self.listen_socket_out,
                    events=EVENT_READ, 
                    data=self.on_accept_out)

            self.fd2node = {}
            for n in chattyNodes:
                key = self.selector.register(fileobj=n._pipe,
                        events=EVENT_READ, 
                        data=self.on_node_data)
                self.fd2node[key.fd] = n

            # Cyclically call handlers for incoming connections and data
            while True:
                try:
                    ev_list = self.selector.select()
                    if ev_list:
                        for key_ev in ev_list:
                            key = key_ev[0]
                            handler = key.data
                            handler(key)
                    else:
                        logger.info("Selector event list is empty")
                        break
                except Exception as e:
                    logger.exception(e)
                    pass
        except Exception as e:
            logger.exception(e)


def run_test(testbed):
    try:
        workspace = sio.loadmat(str(testbed.testDir.joinpath('data.mat')))
        m = Main(workspace)
        m.run(testbed)
    except Exception as e:
        logger.exception(e)


def init_test(test, scenario="std", seed=None):
    '''
    Initialise WCB experiments. This function is highly dependent on the testbed implementation,
    therefore is not reported hereafter.
    
    The main goal of init_test is to execute on a remote host the MATLAB/Simulink script emulating
    the physical system under study (a water irrigation system in our case) with the desired set
    of parameters. The MATLAB/Simulink script will interact with the python orchestrator via the 
    sockets defined in the Main object. MATLAB stdout and stderr can be stored for debug purposes.

    Assuming ssh is a paramiko.SSHClient connected to the remote host that runs the MATLAB script,  
    users can start implementing init_test as follows:

        matlab_param = f"-vers_sim {scenario}"
        if scenario == "noise" and seed is not None:
            matlab_param += f" -seed {seed}"
        stdin, stdout, stderr = ssh.exec_command(f"nohup ~/run_matlab.sh -test_id {test.test_id} {matlab_param} >run_stdout.txt 2>run_stderr.txt < /dev/null &")
        
        stdin.close()
        exitCode = stdout.channel.recv_exit_status()
        logger.info("ExitCode %d for matlab", exitCode)
        stdout.close()
        stderr.close() 
    '''
