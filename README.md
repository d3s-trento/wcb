# Wireless Control Bus

Wireless Control Bus (WCB) is a concurrent transmissions (CTX) based protocol 
stack tailored to event-triggered control (ETC), one of the most promising
aperiodic control paradigms in the literature. 
By operating in conjunction with ETC and carefully orchestrating CTX floods, 
WCB timely and dynamically adapts the network operation to the control demands:
it minimizes the network overhead during quiescent, steady-state periods, while 
ensuring timely and reliable reactions when required by the event-triggered 
controller to retain control performance.  
Experimental results show that ETC over WCB can achieve the same control performance
of periodic control at a fraction of the energy costs, thus unlocking substantial
energy savings.


## Publication

**The Wireless Control Bus: Enabling Efficient Multi-hop Event-Triggered 
Control with Concurrent Transmissions**, Matteo Trobinger, Gabriel de Albuquerque 
Gleizer, Timofei Istomin, Manuel Mazo, Amy L. Murphy, and Gian Pietro Picco. In
*ACM Transaction on Cyber-Physical Systems (TCPS) 6.1 (2022).* [PDF](https://dl.acm.org/doi/10.1145/3485467)


## Status

We implement WCB for the TI's [CC2538](http://www.ti.com/product/CC2538) SoC, targeting
the [Zolertia Firefly](https://zolertia.io/product/firefly) board. Our prototype 
is built atop a Contiki OS port of [Glossy](https://github.com/kasunch/lwb-cc2538/tree/master/net/glossy)
for this SoC. Glossy network flooding is exploited in WCB as a primitive 
to build fast and reliable event detection, sensor readings collection, and 
dissemination of actuation commands.  

The WCB code can be adapted to work with different hardware platforms (e.g., the 
TMote Sky platform), or to be emulated in Cooja. Please contact us if you need a 
version of the WCB code compatible with TMote Sky nodes and Cooja.

We extensively evaluated WCB by using a cyber-physical testbed emulating a water
distribution system (implemented in MATLAB/Simulink) controlled over a real-world 
large-scale multi-hop wireless network. A detailed description of our real-time
network-in-the-loop experimental setup can be found in the 
[paper](https://dl.acm.org/doi/10.1145/3485467). 


## Code structure

The WCB code has the following directory structure:

* `apps/deployment` to statically set the nodes logical IDs given their IEEE addresses;

* `apps/wcb-test` WCB application logic and control related functions;

* `contiki` Contiki source tree;

* `cyber-physical_testbed/WIS` emulated water irrigation system, with and without 
measurement noise;

* `cyber-physical_testbed/orchestrator` to manage the interactions between the 
emulated plant model and the real wireless sensor network;

* `dev/cc2538` overridden SoC specific files of Contiki;

* `exp/example` example configuration of the WCB protocol;

* `net/glossy` Glossy implementation;

* `net/wcb` WCB implementation and default configuration;

* `platform/zoul` overridden Firefly node specific files of Contiki;

* `test-tools` utility scripts to build WCB from a configuration file and parse WCB logs.


## Preparing and running experiments

The instructions to configure and compile WCB for Zolertia Firefly
nodes are reported below.


#### Setting up the tool chain and source
1. Install the ARM tool chain (the ARM GCC version used in the development and testing of
   WCB is 9.2.1)
   
   ```
   https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
   ```  

2. Clone the WCB repository

   ```
   git clone https://github.com/d3s-trento/wcb.git
   ```
3. Get Contiki submodules.

   ```
   git submodule update --init --recursive
   ```

#### Setting up node IDs
Before compiling WCB, nodes should have IDs assigned. To guarantee that every node has 
a unique node ID, a static mappings between node IDs and IEEE addresses is 
done, following the same approach proposed in [lwb-cc2538](https://github.com/kasunch/lwb-cc2538).
Node IDs can be statically set in `apps/deployment/deployment.c` as follows.

```C
struct id_addr {
  uint16_t id;
  uint8_t ieee_addr[IEEE_ADDR_LEN];
};

static struct id_addr id_addr_list[] = {
  { 1, {0x00, 0x12, 0x4B, 0x00, 0x18, 0xD6, 0xF7, 0x9C}},
  { 2, {0x00, 0x12, 0x4b, 0x00, 0x14, 0xb5, 0xd9, 0x76}}, 
  { 3, {0x00, 0x12, 0x4B, 0x00, 0x18, 0xD6, 0xF3, 0x84}},
  { 4, {0x00, 0x12, 0x4B, 0x00, 0x18, 0xD6, 0xF3, 0xEE}},
  { 5, {0x00, 0x12, 0x4B, 0x00, 0x18, 0xD6, 0xF7, 0x92}},

  {0, {0, 0, 0, 0, 0, 0 ,0, 0}}
};
```
To discover the IEEE addresses of your devices, you can use the following command

`python contiki/tools/cc2538-bsl/cc2538-bsl.py -p <serial port>`

which should output something similar to what follows:
```
Opening port /dev/tty.SLAB_USBtoUART, baud 500000
Connecting to target...
CC2538 PG2.0: 512KB Flash, 32KB SRAM, CCFG at 0x0027FFD4
Primary IEEE Address: 00:12:4B:00:14:B5:D8:F1
```

#### Configuring and building WCB
To conveniently build WCB from a configuration file we encourage you to exploit the 
`test_tools/simgen.py` script. It reads the parameter set(s) defined in 
`params.py` and, for each set of parameters, builds a binary.

The `exp/example/params.py` file offers a good starting point for defining your 
parameter set and the list of nodes participating in the experiment.
Upon running `python ../../test_tools/simgen.py`, the `simgen.py` script  
creates one or several subdirectories (if they don't exist) named after the 
individual parameter sets defined in the `params.py` file and puts there the 
binaries compiled from `apps/wcb-test`. You can directly exploit such binaries to
run your experiments.


## Disclaimer
Although we tested the code extensively, it is considered a research prototype that 
likely contains bugs. We take no responsibility for and give no warranties in respect 
of using this code.
