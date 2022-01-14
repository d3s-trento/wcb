#ifndef WCB_CONF_H_
#define WCB_CONF_H_

#ifdef WCB_CONF_PERIOD_MS
#define WCB_CONF_PERIOD ((uint32_t)RTIMER_SECOND*WCB_CONF_PERIOD_MS/1000)
#endif

#ifndef WCB_CONF_PERIOD
#define WCB_CONF_PERIOD RTIMER_SECOND
#endif

#ifndef WCB_CONF_IS_SINK
#define WCB_CONF_IS_SINK 0
#endif

#ifdef WCB_CONF_DEF_CHANNEL
#define WCB_DEF_CHANNEL WCB_CONF_DEF_CHANNEL
#else
#define WCB_DEF_CHANNEL 26
#endif

#ifndef WCB_CONF_NTX_S
#define WCB_CONF_NTX_S 3
#endif

#ifdef WCB_CONF_DUR_S_MS
#define WCB_CONF_DUR_S ((uint32_t)RTIMER_SECOND*WCB_CONF_DUR_S_MS/1000)
#endif

#ifndef WCB_CONF_DUR_S
#define WCB_CONF_DUR_S ((uint32_t)RTIMER_SECOND*10/1000)
#endif

#ifndef WCB_CONF_NTX_T
#define WCB_CONF_NTX_T 3
#endif

#ifdef WCB_CONF_DUR_T_MS
#define WCB_CONF_DUR_T ((uint32_t)RTIMER_SECOND*WCB_CONF_DUR_T_MS/1000)
#endif

#ifndef WCB_CONF_DUR_T
#define WCB_CONF_DUR_T ((uint32_t)RTIMER_SECOND*8/1000)
#endif

#ifndef WCB_CONF_NTX_A
#define WCB_CONF_NTX_A 3
#endif

#ifdef WCB_CONF_DUR_A_MS
#define WCB_CONF_DUR_A ((uint32_t)RTIMER_SECOND*WCB_CONF_DUR_A_MS/1000)
#endif

#ifndef WCB_CONF_DUR_A
#define WCB_CONF_DUR_A ((uint32_t)RTIMER_SECOND*8/1000)
#endif

#ifndef WCB_CONF_NTX_EV
#define WCB_CONF_NTX_EV 3
#endif

#ifdef WCB_CONF_DUR_EV_MS
#define WCB_CONF_DUR_EV ((uint32_t)RTIMER_SECOND*WCB_CONF_DUR_EV_MS/1000)
#endif

#ifndef WCB_CONF_DUR_EV
#define WCB_CONF_DUR_EV ((uint32_t)RTIMER_SECOND*5/1000)
#endif

#ifndef WCB_CONF_NTX_CTRL
#define WCB_CONF_NTX_CTRL 3
#endif

#ifdef  WCB_CONF_DUR_CTRL_MS
#define WCB_CONF_DUR_CTRL ((uint32_t)RTIMER_SECOND*WCB_CONF_DUR_CTRL_MS/1000)
#endif

#ifndef WCB_CONF_DUR_CTRL
#define WCB_CONF_DUR_CTRL ((uint32_t)RTIMER_SECOND*15/1000)
#endif

/* [Recovery phase] The number of empty TA pairs in a row causing a node without data to leave 
 * the Recovery phase (entering to sleep until the beginning of the Dissemination phase) */
#ifndef WCB_CONF_MAX_SILENT_TAS
#define WCB_CONF_MAX_SILENT_TAS 2
#endif

/* [Recovery phase] The number of missing ACKs in a row causing a node with data to leave 
 * the Recovery phase (entering to sleep until the beginning of the Dissemination phase) */
#ifndef WCB_CONF_MAX_MISSING_ACKS
#define WCB_CONF_MAX_MISSING_ACKS 4
#endif

/* Exploit ACK and CTRL messages to synchronize the network */
#ifdef WCB_CONF_SYNC_ACKS
#define WCB_SYNC_ACKS WCB_CONF_SYNC_ACKS
#else
#define WCB_SYNC_ACKS 1
#endif

/* Logging levels */
#define WCB_LOGS_NONE 0
#define WCB_LOGS_EPOCH_STATS 1
#define WCB_LOGS_ALL 2

#ifdef WCB_CONF_LOGLEVEL
#define WCB_LOGLEVEL WCB_CONF_LOGLEVEL
#else
#define WCB_LOGLEVEL WCB_LOGS_NONE
#endif

/* WCB hopping strategy during the bootstrap */
#ifdef WCB_CONF_BSTRAP_CHHOPPING
#define WCB_BSTRAP_CHHOPPING WCB_CONF_BSTRAP_CHHOPPING
#else
#define WCB_BSTRAP_CHHOPPING BSTRAP_nohop
#endif

/* WCB hopping strategy (normal protocol operation) */
#ifdef WCB_CONF_CHHOP_MAPPING
#define WCB_CHHOP_MAPPING WCB_CONF_CHHOP_MAPPING
#else
#define WCB_CHHOP_MAPPING CHMAP_nohop
#endif

/* The maximum WCB packet size */
#ifdef WCB_CONF_PKTBUF_LEN
#define WCB_PKTBUF_LEN WCB_CONF_PKTBUF_LEN
#else
#define WCB_PKTBUF_LEN 127
#endif

/* WCB will notify the application this time before an epoch starts.
 * NB: By default, WCB_CONF_APP_PRE_EPOCH_CB_TIME is defined in apps/project-conf.h */
#ifdef WCB_CONF_APP_PRE_EPOCH_CB_TIME
#define WCB_APP_PRE_EPOCH_CB_TIME WCB_CONF_APP_PRE_EPOCH_CB_TIME
#else
#define WCB_APP_PRE_EPOCH_CB_TIME 1600 // ~ 50 ms 
#endif

/* Number of source nodes */
#ifndef NUM_SOURCES
#define NUM_SOURCES 10
#endif

/* Number of height sensor nodes */
#ifndef NUM_HEIGHT_SENSORS
#define NUM_HEIGHT_SENSORS 5
#endif

/* Number of Actuator nodes */
#ifndef NUM_ACTUATORS
#define NUM_ACTUATORS 5
#endif

/* Number of Event slots in the Event Phase */
#ifndef WCB_EV_SLOTS
#define WCB_EV_SLOTS 2
#endif

/* Number of CTRL slots in the Dissemination Phase */
#ifndef WCB_CONTROL_SLOTS
#define WCB_CONTROL_SLOTS 2
#endif

/* Enable or disable the Recovery Phase*/
#ifdef WCB_RECOVERY_ON
#define RECOVERY_ON WCB_RECOVERY_ON
#else
#define RECOVERY_ON 1
#endif

/* Maximum number of TA pairs that WCB can schedule during the Recovery phase.
 * Remember that the (control) Dissemination phase starts only some time after
 * the *maximum* duration of the Recovery phase, so the bigger WCB_MAX_TAS_RECV 
 * the higher the actuation latency.
*/
#ifndef WCB_MAX_TAS_RECV
#define WCB_MAX_TAS_RECV 3
#endif

/* The processing required to compute new actuation commands is carried out
 * onboard on the controller node (1) or it is offloaded to an external entity (0) */
#ifdef WCB_ON_BOARD
#define ON_BOARD WCB_ON_BOARD
#else
#define ON_BOARD 0
#endif

#ifdef SYNCH_BOOTSTRAP
#define DEP_BOOTSTRAP SYNCH_BOOTSTRAP
#else
#define DEP_BOOTSTRAP 1
#endif

#ifdef TOPOLOGY
#define TESTBED_TOPOLOGY TOPOLOGY
#else
#define TESTBED_TOPOLOGY 1
#endif

#ifndef START_DELAY_SINK
#define START_DELAY_SINK 10
#endif

#ifndef START_DELAY_NONSINK
#define START_DELAY_NONSINK 5
#endif

#ifndef MAX_CORRECT_HOPS
#define MAX_CORRECT_HOPS 30
#endif

#ifndef WCB_ACK_SKEW_ERROR_DETECTION
#define WCB_ACK_SKEW_ERROR_DETECTION 1
#endif

#endif /* WCB_CONF_H */
