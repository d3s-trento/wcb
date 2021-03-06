
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Enable energy estimation */
#define ENERGEST_CONF_ON              1

/* system clock runs at 32 MHz */
#define SYS_CTRL_CONF_SYS_DIV         SYS_CTRL_CLOCK_CTRL_SYS_DIV_32MHZ

/* IO clock runs at 32 MHz */
#define SYS_CTRL_CONF_IO_DIV          SYS_CTRL_CLOCK_CTRL_IO_DIV_32MHZ

#define NETSTACK_CONF_WITH_IPV6       0

#define COFFEE_CONF_SIZE              0

#define WCB_CONF_APP_PRE_EPOCH_CB_TIME (RTIMER_SECOND/4) // Could be shrink

#define LPM_CONF_MAX_PM LPM_PM0

#endif /* PROJECT_CONF_H_ */

