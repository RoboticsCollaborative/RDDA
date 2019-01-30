#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ethercat.h"
#include "rddamain.h"

/* SOEM vars */
char IOmap[4096];
int expectedWKC;
volatile int wkc;
uint8 currentgroup = 0;

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param[in] ifnameptr = NIC interface pointer
 */
void rdda_ecat_config(void *ifnameptr)
{
    char *ifname  = (char *)ifnameptr;
    
    printf("Begin network configuration\n");

    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
	printf("Socket connection on %s succeeded.\n", ifname);
    }
    else
    {
	printf("No socket connection on %s.\nExcecuted as root\n", ifname);
	return;
    }

    /* Find and configure slaves */
    if (ec_config_init(FALSE)>0)
    {
	printf("%d slaves found and configured.\n", ec_slavecount);
    }
    else
    {
	printf("No slaves found!\n");
	printf("Close socket\n");
	ec_close(); /* stop SOEM, close socket */
	return;
    }
}
