#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ethercat.h"
#include "rdda/rddaconfig.h"
#include "rdda/init_BEL.h"
#include "rdda/rddamain.h"


/* SOEM global vars */
char IOmap[4096];
int expectedWKC;
volatile int wkc;
/*
uint8 currentgroup = 0;
*/

/** Declare slave identifier */
rdda_slavet *rdda_slave;


/** Locate and identify EtherCAT slaves
 *
 * @param rdda_slave    = Slave index group.
 * @return 0.
 */
static int slaveIdentify(rdda_slavet *rdda_slave)
{
    uint16 slaveIdx = 0;
    int buf = 0;
    for (slaveIdx = 1; slaveIdx <= ec_slavecount; slaveIdx++)
    {
        /* BEL motor drive */
        if ((ec_slave[slaveIdx].eep_man == 0x000000ab) && (ec_slave[slaveIdx].eep_id == 0x00001110))
        {
            uint32 serial_num;
            buf = sizeof(serial_num);
            ec_SDOread(slaveIdx, 0x1018, 4, FALSE, &buf, &serial_num, EC_TIMEOUTRXM);

            /* motor1 */
            if (serial_num == 0x2098302)
            {
                rdda_slave->motor1 = slaveIdx;
                /* CompleteAccess disabled for BEL drive */
                ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[slaveIdx].name, slaveIdx);
                mapMotorPDOs_callback(slaveIdx);
            }
            /* motor2 */
            if (serial_num == 0x2098303)
            {
                rdda_slave->motor2 = slaveIdx;
                /* CompleteAccess disabled for BEL drive */
                ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[slaveIdx].name, slaveIdx);
                mapMotorPDOs_callback(slaveIdx);
            }
        }
        /* pressure sensor */
        if ((ec_slave[slaveIdx].eep_man == 0x00000002) && (ec_slave[slaveIdx].eep_id == 0x0c1e3052))
        {
            rdda_slave->psensor = slaveIdx;
        }
    }

    return 0;
}


/** Close socket */
void rddaStop()
{
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}


/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
void rddaEcatConfig(void *ifnameptr)
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
	rddaStop();
	return;
    }

    /* Request for PRE-OP mode */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    /* Locate slaves */
    slaveIdentify(rdda_slave);

    /* If Complete Access (CA) disabled => auto-mapping work */
    ec_config_map(&IOmap);

    /* Let DC off for the time being */
    //ec_configdc(); // DC should be launched for each identified slave

    printf("Slaves mapped, state to SAFE_OP\n");
    /* Wait for all salves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    /* Initialize motor params */
    initMotor(rdda_slave->motor1);
    initMotor(rdda_slave->motor2);

    printf("Slaves initialized, state to OP\n");

    /* Check if all slaves are working properly */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /* Request for OP mode */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* Send one valid process data to make outputs in slave happy */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    /* Request OP state for all slaves */
    ec_writestate(0);
    /* Wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves\n");
    }
    else
    {
        printf("Operational state failed\n");
        rddaStop();
        return;
    }
}

