/* rdda_simpletest.c */

//#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>

#include "ethercat.h"
#include "rdda_ecat.h"
#include "rdda_base.h"
#include "shm_data.h"
#include "shm.h"


void rdda_simpletest(void *ifnameptr)
{
    /**
     *  EtherCAT network configuration and slave identification
     */
    char *ifname = ifnameptr;
    RDDA_slave *rddaSlave;
    double theta_rad[2];
    double comp_Nm[2];

    /* Configure ethercat network and slaves. */
    rddaSlave = rddaEcatConfig(ifname);
    if (rddaSlave == NULL)
    {
        fprintf(stderr, "Init data failed.\n");
        exit(1);
    }

    printf("Network configuration succeed.\n");
    /**
     *  PDO transfer
     */
    for (int i=1; i<20000; i++)
    {
        ec_receive_processdata(EC_TIMEOUTRET);

        theta_rad[0] = (double)(rddaSlave->motor[0].in_motor->act_pos)/COUNTS_PER_RADIAN;
        theta_rad[1] = (double)(rddaSlave->motor[1].in_motor->act_pos)/COUNTS_PER_RADIAN;
        comp_Nm[0] = (double)(rddaSlave->psensor.in_pressure->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL;
        comp_Nm[1] = (double)(rddaSlave->psensor.in_pressure->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL;

        printf("theta1: %+2.4lf, theta2: %+2.4lf, pressure1: %+2.4f, pressure2: %+2.4lf\r", theta_rad[0], theta_rad[1], comp_Nm[0], comp_Nm[1]);
        fflush(stdout);

        ec_send_processdata();
    }

    rddaStop(rddaSlave);
}

void rdda_run (void *ifnameptr)
{
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    RDDA_slave *rddaSlave;
    /* User friendly struct */
    JointCommands   *jointCommands;
    JointStates     *jointStates;
    int loopnum;

    /* Configure ethercat network and slaves. */
    rddaSlave = rddaEcatConfig(ifname);
    if (rddaSlave == NULL) {
        fprintf(stderr, "Init rddaSlave failed.\n");
        exit(1);
    }

    /* Initialize user-friendly struct */
    jointCommands = initJointCommands();
    if (jointCommands == NULL) {
        fprintf(stderr, "Init jointCommands failed.\n");
        exit(1);
    }
    jointStates = initJointStates();
    if (jointStates == NULL) {
        fprintf(stderr, "Init jointStates failed.\n");
        exit(1);
    }

    for (loopnum = 0; loopnum < 2000; loopnum ++) {

        rdda_update(rddaSlave, jointCommands, jointStates);
        printf("act_pos[0]: %lf, act_pos[1]: %lf\n", jointStates->act_pos[0], jointStates->act_pos[1]);
    }

    rddaStop(rddaSlave);
}

int main(int argc, char **argv)
{

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1)
    {
//        rdda_simpletest(argv[1]);
        rdda_run(argv[1]);
    }
    else
    {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}