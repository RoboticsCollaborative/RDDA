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


void rdda_simpletest(void *ifnameptr)
{
    /**
     *  EtherCAT network configuration and slave identification
     */
    char *ifname = ifnameptr;
    RDDA_slave *rddaSlave;
    double theta_rad[2];
    double comp_Nm[2];

//    int64 NSEC_PER_SEC = 1000000000;
    double COUNTS_PER_RADIAN = 52151.8917;
    double PASCAL_PER_COUNT = 21.04178;
    double NM_PER_PASCAL = 2.822e-6;

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

int main(int argc, char **argv)
{
    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1)
    {
        rdda_simpletest(argv[1]);
    }
    else
    {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}