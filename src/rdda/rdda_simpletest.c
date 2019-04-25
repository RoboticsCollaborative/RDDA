/* rdda_simpletest.c */

#define _GNU_SOURCE
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
    SlaveIndex *slaveIndex;
    int motor[2], psensor;
//    double theta_rad[2];
//    double comp_Nm[2];

//    int64 NSEC_PER_SEC = 1000000000;
/*
    double COUNTS_PER_RADIAN = 52151.8917;
    double PASCAL_PER_COUNT = 21.04178;
    double NM_PER_PASCAL = 2.822e-6;
*/

    /* Configure ethercat network and slaves. */
    slaveIndex= rddaEcatConfig(ifname);
    motor[0] = slaveIndex->motor[0];
    motor[1] = slaveIndex->motor[1];
    psensor = slaveIndex->psensor;

    printf("motor0: %d, motor1: %d, psensor: %d", motor[0], motor[1], psensor);

    free(slaveIndex);

    /**
     *  Initialize input/ouput interface
     */
/*
    MotorIn *motorIn[2];
    for (int mot_id = 0; mot_id < 2; mot_id ++)
    {
        motorIn[mot_id] = (MotorIn *) ec_slave[motor[mot_id]].inputs;
    }
    PressureIn *pressureIn = (PressureIn *) ec_slave[psensor].inputs;
*/

    /**
     *  PDO transfer
     */
/*
    for (int i=1; i<20000; i++)
    {
        ec_receive_processdata(EC_TIMEOUTRET);

        theta_rad[0] = (double)(motorIn[motor[0]]->act_pos)/COUNTS_PER_RADIAN;
        theta_rad[1] = (double)(motorIn[motor[1]]->act_pos)/COUNTS_PER_RADIAN;
        comp_Nm[0] = (double)(pressureIn->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL;
        comp_Nm[1] = (double)(pressureIn->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL;

        printf("theta1: %+2.4lf, theta2: %+2.4lf, pressure1: %+2.4f, pressure2: %+2.4lf\r", theta_rad[0], theta_rad[1], comp_Nm[0], comp_Nm[1]);
        fflush(stdout);

        ec_send_processdata();
    }
*/

    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    printf("End RDDA, close socket.\n");
    ec_close();
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