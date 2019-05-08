/* rdda_simpletest.c */

//#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include "ethercat.h"
#include "rdda_ecat.h"
#include "rdda_base.h"
#include "shm_data.h"
#include "shm.h"

void rdda_run (void *ifnameptr)
{
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    RDDA_slave *rddaSlave;
    /* User friendly struct */
    JointCommands   *jointCommands;
    JointStates     *jointStates;
//    double current_time;
//    int nsec_per_sec;
    int cycletime;
    int loopnum;

    /* Configure ethercat network and slaves. */
    rddaSlave = rddaEcatConfig(ifname);
    if (rddaSlave == NULL) {
        fprintf(stderr, "Init rddaSlave failed.\n");
        exit(1);
    }
    printf("Network configuration succeed.\n");

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
    printf("Input/output interface succeed.\n");

    /* timer */
    cycletime = 500; /* 500us */

//    nsec_per_sec = 1000000000;
    for (loopnum = 0; loopnum < 20000; loopnum ++) {

        rdda_gettime(rddaSlave);
        rdda_update(rddaSlave, jointStates);
        printf("pos[0]: +%lf, vel[0]: +%lf, tau[0]: +%lf, pos[1]: +%lf, vel[1]: +%lf, tau[1]: +%lf\r, ctime: %lf",
                jointStates->act_pos[0], jointStates->act_vel[0], jointStates->act_tau[0],
                jointStates->act_pos[1], jointStates->act_vel[1], jointStates->act_tau[1],
                (cycletime - (double)(rddaSlave->time.delta_time) / 1000000)
                );

        rdda_gettime(rddaSlave);
        rdda_sleep(rddaSlave, cycletime);
    }

    rddaStop(rddaSlave);
}

int main(int argc, char **argv)
{

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1)
    {
        rdda_run(argv[1]);
    }
    else
    {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}