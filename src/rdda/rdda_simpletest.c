/* rdda_simpletest.c */

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>

#include "rdda_ecat.h"
#include "rdda_base.h"
#include "rdda_control.h"
#include "shm_data.h"
#include "shm.h"

void rdda_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;
    /* User friendly struct */
    Rdda *rdda;
    ControlParams controlParams;
    FilterParams filterParams;
    PreviousVariables previousVariables;
    int cycletime;
    //int start_time, end_time;
    //int delta_time;
    int loopnum;

    /* Configure ethercat network and slaves. */
    ecatSlaves = initEcatConfig(ifname);
    if (ecatSlaves == NULL) {
        fprintf(stderr, "Init ecatslaves failed.\n");
        exit(1);
    }
    printf("Network configuration succeed.\n");

    /* Initialize user-friendly struct */
    rdda = initRdda();
    if (rdda == NULL) {
        fprintf(stderr, "Init rdda failed.\n");
        exit(1);
    }
    printf("Input/output interface succeed.\n");

    /* timer */
    cycletime = 500; /* in microseconds */

    /* Initialize controller */
    //pivGainSDOwrite(ecatSlave->bel[0].slave_id, 100, 10);
    //pivGainSDOwrite(ecatSlave->bel[1].slave_id, 0, 0);
    initRddaStates(ecatSlaves, rdda);
    dobInit(&controlParams, &filterParams, &previousVariables, rdda);

    rdda_gettime(ecatSlaves);
    for (loopnum = 0; loopnum < 120000; loopnum ++) {

        //start_time = rdda_gettime(ecatSlave);

        /* Implement controller */
        rdda_sleep(ecatSlaves, cycletime);
        dobController(rdda, &controlParams, &filterParams, &previousVariables);
        rdda_update(ecatSlaves, rdda);

        printf("tg_pos[0]: %+d, pos[0]: %+2.4lf, vel[0]: %+2.4lf, pre[0]: %+2.4lf, tau_off[0]: %+2.4lf, tg_pos[1]: %+d, pos[1]: %+2.4lf, vel[1]: %+2.4lf, pre[1]: %+2.4lf, tau_off[1]: %+2.4lf\r",
               ecatSlaves->bel[0].out_motor->tg_pos, rdda->motor[0].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->motor[0].motorOut.tau_off,
               ecatSlaves->bel[1].out_motor->tg_pos, rdda->motor[1].motorIn.act_pos, rdda->motor[1].motorIn.act_vel, rdda->psensor.analogIn.val2, rdda->motor[1].motorOut.tau_off
        );

        //end_time = rdda_gettime(ecatSlave);
        //delta_time = cycletime - (end_time - start_time);
        //rdda_sleep(ecatSlave, delta_time);
    }

    rddaStop(ecatSlaves);
}

int main(int argc, char **argv) {
    pthread_t rt_thread;
    struct sched_param param;
    int policy = SCHED_FIFO;

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1) {


        /* Create realtime thread */
        pthread_create(&rt_thread, NULL, (void *)&rdda_run, (void *)argv[1]);
        // rdda_run(argv[1]);

        /* Scheduler */
        memset(&param, 0, sizeof(param));
        param.sched_priority = 40;
        pthread_setschedparam(rt_thread, policy, &param);

        /* Core-Iso */
        cpu_set_t CPU3;
        CPU_ZERO(&CPU3);
        CPU_SET(3, &CPU3);
        pthread_setaffinity_np(rt_thread, sizeof(CPU3), &CPU3);

        /* Wait until sub-thread is finished */
        pthread_join(rt_thread, NULL);
    }
    else {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}