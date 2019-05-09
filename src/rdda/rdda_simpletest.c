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
    int nsec_per_sec;
    int cycletime;
//    int loopnum;

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
    cycletime = 1000; /* 500us */

    nsec_per_sec = 1000000000;
/*
    for (loopnum = 0; loopnum < 20000; loopnum ++) {

        rdda_gettime(rddaSlave);
        rdda_update(rddaSlave, jointStates);
        printf("pos[0]: +%lf, vel[0]: +%lf, tau[0]: +%lf, pos[1]: +%lf, vel[1]: +%lf, tau[1]: +%lf, ctime: %lf\n",
                jointStates->act_pos[0], jointStates->act_vel[0], jointStates->act_tau[0],
                jointStates->act_pos[1], jointStates->act_vel[1], jointStates->act_tau[1],
                (double)(rddaSlave->time.delta_time)
                );

        rdda_gettime(rddaSlave);
        rdda_sleep(rddaSlave, cycletime);
    }
*/
    double current_time;
    rdda_gettime(rddaSlave);
    current_time = (rddaSlave->time.ts.tv_sec * nsec_per_sec + rddaSlave->time.ts.tv_nsec);
    printf("start at: %lf\n", current_time);
//    rdda_update(rddaSlave, jointStates);
//    rdda_sleep(rddaSlave, cycletime);
    usleep(cycletime);
    rdda_gettime(rddaSlave);
    current_time = (rddaSlave->time.ts.tv_sec * nsec_per_sec + rddaSlave->time.ts.tv_nsec);
    printf("end at: %lf\n", current_time);

    rddaStop(rddaSlave);
}

int main(int argc, char **argv)
{
    pthread_t rt_thread;
    struct sched_param param;
    int policy = SCHED_FIFO;

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1)
    {


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
    else
    {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}