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
#include <math.h>

#include "rdda_ecat.h"
#include "rdda_base.h"
#include "rdda_control.h"
#include "tele_control.h"
#include "shm_data.h"
#include "shm.h"

typedef struct {
    double mean;
    double unbias_variance;
    double mean_difference_square;
} Loop;

void online_mean_variance_calculation (Loop *loop, int32 index, int controlInterval) {
    double previous_mean = loop->mean;
    loop->mean = previous_mean + ((double)(controlInterval) - previous_mean) / index;
    loop->mean_difference_square += ((double)(controlInterval) - previous_mean) * ((double)(controlInterval) - loop->mean);
    if (index == 1) loop->unbias_variance = loop->mean_difference_square;
    else loop->unbias_variance = loop->mean_difference_square / (index - 1);
    printf("Control loop time mean is %10.4f us and variance is %10.4f us^2\r", loop->mean, loop->unbias_variance);
}

volatile sig_atomic_t done = 0;
volatile sig_atomic_t error_signal = 0;
void intHandler (int sig) {
    if (sig == SIGINT) {
        done = 1;
        printf("Received interrupt\n");
    }
}

void rdda_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    char *ip;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;
    /* User friendly struct */
    Rdda *rdda;
    /* DOB */
    ControlParams controlParams;
    SecondOrderLowPassFilterParams secondOrderLowPassFilterParams;
    PreviousVariables previousVariables;
    /* Teleoperation */
    TeleParam teleParam;

    /* timer */
    int cycletime = 250; /* in microseconds */;

    /* Configure ethercat network and slaves. */
    if (!strcmp(ifname, "right_gripper")) {
        ip = (char*)("enx000ec682ae62");
    }
    else if (!strcmp(ifname, "left_gripper")) {
        ip = (char*)("enx000ec682b118");
    }
    else if (!strcmp(ifname, "right_glove")) {
        ip = (char*)("enx000ec682b0fb");
    }
    else if (!strcmp(ifname, "left_glove")) {
        ip = (char*)("enxb49cdff1de4f");
    }
    else {
        printf("Wrong hand name called.\n");
        exit(1);
    }

    ecatSlaves = initEcatConfig(ip);
    if (ecatSlaves == NULL) {
        fprintf(stderr, "Init ecatslaves failed.\n");
        exit(1);
    }
    printf("Network configuration succeed.\n");

    /* Initialize user-friendly struct */
    rdda = initRdda(ifname);
    if (rdda == NULL) {
        fprintf(stderr, "Init rdda failed.\n");
        exit(1);
    }
    printf("Input/output interface succeed.\n");

    /* Initialize controller */
    /* These two lines are to initialize master to position mode while re-initializing piv gains,
     * comment out them when running DoB
     */
    pivGainSDOwrite(ecatSlaves->aev[0].slave_id, 0, 0); // Pp 400, Vp 100, Kp 18.6
    pivGainSDOwrite(ecatSlaves->aev[1].slave_id, 0, 0);
    pivGainSDOwrite(ecatSlaves->aev[2].slave_id, 0, 0);
    /**/

    initRddaStates(ecatSlaves, rdda);
    rdda_update(ecatSlaves, rdda);
    dobInit(&controlParams, &secondOrderLowPassFilterParams, &previousVariables, rdda);
    teleInit(&teleParam);

    /* Measure time interval for sleep */
    int usec_per_sec = 1000000;
    int nsec_per_usec = 1000;
    struct timespec startTime, endTime;
    int controlInterval;
    rdda_gettime(ecatSlaves);
    int local_error_signal;
    int remote_error_signal;

    /* Calculate loop rate mean and variance */
    Loop loop;
    loop.mean = 0.0;
    loop.mean_difference_square = 0.0;
    int32 index;

    while (!done && !error_signal) {

        /* Mark start time */
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        mutex_lock(&rdda->mutex);

        teleController(&teleParam, &controlParams, rdda);
        dobController(rdda, &controlParams, &secondOrderLowPassFilterParams, &previousVariables);

        rdda_update(ecatSlaves, rdda);

        /* Error code detection */
        local_error_signal = errorCheck(ecatSlaves);
        rdda->error_signal.error_out = local_error_signal;
        remote_error_signal = rdda->error_signal.error_in;
        error_signal = local_error_signal || remote_error_signal;

        mutex_unlock(&rdda->mutex);

        clock_gettime(CLOCK_MONOTONIC, &endTime);
        controlInterval = (endTime.tv_sec-startTime.tv_sec)*usec_per_sec + (endTime.tv_nsec-startTime.tv_nsec)/nsec_per_usec;

        index++;
        online_mean_variance_calculation(&loop, index, controlInterval);

        if (controlInterval >= cycletime) {
            printf("\nControl interval time exceeds defined cycle time, CT = %d\n", controlInterval);
            continue;
        }
        rdda_sleep(ecatSlaves, cycletime-controlInterval);
    }

    rddaStop(ecatSlaves);

}

int main(int argc, char **argv) {
    /* ctrl-c */
    signal(SIGINT, intHandler);

    pthread_t rt_thread;
    struct sched_param param;
    int policy = SCHED_FIFO;
    char enter;
    char *last_five = &argv[0][strlen(argv[0])-5];

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1) {

        while(!done) {
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

            if (!done && error_signal) {
                if (!strcmp(last_five, "aster")) {
                    printf("Error diagnosed, restart control. Press Enter\n");
                    while(1) {
                        enter = fgetc(stdin);
                        if (enter == 0x0A) break;
                        enter = getchar();
                    }
                }
                else if (!strcmp(last_five, "slave")) {
                    printf("Error diagnosed, restart control\n");
                }
            }
        }
    }
    else {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}
