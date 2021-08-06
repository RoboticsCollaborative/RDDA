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
#include "contact_detect.h"
#include "tele_control.h"
#include "shm_data.h"
#include "shm.h"

volatile sig_atomic_t done = 0;
void intHandler (int sig) {
    if (sig == SIGINT) {
        done = 1;
        printf("Received interrupt");
    }
}

void rdda_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;
    /* User friendly struct */
    Rdda *rdda;
    /* DOB */
    ControlParams controlParams;
    FirstOrderLowPassFilterParams firstOrderLowPassFilterParams;
    SecondOrderLowPassFilterParams secondOrderLowPassFilterParams;
    PreviousVariables previousVariables;
    /* Contact */
    ContactDetectionParams contactDetectionParams;
    ContactDetectionHighPassFilterParams contactDetectionHighPassFilterParams;
    ContactDetectionPreviousVariable contactDetectionPreviousVariable;
    /* Teleoperation */
    TeleParam teleParam;

    int cycletime;
    //int start_time, end_time;
    //int delta_time;
    //int loopnum;
    double time = 0.0;

    /* create a data file */
    FILE *fptr;
    char filename[] = "rdda_log.dat";
    remove(filename);
    fptr = fopen(filename, "w");

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
    /* These two lines are to initialize master to position mode while re-initializing piv gains,
     * comment out them when running DoB
     */
    pivGainSDOwrite(ecatSlaves->bel[0].slave_id, 0, 0); // Pp 400, Vp 100, Kp 18.6
    pivGainSDOwrite(ecatSlaves->bel[1].slave_id, 0, 0);
    /**/

    initRddaStates(ecatSlaves, rdda);
    rdda_update(ecatSlaves, rdda);
    dobInit(&controlParams, &firstOrderLowPassFilterParams, &secondOrderLowPassFilterParams, &previousVariables, rdda);
    teleInit(&teleParam);
    contactDetectionInit(&contactDetectionParams, &contactDetectionHighPassFilterParams, &contactDetectionPreviousVariable, rdda);

    /* Measure time interval for sleep */
    int usec_per_sec = 1000000;
    int nsec_per_usec = 1000;
    struct timespec startTime, endTime;
    int controlInterval;
    rdda_gettime(ecatSlaves);
    /* Initialise timestamps */
    int i = 0;
    //double r = 4.0; //inertia ratio

    while (!done) {

        /* Mark start time */
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        time += 0.5e-3;

        mutex_lock(&rdda->mutex);

        teleController(&teleParam, &controlParams, rdda);
        //contactDetection(&contactDetectionParams, &contactDetectionHighPassFilterParams, &contactDetectionPreviousVariable, rdda);
        dobController(rdda, &controlParams, &firstOrderLowPassFilterParams, &secondOrderLowPassFilterParams, &previousVariables);

        rdda_update(ecatSlaves, rdda);
        i++;
        //printf("tg_pos[0]: %+d, pos[0]: %+2.4lf, vel[0]: %+2.4lf, pre[0]: %+2.4lf, tau_off[0]: %+2.4lf, tg_pos[2]: %+d, pos[2]: %+2.4lf, vel[2]: %+2.4lf, pre[2]: %+2.4lf, tau_off[2]: %+2.4lf, ",
        //       ecatSlaves->bel[0].out_motor->tg_pos, rdda->motor[0].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->motor[0].motorOut.tau_off,
        //       ecatSlaves->bel[2].out_motor->tg_pos, rdda->motor[2].motorIn.act_pos, rdda->motor[2].motorIn.act_vel, rdda->psensor.analogIn.val3, rdda->motor[2].motorOut.tau_off
        //);

        /* Error code detection */
        done = errorCheck(ecatSlaves);

        //printf("%+10d,%+10d", ecatSlaves->bel[2].in_motor->analog_in,ecatSlaves->bel[3].in_motor->analog_in);

        /* save data to file */
        //fprintf(fptr, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", rdda->motor[0].motorIn.act_pos, rdda->motor[2].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->motor[2].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->psensor.analogIn.val3, time);
        //fprintf(fptr, "%lf, %lf, %lf, %lf, %lf\n", rdda->motor[2].motorIn.act_pos, controlParams.filtered_pos[2], rdda->motor[2].motorIn.act_vel, controlParams.filtered_vel[2], time);
        mutex_unlock(&rdda->mutex);
        clock_gettime(CLOCK_MONOTONIC, &endTime);
        controlInterval = (endTime.tv_sec-startTime.tv_sec)*usec_per_sec + (endTime.tv_nsec-startTime.tv_nsec)/nsec_per_usec;
        printf("CT: %4d\r", controlInterval);
        if (controlInterval >= cycletime) {
            printf("\nControl interval time exceeds defined cycle time\n");
            continue;
        }
        rdda_sleep(ecatSlaves, cycletime-controlInterval);
    }

    rddaStop(ecatSlaves);

    /* close file */
    if (fclose(fptr) == 0) {
        printf("Shared memory file pointer closed\n");
    }
    else {
        printf("Failed to close the file stream\n");
    }
}

int main(int argc, char **argv) {
    /* ctrl-c */
    signal(SIGINT, intHandler);

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
