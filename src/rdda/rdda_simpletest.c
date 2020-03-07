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
#include "shm_data.h"
#include "shm.h"

volatile sig_atomic_t done = 0;
void intHandler (int sig) {
    if (sig == SIGINT) {
        done = 1;
    }
}

/* step function */
double stepFunction(double dmax, double dmin, double current_time)
{
    /* function parameters */
    double t0 = 2.0; // time to begin cycles
    double dtw = 4.0; // wait time
    double dto = 2.0; // open time
    double dth = 4.0; // hold time
    double dtc = 0.0; // close time

    double T = 0.0;
    T = dtw + dto + dth + dtc;

    double local_time = 0.0;

    if (current_time < t0) {
        return dmax;
    }
    else {
        local_time = fmod(current_time - t0, T);
        if(local_time < dtw) {
            return dmax;
        }
        else if(local_time < (dtw + dto)) {
            return (dmax - dmin) / 2.0 * cos((local_time - dtw) * M_PI / dto) + (dmax + dmin) / 2.0;
        }
        else if(local_time < (dtw + dto + dth)) {
            return dmin;
        }
        else {
            return -1.0 * (dmax - dmin) / 2.0 * cos((local_time - (dtw + dto + dth)) * M_PI / dtc) + (dmax + dmin) / 2.0;
        }
    }
}

void rdda_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;
    /* User friendly struct */
    Rdda *rdda;
    ControlParams controlParams;
    FirstOrderLowPassFilterParams firstOrderLowPassFilterParams;
    FirstOrderHighPassFilterParams firstOrderHighPassFilterParams;
    SecondOrderLowPassFilterParams secondOrderLowPassFilterParams;
    PreviousVariables previousVariables;

    ContactDetectionParams contactDetectionParams;
    ContactDetectionHighPassFilterParams contactDetectionHighPassFilterParams;
    ContactDetectionPreviousVariable contactDetectionPreviousVariable;
    int cycletime;
    //int start_time, end_time;
    //int delta_time;
    //int loopnum;
    double time = 0.0;

    /* create a data file */
    FILE *fptr;
    char filename[] = "/home/ethercat/rdda.dat";
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
    pivGainSDOwrite(ecatSlaves->bel[0].slave_id, 0, 0);
    pivGainSDOwrite(ecatSlaves->bel[1].slave_id, 0, 0);
    pivGainSDOwrite(ecatSlaves->bel[2].slave_id, 0, 0);
    /**/

    initRddaStates(ecatSlaves, rdda);
    dobInit(&controlParams, &firstOrderLowPassFilterParams, &firstOrderHighPassFilterParams, &secondOrderLowPassFilterParams, &previousVariables, rdda);
    contactDetectionInit(&contactDetectionParams, &contactDetectionHighPassFilterParams, &contactDetectionPreviousVariable, rdda);

    /* Measure time interval for sleep */
    int usec_per_sec = 1000000;
    int nsec_per_usec = 1000;
    struct timespec startTime, endTime;
    int controlInterval;
    rdda_gettime(ecatSlaves);
    /* Initialise timestamps */
    int i = 0;
    /* Gripper open and close test parameters */
    //double dmax[2];
    //double dmin[2];
    double stiffness = 0.5;
    //dmax[0] = rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos;
    //dmax[1] = rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos;
    //dmin[0] = dmax[0] - 0.6;
    //dmin[1] = dmax[1] - 0.6;
    /* PV controller dmax and dmin */
    //dmax[0] = rdda->motor[0].motorIn.act_pos;
    //dmax[1] = rdda->motor[1].motorIn.act_pos

    while (!done) {

        /* Mark start time */
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        time += 0.5e-3;

        mutex_lock(&rdda->mutex);

        /* Gripper open and close test */
        //rdda->motor[0].rosOut.pos_ref = stepFunction(dmax, dmin, time);
        //rdda->motor[1].rosOut.pos_ref = stepFunction(dmax, dmin, time);
        rdda->motor[0].rosOut.stiffness = stiffness;
        rdda->motor[1].rosOut.stiffness = stiffness;

        /* PV control target position */
        //rdda->motor[0].motorOut.tg_pos = stepFunction(dmax[0], dmin[0], time);
        //rdda->motor[1].motorOut.tg_pos = dmax[1];

        /* teleoperation */
        rdda->motor[0].rosOut.pos_ref = rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos;
        rdda->motor[2].motorOut.tau_off = -1.0 * previousVariables.current_reference_force[0] - 0.01 * rdda->motor[2].motorIn.act_vel;

        //contactDetection(&contactDetectionParams, &contactDetectionHighPassFilterParams, &contactDetectionPreviousVariable, rdda);
        dobController(rdda, &controlParams, &firstOrderLowPassFilterParams, &firstOrderHighPassFilterParams, &secondOrderLowPassFilterParams, &previousVariables);

        rdda_update(ecatSlaves, rdda);

        i++;
        //printf("tg_pos[0]: %+d, pos[0]: %+2.4lf, vel[0]: %+2.4lf, pre[0]: %+2.4lf, tau_off[0]: %+2.4lf, act_tau[0]: %+2.4lf, tg_pos[1]: %+d, pos[1]: %+2.4lf, vel[1]: %+2.4lf, pre[1]: %+2.4lf, tau_off[1]: %+2.4lf\r",
        //       ecatSlaves->bel[0].out_motor->tg_pos, rdda->motor[0].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->motor[0].motorOut.tau_off, rdda->motor[0].motorIn.act_tau,
        //       ecatSlaves->bel[1].out_motor->tg_pos, rdda->motor[1].motorIn.act_pos, rdda->motor[1].motorIn.act_vel, rdda->psensor.analogIn.val2, rdda->motor[1].motorOut.tau_off
        //);
        printf("act_tau: %+2.4lf, cpl_tau: %+2.4lf, ref_pos: %+2.4lf, sla_pos: %+2.4lf, act_pos_cnt: %+10d\r",
               (rdda->motor[0].motorIn.act_tau - rdda->motor[2].motorIn.act_tau)*100, previousVariables.current_reference_force[0], rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos, rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos, ecatSlaves->bel[1].in_motor->act_tau);

        /* save data to file */
        //fprintf(fptr, "%lf, %lf, %lf, %lf, %lf, %lf %lf\n", rdda->motor[0].motorIn.act_pos, rdda->motor[1].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->motor[1].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->psensor.analogIn.val2, time);

        mutex_unlock(&rdda->mutex);

        clock_gettime(CLOCK_MONOTONIC, &endTime);
        controlInterval = (endTime.tv_sec-startTime.tv_sec)*usec_per_sec + (endTime.tv_nsec-startTime.tv_nsec)/nsec_per_usec;

        rdda_sleep(ecatSlaves, cycletime-controlInterval);
    }

    rddaStop(ecatSlaves);

    /* close file */
    fclose(fptr);
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
