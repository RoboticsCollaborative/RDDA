/* rddamain.c */

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>

#include "ethercat.h"
#include "rdda/rddaconfig.h"
#include "rdda/rddauser.h"

//#define STACK_SIZE 128000
#define STACK_SIZE (64 * 1024)

//struct sched_param schedp;
volatile sig_atomic_t done = 0;

/** Sig-int handler function */
void intHandler(int sig)
{
    if (sig == SIGINT)
    {
        done = 1;
        printf("User requests termination.\n");
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, intHandler);

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-HAND Run\n");

    if (argc > 1)
    {
        int i;
        int err;
        pthread_t RTthread;
        pthread_t Configthread;
        pthread_t Checkthread;
        rdda_slavet *rdda_slave;

        /*
         *  EtherCAT networks configuration
         */
        /* Create a thread to configure EtherCAT networks */
        err = osal_thread_create(&Configthread, STACK_SIZE, (void *)rddaEcatConfig, (void *)argv[1]);
        if (err == 0)
        {
            fprintf(stderr, "pthread_create failed\n");
            exit(1);
        }

        /* Wait until network configuration is finished */
        err = pthread_join(Configthread, (void **)&rdda_slave);
        if (err != 0)
        {
            fprintf(stderr, "pthread_join failed\n");
            exit(1);
        }

        /*
         *  Realtime cyclic data sharing
         */
        /* Create RT thread for PDO transfer */
        err = osal_thread_create_rt(&RTthread, STACK_SIZE * 2, (void *)&rddaCyclic, (void *)&rdda_slave);
        if (err == 0)
        {
            fprintf(stderr, "pthread_creat_rt failed\n");
            exit(1);
        }

        /* Deploy Core-Iso to RTthread */
        cpu_set_t CPU3;
        CPU_ZERO(&CPU3);
        CPU_SET(3, &CPU3);
        pthread_setaffinity_np(RTthread, sizeof(CPU3), &CPU3);

        /*
         *  Create thread to handle slave error handling in OP
         */
        err = osal_thread_create(&Checkthread, STACK_SIZE * 4, &ecatcheck, NULL);
        if (err == 0)
        {
            fprintf(stderr, "pthread_creat failed\n");
            exit(1);
        }

        i = 0;
        while(!done || i <= 120000)
        {
            i++;
        }
        rddaStop();
    }
    else
    {
        printf("Usage: RDDA ifname1\nifname = enp1s0 for example\n");
    }

    return 0;
}

