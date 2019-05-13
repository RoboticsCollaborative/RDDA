#include "shm.h"
#include "shm_data.h"

int main () {

    /* Local variables */
    void *p;	/* Intermediate pointer */
    // int err; 	/* Error number */
//    int i;	 	/* Loop iterations */
//    double pos;
//    int ticket;

    /* Instanciate input-output data varibles */
    JointCommands *jointCommands;
    JointStates *jointStates;

    /* Map data structs to shared memory */
    /* Open and obtain shared memory pointers for master-input data */
    if (!openSharedMemory(SHARED_IN, &p)) {
        jointCommands = (JointCommands *) p;
    } else {
        fprintf(stderr, "Open(shared_in)\n");
        return -1;
    }

    /* Initialise ticket lock */
    mutex_init(&jointCommands->mutex);

    /* Open and obtain shared memory pointers for master-output data */
    if (!openSharedMemory(SHARED_OUT, &p)) {
        jointStates = (JointStates *) p;
    } else {
        fprintf(stderr, "Open(shared_out)\n");
        return -1;
    }

    /* Initialise ticket lock */
    mutex_init(&jointStates->mutex);

    /* Initialise input-output data */
//    shared_out->act_pos = (double)0.0;
//    shared_in->tg_pos = (double)0.0;

    /* Run in shared memory */
//    while(1) {

        /* Update master input */
        /* Request ticket lock */
//        ticket = ticket_lock(&shared_in->queue);
        /* Receive target position */
//        pos = shared_in->tg_pos;
//        printf("Receive target points: %lf\n", pos);
        /* Release ticket lock */
//        ticket_unlock(&shared_in->queue, ticket);

        /* Update master output */
        /* Request ticket lock */
//        ticket = ticket_lock(&shared_out->queue);
        /* Send actual position */
// //		pos = sin((double)(2*M_PI/10000 * i));
//        shared_out->act_pos = pos;
// //		printf("Receive target points: %lf\n", pos);
        /* Release ticket lock */
//        ticket_unlock(&shared_out->queue, ticket);


//        i++;
//        usleep(1000);
//    }

    return 0;
}