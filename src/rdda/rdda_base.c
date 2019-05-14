/** rdda_base.c */

#include "rdda_base.h"

void delete_rdda_slave(ecat_slave *slave) {
    free(slave);
}

/** Close socket */
void rddaStop(ecat_slave *slave) {
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    delete_rdda_slave(slave);
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}

/** Add ns to timespec.
 *
 * @param ts  =   Structure holding an interval broken down into seconds and nanoseconds
 * @param addtime   =   Elapsed time interval added to previous time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec;
    int64 NSEC_PER_SEC = 1000000000;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;

    if (ts->tv_nsec > NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/** PI calculation to get linux time synced to DC time
 *
 * @param reftime   =   Reference DCtime.
 * @param cycletime    =   Cycle time of PDO transfer.
 * @param offsettime    =   Offset time.
 */
int64 ec_sync(int64 reftime, int64 cycletime) {
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    return -(delta/100)-(integral/20);
}

/** Get system time in microseconds (us)
 *
 * @param rddaSlave     =   rdda structure.
 * @return system time at nearest us.
 */
int rdda_gettime(ecat_slave *rddaSlave) {
    int64 nsec_per_sec = 1000000000;
    clock_gettime(CLOCK_MONOTONIC, &rddaSlave->ts);
    return (int)(rddaSlave->ts.tv_sec * nsec_per_sec + rddaSlave->ts.tv_nsec) / 1000 + 1;
}

/** Sleep and calibrate DC time.
 *
 * @param rddaSlave     =   rdda structure.
 * @param cycletime     =   sleep time.
 */
void rdda_sleep(ecat_slave *rddaSlave, int cycletime) {
    int64 cycletime_ns = cycletime * 1000;
    int toff = 0;
    if (ec_slave[0].hasdc) {
        toff = ec_sync(ec_DCtime, cycletime);
    }
    add_timespec(&rddaSlave->ts, cycletime_ns + toff);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rddaSlave->ts, NULL);
}


/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(ecat_slave *ecatSlave, RDDA_slave *rddaSlave) {
    ec_receive_processdata(EC_TIMEOUTRET);

    /* Update joint states */
    mutex_lock(&rddaSlave->mutex);

    for (int i = 0; i < 2; i++) {
        rddaSlave->motor[i].motorIn.act_pos = (double)(ecatSlave->bel[i].in_motor->act_pos/COUNTS_PER_RADIAN);
        rddaSlave->motor[i].motorIn.act_vel = (double)(ecatSlave->bel[i].in_motor->act_vel/COUNTS_PER_RADIAN/10.0);
    }
    rddaSlave->psensor.analogIn.val1 = (double)(ecatSlave->el3102.in_analog->val1 * PASCAL_PER_COUNT * NM_PER_PASCAL);
    rddaSlave->psensor.analogIn.val2 = (double)(ecatSlave->el3102.in_analog->val2 * PASCAL_PER_COUNT * NM_PER_PASCAL);

    mutex_unlock(&rddaSlave->mutex);

    ec_send_processdata();
}
