/** rdda_base.c */

#include "rdda_base.h"

/** Free EtherCAT slave memory.
 *
 * @param slave
 */
static void
delete_ecat_slave(ecat_slaves *slave) {
    free(slave);
}

/** Close socket */
void rddaStop(ecat_slaves *slave) {
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    delete_ecat_slave(slave);
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}

/** Sleep and calibrate DC time.
 *
 * @param rddaSlave     =   rdda structure.
 * @param cycletime     =   sleep time.
 */
void rdda_sleep(ecat_slaves *rdda, int cycletime) {
    int64 cycletime_ns = cycletime * 1000;
    int64 toff = 0;
    if (ec_slave[0].hasdc) {
        toff = ec_sync(ec_DCtime, cycletime);
    }
    add_timespec(&rdda->ts, cycletime_ns + toff);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rdda->ts, NULL);
}

/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(ecat_slaves *ecatSlaves, Rdda *rdda) {
    ec_receive_processdata(EC_TIMEOUTRET);
    //ec_send_processdata();
    mutex_lock(&rdda->mutex);

    /* Inputs */
    for (int i = 0; i < 2; i++) {
        rdda->motor[i].motorIn.act_pos = (double)(ecatSlaves->bel[i].in_motor->act_pos) / ecatSlaves->bel[i].counts_per_rad;
        rdda->motor[i].motorIn.act_vel = (double)(ecatSlaves->bel[i].in_motor->act_vel) / ecatSlaves->bel[i].counts_per_rad_sec;
    }
    rdda->psensor.analogIn.val1 = (double)(ecatSlaves->el3102.in_analog->val1) * ecatSlaves->bel[0].pascal_per_count * ecatSlaves->bel[0].nm_per_pascal;
    rdda->psensor.analogIn.val2 = (double)(ecatSlaves->el3102.in_analog->val2) * ecatSlaves->bel[1].pascal_per_count * ecatSlaves->bel[1].nm_per_pascal;

    /* Outputs */
    for (int j = 0; j < 2; j++) {
        ecatSlaves->bel[j].out_motor->ctrl_wd = 15;
        ecatSlaves->bel[j].out_motor->tg_pos = (int32)(rdda->motor[j].motorOut.tg_pos * ecatSlaves->bel[j].counts_per_rad);
        ecatSlaves->bel[j].out_motor->vel_off = (int32)(rdda->motor[j].motorOut.vel_off * ecatSlaves->bel[j].counts_per_rad_sec);
        ecatSlaves->bel[j].out_motor->tau_off = (int16)(rdda->motor[j].motorOut.tau_off * ecatSlaves->bel[j].units_per_nm);
    }

    mutex_unlock(&rdda->mutex);
    //ec_receive_processdata(EC_TIMEOUTRET);
    ec_send_processdata();
}

/** Get system time in microseconds (us)
 *
 * @param rddaSlave     =   rdda structure.
 * @return system time at nearest us.
 */
int rdda_gettime(ecat_slaves *ecatSlaves) {
    int64 nsec_per_sec = 1000000000;
    clock_gettime(CLOCK_MONOTONIC, &ecatSlaves->ts);
    return (int)(ecatSlaves->ts.tv_sec * nsec_per_sec + ecatSlaves->ts.tv_nsec) / 1000 + 1;
}

/** Torque saturation
 *
 * @param max_value    =   Maximum value.
 * @param raw_value    =   Raw value.
 * @return filtered value with max value.
 */
double saturation(double max_value, double raw_value) {
    if (raw_value > max_value)
        return max_value;
    else if (raw_value < (-1.0 * max_value))
        return (-1.0 * max_value);
    else
        return raw_value;
}

/** Let motor be static at launch time.
 *
 * @param ecatSlave     =   EtherCAT structure.
 * @param rddaSlave     =   RDDA structure (user-friendly).
 */
void initRddaStates(ecat_slaves *ecatSlaves, Rdda *rdda) {
    int32 initial_theta1_cnts[2];
    uint16  mot_id[2];

    /* Request initial data via SDO */
    for (int i = 0; i < 2; i++) {
        mot_id[i] = ecatSlaves->bel[i].slave_id;
        initial_theta1_cnts[i] = positionSDOread(mot_id[i]);
        /* Init motor position */
        rdda->motor[i].motorOut.tg_pos = (double)(initial_theta1_cnts[i]) / ecatSlaves->bel[i].counts_per_rad;
        rdda->motor[i].motorOut.tau_off = 0.0;
    }
}
