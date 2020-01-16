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

/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(ecat_slaves *ecatSlaves, Rdda *rdda) {

    double limit_int16 = 32767.0;
    double limit_int32 = 2147483647.0;
    int16 pressure_offset = 750;

    ec_receive_processdata(EC_TIMEOUTRET);
    //ec_send_processdata();

    /* Inputs */
    for (int i = 0; i < 2; i++) {
        rdda->motor[i].motorIn.act_pos = (double)(ecatSlaves->bel[i].in_motor->act_pos) / ecatSlaves->bel[i].counts_per_rad;
        rdda->motor[i].motorIn.act_vel = (double)(ecatSlaves->bel[i].in_motor->act_vel) / ecatSlaves->bel[i].counts_per_rad_sec;
        rdda->motor[i].motorIn.act_tau = (double)(ecatSlaves->bel[i].in_motor->act_tau) / ecatSlaves->bel[i].units_per_nm;
    }
    rdda->psensor.analogIn.val1 = (double)(ecatSlaves->el3102.in_analog->val1 - pressure_offset) * ecatSlaves->bel[0].pascal_per_count * ecatSlaves->bel[0].nm_per_pascal;
    rdda->psensor.analogIn.val2 = (double)(ecatSlaves->el3102.in_analog->val2 - pressure_offset) * ecatSlaves->bel[1].pascal_per_count * ecatSlaves->bel[1].nm_per_pascal;

    rdda->ts.nsec = ecatSlaves->ts.tv_nsec;
    rdda->ts.sec = ecatSlaves->ts.tv_sec;

    /* Outputs */
    ecatSlaves->bel[0].out_motor->ctrl_wd = 15;
    ecatSlaves->bel[1].out_motor->ctrl_wd = 15;
    for (int j = 0; j < 2; j++) {
        //ecatSlaves->bel[j].out_motor->ctrl_wd = 0;
        ecatSlaves->bel[j].out_motor->tg_pos = (int32)saturation(limit_int32, ecatSlaves->bel[j].init_pos_cnts + (int32)saturation(limit_int32, rdda->motor[j].motorOut.tg_pos * ecatSlaves->bel[j].counts_per_rad));
        ecatSlaves->bel[j].out_motor->vel_off = (int32)saturation(limit_int32, rdda->motor[j].motorOut.vel_off * ecatSlaves->bel[j].counts_per_rad_sec);
        ecatSlaves->bel[j].out_motor->tau_off = (int16)saturation(limit_int16, rdda->motor[j].motorOut.tau_off * ecatSlaves->bel[j].units_per_nm);
    }

    /* Timestamp */
    rdda->ts.sec = ecatSlaves->ts.tv_sec;
    rdda->ts.nsec = ecatSlaves->ts.tv_nsec;

    //ec_receive_processdata(EC_TIMEOUTRET);
    ec_send_processdata();
}

/** Sleep and calibrate DC time.
 *
 * @param rddaSlave     =   rdda structure.
 * @param cycletime     =   sleep time.
 */
void rdda_sleep(ecat_slaves *ecatSlaves, int cycletime) {
    int64 cycletime_ns = cycletime * 1000;
    int64 toff = 0;
    if (ec_slave[0].hasdc) {
        toff = ec_sync(ec_DCtime, cycletime);
    }
    add_timespec(&ecatSlaves->ts, cycletime_ns + toff);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ecatSlaves->ts, NULL);
}

/** Get system time in microseconds (us)
 *
 * @param ecatslaves     =   Ethercat structure.
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
    uint16  mot_id[2];

    /* Request initial data via SDO */
    for (int i = 0; i < 2; i++) {
        mot_id[i] = ecatSlaves->bel[i].slave_id;
        ecatSlaves->bel[i].init_pos_cnts = positionSDOread(mot_id[i]);
        rdda->motor[i].init_pos = (double)(ecatSlaves->bel[i].init_pos_cnts) / ecatSlaves->bel[i].counts_per_rad;
        /* Init motor position */
        rdda->motor[i].motorOut.tg_pos = 0.0;
        rdda->motor[i].motorOut.tau_off = 0.0;
        /* Init ROS inputs */
        rdda->motor[i].rosIn.contact_flag = 0;
        /* Init ROS outputs */
        rdda->motor[i].rosOut.pos_ref = 0.0;
        rdda->motor[i].rosOut.vel_sat = 5.0;
        rdda->motor[i].rosOut.tau_sat = 5.0;
        rdda->motor[i].rosOut.stiffness = 0.0;
    }
    rdda->freq_anti_alias = 500;
    rdda->ts.sec = rdda->ts.nsec = 0;
}
