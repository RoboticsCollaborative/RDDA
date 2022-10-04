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
    ec_close(); /* stop SOEM, close socket */
    printf("Socket closed\n");

}

/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(ecat_slaves *ecatSlaves, Rdda *rdda) {

    double limit_int16 = 32767.0;
    double limit_int32 = 2147483647.0;
    
    double pre_pressure = 360000; // unit pascal

    ec_receive_processdata(EC_TIMEOUTRET);

    /* Inputs */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        rdda->motor[i].motorIn.act_pos = (double)(ecatSlaves->aev[i].in_motor->act_pos) / ecatSlaves->aev[i].counts_per_rad;
        rdda->motor[i].motorIn.act_vel = (double)(ecatSlaves->aev[i].in_motor->act_vel) / ecatSlaves->aev[i].counts_per_rad_sec;
        rdda->motor[i].motorIn.act_tau = (double)(ecatSlaves->aev[i].in_motor->act_tau) / ecatSlaves->aev[i].units_per_nm;
        // rdda->motor[i].motorIn.load_pos = (double)(ecatSlaves->aev[i].in_motor->load_pos) / ecatSlaves->aev[i].load_counts_per_rad;
        // rdda->motor[i].motorIn.load_vel = (double)(ecatSlaves->aev[i].in_motor->load_vel) / ecatSlaves->aev[i].load_counts_per_rad_sec;
        rdda->motor[i].motorIn.act_pre = (double)(ecatSlaves->aev[i].in_motor->analog_in ) * ecatSlaves->aev[i].pascal_per_count * ecatSlaves->aev[i].nm_per_pascal - pre_pressure * ecatSlaves->aev[i].nm_per_pascal;
    }
    
    rdda->ts.nsec = ecatSlaves->ts.tv_nsec;
    rdda->ts.sec = ecatSlaves->ts.tv_sec;

    /* Outputs */
    // ecatSlaves->aev[3].out_motor->ctrl_wd = 0;//15;
    // ecatSlaves->aev[4].out_motor->ctrl_wd = 0;
    // ecatSlaves->aev[5].out_motor->ctrl_wd = 0;
    for (int j = 0; j < MOTOR_COUNT; j++) {
        ecatSlaves->aev[j].out_motor->ctrl_wd = 0;
        ecatSlaves->aev[j].out_motor->tg_pos = (int32)saturation(limit_int32, ecatSlaves->aev[j].init_pos_cnts + (int32)saturation(limit_int32, rdda->motor[j].motorOut.tg_pos * ecatSlaves->aev[j].counts_per_rad));
        ecatSlaves->aev[j].out_motor->vel_off = (int32)saturation(limit_int32, rdda->motor[j].motorOut.vel_off * ecatSlaves->aev[j].counts_per_rad_sec);
        ecatSlaves->aev[j].out_motor->tau_off = (int16)saturation(limit_int16, rdda->motor[j].motorOut.tau_off * ecatSlaves->aev[j].units_per_nm);
    }

    /* rddaPacket update */
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].rddaPacket.tau = rdda->motor[i].motorIn.act_tau;
        rdda->motor[i].rddaPacket.pos_out = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        rdda->motor[i].rddaPacket.vel_out = rdda->motor[i].motorIn.act_vel;
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

    uint16  mot_id[MOTOR_COUNT];

    /* Request initial data via SDO */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        mot_id[i] = ecatSlaves->aev[i].slave_id;
        ecatSlaves->aev[i].init_pos_cnts = positionSDOread(mot_id[i]);
        rdda->motor[i].init_pos = (double)(ecatSlaves->aev[i].init_pos_cnts) / ecatSlaves->aev[i].counts_per_rad;
        /* Init motor position */
        rdda->motor[i].motorOut.tg_pos = 0.0;
        rdda->motor[i].motorOut.tau_off = 0.0;
        /* Init ROS outputs */
        rdda->motor[i].rddaPacket.contact_flag = 0;
        rdda->motor[i].rddaPacket.wave_out = 0.0;
        rdda->motor[i].rddaPacket.wave_out_aux = 0.0;
        rdda->motor[i].rddaPacket.pos_out = 0.0;
        /* Init ROS inputs */
        rdda->motor[i].vel_sat = 10.0;
        rdda->motor[i].tau_sat = 5.0;
        rdda->motor[i].stiffness = 0.0;
        rdda->motor[i].rddaPacket.pos_in = 0.0;
        rdda->motor[i].rddaPacket.wave_in = 0.0;
        rdda->motor[i].rddaPacket.wave_in_aux = 0.0;
        rdda->motor[i].rddaPacket.pos_ref = 0.0;
        rdda->motor[i].rddaPacket.test = 0.0;
    }
    rdda->freq_anti_alias = 500;
    rdda->ts.sec = rdda->ts.nsec = 0;
    rdda->error_signal.error_in = 0;
    rdda->stamp.delay_cycle = 16;
}

/** Error Check
 *
 * @param ecatSlave     =   EtherCAT structure.
 */

int errorCheck(ecat_slaves *ecatSlaves) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0001) {
            printf("Fault: Data flash CRC failure on aev[%d]. This fault is considered fatal and cannot be cleared\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0002) {
            printf("Fault: Amplifier internal error on aev[%d].This fault is considered fatal and cannot be cleared\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0004) {
            printf("Fault: Short circuit on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0008) {
            printf("Fault: amplifier over temperature on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0010) {
            printf("Fault: Motor over temperature on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0020) {
            printf("Fault: Over voltage on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0040) {
            printf("Fault: Under voltage on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0080) {
            printf("Fault: Feedback fault on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0100) {
            printf("Fault: Phasing error on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0200) {
            printf("Fault: Tracking error on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0400) {
            printf("Fault: Over current on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x0800) {
            printf("Fault: FPGA failure case 1 on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x1000) {
            printf("Fault: Command input lost on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x2000) {
            printf("Fault: FPGA failure case 2 on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x4000) {
            printf("Fault: Safety circuit fault on aev[%d].\n", i);
            return 1;
        }
        else if (ecatSlaves->aev[i].in_motor->latching_fault == 0x8000) {
            printf("Fault: Unable to control current on aev[%d].\n", i);
            return 1;
        }
    }
    return 0;
}