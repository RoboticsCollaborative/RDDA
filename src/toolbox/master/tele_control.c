/**
 *  Functions and auxiliary tools for tele control
 **/

#include "tele_control.h"

double teleFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void matrixMuliply(double first[MOTOR_COUNT][MOTOR_COUNT], double second[MOTOR_COUNT], double result[MOTOR_COUNT]) {
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        result[i] = first[i][0] * second[0] + first[i][1] * second[1] + first[i][2] * second[2];
    }
}

void teleInit(TeleParam *teleParam) {
    /* parameter initialization */
    int num = MOTOR_COUNT;
    teleParam->sample_time = 0.25e-3;
    teleParam->zeta = 0.3;//0.12;
    teleParam->wave_damping = 0.04;

    /* symmetric stiffness */
    teleParam->stiffness[0] = 20.0;
    teleParam->stiffness[1] = 20.0;
    teleParam->stiffness[2] = 15.0;
    for (int i = 0; i < num; i ++) {
        // teleParam->stiffness[i] = 20.0;
        teleParam->motor_inertia[i] = 1.463e-4 / 2.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * teleParam->motor_inertia[i]);
    }

    teleParam->delay_cycle_previous = 4;

    for (int i = 0; i < num; i ++) {
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            teleParam->pred_input_int[i][j] = 0.0;
            teleParam->pred_force_int[i][j] = 0.0;
            for (int k = 0; k < MAX_BUFF; k ++) {
                teleParam->pred_input_history[i][j][k] = 0.0;
                teleParam->pred_force_history[i][j][k] = 0.0;
            }
        }
        teleParam->pred_vs[i] = 0.0;
        teleParam->pred_energy_reservoir[i] = 0.0;
        teleParam->energy_tdpa_local[i] = 0.0;
    }

    teleParam->current_timestamp = 0;

    // for round loop delay at 0.2s
    teleParam->eAT[0][0] = 0.999999999999978;
    teleParam->eAT[0][1] = 0.000000000000000;
    teleParam->eAT[0][2] = 0.001828750000000;
    teleParam->eAT[1][0] = 0.999999999999978;
    teleParam->eAT[1][1] = 0.000000000000000;
    teleParam->eAT[1][2] = 0.001828750000000;
    teleParam->eAT[2][0] = -0.000000000000076;
    teleParam->eAT[2][1] = 0.000000000000000;
    teleParam->eAT[2][2] = -0.000000000000000;

    teleParam->eAdT[0][0] = 0.940188978523100;
    teleParam->eAdT[0][1] = 0.059811021476900;
    teleParam->eAdT[0][2] = 0.000122030965721;
    teleParam->eAdT[1][0] = 0.004180358404841;
    teleParam->eAdT[1][1] = 0.995819641595159;
    teleParam->eAdT[1][2] = 0.000241653008674;
    teleParam->eAdT[2][0] = 32.705958428926962;
    teleParam->eAdT[2][1] = -32.705958428926969;
    teleParam->eAdT[2][2] = 0.933270832141819;

    teleParam->eAdTB[0] = 0.000362909019697e4;
    teleParam->eAdTB[1] = 0.000047184647095e4;
    teleParam->eAdTB[2] = 0.188214770465666e4;

    teleParam->eAdTG[0] = -0.000166822919645e4;
    teleParam->eAdTG[1] = -0.000330352711790e4;
    teleParam->eAdTG[2] = -1.275831622886970e4;

    teleParam->C[0] = -72.292821889918784;
    teleParam->C[1] = 72.292821889918784;
    teleParam->C[2] = 0.138257068694781;

    teleParam->D = 0.022374891789457;

}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num = MOTOR_COUNT;

    double vel[num];
    double wave_input[num];
    double wave_out[num];
    double tele_ratio = 1.2;
    double finger_damping = 0.0065;

    double delay_state[SLAVE_PLANT_STATE_NUM];
    double pred_state[SLAVE_PLANT_STATE_NUM];
    double temp_mm_in[SLAVE_PLANT_STATE_NUM];
    double temp_mm_out[SLAVE_PLANT_STATE_NUM];
    int delay_index;
    int delay_cycle_current = 800; // round loop 0.05s delay
    // double energy_reservoir_coeff = 1e2;

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
        rdda->motor[i].rddaPacket.wave_out_aux = wave_input[i];
    }

    // for (int i = 0; i < num; i ++) {
    //     teleParam->pred_energy_reservoir[i] += (wave_input[i] * wave_input[i] - teleParam->pred_vs[i] * teleParam->pred_vs[i]) * teleParam->sample_time;
    //     teleParam->pred_energy_reservoir[i] = MAX(teleParam->pred_energy_reservoir[i], 0.0);
    //     wave_input[i] = wave_input[i] + (1.0 - exp(-1.0 * energy_reservoir_coeff * teleParam->pred_energy_reservoir[i])) * (teleParam->pred_vs[i] - wave_input[i]);
    //     // if (teleParam->pred_vs[i] * teleParam->pred_vs[i] < wave_input[i] * wave_input[i]) {
    //     //     wave_input[i] = teleParam->pred_vs[i];
    //     // }
    //     rdda->motor[i].rddaPacket.wave_out_aux = wave_input[i];
    // }

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        // Disable coupling torque if tdpa is used instead of wave variable
        controlParams->coupling_torque[i] = -1.0 * (teleParam->wave_damping * vel[i] * tele_ratio + sqrt(2 * teleParam->wave_damping) * wave_input[i]);
        wave_out[i] = -1.0 * sqrt(2 * teleParam->wave_damping) * vel[i] * tele_ratio - wave_input[i];
        rdda->motor[i].rddaPacket.wave_out = wave_out[i];
    }

    delay_index = teleParam->current_timestamp -  delay_cycle_current;
    if (delay_index < 0) {
        delay_index += MAX_BUFF;
    }

    for (int i = 0; i < num; i ++) {
        delay_state[0] = rdda->motor[i].rddaPacket.pos_d_in;
        delay_state[1] = rdda->motor[i].rddaPacket.pos_in;
        delay_state[2] = rdda->motor[i].rddaPacket.vel_in;

        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            temp_mm_in[j] = teleParam->pred_input_int[i][j];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            teleParam->pred_input_int[i][j] = teleParam->eAdT[j][0] * temp_mm_in[0] + teleParam->eAdT[j][1] * temp_mm_in[1] + teleParam->eAdT[j][2] * temp_mm_in[2];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            temp_mm_in[j] = teleParam->pred_input_history[i][j][delay_index];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            temp_mm_out[j] = teleParam->eAT[j][0] * temp_mm_in[0] + teleParam->eAT[j][1] * temp_mm_in[1] + teleParam->eAT[j][2] * temp_mm_in[2];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            teleParam->pred_input_int[i][j] -= temp_mm_out[j] * teleParam->sample_time;
            teleParam->pred_input_int[i][j] += teleParam->eAdTB[j] * wave_out[i] * teleParam->sample_time;
            teleParam->pred_input_history[i][j][teleParam->current_timestamp] = teleParam->eAdTB[j] * wave_out[i];
        }

        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            temp_mm_in[j] = teleParam->pred_force_int[i][j];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            teleParam->pred_force_int[i][j] = teleParam->eAdT[j][0] * temp_mm_in[0] + teleParam->eAdT[j][1] * temp_mm_in[1] + teleParam->eAdT[j][2] * temp_mm_in[2];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            temp_mm_in[j] = teleParam->pred_force_history[i][j][delay_index];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            temp_mm_out[j] = teleParam->eAT[j][0] * temp_mm_in[0] + teleParam->eAT[j][1] * temp_mm_in[1] + teleParam->eAT[j][2] * temp_mm_in[2];
        }
        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++){
            teleParam->pred_force_int[i][j] -= temp_mm_out[j] * teleParam->sample_time;
            teleParam->pred_force_int[i][j] += teleParam->eAdTG[j] * (-1.0 * rdda->motor[i].rddaPacket.pre_in) * teleParam->sample_time;
            teleParam->pred_force_history[i][j][teleParam->current_timestamp] = teleParam->eAdTG[j] * (-1.0 * rdda->motor[i].rddaPacket.pre_in);
        }

        for (int j = 0; j < SLAVE_PLANT_STATE_NUM; j ++) {
            pred_state[j] = teleParam->eAT[j][0] * delay_state[0] + teleParam->eAT[j][1] * delay_state[1] + teleParam->eAT[j][2] * delay_state[2];
            pred_state[j] += teleParam->pred_input_int[i][j] + teleParam->pred_force_int[i][j];
        }

        teleParam->pred_vs[i] = teleParam->C[0] * pred_state[0] + teleParam->C[1] * pred_state[1] + teleParam->C[2] * pred_state[2]
                            + teleParam->D * wave_out[i];

        rdda->motor[i].rddaPacket.test = teleParam->pred_vs[i];
    }

    teleParam->current_timestamp++;
    if (teleParam->current_timestamp >= MAX_BUFF) {
        teleParam->current_timestamp = 0;
    }

    // varying delay energy reservoir
    for (int i = 0; i < num; i ++) {
        rdda->motor[i].rddaPacket.delay_energy_reservior += ((finger_damping + controlParams->motor_damping[i]) * vel[i] * vel[i]
        -0.5 * (rdda->ts.delay_cycle - teleParam->delay_cycle_previous) * wave_input[i] * wave_input[i]) * teleParam->sample_time;
        teleParam->delay_cycle_previous = rdda->ts.delay_cycle;
    }

    // // TDPA
    // double energy_change[num];
    // double coupling_torque[num];
    // double eps = 1e-6;
    // double energy_net_in[num];
    // for (int i = 0; i < num; i ++) {
    //     coupling_torque[i] = rdda->motor[i].rddaPacket.coupling_torque_in;
    //     energy_change[i] = coupling_torque[i] * vel[i] * teleParam->sample_time;;
        
    //     if (energy_change[i] > 0) rdda->motor[i].rddaPacket.energy_tdpa_out += energy_change[i];
    //     else if (energy_change[i] < 0) teleParam->energy_tdpa_local[i] -= energy_change[i];

    //     energy_net_in[i] = teleParam->energy_tdpa_local[i] - rdda->motor[i].rddaPacket.energy_tdpa_in;
    //     if (energy_net_in[i] > 0 && fabs(vel[i]) > eps) controlParams->coupling_torque[i] = -1.0 * (coupling_torque[i] + energy_net_in[i] / teleParam->sample_time / vel[i]);
    //     else controlParams->coupling_torque[i] = -1.0 * coupling_torque[i];

    //     rdda->motor[i].rddaPacket.pos_out = pos[i];
    //     rdda->motor[i].rddaPacket.vel_out = vel[i];
    // }

}
