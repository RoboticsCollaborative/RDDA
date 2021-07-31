/**
 *  Functions and auxiliary tools for tele control
 **/

#include "tele_control.h"

double teleFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void teleInit(TeleParam *teleParam) {
    /* parameter initialization */
    teleParam->num = 2;
    teleParam->sample_time = 0.5e-3;
    teleParam->inertia[0] = 1.078e-3;
    teleParam->inertia[1] = 1.078e-3;
//    teleParam->inertia[0] = 0.2e-3; //1.463e-4;
//    teleParam->inertia[1] = 0.2e-3; //1.463e-4;
    teleParam->resonant_frequency = 100; // rad/s
    teleParam->zeta = 0.12;
    teleParam->wave_damping = 0.01;
    teleParam->vel_tar[0] = 0.0;
    teleParam->vel_tar[1] = 0.0;
    teleParam->lambda = 1e-3;

    /* symmetric stiffness */
    for (int i = 0; i < teleParam->num; i ++) {
        teleParam->stiffness[i] = 10.0;//4.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * 1.1e-3);
    }

    /* asymmetric stiffness based on same resonant frequency*/ /*
    for (int i = 0; i < teleParam->num; i++) {
        teleParam->stiffness[i] = teleParam->resonant_frequency * teleParam->resonant_frequency * teleParam->inertia[i];
        teleParam->damping[i] = 2.0 * teleParam->zeta * teleParam->resonant_frequency * teleParam->inertia[i];
    } */

}


void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num;
    num = teleParam->num;

    double pos[num];
    double vel[num];
    double wave_input[num];

    double pos_tar[num];
    double vel_tar[num];

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
    }

    /* virtual coupling */
//    controlParams->external_force[2] = teleParam->stiffness[2] * ((rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos) - (rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos)) + teleParam->damping[2] * (rdda->motor[0].motorIn.act_vel - rdda->motor[2].motorIn.act_vel);
//    controlParams->external_force[3] = teleParam->stiffness[3] * (-1.0 * (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos) - (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos)) + teleParam->damping[3] * (-1.0 * rdda->motor[1].motorIn.act_vel - rdda->motor[3].motorIn.act_vel);
//    controlParams->external_force[0] = -1.0 * rdda->motor[2].motorOut.tau_off;
//    controlParams->external_force[1] = rdda->motor[3].motorOut.tau_off;
    //controlParams->coupling_torque[0] = 0.0;
    //controlParams->coupling_torque[1] = 0.0;

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        teleParam->pos_tar_int[i] += teleParam->vel_tar[i] * teleParam->sample_time;
        pos_tar[i] = teleParam->pos_tar_int[i] + teleParam->lambda * (rdda->motor[i].rddaPacket.pos_in - pos[i]);
        vel_tar[i] = (sqrt(2.0 * teleParam->wave_damping) * wave_input[i] + teleParam->damping[i] * vel[i] + teleParam->stiffness[i] * (pos[i] - pos_tar[i])) / (teleParam->damping[i] + teleParam->wave_damping);
        controlParams->coupling_torque[i] = teleParam->stiffness[i] * (pos_tar[i] - pos[i]) + teleParam->damping[i] * (vel_tar[i] - vel[i]);
        rdda->motor[i].rddaPacket.wave_out = rdda->motor[i].rddaPacket.wave_in - sqrt(2.0 / teleParam->wave_damping) * controlParams->coupling_torque[i];
        rdda->motor[i].rddaPacket.pos_out = pos[i];
    }
    //printf("tau[0]: %+2.4lf, tau[1]: %+2.4lf,", vel_tar[0], vel_tar[1]);

}
