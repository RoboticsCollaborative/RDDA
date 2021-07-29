/**
 *  Functions and auxiliary tools for tele control
 **/

#include "tele_control.h"

double teleFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void teleInit(TeleParam *teleParam, TeleFilterVariable *teleFilterVariable, TeleFirstOrderLowPassFilterParams *teleFirstOrderLowPassFilterParams, Rdda *rdda) {
    /* parameter initialization */
    teleParam->num = 2;
    teleParam->sample_time = 0.5e-3;
/*    teleParam->inertia[0] = 1.078e-3;
    teleParam->inertia[1] = 1.078e-3;*/
    teleParam->inertia[0] = 0.2e-3; //1.463e-4;
    teleParam->inertia[1] = 0.2e-3; //1.463e-4;
    teleParam->resonant_frequency = 100; // rad/s
    teleParam->zeta = 0.12;

    /* symmetric stiffness */
    for (int i = 0; i < teleParam->num; i++) {
        teleParam->stiffness[i] = 10.0;//4.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * 1.1e-3);
    }

    /* asymmetric stiffness based on same resonant frequency*/ /*
    for (int i = 0; i < teleParam->num; i++) {
        teleParam->stiffness[i] = teleParam->resonant_frequency * teleParam->resonant_frequency * teleParam->inertia[i];
        teleParam->damping[i] = 2.0 * teleParam->zeta * teleParam->resonant_frequency * teleParam->inertia[i];
    } */

    teleFirstOrderLowPassFilterParams->cutoff_frequency_LPF = 20.0; // position & velocity filter
    teleFirstOrderLowPassFilterParams->lambda = 2.0 * M_PI * teleFirstOrderLowPassFilterParams->cutoff_frequency_LPF;

    for (int i = 0; i < teleParam->num; i++) {
        teleFilterVariable->pre_pos[i] = rdda->motor[i].motorIn.act_pos;
        teleFilterVariable->pre_filtered_pos[i] = rdda->motor[i].motorIn.act_pos;
        teleFilterVariable->pre_vel[i] = rdda->motor[i].motorIn.act_vel;
        teleFilterVariable->pre_filtered_vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    teleFilterVariable->pre_pressure[0] = rdda->psensor.analogIn.val1;
    teleFilterVariable->pre_filtered_pressure[0] = rdda->psensor.analogIn.val1;
    teleFilterVariable->pre_pressure[1] = rdda->psensor.analogIn.val2;
    teleFilterVariable->pre_filtered_pressure[1] = rdda->psensor.analogIn.val2;

    teleFirstOrderLowPassFilterParams->a1 = -1.0 * (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time - 2.0) / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);
    teleFirstOrderLowPassFilterParams->b0 = teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);
    teleFirstOrderLowPassFilterParams->b1 = teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);

}


void teleController(TeleParam *teleParam, TeleFilterVariable *teleFilterVariable, TeleFirstOrderLowPassFilterParams *teleFirstOrderLowPassFilterParams, ControlParams *controlParams, Rdda *rdda) {
    int num;
    num = teleParam->num;

    double pos[num];
    double vel[num];
    double pressure[num];
    double filtered_pos[num];
    double filtered_vel[num];
    double filtered_pressure[num];

    /* pos, vel & pressure filtering */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        filtered_pos[i] = teleFirstOrderIIRFilter(pos[i], teleFilterVariable->pre_pos[i], teleFilterVariable->pre_filtered_pos[i], teleFirstOrderLowPassFilterParams->b0, teleFirstOrderLowPassFilterParams->b1, teleFirstOrderLowPassFilterParams->a1);
        filtered_vel[i] = teleFirstOrderIIRFilter(vel[i], teleFilterVariable->pre_vel[i],  teleFilterVariable->pre_filtered_vel[i], teleFirstOrderLowPassFilterParams->b0, teleFirstOrderLowPassFilterParams->b1, teleFirstOrderLowPassFilterParams->a1);
        filtered_pressure[i] = teleFirstOrderIIRFilter(pressure[i], teleFilterVariable->pre_pressure[i], teleFilterVariable->pre_filtered_pressure[i], teleFirstOrderLowPassFilterParams->b0, teleFirstOrderLowPassFilterParams->b1, teleFirstOrderLowPassFilterParams->a1);
    }

    /* virtual coupling */
//    controlParams->external_force[2] = teleParam->stiffness[2] * ((rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos) - (rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos)) + teleParam->damping[2] * (rdda->motor[0].motorIn.act_vel - rdda->motor[2].motorIn.act_vel);
//    controlParams->external_force[3] = teleParam->stiffness[3] * (-1.0 * (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos) - (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos)) + teleParam->damping[3] * (-1.0 * rdda->motor[1].motorIn.act_vel - rdda->motor[3].motorIn.act_vel);
//    controlParams->external_force[0] = -1.0 * rdda->motor[2].motorOut.tau_off;
//    controlParams->external_force[1] = rdda->motor[3].motorOut.tau_off;
    controlParams->coupling_torque[0] = 0.0;
    controlParams->coupling_torque[1] = 0.0;


    /* previous variable update */
    for (int i = 0; i < num; i++) {
        teleFilterVariable->pre_pos[i] = pos[i];
        teleFilterVariable->pre_vel[i] = vel[i];
        teleFilterVariable->pre_pressure[i] = pressure[i];
        teleFilterVariable->pre_filtered_pos[i] = filtered_pos[i];
        teleFilterVariable->pre_filtered_vel[i] = filtered_vel[i];
        teleFilterVariable->pre_filtered_pressure[i] = filtered_pressure[i];
    }

}
