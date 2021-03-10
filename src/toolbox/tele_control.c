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
    int num = 4;
    teleParam->sample_time = 0.5e-3;
    teleParam->inertia[0] = 1.078e-3;
    teleParam->inertia[1] = 1.078e-3;
    teleParam->inertia[2] = 0.2e-3; //1.463e-4;
    teleParam->inertia[3] = 0.2e-3; //1.463e-4;
    teleParam->resonant_frequency = 100; // rad/s
    teleParam->zeta = 0.5;

    /* symmetric stiffness */ /*
    for (int i = 0; i < num; i++) {
        teleParam->stiffness[i] = 10.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * 1.0e-3);
    } */

    /* asymmetric stiffness based on same resonant frequency*/
    for (int i = 0; i < num; i++) {
        teleParam->stiffness[i] = teleParam->resonant_frequency * teleParam->resonant_frequency * teleParam->inertia[i];
        teleParam->damping[i] = 2.0 * teleParam->zeta * teleParam->resonant_frequency * teleParam->inertia[i];
    }
    
    teleFirstOrderLowPassFilterParams->cutoff_frequency_LPF = 20.0; // position & velocity filter
    teleFirstOrderLowPassFilterParams->lambda = 2.0 * M_PI * teleFirstOrderLowPassFilterParams->cutoff_frequency_LPF;

    for (int i = 0; i < num; i++) {
        teleFilterVariable->pre_pos[i] = rdda->motor[i].motorIn.act_pos;
        teleFilterVariable->pre_filtered_pos[i] = rdda->motor[i].motorIn.act_pos;
        teleFilterVariable->pre_vel[i] = rdda->motor[i].motorIn.act_vel;
        teleFilterVariable->pre_filtered_vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    teleFirstOrderLowPassFilterParams->a1 = -1.0 * (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time - 2.0) / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);
    teleFirstOrderLowPassFilterParams->b0 = teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);
    teleFirstOrderLowPassFilterParams->b1 = teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time / (teleFirstOrderLowPassFilterParams->lambda * teleParam->sample_time + 2.0);

}


void teleController(TeleParam *teleParam, TeleFilterVariable *teleFilterVariable, TeleFirstOrderLowPassFilterParams *teleFirstOrderLowPassFilterParams, ControlParams *controlParams, Rdda *rdda) {
    int num = 4;
    double pos[num];
    double vel[num];
    double filtered_pos[num];
    double filtered_vel[num];

    /* pos & vel filtering */
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        filtered_pos[i] = teleFirstOrderIIRFilter(pos[i], teleFilterVariable->pre_pos[i], teleFilterVariable->pre_filtered_pos[i], teleFirstOrderLowPassFilterParams->b0, teleFirstOrderLowPassFilterParams->b1, teleFirstOrderLowPassFilterParams->a1);
        filtered_vel[i] = teleFirstOrderIIRFilter(vel[i], teleFilterVariable->pre_vel[i],  teleFilterVariable->pre_filtered_vel[i], teleFirstOrderLowPassFilterParams->b0, teleFirstOrderLowPassFilterParams->b1, teleFirstOrderLowPassFilterParams->a1);
    }

    /* virtual coupling */
    rdda->motor[2].motorOut.tau_off = teleParam->stiffness[2] * ((rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos) - (rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos)) + teleParam->damping[2] * (rdda->motor[0].motorIn.act_vel - rdda->motor[2].motorIn.act_vel);
    rdda->motor[3].motorOut.tau_off = teleParam->stiffness[3] * (-1.0 * (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos) - (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos)) + teleParam->damping[3] * (-1.0 * rdda->motor[1].motorIn.act_vel - rdda->motor[3].motorIn.act_vel);

    rdda->motor[0].motorOut.tau_off = teleParam->stiffness[0] * ((rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos) - (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos)) + teleParam->damping[0] * (rdda->motor[2].motorIn.act_vel - rdda->motor[0].motorIn.act_vel);
    rdda->motor[1].motorOut.tau_off = teleParam->stiffness[1] * (-1.0 * (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos) - (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos)) + teleParam->damping[1] * (-1.0 * rdda->motor[3].motorIn.act_vel - rdda->motor[1].motorIn.act_vel);

    /* DOB enabled */
    controlParams->external_force[0] = -1.0 * rdda->motor[2].motorOut.tau_off;
    controlParams->external_force[1] = rdda->motor[3].motorOut.tau_off;
    /* simple spring-damper connection */
    //rdda->motor[0].motorOut.tau_off = -1.0 * rdda->motor[2].motorOut.tau_off;
    //rdda->motor[1].motorOut.tau_off = rdda->motor[3].motorOut.tau_off;

    /* force & position scaling */
    //rdda->motor[2].motorOut.tau_off = link_stiffness * ((rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos) / r - (rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos)) + 2 * zeta * sqrt(link_stiffness * 1.0e-3) * (rdda->motor[0].motorIn.act_vel / r - rdda->motor[2].motorIn.act_vel);
    //rdda->motor[0].motorOut.tau_off = link_stiffness * ((rdda->motor[2].motorIn.act_pos - rdda->motor[2].init_pos) * r  - (rdda->motor[0].motorIn.act_pos - rdda->motor[0].init_pos)) + 2 * zeta * sqrt(link_stiffness * 1.0e-3) * (rdda->motor[2].motorIn.act_vel * r - rdda->motor[0].motorIn.act_vel);
    //rdda->motor[3].motorOut.tau_off = link_stiffness * (-1.0 * (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos) / r - (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos)) + 2 * zeta * sqrt(link_stiffness * 1.0e-3) * (-1.0 * rdda->motor[1].motorIn.act_vel / r - rdda->motor[3].motorIn.act_vel);
    //rdda->motor[1].motorOut.tau_off = link_stiffness * (-1.0 * (rdda->motor[3].motorIn.act_pos - rdda->motor[3].init_pos) * r  - (rdda->motor[1].motorIn.act_pos - rdda->motor[1].init_pos)) + 2 * zeta * sqrt(link_stiffness * 1.0e-3) * (-1.0 * rdda->motor[3].motorIn.act_vel * r - rdda->motor[1].motorIn.act_vel);

    /* previous variable update */
    for (int i = 0; i < num; i++) {
        teleFilterVariable->pre_pos[i] = pos[i];
        teleFilterVariable->pre_vel[i] = vel[i];
        teleFilterVariable->pre_filtered_pos[i] = filtered_pos[i];
        teleFilterVariable->pre_filtered_vel[i] = filtered_vel[i];
    }

}