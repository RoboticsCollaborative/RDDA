/**
 *  Functions and auxiliary tools for tele control
 **/

#include "rdda_homing.h"

void rddaHoming(ecat_slaves *ecatSlaves, Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables) {
    const double torque_lower_bound = 0.3;
    const double torque_upper_bound = 0.6;
    const double torque_collision = 0.3;
    const double pos_range = 0.8;
    const double pos_stop_error = 0.1;
    const double pos_origin_error = 0.2;
    const int cycle_time = 500;
    const double sample_time = 0.5e-3;
    const double homing_vel = 0.5;
    const double homing_stiffness = 10.0;
    const int num = 2;
    int done = 0;
    double act_torque[2];
    double act_pos[2];
    double pos_ref[2];
    double pos_lower_bound[2];
//    double pos_upper_bound[2];
//    double pos_origin[2];

    /* pre homing */
    rdda->motor[0].stiffness = homing_stiffness;
    rdda->motor[1].stiffness = homing_stiffness;

    /* find finger lower bound */
    for (int i = 0; i < num; i ++) {
        pos_ref[i] = rdda->motor[i].motorIn.act_pos;
        while (!done) {
            act_torque[i] = rdda->motor[i].motorIn.act_tau;
            if (fabs(act_torque[i]) > torque_lower_bound) {
                pos_lower_bound[i] = rdda->motor[i].motorIn.act_pos;
                done = 1;
            }
            else {
                pos_ref[i] += -1.0 * homing_vel * sample_time;
                controlParams->pos_ref[i] = pos_ref[i];
            }
            dobController(rdda, controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams, previousVariables);
            rdda_update(ecatSlaves, rdda);
            rdda_sleep(ecatSlaves, cycle_time);
        }
        done = 0;
    }

    /* find finger upper bound */
    for (int i = 0; i < num; i ++) {
        pos_ref[i] = rdda->motor[i].motorIn.act_pos;
        while (!done) {
            act_torque[i] = rdda->motor[i].motorIn.act_tau;
            act_pos[i] = rdda->motor[i].motorIn.act_pos;
            if (fabs(act_torque[i]) > torque_upper_bound || fabs(act_pos[i] - pos_lower_bound[i]) > pos_range) {
//                pos_upper_bound[i] = act_pos[i];
                done = 1;
            }
            else {
                pos_ref[i] += 1.0 * homing_vel * sample_time;
                controlParams->pos_ref[i] = pos_ref[i];
            }
            dobController(rdda, controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams, previousVariables);
            rdda_update(ecatSlaves, rdda);
            rdda_sleep(ecatSlaves, cycle_time);
        }
        done = 0;
        controlParams->pos_ref[i] = pos_lower_bound[i];
        while (!done) {
            act_pos[i] = rdda->motor[i].motorIn.act_pos;
            if (fabs(act_pos[i] - pos_lower_bound[i]) < pos_stop_error) {
                done = 1;
            }
            dobController(rdda, controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams, previousVariables);
            rdda_update(ecatSlaves, rdda);
            rdda_sleep(ecatSlaves, cycle_time);
        }
        done = 0;
    }

    /* find collision position as origin */
    pos_ref[0] = rdda->motor[0].motorIn.act_pos;
    pos_ref[1] = rdda->motor[1].motorIn.act_pos;
    while (!done) {
        for (int i = 0; i < num; i ++) {
            act_torque[i] = rdda->motor[i].motorIn.act_tau;
            act_pos[i] = rdda->motor[i].motorIn.act_pos;
            if (fabs(act_torque[i]) > torque_collision && fabs(act_pos[i] - pos_lower_bound[i]) > pos_origin_error) {
                done = 1;
            }
            else {
                pos_ref[i] += 1.0 * homing_vel * sample_time;
            }
        }
        dobController(rdda, controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams, previousVariables);
        rdda_update(ecatSlaves, rdda);
        rdda_sleep(ecatSlaves, cycle_time);
    }

    /* post homing */
    rdda->motor[0].init_pos = act_pos[0];
    rdda->motor[1].init_pos = act_pos[1];
    rdda->motor[0].stiffness = 0.0;
    rdda->motor[1].stiffness = 0.0;
    rdda_update(ecatSlaves, rdda);

}