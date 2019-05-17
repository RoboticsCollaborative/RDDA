/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

void dobInit(ControlParams *controlParams, FilterParams *filterParams, PreviousVariables *previousVariables, Rdda *rdda) {
    /* control parameters initialization */
    controlParams->motor_inertia[0] = 1.1051e-3;
    controlParams->motor_inertia[1] = 1.1051e-3;
    controlParams->motor_damping[0] = 0.0;
    controlParams->motor_damping[1] = 0.0;
    controlParams->finger_damping[0] = 0.014289;
    controlParams->finger_damping[1] = 0.014289;
    controlParams->finger_stiffness[0] = 0.04494;
    controlParams->finger_stiffness[1] = 0.04494;
    controlParams->hydraulic_damping = 0.0092573;
    controlParams->hydraulic_stiffness = 12.76140;
    controlParams->cutoff_frequency[0] = 20;
    controlParams->cutoff_frequency[1] = 20;
    controlParams->cutoff_frequency[2] = 20;
    controlParams->pos_gain = 0.0;
    controlParams->vel_gain = 0.0;
    controlParams->acc_gain = 0.0;
    controlParams->pressure_offset = 0.04;
    controlParams->max_inner_loop_torque_Nm = 0.3;
    controlParams->max_torque_Nm = 5.0;
    controlParams->hysteresis_sigma = 400;
    controlParams->hysteresis_friction = 0.016;
    controlParams->gripper_angle_difference = 0.5;
    controlParams->sample_time = 0.5e-3;

    /* filter parameters initialization */
    for (int i = 0; i < 3; i ++) {
        filterParams->lambda[i] = 2.0 * M_PI * controlParams->cutoff_frequency[i];
        filterParams->a1[i] = -1.0 * (filterParams->lambda[i] * controlParams->sample_time - 2.0) / (filterParams->lambda[i] * controlParams->sample_time + 2.0);
        filterParams->b0[i] = filterParams->lambda[i] * controlParams->sample_time / (filterParams->lambda[i] * controlParams->sample_time + 2.0);
        filterParams->b1[i] = filterParams->lambda[i] * controlParams->sample_time / (filterParams->lambda[i] * controlParams->sample_time + 2.0);
    }

    /* previous variables initialization */
    previousVariables->psensor[0] = rdda->psensor.analogIn.val1 - controlParams->pressure_offset;
    previousVariables->psensor[1] = rdda->psensor.analogIn.val2 - controlParams->pressure_offset;
    for (int i = 0; i < 2; i ++) {
        previousVariables->motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        previousVariables->motor_vel[i] = rdda->motor[i].motorIn.act_vel;
        previousVariables->nominal_force[i] = 0.0;
        previousVariables->filtered_nominal_force[i] = 0.0;
        previousVariables->output_force[i] = 0.0;
        previousVariables->integral_output_force[i] = 0.0;
        previousVariables->filtered_output_force[i] = 0.0;
    }
}

double firstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void dobController(Rdda *rdda, ControlParams *controlParams, FilterParams *filterParams, PreviousVariables *previousVariables) {
    /* dob parameters */
    int num = 2;
    double motor_pos[num];
    double motor_vel[num];
    double motor_acc[num];
    double psensor[num];
    double nominal_force[num];
    double filtered_nominal_force[num];
    double output_force[num];
    double integral_output_force[num];
    double filtered_output_force[num];

    /* sensor reading */
    psensor[0] = rdda->psensor.analogIn.val1 - controlParams->pressure_offset;
    psensor[1] = rdda->psensor.analogIn.val2 - controlParams->pressure_offset;
    for (int i = 0; i < num; i ++) {
        motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        motor_vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    /* motor acceleration calculation */
    for (int i = 0; i < num; i ++) {
        motor_acc[i] = (motor_vel[i] - previousVariables->motor_vel[i]) / controlParams->sample_time;
    }

    /* disturbance observer */

    /* nominal force */
    for (int i = 0; i < num; i ++) {
        nominal_force[i] = controlParams->motor_inertia[i] * motor_acc[i] + controlParams->motor_damping[i] * motor_vel[i];
        filtered_nominal_force[i] = firstOrderIIRFilter(nominal_force[i], previousVariables->nominal_force[i], previousVariables->filtered_nominal_force[i], filterParams->b0[1], filterParams->b1[1], filterParams->a1[1]);
    }

    /* output force */
    for (int i = 0; i < num; i ++) {
        integral_output_force[i] = previousVariables->integral_output_force[i] + filterParams->lambda[0] * controlParams->sample_time * (psensor[i] - filtered_nominal_force[i]);
        output_force[i] = (psensor[i] - filtered_nominal_force[i]) + integral_output_force[i];
    }

    /* dob inner loop saturation */
    for (int i = 0; i < num; i ++) {
        filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], filterParams->b0[0], filterParams->b1[0], filterParams->a1[0]);
        if ((filtered_output_force[i] - filtered_nominal_force[i]) > controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = controlParams->max_inner_loop_torque_Nm + psensor[i];
            filtered_nominal_force[i] = firstOrderIIRFilter(nominal_force[i], previousVariables->nominal_force[i], previousVariables->filtered_nominal_force[i], filterParams->b0[0], filterParams->b1[0], filterParams->a1[0]);
        }
        else if ((filtered_output_force[i] - filtered_nominal_force[i]) < -1.0 * controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = -1.0 * controlParams->max_inner_loop_torque_Nm + psensor[i];
            filtered_nominal_force[i] = firstOrderIIRFilter(nominal_force[i], previousVariables->nominal_force[i], previousVariables->filtered_nominal_force[i], filterParams->b0[0], filterParams->b1[0], filterParams->a1[0]);
        }
    }

    /* previous variables update */
    for (int i = 0; i < num; i ++) {
        previousVariables->motor_pos[i] = motor_pos[i];
        previousVariables->motor_vel[i] = motor_vel[i];
        previousVariables->nominal_force[i] = nominal_force[i];
        previousVariables->filtered_nominal_force[i] = filtered_nominal_force[i];
        previousVariables->output_force[i] = output_force[i];
        previousVariables->integral_output_force[i] = integral_output_force[i];
        previousVariables->filtered_output_force[i] = filtered_output_force[i];
    }

    /* motor output with saturation */
    for (int i = 0; i < num; i ++) {
        rdda->motor[i].tau_max = controlParams->max_torque_Nm;
        rdda->motor[i].motorOut.tau_off = saturation(rdda->motor[i].tau_max, output_force[i]);
    }

}