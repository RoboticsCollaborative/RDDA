/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

double firstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

double secondOrderIIRFilter(double input, double input_prev, double input_prev2, double output_prev, double output_prev2, double b0, double b1, double b2, double a1, double a2) {
    double output;
    output = b0 * input + b1 * input_prev + b2 * input_prev2 + a1 * output_prev + a2 * output_prev2;
    return output;
}

double trajectoryGenerator(double input, double pre_output, double max_vel, double dt) {
    double output;
    if (input - pre_output > max_vel * dt) {
        output = pre_output + max_vel * dt;
    }
    else if (input - pre_output < -max_vel * dt) {
        output = pre_output - max_vel * dt;
    }
    else {
        output = input;
    }
    return output;
}

void dobInit(ControlParams *controlParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda) {
    /* control parameters initialization */
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        controlParams->motor_inertia[i] = 1.463e-4;;
        controlParams->motor_damping[i] = 0.0;
        controlParams->Kp[i] = 0.0; // max stable value 60 (40) with zeta = 0.3 and max_velocity <= 5.0 when DOB turned off
        controlParams->Pp[i] = 0.0;
        controlParams->Vp[i] = 0.0;
        controlParams->max_external_torque[i] = 2.0; // ACD motor
        controlParams->coupling_torque[i] = 0.0;
        controlParams->Kf[i] = 1.0;
    }

    controlParams->cutoff_frequency_LPF[0] = 20.0; // Q for overall DOB
    controlParams->cutoff_frequency_LPF[1] = 10.0; // target position filter
    controlParams->lambda[0] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[0];
    controlParams->lambda[1] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[1];
    controlParams->zeta = 0.3;
    controlParams->max_inner_loop_torque_Nm = 0.5;
    controlParams->max_torque_Nm = 2.0; // max continuous torque limit
    controlParams->max_velocity = 10.0; 
    controlParams->max_stiffness = 40.0;
    controlParams->sample_time = 0.25e-3;
    controlParams->gear_ratio = 1.33;

    /* target position low-pass filter */
    secondOrderLowPassFilterParams->a1 = - (-8.0 + 2.0 * pow((controlParams->lambda[1] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->a2 = - (4.0 - 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b0 = pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b1 = 2.0 * pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b2 = pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));

    /* previous variables initialization */
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        previousVariables->pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
        previousVariables->prev_pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
        previousVariables->filtered_pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
        previousVariables->prev_filtered_pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
        previousVariables->stiffness[i] = rdda->motor[i].stiffness;
        previousVariables->prev_stiffness[i] = rdda->motor[i].stiffness;
        previousVariables->filtered_stiffness[i] = rdda->motor[i].stiffness;
        previousVariables->prev_filtered_stiffness[i] = rdda->motor[i].stiffness;
        previousVariables->vel_sat[i] = rdda->motor[i].vel_sat;
        previousVariables->prev_vel_sat[i] = rdda->motor[i].vel_sat;
        previousVariables->filtered_vel_sat[i] = rdda->motor[i].vel_sat;
        previousVariables->prev_filtered_vel_sat[i] = rdda->motor[i].vel_sat;
        previousVariables->tau_sat[i] = rdda->motor[i].tau_sat;
        previousVariables->prev_tau_sat[i] = rdda->motor[i].tau_sat;
        previousVariables->filtered_tau_sat[i] = rdda->motor[i].tau_sat;
        previousVariables->prev_filtered_tau_sat[i] = rdda->motor[i].tau_sat;
        previousVariables->pos_ref[i] = rdda->motor[i].rddaPacket.pos_in;
        previousVariables->integral_control_force[i] = 0.0;
        previousVariables->vel[i] = rdda->motor[i].motorIn.act_vel;
    }
}

void dobController(Rdda *rdda, ControlParams *controlParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables) {
    /* dob parameters */
    int num = MOTOR_COUNT;
    double motor_pos[num];
    double motor_vel[num];

    double pressure[num];
    double nominal_force_integration[num];
    //double nominal_force[num];

    double output_force[num];
    double coupling_torque[num];
    double integral_control_force[num];
    double saturated_feedback_force[num];

    double reference_force[num];
    double ff_force[num];

    double pos_tar[num];
    double filtered_pos_tar[num];
    double pos_ref[num];
    double vel_ref[num];

    double stiffness[num];
    double filtered_stiffness[num];

    double vel_sat[num];
    double filtered_vel_sat[num];

    double tau_sat[num];
    double filtered_tau_sat[num];

    /* position reference considering max velocity */
    for (int i = 0; i < num; i ++) {
        pos_tar[i] = rdda->motor[i].rddaPacket.pos_ref;
        filtered_pos_tar[i] = secondOrderIIRFilter(pos_tar[i], previousVariables->pos_tar[i], previousVariables->prev_pos_tar[i], previousVariables->filtered_pos_tar[i], previousVariables->prev_filtered_pos_tar[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        vel_sat[i] = rdda->motor[i].vel_sat;
        filtered_vel_sat[i] = secondOrderIIRFilter(vel_sat[i], previousVariables->vel_sat[i], previousVariables->prev_vel_sat[i], previousVariables->filtered_vel_sat[i], previousVariables->prev_filtered_vel_sat[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        filtered_vel_sat[i] = MIN(controlParams->max_velocity, filtered_vel_sat[i]);
        pos_ref[i] = trajectoryGenerator(filtered_pos_tar[i], previousVariables->pos_ref[i], filtered_vel_sat[i], controlParams->sample_time);
    }

    /* sensor reading */
    for (int i = 0; i < num; i ++) {
        motor_pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        motor_vel[i] = rdda->motor[i].motorIn.act_vel;
        pressure[i] = rdda->motor[i].motorIn.act_pre;
    }

    /* Stiffness reading */
    for (int i = 0; i < num; i ++) {
        stiffness[i] = rdda->motor[i].stiffness;
        filtered_stiffness[i] = secondOrderIIRFilter(stiffness[i], previousVariables->stiffness[i], previousVariables->prev_stiffness[i], previousVariables->filtered_stiffness[i], previousVariables->prev_filtered_stiffness[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        if (filtered_stiffness[i] < 0) {
            filtered_stiffness[i] = 0.0;
        }
        else {
            filtered_stiffness[i] = MIN(filtered_stiffness[i], controlParams->max_stiffness);
        }
    }

    /* PV gain calculation based on Kp */
    for (int i = 0; i < num; i ++) {
        controlParams->Vp[i] = 2 * controlParams->zeta * sqrt(filtered_stiffness[i] * controlParams->motor_inertia[i]);
        controlParams->Pp[i] = sqrt(filtered_stiffness[i] / controlParams->motor_inertia[i]) / (2 * controlParams->zeta);
    }

    /* PV controller */
    for (int i = 0; i < num; i ++) {
        vel_ref[i] = controlParams->Pp[i] * (pos_ref[i] - motor_pos[i]);
    }

    /* reference force */
    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->Vp[i] * (vel_ref[i] - motor_vel[i]) + rdda->motor[i].rddaPacket.tau_ref;
    }

    /* feedforward force */
    for (int i = 0; i < num; i ++) {
        coupling_torque[i] = saturation(controlParams->max_external_torque[i], controlParams->coupling_torque[i]);
        ff_force[i] = controlParams->Kf[i] * (pressure[i] + reference_force[i] + coupling_torque[i]);
    }

    /* disturbance observer */
    /* nominal force */
    for (int i = 0; i < num; i ++) {
        nominal_force_integration[i] = controlParams->lambda[0] * (controlParams->motor_inertia[i] * motor_vel[i] + controlParams->motor_damping[i] * motor_pos[i]);
    }

    /* output force */
    for (int i = 0; i < num; i ++) {
        /* direct equation */
        integral_control_force[i] = previousVariables->integral_control_force[i] + controlParams->lambda[0] * controlParams->sample_time * (reference_force[i] + pressure[i] + coupling_torque[i] + ff_force[i]);
        saturated_feedback_force[i] = saturation(controlParams->max_inner_loop_torque_Nm, integral_control_force[i] - nominal_force_integration[i]);
        integral_control_force[i] = saturated_feedback_force[i] + nominal_force_integration[i];
        output_force[i] = coupling_torque[i] + reference_force[i] + ff_force[i] + saturated_feedback_force[i];
    }

    /* motor output with torque saturation */
    for (int i = 0; i < num; i ++) {
        tau_sat[i] = rdda->motor[i].tau_sat;
        filtered_tau_sat[i] = secondOrderIIRFilter(tau_sat[i], previousVariables->tau_sat[i], previousVariables->prev_tau_sat[i], previousVariables->filtered_tau_sat[i], previousVariables->prev_filtered_tau_sat[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        if (filtered_tau_sat[i] < 0) {
            filtered_tau_sat[i] = 0.0;
        }
        else {
            filtered_tau_sat[i] = MIN(controlParams->max_torque_Nm, filtered_tau_sat[i]);
        }
        rdda->motor[i].motorOut.tau_off = saturation(filtered_tau_sat[i], output_force[i]);
    }

    /* previous variables update */
    for (int i = 0; i < num; i ++) {
        previousVariables->prev_pos_tar[i] = previousVariables->pos_tar[i];
        previousVariables->pos_tar[i] = pos_tar[i];
        previousVariables->prev_filtered_pos_tar[i] = previousVariables->filtered_pos_tar[i];
        previousVariables->filtered_pos_tar[i] = filtered_pos_tar[i];
        previousVariables->prev_stiffness[i] = previousVariables->stiffness[i];
        previousVariables->stiffness[i] = stiffness[i];
        previousVariables->prev_filtered_stiffness[i] = previousVariables->filtered_stiffness[i];
        previousVariables->filtered_stiffness[i] = filtered_stiffness[i];
        previousVariables->prev_vel_sat[i] = previousVariables->vel_sat[i];
        previousVariables->vel_sat[i] = vel_sat[i];
        previousVariables->prev_filtered_vel_sat[i] = previousVariables->filtered_vel_sat[i];
        previousVariables->filtered_vel_sat[i] = filtered_vel_sat[i];
        previousVariables->prev_tau_sat[i] = previousVariables->tau_sat[i];
        previousVariables->tau_sat[i] = tau_sat[i];
        previousVariables->prev_filtered_tau_sat[i] = previousVariables->filtered_tau_sat[i];
        previousVariables->filtered_tau_sat[i] = filtered_tau_sat[i];
        previousVariables->pos_ref[i] = pos_ref[i];
        previousVariables->integral_control_force[i] = integral_control_force[i];
        previousVariables->vel[i] = motor_vel[i];
    }

}