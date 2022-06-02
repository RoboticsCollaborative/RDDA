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

void dobInit(ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda) {
    /* control parameters initialization */
    // controlParams->motor_inertia[0] = 1.0*1.11e-3;//1.1144e-3;
    // controlParams->motor_inertia[1] = 1.0*1.11e-3;//1.1144e-3;
    controlParams->motor_inertia[0] = 1.463e-4;//1.6*1.463e-4;//1.1144e-3;
    controlParams->motor_inertia[1] = 1.463e-4;//1.6*1.463e-4;//1.1144e-3;
    controlParams->motor_damping[0] = 0.0;
    controlParams->motor_damping[1] = 0.0;
    controlParams->finger_damping[0] = 1.0933e-2;//1.6933e-2;
    controlParams->finger_damping[1] = 1.0933e-2;//1.6933e-2;
    controlParams->finger_stiffness[0] = 0.0;//0.0235;
    controlParams->finger_stiffness[1] = 0.0;//0.0235;
    controlParams->hydraulic_damping = 0.009257;
    controlParams->hydraulic_stiffness = 13.0948;
    controlParams->cutoff_frequency_LPF[0] = 20.0; // Q for overall DOB
    controlParams->cutoff_frequency_LPF[1] = 10.0; // target position filter
    controlParams->lambda[0] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[0];
    controlParams->lambda[1] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[1];
    controlParams->Kp[0] = 0.0; // max stable value 60 (40) with zeta = 0.3 and max_velocity <= 5.0 when DOB turned off
    controlParams->Pp[0] = 0.0;
    controlParams->Vp[0] = 0.0;
    controlParams->Kp[1] = 0.0;
    controlParams->Pp[1] = 0.0;
    controlParams->Vp[1] = 0.0;
    controlParams->zeta = 0.5;
    controlParams->max_external_torque[0] = 2.0; // ACD motor
    controlParams->max_external_torque[1] = 2.0;
    controlParams->max_inner_loop_torque_Nm = 0.5;
    controlParams->max_torque_Nm = 2.0; // max continuous torque limit
    controlParams->max_velocity = 10.0; // stable for Kp = 20 and cutoff_frequency_LPF[0] = 14
    controlParams->max_stiffness = 40.0;
    controlParams->hysteresis_sigma = 400;
    controlParams->hysteresis_friction = 0.016;
    controlParams->sample_time = 0.25e-3;
    controlParams->gear_ratio = 1.33;
    controlParams->coupling_torque[0] = 0.0;
    controlParams->coupling_torque[1] = 0.0;
    controlParams->pos_ref[0] = 0.0;
    controlParams->pos_ref[1] = 0.0;

    /* friction compensation and hysteresis filter parameters initialization */
    for (int i = 0; i < 2; i++) {
        firstOrderLowPassFilterParams->friction_cmp_a1[i] = (2.0 * controlParams->hydraulic_damping - controlParams->hydraulic_stiffness * controlParams->sample_time) / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
        firstOrderLowPassFilterParams->friction_cmp_b0[i] = (controlParams->finger_stiffness[i] * controlParams->sample_time + 2.0 * controlParams->finger_damping[i]) / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
        firstOrderLowPassFilterParams->friction_cmp_b1[i] = (controlParams->finger_stiffness[i] * controlParams->sample_time - 2.0 * controlParams->finger_damping[i]) / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    }
    firstOrderLowPassFilterParams->hysteresis_a1 = firstOrderLowPassFilterParams->friction_cmp_a1[0];
    firstOrderLowPassFilterParams->hysteresis_b0 = 2.0 / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    firstOrderLowPassFilterParams->hysteresis_b1 = -1.0 * firstOrderLowPassFilterParams->hysteresis_b0;

    /* target position low-pass filter */
    secondOrderLowPassFilterParams->a1 = - (-8.0 + 2.0 * pow((controlParams->lambda[1] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->a2 = - (4.0 - 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b0 = pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b1 = 2.0 * pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b2 = pow((controlParams->lambda[1] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * controlParams->lambda[1] * controlParams->sample_time + pow((controlParams->lambda[1] * controlParams->sample_time), 2.0));

    /* previous variables initialization */
    previousVariables->pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[0] = rdda->psensor.analogIn.val1 * controlParams->finger_stiffness[0] / controlParams->hydraulic_stiffness;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[1] = rdda->psensor.analogIn.val2 * controlParams->finger_stiffness[1] / controlParams->hydraulic_stiffness;

    for (int i = 0; i < 2; i ++) {
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
        previousVariables->hysteresis_force[i] = 0.0;
        previousVariables->finger_vel_pressure_part[i] = 0.0;
        previousVariables->integral_control_force[i] = 0.0;
        previousVariables->vel[i] = rdda->motor[i].motorIn.act_vel;
    }
}

void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables) {
    /* dob parameters */
    int num = 2;
    double motor_pos[num];
    double motor_vel[num];
    //double motor_acc[num];

    double finger_vel[num];
    double finger_vel_pressure_part[num];

    double pressure[num];
    double nominal_force_integration[num];
    //double nominal_force[num];

    double finger_bk_comp_force_position_part[num];
    double filtered_finger_bk_comp_force_pressure_part[num];
    double finger_bk_comp_force[num];
    double hysteresis_force[num];

    double output_force[num];
    double coupling_torque[num];
    double integral_control_force[num];
    double saturated_feedback_force[num];

    double reference_force[num];

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
    
    // double Kp[num];
    // double Kd[num];

    /* position reference considering max velocity */
    for (int i = 0; i < num; i ++) {
        pos_tar[i] = rdda->motor[i].rddaPacket.pos_ref;
        // pos_tar[i] = controlParams->pos_ref[i];
        filtered_pos_tar[i] = secondOrderIIRFilter(pos_tar[i], previousVariables->pos_tar[i], previousVariables->prev_pos_tar[i], previousVariables->filtered_pos_tar[i], previousVariables->prev_filtered_pos_tar[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        //filtered_pos_tar[i] = secondOrderLowPassFilterParams->a1[2] * previousVariables->filtered_pos_tar[i] + secondOrderLowPassFilterParams->a2[2] * previousVariables->prev_filtered_pos_tar[i] + secondOrderLowPassFilterParams->b0[2] * pos_tar[i] + secondOrderLowPassFilterParams->b1[2] * previousVariables->pos_tar[i] + secondOrderLowPassFilterParams->b2[2] * previousVariables->prev_pos_tar[i];
        vel_sat[i] = rdda->motor[i].vel_sat;
        filtered_vel_sat[i] = secondOrderIIRFilter(vel_sat[i], previousVariables->vel_sat[i], previousVariables->prev_vel_sat[i], previousVariables->filtered_vel_sat[i], previousVariables->prev_filtered_vel_sat[i], secondOrderLowPassFilterParams->b0, secondOrderLowPassFilterParams->b1, secondOrderLowPassFilterParams->b2, secondOrderLowPassFilterParams->a1, secondOrderLowPassFilterParams->a2);
        filtered_vel_sat[i] = MIN(controlParams->max_velocity, filtered_vel_sat[i]);
        pos_ref[i] = trajectoryGenerator(filtered_pos_tar[i], previousVariables->pos_ref[i], filtered_vel_sat[i], controlParams->sample_time);
        // pos_ref[i] = rdda->motor[i].rddaPacket.pos_ref;
    }

    /* sensor reading */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        motor_pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        motor_vel[i] = rdda->motor[i].motorIn.act_vel;
        //motor_acc[i] = (rdda->motor[i].motorIn.act_vel - previousVariables->vel[i]) / controlParams->sample_time;
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
        //filtered_stiffness[i] = controlParams->Kp[i]; // max stiffness test
    }

    /* cutoff frequency update based on Kp */
    //if ((MAX(filtered_stiffness[0], filtered_stiffness[1])) < 28.0) {
    //    controlParams->cutoff_frequency_LPF[0] = 20.0 * (1.0 - (MAX(filtered_stiffness[0], filtered_stiffness[1])) / 28.0);
    //}
    //else {
    //    controlParams->cutoff_frequency_LPF[0] = 0.0;
    //}

    /* integral gain update*/
    controlParams->lambda[0] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[0];

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
        reference_force[i] = controlParams->Vp[i] * (vel_ref[i] - motor_vel[i]);
    }

    /* PD gain calculation based on endpoint impedance */
/*    for (int i = 0; i < num; i ++) {
        controlParams->pos_gain[i] = controlParams->hydraulic_stiffness * (rdda->motor[i].rosIn.stiffness - controlParams->finger_stiffness[i] * pow(controlParams->gear_ratio, 2)) / ((controlParams->hydraulic_stiffness + controlParams->finger_stiffness[i]) * pow(controlParams->gear_ratio, 2) - rdda->motor[i].rosIn.stiffness);
        if (controlParams->pos_gain[i] <= 10.0) {
            controlParams->vel_gain[i] = controlParams->pos_gain[i] / 100;
        }
        else controlParams->vel_gain[i] = 0.1;
    }*/

    /* impedance controller */
/*    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->pos_gain[i] * (pos_ref[i] - (motor_pos[i] - rdda->motor[i].init_pos)) + controlParams->vel_gain[i] * (vel_ref[i] - motor_vel[i]);
        filtered_reference_force[i] = firstOrderIIRFilter(reference_force[i], previousVariables->reference_force[i], previousVariables->filtered_reference_force[i], firstOrderLowPassFilterParams->b0[1], firstOrderLowPassFilterParams->b1[1], firstOrderLowPassFilterParams->a1[1]);
    }*/

    /* disturbance observer */
    /* nominal force */
    for (int i = 0; i < num; i ++) {
        nominal_force_integration[i] = controlParams->lambda[0] * (controlParams->motor_inertia[i] * motor_vel[i] + controlParams->motor_damping[i] * motor_pos[i]);
        //nominal_force[i] = controlParams->motor_inertia[i] * motor_acc[i] + controlParams->motor_damping[i] * motor_vel[i];
    }

    /* finger damping and stiffness compensation */
    for (int i = 0; i < num; i ++) {
        /* position part*/
        finger_bk_comp_force_position_part[i] = controlParams->finger_damping[i] * motor_vel[i] + controlParams->finger_stiffness[i] * motor_pos[i];
        /* pressure part*/
        filtered_finger_bk_comp_force_pressure_part[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->filtered_finger_bk_comp_force_pressure_part[i], firstOrderLowPassFilterParams->friction_cmp_b0[i], firstOrderLowPassFilterParams->friction_cmp_b1[i], firstOrderLowPassFilterParams->friction_cmp_a1[i]);
        /* total */
        finger_bk_comp_force[i] = finger_bk_comp_force_position_part[i] + filtered_finger_bk_comp_force_pressure_part[i];
        finger_bk_comp_force[i] = 0.0;
    }

    /* hysteresis compensation */
    for (int i = 0; i < num; i ++) {
        /* finger velocity */
        finger_vel_pressure_part[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->finger_vel_pressure_part[i], firstOrderLowPassFilterParams->hysteresis_b0, firstOrderLowPassFilterParams->hysteresis_b1, firstOrderLowPassFilterParams->hysteresis_a1);
        finger_vel[i] = finger_vel_pressure_part[i] + motor_vel[i];
        /* hysteresis force */
        hysteresis_force[i] = (previousVariables->hysteresis_force[i] + controlParams->sample_time * controlParams->hysteresis_sigma * finger_vel[i] * controlParams->hysteresis_friction) / (1.0 + controlParams->sample_time * controlParams->hysteresis_sigma * fabs(finger_vel[i]));
        hysteresis_force[i] = 0.0;
    }

    /* output force */
    for (int i = 0; i < num; i ++) {
        /* direct equation */
        coupling_torque[i] = saturation(controlParams->max_external_torque[i], controlParams->coupling_torque[i]);
        // Kp[i] = 1.0;
        // Kd[i] = MIN(2.0 * 0.5 * sqrt(Kp[i] * 1.463e-4), 0.08);
        // coupling_torque[i] = -1.0 * Kp[i] * motor_pos[i] - 1.0 * Kd[i] * motor_vel[i];
        integral_control_force[i] = previousVariables->integral_control_force[i] + controlParams->lambda[0] * controlParams->sample_time * (reference_force[i] + pressure[i] + finger_bk_comp_force[i] + hysteresis_force[i] + coupling_torque[i]);
        //output_force[i] = integral_control_force[i] + reference_force[i] + finger_bk_comp_force[i] + hysteresis_force[i];// + 0.5 * pressure[i];
        saturated_feedback_force[i] = saturation(controlParams->max_inner_loop_torque_Nm, integral_control_force[i] - nominal_force_integration[i]);
        integral_control_force[i] = saturated_feedback_force[i] + nominal_force_integration[i];
        output_force[i] = coupling_torque[i] + reference_force[i] + saturated_feedback_force[i];// + 0.5 * pressure[i];
    }
    // printf("%+2.4lf, %+2.4lf\r", integral_control_force[0], saturated_feedback_force[0]);

    /* Disable DOB on new motors */
    //output_force[0] = controlParams->coupling_torque[0];
    //output_force[1] = controlParams->coupling_torque[1];
    // printf("tau[0]: %+2.4lf, tau[1]: %+2.4lf,", output_force[0], output_force[1]);

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
        previousVariables->finger_vel_pressure_part[i] = finger_vel_pressure_part[i];
        previousVariables->pressure[i] = pressure[i];
        previousVariables->hysteresis_force[i] = hysteresis_force[i];
        previousVariables->filtered_finger_bk_comp_force_pressure_part[i] = filtered_finger_bk_comp_force_pressure_part[i];
        previousVariables->integral_control_force[i] = integral_control_force[i];
        previousVariables->vel[i] = motor_vel[i];
    }

}