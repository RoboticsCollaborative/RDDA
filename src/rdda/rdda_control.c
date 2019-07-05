/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

#define MIN(x,y) (x)<(y)?(x):(y)

void dobInit(ControlParams *controlParams, FirstOrderFilterParams *firstOrderFilterParams, SecondOrderFilterParams *secondOrderFilterParams, PreviousVariables *previousVariables, Rdda *rdda) {
    /* control parameters initialization */
    controlParams->motor_inertia[0] = 1.11e-3;//1.1144e-3;
    controlParams->motor_inertia[1] = 1.11e-3;//1.1144e-3;
    controlParams->motor_damping[0] = 0.0;
    controlParams->motor_damping[1] = 0.0;
    controlParams->finger_damping[0] = 1.0933e-2;//1.6933e-2;
    controlParams->finger_damping[1] = 1.0933e-2;//1.6933e-2;
    controlParams->finger_stiffness[0] = 0.0;//0.0235;
    controlParams->finger_stiffness[1] = 0.0;//0.0235;
    controlParams->hydraulic_damping = 0.009257;
    controlParams->hydraulic_stiffness = 13.0948;
    controlParams->cutoff_frequency[0] = 14; // Q_A for overall DOB
    controlParams->cutoff_frequency[1] = 20; // Q_B for nominal plant
    controlParams->cutoff_frequency[2] = 20; // Q_C for pressure
    controlParams->Kp[0] = 0.0;
    controlParams->Pp[0] = 0.0;
    controlParams->Vp[0] = 0.0;
    controlParams->Kp[1] = 0.0;
    controlParams->Pp[1] = 0.0;
    controlParams->Vp[1] = 0.0;
    //controlParams->pressure_offset = 0.04;
    controlParams->max_inner_loop_torque_Nm = 0.5;
    controlParams->max_torque_Nm = 5.0;
    controlParams->max_velocity = 5.0;
    controlParams->max_stiffness = 10.0;
    controlParams->hysteresis_sigma = 400;
    controlParams->hysteresis_friction = 0.016;
    //controlParams->gripper_angle_difference = 0.5;
    controlParams->sample_time = 0.5e-3;
    controlParams->gear_ratio = 1.33;

    /* filter parameters initialization */
    /* first order first initialization */
    for (int i = 0; i < 3; i ++) {
        firstOrderFilterParams->lambda[i] = 2.0 * M_PI * controlParams->cutoff_frequency[i];
        firstOrderFilterParams->a1[i] = -1.0 * (firstOrderFilterParams->lambda[i] * controlParams->sample_time - 2.0) / (firstOrderFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderFilterParams->b0[i] = firstOrderFilterParams->lambda[i] * controlParams->sample_time / (firstOrderFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderFilterParams->b1[i] = firstOrderFilterParams->lambda[i] * controlParams->sample_time / (firstOrderFilterParams->lambda[i] * controlParams->sample_time + 2.0);
    }
    firstOrderFilterParams->a1[3] = (2.0 * controlParams->hydraulic_damping - controlParams->hydraulic_stiffness * controlParams->sample_time) / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    firstOrderFilterParams->b0[3] = 2.0 / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    firstOrderFilterParams->b1[3] = -2.0 / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    /* second order filter initialization */
    secondOrderFilterParams->a1 = -1.0 * ((firstOrderFilterParams->lambda[2] * controlParams->sample_time - 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time) + (2.0 + firstOrderFilterParams->lambda[2] * controlParams->sample_time) * (controlParams->hydraulic_stiffness * controlParams->sample_time - 2.0 * controlParams->hydraulic_damping)) / ((firstOrderFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
    secondOrderFilterParams->a2 = -1.0 * (firstOrderFilterParams->lambda[2] * controlParams->sample_time - 2.0) * (controlParams->hydraulic_stiffness * controlParams->sample_time - 2.0 * controlParams->hydraulic_damping) / ((firstOrderFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
    for (int i = 0; i < 2; i ++) {
        secondOrderFilterParams->b0[i] = firstOrderFilterParams->lambda[2] * controlParams->sample_time * (2.0 * controlParams->finger_damping[i] + controlParams->finger_stiffness[i] * controlParams->sample_time) / ((firstOrderFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderFilterParams->b1[i] = firstOrderFilterParams->lambda[2] * controlParams->sample_time * 2.0 * controlParams->finger_stiffness[i] * controlParams->sample_time / ((firstOrderFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderFilterParams->b2[i] = firstOrderFilterParams->lambda[2] * controlParams->sample_time * (controlParams->finger_stiffness[i] * controlParams->sample_time - 2.0 * controlParams->finger_damping[i]) / ((firstOrderFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
    }

    /* previous variables initialization */
    previousVariables->pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->filtered_pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->prev_pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->prev_pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[0] = rdda->psensor.analogIn.val1 * controlParams->finger_stiffness[0] / controlParams->hydraulic_stiffness;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[1] = rdda->psensor.analogIn.val2 * controlParams->finger_stiffness[1] / controlParams->hydraulic_stiffness;
    previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[0] = rdda->psensor.analogIn.val1 * controlParams->finger_stiffness[0] / controlParams->hydraulic_stiffness;
    previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[1] = rdda->psensor.analogIn.val2 * controlParams->finger_stiffness[1] / controlParams->hydraulic_stiffness;

    for (int i = 0; i < 2; i ++) {
        previousVariables->motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        previousVariables->motor_vel[i] = rdda->motor[i].motorIn.act_vel;
        previousVariables->nominal_force[i] = 0.0;
        previousVariables->filtered_nominal_force[i] = 0.0;
        previousVariables->finger_bk_comp_force_position_part[i] = 0.0;
        previousVariables->filtered_finger_bk_comp_force_position_part[i] = 0.0;
        previousVariables->hysteresis_force[i] = 0.0;
        previousVariables->filtered_hysteresis_force[i] = 0.0;
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

void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderFilterParams *firstOrderFilterParams, SecondOrderFilterParams *secondOrderFilterParams, PreviousVariables *previousVariables) {
    /* dob parameters */
    int num = 2;
    double motor_pos[num];
    double motor_vel[num];
    double motor_acc[num];
    double finger_vel[num];
    double finger_vel_pressure_part[num];
    double pressure[num];
    double filtered_pressure[num];
    double nominal_force[num];
    double filtered_nominal_force[num];

    double finger_bk_comp_force_position_part[num];
    double filtered_finger_bk_comp_force_position_part[num];
    double filtered_finger_bk_comp_force_pressure_part[num];
    double filtered_finger_bk_comp_force[num];
    double hysteresis_force[num];

    double output_force[num];
    double integral_output_force[num];
    double filtered_output_force[num];

    double reference_force[num];

    double max_torque_Nm[num];

    double pos_ref[num];
    double vel_ref[num];

    /* position reference by ros */
    for (int i = 0; i < num; i ++) {
        pos_ref[i] = rdda->motor[i].rosOut.pos_ref;
    }

    /* sensor reading */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        motor_vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    /* pressure filtering */
    for (int i = 0; i < num; i ++) {
        filtered_pressure[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->filtered_pressure[i], firstOrderFilterParams->b0[0], firstOrderFilterParams->b1[0], firstOrderFilterParams->a1[0]);
    }

    /* motor acceleration calculation */
    for (int i = 0; i < num; i ++) {
        motor_acc[i] = (motor_vel[i] - previousVariables->motor_vel[i]) / controlParams->sample_time;
    }

    /* PD gain calculation based on endpoint impedance */
/*    for (int i = 0; i < num; i ++) {
        controlParams->pos_gain[i] = controlParams->hydraulic_stiffness * (rdda->motor[i].rosOut.stiffness - controlParams->finger_stiffness[i] * pow(controlParams->gear_ratio, 2)) / ((controlParams->hydraulic_stiffness + controlParams->finger_stiffness[i]) * pow(controlParams->gear_ratio, 2) - rdda->motor[i].rosOut.stiffness);
        if (controlParams->pos_gain[i] <= 10.0) {
            controlParams->vel_gain[i] = controlParams->pos_gain[i] / 100;
        }
        else controlParams->vel_gain[i] = 0.1;
    }*/

    /* PV gain calculation based on Kp */
    for (int i = 0; i < num; i ++) {
        if (rdda->motor[i].rosOut.stiffness < 0) {
            controlParams->Kp[i] = 0.0;
        }
        else {
            controlParams->Kp[i] = MIN(rdda->motor[i].rosOut.stiffness, controlParams->max_stiffness);
        }
        controlParams->Vp[i] = 0.6 * sqrt(controlParams->Kp[i] * controlParams->motor_inertia[i]);
        controlParams->Pp[i] = sqrt(controlParams->Kp[i] / controlParams->motor_inertia[i]) / 0.6;
    }

    /* PV controller */
    /* velocity reference calculation with saturation */
    for (int i = 0; i < num; i ++) {
        vel_ref[i] = saturation(saturation(controlParams->max_velocity, rdda->motor[i].rosOut.vel_sat), controlParams->Pp[i] * (pos_ref[i] - (motor_pos[i] - rdda->motor[i].init_pos)));
    }

    /* reference force */
    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->Vp[i] * (vel_ref[i] - motor_vel[i]);
    }

    /* impedance controller */
/*    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->pos_gain[i] * (pos_ref[i] - (motor_pos[i] - rdda->motor[i].init_pos)) + controlParams->vel_gain[i] * (vel_ref[i] - motor_vel[i]);
        filtered_reference_force[i] = firstOrderIIRFilter(reference_force[i], previousVariables->reference_force[i], previousVariables->filtered_reference_force[i], firstOrderFilterParams->b0[1], firstOrderFilterParams->b1[1], firstOrderFilterParams->a1[1]);
    }*/

    /* disturbance observer */

    /* nominal force */
    for (int i = 0; i < num; i ++) {
        nominal_force[i] = controlParams->motor_inertia[i] * motor_acc[i] + controlParams->motor_damping[i] * motor_vel[i];
        filtered_nominal_force[i] = firstOrderIIRFilter(nominal_force[i], previousVariables->nominal_force[i], previousVariables->filtered_nominal_force[i], firstOrderFilterParams->b0[1], firstOrderFilterParams->b1[1], firstOrderFilterParams->a1[1]);
    }

    /* finger damping and stiffness compensation */
    for (int i = 0; i < num; i ++) {
        /* position part*/
        finger_bk_comp_force_position_part[i] = controlParams->finger_damping[i] * motor_vel[i] + controlParams->finger_stiffness[i] * motor_pos[i];
        filtered_finger_bk_comp_force_position_part[i] = firstOrderIIRFilter(finger_bk_comp_force_position_part[i], previousVariables->finger_bk_comp_force_position_part[i], previousVariables->filtered_finger_bk_comp_force_position_part[i], firstOrderFilterParams->b0[2], firstOrderFilterParams->b1[2], firstOrderFilterParams->a1[2]);
        /* pressure part*/
        filtered_finger_bk_comp_force_pressure_part[i] = secondOrderFilterParams->b0[i] * pressure[i] + secondOrderFilterParams->b1[i] * previousVariables->pressure[i] + secondOrderFilterParams->b2[i] * previousVariables->prev_pressure[i] + secondOrderFilterParams->a1 * previousVariables->filtered_finger_bk_comp_force_pressure_part[i] + secondOrderFilterParams->a2 * previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[i];
        /* total */
        filtered_finger_bk_comp_force[i] = filtered_finger_bk_comp_force_position_part[i] + filtered_finger_bk_comp_force_pressure_part[i];
        filtered_finger_bk_comp_force[i] = 0.0;
    }

    /* hysteresis compensation */
    for (int i = 0; i < num; i ++) {
        /* finger velocity */
        finger_vel_pressure_part[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->finger_vel_pressure_part[i], firstOrderFilterParams->b0[3], firstOrderFilterParams->b1[3], firstOrderFilterParams->a1[3]);
        finger_vel[i] = finger_vel_pressure_part[i] + motor_vel[i];
        /* hysteresis force */
        hysteresis_force[i] = (previousVariables->hysteresis_force[i] + controlParams->sample_time * controlParams->hysteresis_sigma * finger_vel[i] * controlParams->hysteresis_friction) / (1.0 + controlParams->sample_time * controlParams->hysteresis_sigma * fabs(finger_vel[i]));
        hysteresis_force[i] = 0.0;
    }

    /* output force */
    for (int i = 0; i < num; i ++) {
        integral_output_force[i] = previousVariables->integral_output_force[i] + firstOrderFilterParams->lambda[0] * controlParams->sample_time * (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        output_force[i] = (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]) + integral_output_force[i];
        //output_force[i] = reference_force[i];
    }

    /* dob inner loop saturation */
    /*for (int i = 0; i < num; i ++) {
        filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderFilterParams->b0[0], firstOrderFilterParams->b1[0], firstOrderFilterParams->a1[0]);
        if ((filtered_output_force[i] + filtered_pressure[i] - filtered_nominal_force[i]) > controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = controlParams->max_inner_loop_torque_Nm + filtered_reference_force[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i];
            filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderFilterParams->b0[0], firstOrderFilterParams->b1[0], firstOrderFilterParams->a1[0]);
        }
        else if ((filtered_output_force[i] + filtered_pressure[i] - filtered_nominal_force[i]) < -1.0 * controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = -1.0 * controlParams->max_inner_loop_torque_Nm + filtered_reference_force[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i];
            filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderFilterParams->b0[0], firstOrderFilterParams->b1[0], firstOrderFilterParams->a1[0]);
        }
    }*/

    /* previous variables update */
    for (int i = 0; i < num; i ++) {
        previousVariables->motor_pos[i] = motor_pos[i];
        previousVariables->motor_vel[i] = motor_vel[i];
        previousVariables->finger_vel_pressure_part[i] = finger_vel_pressure_part[i];
        previousVariables->prev_pressure[i] = previousVariables->pressure[i];
        previousVariables->pressure[i] = pressure[i];
        previousVariables->filtered_pressure[i] = filtered_pressure[i];
        previousVariables->nominal_force[i] = nominal_force[i];
        previousVariables->filtered_nominal_force[i] = filtered_nominal_force[i];
        previousVariables->finger_bk_comp_force_position_part[i] = finger_bk_comp_force_position_part[i];
        previousVariables->filtered_finger_bk_comp_force_position_part[i] = filtered_finger_bk_comp_force_position_part[i];
        previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[i] = previousVariables->filtered_finger_bk_comp_force_pressure_part[i];
        previousVariables->filtered_finger_bk_comp_force_pressure_part[i] = filtered_finger_bk_comp_force_pressure_part[i];
        previousVariables->output_force[i] = output_force[i];
        previousVariables->integral_output_force[i] = integral_output_force[i];
        previousVariables->filtered_output_force[i] = filtered_output_force[i];
    }

    /* motor output with torque saturation */
    for (int i = 0; i < num; i ++) {
        if (rdda->motor[i].rosOut.tau_sat < 0) {
            max_torque_Nm[i] = 0.0;
        }
        else {
            max_torque_Nm[i] = MIN(controlParams->max_torque_Nm, rdda->motor[i].rosOut.tau_sat);
        }
        rdda->motor[i].tau_max = max_torque_Nm[i];
        if (output_force[i] > max_torque_Nm[i]) {
            previousVariables->integral_output_force[i] = max_torque_Nm[i] - (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        }
        else if (output_force[i] < -max_torque_Nm[i]) {
            previousVariables->integral_output_force[i] = -max_torque_Nm[i] - (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        }
        rdda->motor[i].motorOut.tau_off = saturation(rdda->motor[i].tau_max, output_force[i]);
    }

}