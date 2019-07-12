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

void lowPassFilterParamsUpdate(ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams) {
    /* first order filter update */
    for (int i = 0; i < 4; i ++) {
        firstOrderLowPassFilterParams->lambda[i] = 2.0 * M_PI * controlParams->cutoff_frequency_LPF[i];
        firstOrderLowPassFilterParams->a1[i] = -1.0 * (firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time - 2.0) / (firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderLowPassFilterParams->b0[i] = firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time / (firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderLowPassFilterParams->b1[i] = firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time / (firstOrderLowPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
    }
    firstOrderLowPassFilterParams->a1[4] = (2.0 * controlParams->hydraulic_damping - controlParams->hydraulic_stiffness * controlParams->sample_time) / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    firstOrderLowPassFilterParams->b0[4] = 2.0 / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    firstOrderLowPassFilterParams->b1[4] = -2.0 / (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time);
    /* second order filter update */

    for (int i = 0; i < 2; i ++) {
        secondOrderLowPassFilterParams->a1[i] = -1.0 * ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time - 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time) + (2.0 + firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time) * (controlParams->hydraulic_stiffness * controlParams->sample_time - 2.0 * controlParams->hydraulic_damping)) / ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderLowPassFilterParams->a2[i] = -1.0 * (firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time - 2.0) * (controlParams->hydraulic_stiffness * controlParams->sample_time - 2.0 * controlParams->hydraulic_damping) / ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderLowPassFilterParams->b0[i] = firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time * (2.0 * controlParams->finger_damping[i] + controlParams->finger_stiffness[i] * controlParams->sample_time) / ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderLowPassFilterParams->b1[i] = firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time * 2.0 * controlParams->finger_stiffness[i] * controlParams->sample_time / ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
        secondOrderLowPassFilterParams->b2[i] = firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time * (controlParams->finger_stiffness[i] * controlParams->sample_time - 2.0 * controlParams->finger_damping[i]) / ((firstOrderLowPassFilterParams->lambda[2] * controlParams->sample_time + 2.0) * (2.0 * controlParams->hydraulic_damping + controlParams->hydraulic_stiffness * controlParams->sample_time));
    }
}

void dobInit(ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, FirstOrderHighPassFilterParams *firstOrderHighPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda) {
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
    controlParams->cutoff_frequency_LPF[0] = 20.0; // Q_A for overall DOB
    controlParams->cutoff_frequency_LPF[1] = controlParams->cutoff_frequency_LPF[0]; // Q_B for nominal plant
    controlParams->cutoff_frequency_LPF[2] = controlParams->cutoff_frequency_LPF[0]; // Q_C for finger damping compensation
    controlParams->cutoff_frequency_LPF[3] = controlParams->cutoff_frequency_LPF[0]; // Q_D for reference input
    controlParams->cutoff_frequency_LPF[4] = 1000.0; // target position filter
    controlParams->cutoff_frequency_HPF[0] = 0.1; // for pressure
    controlParams->cutoff_frequency_HPF[1] = 0.1; // for nominal plant
    controlParams->Kp[0] = 0.0; // max stable value 40 with zeta = 0.3 and max_velocity <= 5.0 when DOB turned off
    controlParams->Pp[0] = 0.0;
    controlParams->Vp[0] = 0.0;
    controlParams->Kp[1] = controlParams->Kp[0];
    controlParams->Pp[1] = 0.0;
    controlParams->Vp[1] = 0.0;
    controlParams->zeta = 0.3;
    controlParams->max_inner_loop_torque_Nm = 0.5;
    controlParams->max_output_torque_integral_part_Nm = 0.5;
    controlParams->max_torque_Nm = 5.0;
    controlParams->max_velocity = 10.0; // stable for Kp = 20 and cutoff_frequency_LPF[0] = 14
    controlParams->max_stiffness = 40.0;
    controlParams->hysteresis_sigma = 400;
    controlParams->hysteresis_friction = 0.016;
    controlParams->sample_time = 0.5e-3;
    controlParams->gear_ratio = 1.33;

    /* low-pass filter parameters initialization */
    lowPassFilterParamsUpdate(controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams);

    /* target position low-pass filter */
    firstOrderLowPassFilterParams->lambda[4] = 2 * M_PI * controlParams->cutoff_frequency_LPF[4];
    secondOrderLowPassFilterParams->a1[2] = - (8.0 + 2.0 * pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->a2[2] = - (4.0 - 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0)) / (4.0 + 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b0[2] = pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b1[2] = 2.0 * pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0));
    secondOrderLowPassFilterParams->b2[2] = pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0) / (4.0 + 4.0 * 0.707 * firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time + pow((firstOrderLowPassFilterParams->lambda[4] * controlParams->sample_time), 2.0));

    /* high-pass filter parameters initialization */
    for (int i = 0; i < 2; i ++) {
        firstOrderHighPassFilterParams->lambda[i] = 2.0 * M_PI * controlParams->cutoff_frequency_HPF[i];
        firstOrderHighPassFilterParams->a1[i] = -1.0 * (firstOrderHighPassFilterParams->lambda[i] * controlParams->sample_time - 2.0) / (firstOrderHighPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderHighPassFilterParams->b0[i] = 2.0 / (firstOrderHighPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
        firstOrderHighPassFilterParams->b1[i] = -2.0 / (firstOrderHighPassFilterParams->lambda[i] * controlParams->sample_time + 2.0);
    }

    /* previous variables initialization */
    previousVariables->pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->filtered_pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_pressure_HPF[0] = rdda->psensor.analogIn.val1;
    previousVariables->filtered_pressure_HPF[1] = rdda->psensor.analogIn.val2;
    previousVariables->prev_pressure[0] = rdda->psensor.analogIn.val1;
    previousVariables->prev_pressure[1] = rdda->psensor.analogIn.val2;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[0] = rdda->psensor.analogIn.val1 * controlParams->finger_stiffness[0] / controlParams->hydraulic_stiffness;
    previousVariables->filtered_finger_bk_comp_force_pressure_part[1] = rdda->psensor.analogIn.val2 * controlParams->finger_stiffness[1] / controlParams->hydraulic_stiffness;
    previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[0] = rdda->psensor.analogIn.val1 * controlParams->finger_stiffness[0] / controlParams->hydraulic_stiffness;
    previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[1] = rdda->psensor.analogIn.val2 * controlParams->finger_stiffness[1] / controlParams->hydraulic_stiffness;

    for (int i = 0; i < 2; i ++) {
        previousVariables->pos_tar[i] = rdda->motor[i].rosOut.pos_ref;
        previousVariables->prev_pos_tar[i] = rdda->motor[i].rosOut.pos_ref;
        previousVariables->filtered_pos_tar[i] = rdda->motor[i].rosOut.pos_ref;
        previousVariables->prev_filtered_pos_tar[i] = rdda->motor[i].rosOut.pos_ref;
        previousVariables->pos_ref[i] = rdda->motor[i].rosOut.pos_ref;
        previousVariables->motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        previousVariables->motor_vel[i] = rdda->motor[i].motorIn.act_vel;
        previousVariables->nominal_force[i] = 0.0;
        previousVariables->filtered_nominal_force[i] = 0.0;
        previousVariables->filtered_nominal_force_HPF[i] = 0.0;
        previousVariables->finger_bk_comp_force_position_part[i] = 0.0;
        previousVariables->filtered_finger_bk_comp_force_position_part[i] = 0.0;
        previousVariables->hysteresis_force[i] = 0.0;
        previousVariables->filtered_hysteresis_force[i] = 0.0;
        previousVariables->output_force[i] = 0.0;
        previousVariables->integral_output_force[i] = 0.0;
        previousVariables->filtered_output_force[i] = 0.0;
        previousVariables->reference_force[i] = 0.0;
        previousVariables->filtered_reference_force[i] = 0.0;
    }
}

void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, FirstOrderHighPassFilterParams *firstOrderHighPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables) {
    /* dob parameters */
    int num = 2;
    double motor_pos[num];
    double motor_vel[num];
    double motor_acc[num];
    double finger_vel[num];
    double finger_vel_pressure_part[num];

    double pressure[num];
    double filtered_pressure[num];
    double filtered_pressure_HPF[num];
    double nominal_force[num];
    double filtered_nominal_force[num];
    double filtered_nominal_force_HPF[num];

    double finger_bk_comp_force_position_part[num];
    double filtered_finger_bk_comp_force_position_part[num];
    double filtered_finger_bk_comp_force_pressure_part[num];
    double filtered_finger_bk_comp_force[num];
    double hysteresis_force[num];

    double output_force[num];
    double integral_output_force[num];
    double filtered_output_force[num];

    double reference_force[num];
    double filtered_reference_force[num];

    double max_torque_Nm[num];

    double pos_tar[num];
    double filtered_pos_tar[num];
    double pos_ref[num];
    double vel_ref[num];

    /* position reference considering max velocity */
    for (int i = 0; i < num; i ++) {
        pos_tar[i] = rdda->motor[i].rosOut.pos_ref;
        filtered_pos_tar[i] = secondOrderLowPassFilterParams->a1[2] * previousVariables->filtered_pos_tar[i] + secondOrderLowPassFilterParams->a2[2] * previousVariables->prev_filtered_pos_tar[i] + secondOrderLowPassFilterParams->b0[2] * pos_tar[i] + secondOrderLowPassFilterParams->b1[2] * previousVariables->pos_tar[i] + secondOrderLowPassFilterParams->b2[2] * previousVariables->prev_pos_tar[i];
        pos_ref[i] = trajectoryGenerator(filtered_pos_tar[i], previousVariables->pos_ref[i], controlParams->max_velocity, controlParams->sample_time);
    }

    /* sensor reading */
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        motor_pos[i] = rdda->motor[i].motorIn.act_pos;
        motor_vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    /* Stiffness reading */
    for (int i = 0; i < num; i ++) {
        if (rdda->motor[i].rosOut.stiffness < 0) {
            controlParams->Kp[i] = 0.0;
        }
        else {
            controlParams->Kp[i] = MIN(rdda->motor[i].rosOut.stiffness, controlParams->max_stiffness);
        }
    }

    /* cutoff frequency update based on Kp */
    for (int i = 0; i < 4; i ++) {
        if ((MAX(controlParams->Kp[0], controlParams->Kp[1])) < 28.0) {
            controlParams->cutoff_frequency_LPF[i] = 20.0 * (1.0 - (MAX(controlParams->Kp[0], controlParams->Kp[1])) / 28.0);
        }
        else {
            controlParams->cutoff_frequency_LPF[i] = 0.0;
        }
    }

    /* low-pass filter parameters update*/
    lowPassFilterParamsUpdate(controlParams, firstOrderLowPassFilterParams, secondOrderLowPassFilterParams);

    /* PV gain calculation based on Kp */
    for (int i = 0; i < num; i ++) {
        controlParams->Vp[i] = 2 * controlParams->zeta * sqrt(controlParams->Kp[i] * controlParams->motor_inertia[i]);
        controlParams->Pp[i] = sqrt(controlParams->Kp[i] / controlParams->motor_inertia[i]) / (2 * controlParams->zeta);
    }

    /* PV controller */
    for (int i = 0; i < num; i ++) {
        vel_ref[i] = controlParams->Pp[i] * (pos_ref[i] - (motor_pos[i] - rdda->motor[i].init_pos));
    }

    /* reference force */
    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->Vp[i] * (vel_ref[i] - motor_vel[i]);
        filtered_reference_force[i] = firstOrderIIRFilter(reference_force[i], previousVariables->reference_force[i], previousVariables->filtered_reference_force[i], firstOrderLowPassFilterParams->b0[3], firstOrderLowPassFilterParams->b1[3], firstOrderLowPassFilterParams->a1[3]);
    }

    /* pressure filtering */
    for (int i = 0; i < num; i ++) {
        filtered_pressure[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->filtered_pressure[i], firstOrderLowPassFilterParams->b0[0], firstOrderLowPassFilterParams->b1[0], firstOrderLowPassFilterParams->a1[0]);
        filtered_pressure_HPF[i] = firstOrderIIRFilter(filtered_pressure[i], previousVariables->filtered_pressure[i], previousVariables->filtered_pressure_HPF[i], firstOrderHighPassFilterParams->b0[0], firstOrderHighPassFilterParams->b1[0], firstOrderHighPassFilterParams->a1[0]);
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

    /* impedance controller */
/*    for (int i = 0; i < num; i ++) {
        reference_force[i] = controlParams->pos_gain[i] * (pos_ref[i] - (motor_pos[i] - rdda->motor[i].init_pos)) + controlParams->vel_gain[i] * (vel_ref[i] - motor_vel[i]);
        filtered_reference_force[i] = firstOrderIIRFilter(reference_force[i], previousVariables->reference_force[i], previousVariables->filtered_reference_force[i], firstOrderLowPassFilterParams->b0[1], firstOrderLowPassFilterParams->b1[1], firstOrderLowPassFilterParams->a1[1]);
    }*/

    /* disturbance observer */

    /* nominal force */
    for (int i = 0; i < num; i ++) {
        nominal_force[i] = controlParams->motor_inertia[i] * motor_acc[i] + controlParams->motor_damping[i] * motor_vel[i];
        filtered_nominal_force[i] = firstOrderIIRFilter(nominal_force[i], previousVariables->nominal_force[i], previousVariables->filtered_nominal_force[i], firstOrderLowPassFilterParams->b0[1], firstOrderLowPassFilterParams->b1[1], firstOrderLowPassFilterParams->a1[1]);
        filtered_nominal_force_HPF[i] = firstOrderIIRFilter(filtered_nominal_force[i], previousVariables->filtered_nominal_force[i], previousVariables->filtered_nominal_force_HPF[i], firstOrderHighPassFilterParams->b0[1], firstOrderHighPassFilterParams->b1[1], firstOrderHighPassFilterParams->a1[1]);
    }

    /* finger damping and stiffness compensation */
    for (int i = 0; i < num; i ++) {
        /* position part*/
        finger_bk_comp_force_position_part[i] = controlParams->finger_damping[i] * motor_vel[i] + controlParams->finger_stiffness[i] * motor_pos[i];
        filtered_finger_bk_comp_force_position_part[i] = firstOrderIIRFilter(finger_bk_comp_force_position_part[i], previousVariables->finger_bk_comp_force_position_part[i], previousVariables->filtered_finger_bk_comp_force_position_part[i], firstOrderLowPassFilterParams->b0[2], firstOrderLowPassFilterParams->b1[2], firstOrderLowPassFilterParams->a1[2]);
        /* pressure part*/
        filtered_finger_bk_comp_force_pressure_part[i] = secondOrderLowPassFilterParams->b0[i] * pressure[i] + secondOrderLowPassFilterParams->b1[i] * previousVariables->pressure[i] + secondOrderLowPassFilterParams->b2[i] * previousVariables->prev_pressure[i] + secondOrderLowPassFilterParams->a1[i] * previousVariables->filtered_finger_bk_comp_force_pressure_part[i] + secondOrderLowPassFilterParams->a2[i] * previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[i];
        /* total */
        filtered_finger_bk_comp_force[i] = filtered_finger_bk_comp_force_position_part[i] + filtered_finger_bk_comp_force_pressure_part[i];
        filtered_finger_bk_comp_force[i] = 0.0;
    }

    /* hysteresis compensation */
    for (int i = 0; i < num; i ++) {
        /* finger velocity */
        finger_vel_pressure_part[i] = firstOrderIIRFilter(pressure[i], previousVariables->pressure[i], previousVariables->finger_vel_pressure_part[i], firstOrderLowPassFilterParams->b0[4], firstOrderLowPassFilterParams->b1[4], firstOrderLowPassFilterParams->a1[4]);
        finger_vel[i] = finger_vel_pressure_part[i] + motor_vel[i];
        /* hysteresis force */
        hysteresis_force[i] = (previousVariables->hysteresis_force[i] + controlParams->sample_time * controlParams->hysteresis_sigma * finger_vel[i] * controlParams->hysteresis_friction) / (1.0 + controlParams->sample_time * controlParams->hysteresis_sigma * fabs(finger_vel[i]));
        hysteresis_force[i] = 0.0;
    }

    /* output force */
    for (int i = 0; i < num; i ++) {
        integral_output_force[i] = previousVariables->integral_output_force[i] + firstOrderLowPassFilterParams->lambda[0] * controlParams->sample_time * (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        integral_output_force[i] = saturation(controlParams->max_output_torque_integral_part_Nm, integral_output_force[i]);
        output_force[i] = (reference_force[i] + filtered_pressure[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]) + integral_output_force[i];
    }

    /* dob inner loop saturation */
    /*for (int i = 0; i < num; i ++) {
        filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderLowPassFilterParams->b0[0], firstOrderLowPassFilterParams->b1[0], firstOrderLowPassFilterParams->a1[0]);
        if ((filtered_output_force[i] + filtered_pressure[i] - filtered_nominal_force[i]) > controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = controlParams->max_inner_loop_torque_Nm + filtered_reference_force[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i];
            filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderLowPassFilterParams->b0[0], firstOrderLowPassFilterParams->b1[0], firstOrderLowPassFilterParams->a1[0]);
        }
        else if ((filtered_output_force[i] + filtered_pressure[i] - filtered_nominal_force[i]) < -1.0 * controlParams->max_inner_loop_torque_Nm) {
            output_force[i] = -1.0 * controlParams->max_inner_loop_torque_Nm + filtered_reference_force[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i];
            filtered_output_force[i] = firstOrderIIRFilter(output_force[i], previousVariables->output_force[i], previousVariables->filtered_output_force[i], firstOrderLowPassFilterParams->b0[0], firstOrderLowPassFilterParams->b1[0], firstOrderLowPassFilterParams->a1[0]);
        }
    }*/

    /* previous variables update */
    for (int i = 0; i < num; i ++) {
        previousVariables->prev_pos_tar[i] = previousVariables->pos_tar[i];
        previousVariables->pos_tar[i] = pos_tar[i];
        previousVariables->prev_filtered_pos_tar[i] = previousVariables->filtered_pos_tar[i];
        previousVariables->filtered_pos_tar[i] = filtered_pos_tar[i];
        previousVariables->pos_ref[i] = pos_ref[i];
        previousVariables->motor_pos[i] = motor_pos[i];
        previousVariables->motor_vel[i] = motor_vel[i];
        previousVariables->finger_vel_pressure_part[i] = finger_vel_pressure_part[i];
        previousVariables->prev_pressure[i] = previousVariables->pressure[i];
        previousVariables->pressure[i] = pressure[i];
        previousVariables->filtered_pressure[i] = filtered_pressure[i];
        previousVariables->filtered_pressure_HPF[i] = filtered_pressure_HPF[i];
        previousVariables->nominal_force[i] = nominal_force[i];
        previousVariables->filtered_nominal_force[i] = filtered_nominal_force[i];
        previousVariables->filtered_nominal_force_HPF[i] = filtered_nominal_force_HPF[i];
        previousVariables->finger_bk_comp_force_position_part[i] = finger_bk_comp_force_position_part[i];
        previousVariables->filtered_finger_bk_comp_force_position_part[i] = filtered_finger_bk_comp_force_position_part[i];
        previousVariables->prev_filtered_finger_bk_comp_force_pressure_part[i] = previousVariables->filtered_finger_bk_comp_force_pressure_part[i];
        previousVariables->filtered_finger_bk_comp_force_pressure_part[i] = filtered_finger_bk_comp_force_pressure_part[i];
        previousVariables->output_force[i] = output_force[i];
        previousVariables->integral_output_force[i] = integral_output_force[i];
        previousVariables->filtered_output_force[i] = filtered_output_force[i];
        previousVariables->reference_force[i] = reference_force[i];
        previousVariables->filtered_reference_force[i] = filtered_reference_force[i];
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
        /*if (output_force[i] > max_torque_Nm[i]) {
            previousVariables->integral_output_force[i] = max_torque_Nm[i] - (reference_force[i] + filtered_pressure_HPF[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        }
        else if (output_force[i] < -max_torque_Nm[i]) {
            previousVariables->integral_output_force[i] = -max_torque_Nm[i] - (reference_force[i] + filtered_pressure_HPF[i] + filtered_finger_bk_comp_force[i] + hysteresis_force[i] - filtered_nominal_force[i]);
        }*/
        rdda->motor[i].motorOut.tau_off = saturation(rdda->motor[i].tau_max, output_force[i]);
    }

}