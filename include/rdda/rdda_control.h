#ifndef RDDA_CONTROL_H
#define RDDA_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm_data.h"
#include "rdda_base.h"

typedef struct
{
    double motor_inertia[2];
    double motor_damping[2];
    double finger_damping[2];
    double finger_stiffness[2];
    double hydraulic_stiffness;
    double hydraulic_damping;
    double cutoff_frequency_LPF[5];
    double cutoff_frequency_HPF[2];
    double Kp[2];
    double Pp[2];
    double Vp[2];
    double zeta;
    double max_inner_loop_torque_Nm;
    double max_output_torque_integral_part_Nm;
    double max_torque_Nm;
    double max_velocity;
    double max_stiffness;
    double hysteresis_sigma;
    double hysteresis_friction;
    double sample_time;
    double gear_ratio;
} ControlParams;

typedef struct
{
    double lambda[5]; // cut-off frequency(rad/s)
    double a1[5];
    double b0[5];
    double b1[5];
} FirstOrderLowPassFilterParams;

typedef struct
{
    double lambda[2];
    double a1[2];
    double b0[2];
    double b1[2];
} FirstOrderHighPassFilterParams;

typedef struct
{
    double b0[3];
    double b1[3];
    double b2[3];
    double a1[3];
    double a2[3];
} SecondOrderLowPassFilterParams;

typedef struct
{
    double pos_tar[2];
    double prev_pos_tar[2];
    double filtered_pos_tar[2];
    double prev_filtered_pos_tar[2];
    double stiffness[2];
    double prev_stiffness[2];
    double filtered_stiffness[2];
    double prev_filtered_stiffness[2];
    double vel_sat[2];
    double prev_vel_sat[2];
    double filtered_vel_sat[2];
    double prev_filtered_vel_sat[2];
    double tau_sat[2];
    double prev_tau_sat[2];
    double filtered_tau_sat[2];
    double prev_filtered_tau_sat[2];
    double pos_ref[2];
    double motor_pos[2];
    double motor_vel[2];
    double finger_vel_pressure_part[2];
    double pressure[2];
    double prev_pressure[2];
    double filtered_pressure[2];
    double filtered_pressure_HPF[2];
    double nominal_force[2];
    double filtered_nominal_force[2];
    double filtered_nominal_force_HPF[2];
    double finger_bk_comp_force_position_part[2];
    double filtered_finger_bk_comp_force_position_part[2];
    double filtered_finger_bk_comp_force_pressure_part[2];
    double prev_filtered_finger_bk_comp_force_pressure_part[2];
    double hysteresis_force[2];
    double filtered_hysteresis_force[2];
    double output_force[2];
    double integral_output_force[2];
    double filtered_output_force[2];
    double reference_force[2];
    double filtered_reference_force[2];
} PreviousVariables;

void dobInit(ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, FirstOrderHighPassFilterParams *firstOrderHighPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda);
double firstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1);
void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, FirstOrderHighPassFilterParams *firstOrderHighPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables);

#endif //RDDA_CONTROL_H
