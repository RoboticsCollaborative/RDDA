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
    double motor_inertia[4];
    double motor_damping[4];
    double finger_damping[4];
    double finger_stiffness[4];
    double hydraulic_stiffness;
    double hydraulic_damping;
    double cutoff_frequency_LPF[2];
    double lambda[2]; // cut-off frequency(rad/s)
    double Kp[4];
    double Pp[4];
    double Vp[4];
    double zeta;
    double max_inner_loop_torque_Nm[4];
    double max_torque_Nm;
    double max_velocity;
    double max_stiffness;
    double hysteresis_sigma;
    double hysteresis_friction;
    double sample_time;
    double gear_ratio;
    double external_force[4];
} ControlParams;

typedef struct
{
    double friction_cmp_a1[4];
    double friction_cmp_b0[4];
    double friction_cmp_b1[4];
    double hysteresis_a1;
    double hysteresis_b0;
    double hysteresis_b1;
} FirstOrderLowPassFilterParams;

typedef struct
{
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;
} SecondOrderLowPassFilterParams;

typedef struct
{
    double pos_tar[4];
    double prev_pos_tar[4];
    double filtered_pos_tar[4];
    double prev_filtered_pos_tar[4];
    double stiffness[4];
    double prev_stiffness[4];
    double filtered_stiffness[4];
    double prev_filtered_stiffness[4];
    double vel_sat[4];
    double prev_vel_sat[4];
    double filtered_vel_sat[4];
    double prev_filtered_vel_sat[4];
    double tau_sat[4];
    double prev_tau_sat[4];
    double filtered_tau_sat[4];
    double pos_ref[4];
    double pressure[4];
    double prev_filtered_tau_sat[4];
    double finger_vel_pressure_part[4];
    double hysteresis_force[4];
    double filtered_finger_bk_comp_force_pressure_part[4];
    double integral_control_force[4];
} PreviousVariables;

void dobInit(ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda);
//double firstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1);
void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables);

#endif //RDDA_CONTROL_H
