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
    double cutoff_frequency[4];
    double Kp[2];
    double Pp[2];
    double Vp[2];
    //double pressure_offset;
    double max_inner_loop_torque_Nm;
    double max_output_torque_integral_part_Nm;
    double max_torque_Nm;
    double max_velocity;
    double max_stiffness;
    double hysteresis_sigma;
    double hysteresis_friction;
    //double gripper_angle_difference;
    double sample_time;
    double gear_ratio;
} ControlParams;

typedef struct
{
    double lambda[4]; // cut-off frequency(rad/s)
    double a1[5];
    double b0[5];
    double b1[5];
} FirstOrderFilterParams;

typedef struct
{
    double b0[2];
    double b1[2];
    double b2[2];
    double a1;
    double a2;
} SecondOrderFilterParams;

typedef struct
{
    double motor_pos[2];
    double motor_vel[2];
    double finger_vel_pressure_part[2];
    double pressure[2];
    double prev_pressure[2];
    double filtered_pressure[2];
    double nominal_force[2];
    double filtered_nominal_force[2];
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

void dobInit(ControlParams *controlParams, FirstOrderFilterParams *firstOrderFilterParams, SecondOrderFilterParams *secondOrderFilterParams, PreviousVariables *previousVariables, Rdda *rdda);
double firstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1);
void dobController(Rdda *rdda, ControlParams *controlParams, FirstOrderFilterParams *firstOrderFilterParams, SecondOrderFilterParams *secondOrderFilterParams, PreviousVariables *previousVariables);

#endif //RDDA_CONTROL_H
