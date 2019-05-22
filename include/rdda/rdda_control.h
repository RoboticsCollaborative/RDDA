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
    double cutoff_frequency[3];
    double pos_gain;
    double vel_gain;
    double acc_gain;
    double pressure_offset;
    double max_inner_loop_torque_Nm;
    double max_torque_Nm;
    double hysteresis_sigma;
    double hysteresis_friction;
    double gripper_angle_difference;
    double sample_time;
} ControlParams;

typedef struct
{
    double lambda[3]; // cut-off frequency(rad/s)
    double a1[3];
    double b0[3];
    double b1[3];
} FilterParams;

typedef struct
{
    double motor_pos[2];
    double motor_vel[2];
    double pressure[2];
    double filtered_pressure[2];
    double nominal_force[2];
    double filtered_nominal_force[2];
    double output_force[2];
    double integral_output_force[2];
    double filtered_output_force[2];
} PreviousVariables;

void dobInit(ControlParams *controlParams, FilterParams *filterParams, PreviousVariables *previousVariables, Rdda *rdda);
double firstOrderIIRFilter(double input, double input_prev, double output_prev, double a1, double b0, double b1);
void dobController(Rdda *rdda, ControlParams *controlParams, FilterParams *filterParams, PreviousVariables *previousVariables);

#endif //RDDA_CONTROL_H
