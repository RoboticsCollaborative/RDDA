#ifndef RDDA_CONTROL_H
#define RDDA_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm_data.h"
#include "rdda_base.h"

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

typedef struct
{
    double motor_inertia[MOTOR_COUNT];
    double motor_damping[MOTOR_COUNT];
    double cutoff_frequency_LPF[MOTOR_COUNT];
    double lambda[MOTOR_COUNT]; // cut-off frequency(rad/s)
    double Kp[MOTOR_COUNT];
    double Pp[MOTOR_COUNT];
    double Vp[MOTOR_COUNT];
    double zeta;
    double max_external_torque[MOTOR_COUNT];
    double max_inner_loop_torque_Nm;
    double max_torque_Nm;
    double max_velocity;
    double max_stiffness;
    double sample_time;
    double gear_ratio;
    double coupling_torque[MOTOR_COUNT];
    double pos_ref[MOTOR_COUNT];
    double Kf[MOTOR_COUNT];
} ControlParams;

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
    double pos_tar[MOTOR_COUNT];
    double prev_pos_tar[MOTOR_COUNT];
    double filtered_pos_tar[MOTOR_COUNT];
    double prev_filtered_pos_tar[MOTOR_COUNT];
    double stiffness[MOTOR_COUNT];
    double prev_stiffness[MOTOR_COUNT];
    double filtered_stiffness[MOTOR_COUNT];
    double prev_filtered_stiffness[MOTOR_COUNT];
    double vel_sat[MOTOR_COUNT];
    double prev_vel_sat[MOTOR_COUNT];
    double filtered_vel_sat[MOTOR_COUNT];
    double prev_filtered_vel_sat[MOTOR_COUNT];
    double tau_sat[MOTOR_COUNT];
    double prev_tau_sat[MOTOR_COUNT];
    double filtered_tau_sat[MOTOR_COUNT];
    double prev_filtered_tau_sat[MOTOR_COUNT];
    double pos_ref[MOTOR_COUNT];
    double integral_control_force[MOTOR_COUNT];
    double vel[MOTOR_COUNT];
} PreviousVariables;

void dobInit(ControlParams *controlParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables, Rdda *rdda);
void dobController(Rdda *rdda, ControlParams *controlParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables);

#endif //RDDA_CONTROL_H
