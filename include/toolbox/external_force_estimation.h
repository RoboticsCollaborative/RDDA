#ifndef EXTERNAL_FORCE_ESTIMATION_H
#define EXTERNAL_FORCE_ESTIMATION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "shm_data.h"
#include "rdda_base.h"

typedef struct
{
    double joint_pos_est_coeffs[10];
    double joint_torque_est_coeffs[14];
    double b0[2];
    double b1[2];
    double a1[2];
    double joint_motor_ratio;
    double joint_contact_radius;
} ExForceEstParams;

typedef struct
{
    double prev_pressure_diff[6][2];
    double prev_vel_diff[6][2];
    double prev_pressure[2];
    double prev_vel[2];
    double prev_pos[2];
    double prev_joint_torque_est[2];
    double prev_joint_pos_est[2];
    double prev_filtered_joint_torque_est[2];
    double prev_filtered_joint_pos_est[2];
    double prev_HP_filtered_joint_torque_est[2];
    double initial_joint_torque_est[2];
} ExForceEstVars;

void externalForceEstimationInit(ExForceEstParams *exForceEstParams, ExForceEstVars *exForceEstVars, Rdda *rdda);
void externalForceEstimation(ExForceEstParams *exForceEstParams, ExForceEstVars *exForceEstVars, Rdda *rdda);


#endif