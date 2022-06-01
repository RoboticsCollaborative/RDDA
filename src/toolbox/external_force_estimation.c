/**
 *  Functions and auxiliary tools for external force estimation
 **/

#include "external_force_estimation.h"

#define MIN(x,y) (x)<(y)?(x):(y)

double firstOrderFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

double frictionModelling(double pos, double prev_pos, double prev_friction) {
    double friction;
    double sigma = 100;
    double friction_const = 0.0045;
    double friction_offset = -0.012;
    friction = (prev_friction - friction_offset + sigma * (pos - prev_pos)) / (1 + fabs(pos - prev_pos) * sigma / friction_const) + friction_offset;
    return friction;
}

void externalForceEstimationInit(ExForceEstParams *exForceEstParams, ExForceEstVars *exForceEstVars, Rdda *rdda) {
    // params init
    exForceEstParams->joint_pos_est_coeffs[0] = 1.0;
    exForceEstParams->joint_pos_est_coeffs[1] = 0.00217186;
    exForceEstParams->joint_pos_est_coeffs[2] = 0.05230454;
    exForceEstParams->joint_pos_est_coeffs[3] = 0.00614086;
    exForceEstParams->joint_pos_est_coeffs[4] = 0.05580743;
    exForceEstParams->joint_pos_est_coeffs[5] = 0.22967284;
    exForceEstParams->joint_pos_est_coeffs[6] = 0.07569077;
    exForceEstParams->joint_pos_est_coeffs[7] = 0.80566290;
    exForceEstParams->joint_pos_est_coeffs[8] = 0.11266912;
    exForceEstParams->joint_pos_est_coeffs[9] = 0.55576023;


    exForceEstParams->joint_torque_est_coeffs[0] = 0.14173630;
    exForceEstParams->joint_torque_est_coeffs[1] = 0.01600068;
    exForceEstParams->joint_torque_est_coeffs[2] = 0.27446144;
    exForceEstParams->joint_torque_est_coeffs[3] = 0.05968793;
    exForceEstParams->joint_torque_est_coeffs[4] = 0.14992540;
    exForceEstParams->joint_torque_est_coeffs[5] = 0.05635813;
    exForceEstParams->joint_torque_est_coeffs[6] = 0.14469174;
    exForceEstParams->joint_torque_est_coeffs[7] = 1.03255298;
    exForceEstParams->joint_torque_est_coeffs[8] = 0.58367067;
    exForceEstParams->joint_torque_est_coeffs[9] = 3.70322281;
    exForceEstParams->joint_torque_est_coeffs[10] = 1.00067372;
    exForceEstParams->joint_torque_est_coeffs[11] = 1.99906299;
    exForceEstParams->joint_torque_est_coeffs[12] = 0.08020776;
    exForceEstParams->joint_torque_est_coeffs[13] = 0.0;

    double Ts = 0.25e-3;
    double wc = 2 * M_PI * 50.0;
    exForceEstParams->a1[0] = (2.0 - wc * Ts) / (2.0 + wc * Ts);
    exForceEstParams->b0[0] = wc * Ts / (2.0 + wc * Ts);
    exForceEstParams->b1[0] = wc * Ts / (2.0 + wc * Ts);

    double wHP = 2 * M_PI * 1e-100;
    exForceEstParams->a1[1] = (2.0 - wc * Ts) / (2.0 + wHP * Ts);
    exForceEstParams->b0[1] = 2.0 / (2.0 + wHP * Ts);
    exForceEstParams->b1[1] = -2.0 / (2.0 + wHP * Ts);

    exForceEstParams->joint_motor_ratio = 14 / 10.5;
    exForceEstParams->joint_contact_radius = 0.12;

    // variables init
    int num = 2;
    int max_diff_order = 12;
    for (int i = 0; i < num; i ++) {
        for (int j = 0; j < max_diff_order; j ++) {
            exForceEstVars->prev_pressure_diff[j][i] = 0.0;
            exForceEstVars->prev_vel_diff[j][i] = 0.0;
        }
        exForceEstVars->prev_pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        exForceEstVars->prev_vel[i] = rdda->motor[i].motorIn.act_vel;
        exForceEstVars->prev_joint_torque_est[i] = 0.0;
        exForceEstVars->prev_filtered_joint_torque_est[i] = 0.0;
        exForceEstVars->prev_joint_pos_est[i] = 0.0;
        exForceEstVars->prev_filtered_joint_pos_est[i] = 0.0;
        exForceEstVars->prev_HP_filtered_joint_torque_est[i] = 0.0;
    }
    exForceEstVars->initial_joint_torque_est[0] = 1.0 / 2 * rdda->psensor.analogIn.val1;// + exForceEstParams->joint_torque_est_coeffs[0] * exForceEstVars->prev_pos[0];
    exForceEstVars->initial_joint_torque_est[1] = 1.0 / 2 * rdda->psensor.analogIn.val2;// + exForceEstParams->joint_torque_est_coeffs[0] * exForceEstVars->prev_pos[1];
    exForceEstVars->prev_pressure[0] = rdda->psensor.analogIn.val1;
    exForceEstVars->prev_pressure[1] = rdda->psensor.analogIn.val2;
}

void externalForceEstimation(ExForceEstParams *exForceEstParams, ExForceEstVars *exForceEstVars, Rdda *rdda) {
    int num = 2;
    int hose_model_dof = 2;
    double pressure[num];
    double pos[num];
    double vel[num];
    double pressure_diff[2 * (hose_model_dof + 1)][num];
    double vel_diff[2 * (hose_model_dof + 1)][num];
    double joint_pos_est[num];
    double joint_torque_est[num];
    double filtered_joint_pos_est[num];
    double filtered_joint_torque_est[num];
    double HP_filtered_joint_torque_est[num];
    // double filtered_joint_contact_force_est[num];
    
    pressure[0] = rdda->psensor.analogIn.val1;
    pressure[1] = rdda->psensor.analogIn.val2;
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
    }

    for (int i = 0; i < num; i ++) {
        pressure_diff[0][i] = pressure[i] - exForceEstVars->prev_pressure[i];
        vel_diff[0][i] = vel[i] - exForceEstVars->prev_vel[i];
        exForceEstVars->prev_pressure[i] = pressure[i];
        exForceEstVars->prev_vel[i] = vel[i];
        for (int j = 1; j < 2 * (hose_model_dof + 1); j ++) {
            pressure_diff[j][i] = pressure_diff[j-1][i] - exForceEstVars->prev_pressure_diff[j-1][i];
            vel_diff[j][i] = vel_diff[j-1][i] - exForceEstVars->prev_vel_diff[j-1][i];
            exForceEstVars->prev_pressure_diff[j-1][i] = pressure_diff[j-1][i];
            exForceEstVars->prev_vel_diff[j-1][i] = vel_diff[j-1][i];
        }
        exForceEstVars->prev_pressure_diff[2 * (hose_model_dof + 1)-1][i] = pressure_diff[2 * (hose_model_dof + 1)-1][i];
        exForceEstVars->prev_vel_diff[2 * (hose_model_dof + 1)-1][i] = vel_diff[2 * (hose_model_dof + 1)-1][i];
    }

    for (int i = 0; i < num; i ++) {
        joint_pos_est[i] = exForceEstParams->joint_pos_est_coeffs[0] * pos[i] + exForceEstParams->joint_pos_est_coeffs[2 * hose_model_dof + 1] * pressure[i];
        joint_pos_est[i] += exForceEstParams->joint_pos_est_coeffs[1] * vel[i] + exForceEstParams->joint_pos_est_coeffs[2 * hose_model_dof + 2] * pressure_diff[0][i];
        for (int j = 1; j < 2 * hose_model_dof; j ++) {
            joint_pos_est[i] += (exForceEstParams->joint_pos_est_coeffs[j+1] * vel_diff[j-1][i] + exForceEstParams->joint_pos_est_coeffs[2 * hose_model_dof + j + 2] * pressure_diff[j][i]);
        }
        filtered_joint_pos_est[i] = firstOrderFilter(joint_pos_est[i], exForceEstVars->prev_joint_pos_est[i], exForceEstVars->prev_filtered_joint_pos_est[i], exForceEstParams->b0[0], exForceEstParams->b1[0], exForceEstParams->a1[0]);
        exForceEstVars->prev_joint_pos_est[i] = joint_pos_est[i];
        exForceEstVars->prev_filtered_joint_pos_est[i] = filtered_joint_pos_est[i];

        joint_torque_est[i] = exForceEstParams->joint_torque_est_coeffs[0] * pos[i] + exForceEstParams->joint_torque_est_coeffs[2 * (hose_model_dof + 1) + 1] * pressure[i]  - exForceEstVars->initial_joint_torque_est[i];// + 0.01 * ((filtered_joint_pos_est[i] > exForceEstVars->prev_filtered_joint_torque_est[i]) - (filtered_joint_pos_est[i] < exForceEstVars->prev_filtered_joint_torque_est[i]));
        joint_torque_est[i] += exForceEstParams->joint_torque_est_coeffs[1] * vel[i] + exForceEstParams->joint_torque_est_coeffs[2 * (hose_model_dof + 1) + 2] * pressure_diff[0][i];
        for (int j = 1; j < 2 * (hose_model_dof + 1); j ++) {
            joint_torque_est[i] += (exForceEstParams->joint_torque_est_coeffs[j+1] * vel_diff[j-1][i] + exForceEstParams->joint_torque_est_coeffs[2 * (hose_model_dof + 1) + j + 2] * pressure_diff[j][i]);
        }
        filtered_joint_torque_est[i] = firstOrderFilter(joint_torque_est[i], exForceEstVars->prev_joint_torque_est[i], exForceEstVars->prev_filtered_joint_torque_est[i], exForceEstParams->b0[0], exForceEstParams->b1[0], exForceEstParams->a1[0]);
        HP_filtered_joint_torque_est[i] = firstOrderFilter(filtered_joint_torque_est[i], exForceEstVars->prev_filtered_joint_torque_est[i], exForceEstVars->prev_HP_filtered_joint_torque_est[i], exForceEstParams->b0[1], exForceEstParams->b1[1], exForceEstParams->a1[1]);
        exForceEstVars->prev_joint_torque_est[i] = joint_torque_est[i];
        exForceEstVars->prev_filtered_joint_torque_est[i] = filtered_joint_torque_est[i];
        exForceEstVars->prev_HP_filtered_joint_torque_est[i] = HP_filtered_joint_torque_est[i];
        // filtered_joint_contact_force_est[i] = filtered_joint_torque_est[i] / exForceEstParams->joint_contact_radius * exForceEstParams->joint_motor_ratio;
    }

    rdda->motor[0].rddaPacket.test = filtered_joint_torque_est[1];
    // rdda->motor[1].rddaPacket.test = exForceEstParams->joint_torque_est_coeffs[0] * pos[1] + exForceEstParams->joint_torque_est_coeffs[2 * (hose_model_dof + 1) + 1] * pos[1] * 1.0 - 0.01;
    rdda->motor[1].rddaPacket.test = pos[1] * 10.0;

}