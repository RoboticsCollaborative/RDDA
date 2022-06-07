/**
 *  Functions and auxiliary tools for tele control
 **/

#include "tele_control.h"

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

double teleFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void teleInit(TeleParam *teleParam) {
    /* parameter initialization */
    int num = MOTOR_NUM;
    teleParam->sample_time = 0.25e-3;
    teleParam->zeta = 0.5;//0.12;
    teleParam->wave_damping = 0.02;
    teleParam->vel_tar[0] = 0.0;
    teleParam->vel_tar[1] = 0.0;
    teleParam->lambda = 10.0; //20.0; //0.1;
    teleParam->pos_tar_int[0] = 0.0;
    teleParam->pos_tar_int[1] = 0.0;
    teleParam->pos_tar[0] = 0.0;
    teleParam->pos_tar[1] = 0.0;
    teleParam->delay_current_index = 0;
    teleParam->delay_cycle_previous = 6;

    /* symmetric stiffness */
    for (int i = 0; i < num; i ++) {
        teleParam->stiffness[i] = 20.0;
        teleParam->motor_inertia[i] = 1.463e-4;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * teleParam->motor_inertia[i]);
    }

    for (int i = 0; i < num; i ++) {
        for (int j = 0; j < MAX_BUFF; j ++) {
            teleParam->wave_history[i][j] = 0.0;
        }
    }

}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num = MOTOR_NUM;

    double pos[num];
    double vel[num];
    double wave_input[num];
    double wave_output[num];

    double pos_master[num];
    double energy_diff[num];
    double wave_comp_coeff[num];

    double actual_pos_error[num];
    double predict_pos_error[num];
    double error_difference[num];
    double wave_correction[num];

    double energy_ratio = 10.0;
    double beta = 200.0;
    double wave_comp_coeff_upper_bound = 0.99;
    double wave_comp_coeff_lower_bound = -1.0;

    int delay_index;
    int delay_difference;
    int delay_cycle_current = 20;

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        pos_master[i] = rdda->motor[i].rddaPacket.pos_in;
        energy_diff[i] = 0.5 * teleParam->stiffness[i] * (pos_master[i] - pos[i]) * (pos_master[i] - pos[i]) - 0.5 * teleParam->motor_inertia[i] * vel[i] * vel[i] * energy_ratio;
        if (energy_diff[i] >= 0.0) wave_comp_coeff[i] = MIN(1.0 - exp(-energy_diff[i] * beta), wave_comp_coeff_upper_bound);
        else wave_comp_coeff[i] = MAX(-1.0 + exp(energy_diff[i] * beta), wave_comp_coeff_lower_bound);
        rdda->motor[i].rddaPacket.test = wave_comp_coeff[i];
        // wave_comp_coeff[i] = 0.0;
        // wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in + (rdda->motor[i].rddaPacket.wave_in_aux - rdda->motor[i].rddaPacket.wave_out) * wave_comp_coeff[i];
        rdda->motor[i].rddaPacket.wave_out_aux = wave_input[i];
    }

    /* wave tele */
    delay_index = teleParam->delay_current_index - delay_cycle_current;
    delay_difference = delay_cycle_current - teleParam->delay_cycle_previous;
    if (delay_index < 0) {
        delay_index += MAX_BUFF;
    }

    for (int i = 0; i < num; i ++) {
        teleParam->pos_tar_int[i] += teleParam->vel_tar[i] * teleParam->sample_time;
        /* intuitive pos drift correction */
        // teleParam->pos_tar[i] = teleParam->pos_tar_int[i] + teleParam->lambda * (rdda->motor[i].rddaPacket.pos_in - teleParam->pos_tar[i]);
        teleParam->pos_tar[i] = teleParam->pos_tar_int[i];
        teleParam->vel_tar[i] = (sqrt(2.0 * teleParam->wave_damping) * wave_input[i] + teleParam->damping[i] * vel[i] + teleParam->stiffness[i] * (pos[i] - teleParam->pos_tar[i])) / (teleParam->damping[i] + teleParam->wave_damping);
        controlParams->coupling_torque[i] = teleParam->stiffness[i] * (teleParam->pos_tar[i] - pos[i]) + teleParam->damping[i] * (teleParam->vel_tar[i] - vel[i]);
	    wave_output[i] = wave_input[i] - sqrt(2.0 / teleParam->wave_damping) * controlParams->coupling_torque[i];
        
        /* pos drift correction */
        actual_pos_error[i] = pos_master[i] - teleParam->pos_tar[i];
        predict_pos_error[i] = -1.0 / sqrt(2.0 * teleParam->wave_damping) * teleParam->wave_int[i];
        error_difference[i] = predict_pos_error[i] - actual_pos_error[i];
        wave_correction[i] = 1.0 * sqrt(2.0 * teleParam->wave_damping) * (2.0 * M_PI * teleParam->lambda) * error_difference[i];
        if (error_difference[i] * wave_output[i] <= 0) {
            if (fabs(wave_correction[i]) < fabs(wave_output[i])) {
                wave_output[i] += wave_correction[i];
            }
            else {
                wave_output[i] = 0.0;
            }
        }
        /* check time-varying current delay cycle */
        if (delay_difference == 0) {
            teleParam->wave_int[i] -= teleParam->wave_history[i][delay_index] * teleParam->sample_time;
        }
        else if (delay_difference > 0) {
            for (int j = 1; j <= delay_difference; j ++) {
                if (delay_index + j >= MAX_BUFF) teleParam->wave_int[i] += teleParam->wave_history[i][delay_index + j - MAX_BUFF] * teleParam->sample_time;
                else teleParam->wave_int[i] += teleParam->wave_history[i][delay_index + j] * teleParam->sample_time;
            }
        }
        else {
            for (int j = 0; j < -delay_difference; j ++) {
                if (delay_index - j < 0) teleParam->wave_int[i] -= teleParam->wave_history[i][delay_index - j + MAX_BUFF] * teleParam->sample_time;
                else teleParam->wave_int[i] -= teleParam->wave_history[i][delay_index - j] * teleParam->sample_time;
            }
        }
        teleParam->wave_int[i] += wave_output[i] * teleParam->sample_time;
        teleParam->delay_cycle_previous = delay_cycle_current;

        teleParam->wave_history[i][teleParam->delay_current_index] = wave_output[i];
        rdda->motor[i].rddaPacket.wave_out = wave_output[i];
    }

    teleParam->delay_current_index++;
    if (teleParam->delay_current_index >= MAX_BUFF) {
        teleParam->delay_current_index = 0;
    }

    // for (int i = 0; i < num; i ++) {
    //     teleParam->pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
    //     teleParam->vel_tar[i] = wave_input[i];
    //     controlParams->coupling_torque[i] = teleParam->stiffness[i] * (teleParam->pos_tar[i] - pos[i]) + teleParam->damping[i] * (teleParam->vel_tar[i] - vel[i]);
    //     rdda->motor[i].rddaPacket.wave_out = vel[i];
    // }
    // printf("%+2.4lf, %+2.4lf, %+2.4lf, %+2.4lf\r", pos[0], pos[1], rdda->psensor.analogIn.val1, rdda->psensor.analogIn.val2);

}
