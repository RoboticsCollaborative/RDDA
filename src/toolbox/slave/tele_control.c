/**
 *  Functions and auxiliary tools for tele control
 **/

#include "tele_control.h"

double teleFirstOrderIIRFilter(double input, double input_prev, double output_prev, double b0, double b1, double a1) {
    double output;
    output = b0 * input + b1 * input_prev + a1 * output_prev;
    return output;
}

void teleInit(TeleParam *teleParam) {
    /* parameter initialization */
    teleParam->num = 2;
    teleParam->sample_time = 0.5e-3;
    teleParam->zeta = 0.12;
    teleParam->wave_damping = 0.01;
    teleParam->vel_tar[0] = 0.0;
    teleParam->vel_tar[1] = 0.0;
    teleParam->lambda = 10.0; //0.1;
    teleParam->pos_tar_int[0] = 0.0;
    teleParam->pos_tar_int[1] = 0.0;
    teleParam->pos_tar[0] = 0.0;
    teleParam->pos_tar[1] = 0.0;
    teleParam->delay_index = 0;
    teleParam->delay_const = 8;

    /* symmetric stiffness */
    for (int i = 0; i < teleParam->num; i ++) {
        teleParam->stiffness[i] = 20.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * 1.1e-3);
    }

    for (int i = 0; i < teleParam->num; i ++) {
        for (int j = 0; j < teleParam->delay_const; j ++) {
            teleParam->wave_history[i][j] = 0.0;
        }
    }

}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num;
    num = teleParam->num;

    double pos[num];
    double vel[num];
    double wave_input[num];
    double wave_output[num];
    double wave_test[num];

    double actual_pos_error[num];
    double predict_pos_error[num];
    double error_difference[num];
    double wave_correction[num];

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
    }

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        teleParam->pos_tar_int[i] += teleParam->vel_tar[i] * teleParam->sample_time;
//        teleParam->pos_tar[i] = teleParam->pos_tar_int[i] + teleParam->lambda * (rdda->motor[i].rddaPacket.pos_in - teleParam->pos_tar[i]);
        teleParam->pos_tar[i] = teleParam->pos_tar_int[i];
        teleParam->vel_tar[i] = (sqrt(2.0 * teleParam->wave_damping) * wave_input[i] + teleParam->damping[i] * vel[i] + teleParam->stiffness[i] * (pos[i] - teleParam->pos_tar[i])) / (teleParam->damping[i] + teleParam->wave_damping);
        controlParams->coupling_torque[i] = teleParam->stiffness[i] * (teleParam->pos_tar[i] - pos[i]) + teleParam->damping[i] * (teleParam->vel_tar[i] - vel[i]);
        wave_output[i] = rdda->motor[i].rddaPacket.wave_in - sqrt(2.0 / teleParam->wave_damping) * controlParams->coupling_torque[i];
        wave_test[i] = wave_output[i];

        actual_pos_error[i] = rdda->motor[i].rddaPacket.pos_in - pos[i];
        predict_pos_error[i] = -1.0 / sqrt(2.0 * teleParam->wave_damping) * teleParam->wave_int[i];
        error_difference[i] = predict_pos_error[i] - actual_pos_error[i];
        wave_correction[i] = -1.0 * sqrt(2.0 * teleParam->wave_damping) * (2.0 * M_PI * teleParam->lambda) * error_difference[i];
        if (error_difference[i] * wave_output[i] >= 0) {
            if (fabs(wave_correction[i]) < fabs(wave_output[i])) {
                wave_output[i] -= wave_correction[i];
            }
            else {
                wave_output[i] = 0.0;
            }
        }
        teleParam->wave_int[i] += wave_output[i] - teleParam->wave_history[i][teleParam->delay_index];
        teleParam->wave_history[i][teleParam->delay_index] = wave_output[i];
        rdda->motor[i].rddaPacket.wave_out = wave_output[i];
    }
    teleParam->delay_index++;
    if (teleParam->delay_index >= teleParam->delay_const) {
        teleParam->delay_index = 0;
    }
    printf("ppe: %+2.4lf, ape: %+2.4lf, ed: %+2.4lf, raww: %+2.4lf, corw: %+2.4lf\r", predict_pos_error[0], actual_pos_error[0], error_difference[0], wave_test[0], wave_output[0]);

}
