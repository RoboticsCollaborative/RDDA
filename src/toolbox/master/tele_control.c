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
    int num = MOTOR_COUNT;
    teleParam->sample_time = 0.25e-3;
    teleParam->zeta = 0.5;//0.12;
    teleParam->wave_damping = 0.04;

    /* symmetric stiffness */
    for (int i = 0; i < num; i ++) {
        teleParam->stiffness[i] = 30.0;
        teleParam->motor_inertia[i] = 1.463e-4;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * teleParam->motor_inertia[i]);
    }
}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num = MOTOR_COUNT;

    double vel[num];
    double wave_input[num];
    double tele_ratio = 1.0;

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
        rdda->motor[i].rddaPacket.wave_out_aux = wave_input[i];
    }

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        controlParams->coupling_torque[i] = -1.0 * (teleParam->wave_damping * vel[i] + sqrt(2 * teleParam->wave_damping) * wave_input[i]);
        rdda->motor[i].rddaPacket.wave_out = -1.0 * sqrt(2 * teleParam->wave_damping) * vel[i] * tele_ratio - wave_input[i];
    }

}
