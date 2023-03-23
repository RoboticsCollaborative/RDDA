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
        teleParam->stiffness[i] = 20.0;
        teleParam->motor_inertia[i] = 1.463e-4 / 2.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * teleParam->motor_inertia[i]);
    }

    teleParam->delay_cycle_previous = 4;

}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num = MOTOR_COUNT;

    double vel[num];
    double wave_input[num];
    double tele_ratio = 1.0;
    double finger_damping = 1e-4;

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
        rdda->motor[i].rddaPacket.wave_out_aux = wave_input[i];
    }

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        controlParams->coupling_torque[i] = -1.0 * (teleParam->wave_damping * vel[i] * tele_ratio + sqrt(2 * teleParam->wave_damping) * wave_input[i]);
        rdda->motor[i].rddaPacket.wave_out = -1.0 * sqrt(2 * teleParam->wave_damping) * vel[i] * tele_ratio - wave_input[i];
    }

    // energy observer
    for (int i = 0; i < num; i ++) {
        rdda->motor[i].rddaPacket.energy_observer += (finger_damping * vel[i] * vel[i]
        -0.5 * (rdda->ts.delay_cycle - teleParam->delay_cycle_previous) * wave_input[i] * wave_input[i]) * teleParam->sample_time;
        teleParam->delay_cycle_previous = rdda->ts.delay_cycle;
    }

}
