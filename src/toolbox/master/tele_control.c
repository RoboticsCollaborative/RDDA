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
    int num = MOTOR_NUM;
    teleParam->sample_time = 0.5e-3;
    teleParam->zeta = 0.3;//0.12;
    teleParam->wave_damping = 0.04;

    /* symmetric stiffness */
    for (int i = 0; i < num; i ++) {
        teleParam->stiffness[i] = 20.0;
        teleParam->damping[i] = 2.0 * teleParam->zeta * sqrt(teleParam->stiffness[i] * 1.1e-4);
    }
}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num = MOTOR_NUM;

    double pos[num];
    double vel[num];
    double wave_input[num];

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
    }

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        controlParams->coupling_torque[i] = -1.0 * (teleParam->wave_damping * vel[i] - sqrt(2 * teleParam->wave_damping) * wave_input[i]);
        rdda->motor[i].rddaPacket.wave_out = sqrt(2 * teleParam->wave_damping) * vel[i] - wave_input[i];
    }

    // for (int i = 0; i < num; i ++) {
    //     teleParam->pos_tar[i] = rdda->motor[i].rddaPacket.pos_in;
    //     teleParam->vel_tar[i] = wave_input[i];
    //     controlParams->coupling_torque[i] = teleParam->stiffness[i] * (teleParam->pos_tar[i] - pos[i]) + teleParam->damping[i] * (teleParam->vel_tar[i] - vel[i]);
    //     rdda->motor[i].rddaPacket.wave_out = vel[i];
    // }
    // printf("%+2.4lf, %+2.4lf\r", pos[0], pos[1]);
    printf("%+2.4lf, %+2.4lf, %+2.4lf, %+2.4lf\r", pos[0], pos[1], rdda->psensor.analogIn.val1, rdda->psensor.analogIn.val2);

}
