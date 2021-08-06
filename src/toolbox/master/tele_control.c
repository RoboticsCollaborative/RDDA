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
}

void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda) {
    int num;
    num = teleParam->num;

//    double pos[num];
    double vel[num];
    double wave_input[num];

    /* pos, vel & wave input */
    for (int i = 0; i < num; i ++) {
//        pos[i] = rdda->motor[i].motorIn.act_pos - rdda->motor[i].init_pos;
        vel[i] = rdda->motor[i].motorIn.act_vel;
        wave_input[i] = rdda->motor[i].rddaPacket.wave_in;
    }
//    pos[1] = rdda->motor[1].motorIn.act_pos + rdda->motor[1].init_pos;

    /* wave tele */
    for (int i = 0; i < num; i ++) {
        controlParams->coupling_torque[i] = -1.0 * (teleParam->wave_damping * vel[i] - sqrt(2 * teleParam->wave_damping) * wave_input[i]);
        rdda->motor[i].rddaPacket.wave_out = sqrt(2 * teleParam->wave_damping) * vel[i] - wave_input[i];
//        rdda->motor[i].rddaPacket.pos_out = pos[i];
    }
//    printf("p[0]: %+2.4lf, v[0]: %+2.4lf, t[0]: %+2.4lf, p[1]: %+2.4lf, v[1]: %+2.4lf, t[1]: %+2.4lf,", rdda->motor[0].motorIn.act_pos, rdda->motor[0].motorIn.act_vel, rdda->psensor.analogIn.val1, rdda->motor[1].motorIn.act_pos, rdda->motor[1].motorIn.act_vel, rdda->psensor.analogIn.val2);

}
