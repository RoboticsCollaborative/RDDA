#ifndef RDDA_TELE_CONTROL_H
#define RDDA_TELE_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rdda_control.h"

#include "shm_data.h"
#include "rdda_base.h"

#define MAX_BUFF 2000

typedef struct
{
    double sample_time;
    double zeta;
    double stiffness[MOTOR_COUNT];
    double damping[MOTOR_COUNT];
    double motor_inertia[MOTOR_COUNT];
    double wave_damping;
    double pos_tar_int[MOTOR_COUNT];
    double pos_tar[MOTOR_COUNT];
    double vel_tar[MOTOR_COUNT];
    double lambda;
    int delay_current_index;
    int delay_cycle_previous;
    double wave_int[MOTOR_COUNT];
    double wave_history[MOTOR_COUNT][MAX_BUFF];
} TeleParam;

void teleInit(TeleParam *teleParam);
void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda);

#endif //RDDA_TELE_CONTROL_H
