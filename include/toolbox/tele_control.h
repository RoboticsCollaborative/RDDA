#ifndef RDDA_TELE_CONTROL_H
#define RDDA_TELE_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rdda_control.h"

#include "shm_data.h"
#include "rdda_base.h"

#define MOTOR_NUM 2
#define MAX_BUFF 2000

typedef struct
{
    double sample_time;
    double zeta;
    double stiffness[MOTOR_NUM];
    double damping[MOTOR_NUM];
    double motor_inertia[MOTOR_NUM];
    double wave_damping;
    double pos_tar_int[MOTOR_NUM];
    double pos_tar[MOTOR_NUM];
    double vel_tar[MOTOR_NUM];
    double lambda;
    int delay_current_index;
    int delay_cycle_previous;
    double wave_int[MOTOR_NUM];
    double wave_history[MOTOR_NUM][MAX_BUFF];
} TeleParam;

void teleInit(TeleParam *teleParam);
void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda);

#endif //RDDA_TELE_CONTROL_H
