#ifndef RDDA_TELE_CONTROL_H
#define RDDA_TELE_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rdda_control.h"

#include "shm_data.h"
#include "rdda_base.h"

typedef struct
{
    int num;
    double sample_time;
    double resonant_frequency;
    double zeta;
    double inertia[2];
    double stiffness[2];
    double damping[2];
    double wave_damping;
    double pos_tar_int[2];
    double vel_tar[2];
    double lambda;
} TeleParam;

void teleInit(TeleParam *teleParam);
void teleController(TeleParam *teleParam, ControlParams *controlParams, Rdda *rdda);

#endif //RDDA_TELE_CONTROL_H
