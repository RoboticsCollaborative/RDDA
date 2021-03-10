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
    double sample_time;
    double resonant_frequency;
    double zeta;
    double inertia[4];
    double stiffness[4];
    double damping[4];
} TeleParam;

typedef struct
{
    double pre_pos[4];
    double pre_vel[4];
    double pre_filtered_pos[4];
    double pre_filtered_vel[4];
} TeleFilterVariable;

typedef struct
{
    double cutoff_frequency_LPF;
    double lambda;
    double a1;
    double b0;
    double b1;
} TeleFirstOrderLowPassFilterParams;

void teleInit(TeleParam *teleParam, TeleFilterVariable *teleFilterVariable, TeleFirstOrderLowPassFilterParams *teleFirstOrderLowPassFilterParams, Rdda *rdda);
void teleController(TeleParam *teleParam, TeleFilterVariable *teleFilterVariable, TeleFirstOrderLowPassFilterParams *teleFirstOrderLowPassFilterParams, ControlParams *controlParams, Rdda *rdda);

#endif //RDDA_TELE_CONTROL_H
