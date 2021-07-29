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
} TeleParam;

typedef struct
{
    double pre_pos[2];
    double pre_vel[2];
    double pre_pressure[2];
    double pre_filtered_pos[2];
    double pre_filtered_vel[2];
    double pre_filtered_pressure[2];
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
