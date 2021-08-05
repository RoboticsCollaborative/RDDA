#ifndef RDDA_RDDA_HOMING_H
#define RDDA_RDDA_HOMING_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm_data.h"
#include "rdda_base.h"
#include "rdda_control.h"

void rddaHoming(ecat_slaves *ecatSlaves, Rdda *rdda, ControlParams *controlParams, FirstOrderLowPassFilterParams *firstOrderLowPassFilterParams, SecondOrderLowPassFilterParams *secondOrderLowPassFilterParams, PreviousVariables *previousVariables);

#endif //RDDA_RDDA_HOMING_H
