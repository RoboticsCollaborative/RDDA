#ifndef RDDA_BASE_H
#define RDDA_BASE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "rdda_ecat.h"
#include "shm_data.h"

void rdda_update(ecat_slaves *ecatSlaves, Rdda *rdda);
void rddaStop(ecat_slaves *rdda);
int rdda_gettime(ecat_slaves *rdda);
void rdda_sleep(ecat_slaves *rddaSlave, int cycletime);
void initRddaStates(ecat_slaves *ecatSlaves, Rdda *rdda);
double saturation(double max_torque, double raw_torque);

#endif //RDDA_BASE_H
