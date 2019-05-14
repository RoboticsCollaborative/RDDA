#ifndef RDDA_BASE_H
#define RDDA_BASE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ethercat.h"
#include "rdda_ecat.h"
#include "shm_data.h"

void rdda_update(ecat_slave *ecatSlave, RDDA_slave *rddaSlave);
void rddaStop(ecat_slave *rddaSlave);
int rdda_gettime(ecat_slave *rddaSlave);
void rdda_sleep(ecat_slave *rddaSlave, int cycletime);
double torque_saturation(double max_torque, double raw_torque);

#endif //RDDA_BASE_H
