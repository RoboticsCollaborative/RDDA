#ifndef RDDA_INIT_BEL_H
#define RDDA_INIT_BEL_H

#include <stdio.h>
#include <stdlib.h>

#include "ethercat.h"

int mapMotorPDOs_callback(uint16 slaveIdx);
int initMotor(uint16 slaveIdx);

#endif //RDDA_INIT_BEL_H
