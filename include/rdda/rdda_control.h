#ifndef RDDA_CONTROL_H
#define RDDA_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm_data.h"

void initDobController(RDDA_slave *rddaSlave);
void dobController(RDDA_slave *rddaSlave);

#endif //RDDA_CONTROL_H
