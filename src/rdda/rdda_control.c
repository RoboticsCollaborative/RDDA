/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

void dobController(RDDA_slave *rddaSlave) {
    /* Assign analog input from pressure sensor to motor torque */
    rddaSlave->motor[0].motorOut.tau_off = rddaSlave->psensor.analogIn.val1;
}