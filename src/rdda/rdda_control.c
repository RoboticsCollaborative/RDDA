/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

void dobController(Rdda *rdda) {
    /* Assign analog input from pressure sensor to motor torque */
    rdda->motor[0].motorOut.tau_off = rdda->psensor.analogIn.val1;
}