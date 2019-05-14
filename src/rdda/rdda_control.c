/**
 *  Put controller functions in this file.
 **/

#include "rdda_control.h"

void initDobController(RDDA_slave *rddaSlave) {
    /* Init motor position */
    rddaSlave->motor[0].motorIn.act_pos = 0;    // motor1
    rddaSlave->motor[1].motorIn.act_pos = 0;    // motor2
}

void dobController(RDDA_slave *rddaSlave) {
    /* Assign analog input from pressure sensor to motor torque */
    rddaSlave->motor[0].motorOut.tau_off = rddaSlave->psensor.analogIn.val1;
}