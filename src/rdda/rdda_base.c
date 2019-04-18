/** rdda_base.c */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ethercat.h"
#include "rdda_ecat.h"
#include "rdda_base.h"

/** Initialize a struct pointer for slave
 *
 * @param slave_id  =   Slave indices.
 * @return a strcut pointer.
 */
/*
rdda_slavet *initRddaSlave(Slave_id *slave_id)
{
    Slave_id *id = slave_id;
    rdda_slavet *slave;

    slave = (rdda_slavet *)malloc(sizeof(rdda_slavet));
    if (slave == NULL) {
        return NULL;
    }

    slave->motor = (BEL_slave *)malloc(2*sizeof(BEL_slave));
    if (slave->motor == NULL) {
        free(slave);
        return NULL;
    }
    memset(slave->motor, 0, 2*sizeof(BEL_slave));
    slave->motor[0].slave_id = id->motor1;
    slave->motor[1].slave_id = id->motor2;
    for (int i = 0; i < 2; i++) {
        slave->motor[i].in_motor = (in_motor_t *)ec_slave[slave->motor[i].slave_id].inputs;
        slave->motor[i].out_motor = (out_motor_t *)ec_slave[slave->motor[i].slave_id].outputs;
    }

    slave->psensor = (EL3102_slave *)malloc(sizeof(EL3102_slave));
    if (slave->psensor == NULL) {
        free(slave);
        free(slave->motor);
        return NULL;
    }
    memset(slave->psensor, 0, sizeof(EL3102_slave));
    slave->psensor[0].slave_id = id->psensor;
    slave->psensor->in_pressure = (in_pressure_t *)ec_slave[slave->psensor[0].slave_id].inputs;

    return slave;
}
*/

/** Initialize RDDA_slave struct.
 *
 * @param slave_id  =   Slave index.
 * @return struct pointer.
 */
 /*
RDDA_slave *init_RDDA_slave(SlaveIndex *slave_id)
{
    SlaveIndex *idx = slave_id;
    RDDA_slave *slave;

    slave = (RDDA_slave *)malloc(sizeof(RDDA_slave));
    if (slave == NULL) {
        return NULL;
    }

    slave->motor[0].slave_id = idx->motor1;
    slave->motor[1].slave_id = idx->motor2;
    for (int i = 0; i < 2; i++) {
        slave->motor[i].in_motor = (MotorIn *)ec_slave[slave->motor[i].slave_id].inputs;
        slave->motor[i].out_motor = (MotorOut *)ec_slave[slave->motor[i].slave_id].outputs;
    }

    slave->psensor->slave_id = idx->psensor;
    slave->psensor->in_pressure = (PressureIn *)ec_slave[slave->psensor->slave_id].inputs;
}
  */

/** Free RDDA_slave struct.
 *
 * @param rdda_slave    =   RDDA_slave struct.
 */
void free_RDDA_slave(RDDA_slave *rdda_slave)
{
    free(rdda_slave);
}