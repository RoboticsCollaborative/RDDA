#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ethercat.h"
#include "rdda/rddaconfig.h"
#include "rdda/rddamain.h"

/** BEL drive CSP Mode inputs to master */
PACKED_BEGIN
typedef struct PACKED
{
    uint16 stat_wd;   /* status word (0x6041) */
    int32 act_pos;    /* position actual value (0x6064) */
    int32 pos_err;    /* position error (0x60F4) */
    int32 act_vel;    /* actual velocity (0x606C) */
    int16 act_tau;    /* torque actual value (0x6077) */
    int32 load_vel;   /* load encoder velocity (0x2231) */
    int32 load_pos;   /* load encoder position (0x2242) */
} in_motor_t;
PACKED_END

/** BEL drive CSP Mode outputs from master */
PACKED_BEGIN
typedef struct PACKED
{
    uint16 ctrl_wd;   /* control word (0x6040) */
    int32 tg_pos;     /* target position (0x607A) */
    int32 vel_off;    /* velocity offset (0x60B1) */
    int16 tau_off;    /* torque offset (0x60B2) */
} out_motor_t;
PACKED_END

/** EL3102 pressure sensor inputs to master */
PACKED_BEGIN
typedef struct PACKED
{
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} in_pressure_t;
PACKED_END


/** General macros */
#define     COUNTS_PER_RADIAN   52151.8917
#define     PASCAL_PER_COUNT    21.04178
#define     NM_PER_PASCAL       2.822e-6
#define     UNITS_PER_NM        5000


/** Read motor actual position via motor encoder [units in radian].
 *
 * @param slaveIdx  =   Slave index.
 * @return position actual value.
 */
static double readMotorPos_t(uint16 slaveIdx)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_motor->act_pos)/COUNTS_PER_RADIAN;
}


/** Read motor actual velocity via motor encoder [units in radian/s].
 *
 * @param slaveIdx  =   Slave index.
 * @return velocity actual value.
 */
static double readMotorVel_t(uint16 slaveIdx)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_motor->act_vel)/COUNTS_PER_RADIAN/10.0;
}


/** Read finger actual position via loaded encoder [units in radian].
 *
 * @param slaveIdx  =   Slave index.
 * @return loaded position actual value.
 */
static double readLoadPos_t(uint16 slaveIdx)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_motor->load_pos)/COUNTS_PER_RADIAN;
}


/** Read finger actual velocity via loaded encoder [units in radian/s].
 *
 * @param slaveIdx
 * @return loaded velocity actual value.
 */
static double readLoadVel_t(uint16 slaveIdx)
{
    in_motor_t *in_motor = (in_motor_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_motor->load_vel)/COUNTS_PER_RADIAN/10.0;
}


/** Read pressure value on finger1 [units in Nm].
 *
 * @param slaveIdx  =   Slave index.
 * @return pressure actual value.
 */
static double readPre1Val_t(uint16 slaveIdx)
{
    in_pressure_t *in_pressure = (in_pressure_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_pressure->val1) * PASCAL_PER_COUNT * NM_PER_PASCAL;
}


/** Read pressure value on finger2 [units in Nm].
 *
 * @param slaveIdx  =   Slave index.
 * @return pressure actual value.
 */
static double readPre2Val_t(uint16 slaveIdx)
{
    in_pressure_t *in_pressure = (in_pressure_t *)ec_slave[slaveIdx].inputs;
    return (double)(in_pressure->val2) * PASCAL_PER_COUNT * NM_PER_PASCAL;
}


/** Write control word to motor.
 *  control word 15 =>  run motor.
 *
 * @param slavIdx   =   Slave index.
 * @param value     =   Control word.
 * @return 0.
 */
static int writeCtrlWd_t(uint16 slaveIdx, uint16 value)
{
    out_motor_t *out_motor = (out_motor_t *)ec_slave[slaveIdx].outputs;
    out_motor->ctrl_wd = value;

    return 0;
}


/** Write target position value to motor [units in radian].
 *
 * @param slavIdx   =   Slave index.
 * @param value     =   Motor angle.
 * @return 0.
 */
static int writeTarPos_t(uint16 slaveIdx, double value)
{
    out_motor_t *out_motor = (out_motor_t *)ec_slave[slaveIdx].outputs;
    out_motor->tg_pos = (int32)(value * COUNTS_PER_RADIAN);

    return 0;
}


/** Write torque offset value to motor [Units in Nm].
 *
 * @param slaveIdx  =   Slave index.
 * @param value     =   Motor torque.
 * @return 0.
 */
static int writeTauOffset_t(uint16 slaveIdx, double value)
{
    out_motor_t *out_motor = (out_motor_t *)ec_slave[slaveIdx].outputs;
    out_motor->tau_off = (int16)(value * (double)UNITS_PER_NM);

    return 0;
}


double readMotor1Pos()
{
    return readMotorPos_t(rdda_slave->motor1);
}


double readMotor2Pos()
{
    return readMotorPos_t(rdda_slave->motor2);
}


double readMotor1Vel()
{
    return readMotorVel_t(rdda_slave->motor1);
}


double readMotor2Vel()
{
    return readMotorVel_t(rdda_slave->motor2);
}


double readLoad2Pos()
{
    return readLoadPos_t(rdda_slave->motor2);
}


double readLoad2Vel()
{
    return readLoadVel_t(rdda_slave->motor2);
}


double readPre1Val()
{
    return readPre1Val_t(rdda_slave->psensor);
}


double readPre2Val()
{
    return readPre2Val_t(rdda_slave->psensor);
}


void writeMotor1CtrlWd(uint16 value)
{
    writeCtrlWd_t(rdda_slave->motor1, value);
}


void writeMotor2CtrlWd(uint16 value)
{
    writeCtrlWd_t(rdda_slave->motor2, value);
}


void writeMotor1Pos(double value)
{
    writeTarPos_t(rdda_slave->motor1, value);
}


void writeMotor2Pos(double value)
{
    writeTarPos_t(rdda_slave->motor2, value);
}


void writeMotor1Tau(double value)
{
    writeTauOffset_t(rdda_slave->motor1, value);
}


void writeMotor2Tau(double value)
{
    writeTauOffset_t(rdda_slave->motor2, value);
}


/** Connect local  with EtherCAT
 *
 * @param in_motor
 * @param out_motor
 * @param rdda_slave
 * @param slaveIdx
 * @return
 */
/*
int motorConnect(in_motor_t **in_motor, out_motor_t **out_motor, rdda_slavet *rdda_slave, uint16 slaveIdx)
{
   *in_motor = (in_motor_t *)ec_slave[slaveIdx].inputs;
   *out_motor = (out_motor_t *)ec_slave[slaveIdx].outputs;

   return 0;
}
*/