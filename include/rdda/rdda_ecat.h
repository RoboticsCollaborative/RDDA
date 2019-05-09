#ifndef RDDA_ECAT_H
#define RDDA_ECAT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "ethercat.h"
#include "init_BEL.h"
#include "shm_data.h"
#include "shm.h"

// #define NSEC_PER_SEC 1000000000
// #define COUNTS_PER_RADIAN 52151.8917

#define COUNTS_PER_RADIAN   52151.8917
#define COUNTS_PER_REV      327680
#define LOAD_COUNTS_PER_REV 40000
#define UNITS_PER_NM        5000
#define MAX_NM              5.0
#define PASCAL_PER_COUNT    21.04178
#define NM_PER_PASCAL 2.822e-6

/** BEL drive CSP Mode inputs to master */
typedef struct
{
    /* PDO */
    uint16 stat_wd;   /* status word (0x6041) */
    int32 act_pos;    /* position actual value (0x6064) */
    int32 pos_err;    /* position error (0x60F4) */
    int32 act_vel;    /* actual velocity (0x606C) */
    int16 act_tau;    /* torque actual value (0x6077) */
    int32 load_vel;   /* load encoder velocity (0x2231) */
    int32 load_pos;   /* load encoder position (0x2242) */
} MotorIn;

/** BEL drive CSP Mode outputs from master */
typedef struct
{
    uint16 ctrl_wd;   /* control word (0x6040) */
    int32 tg_pos;     /* target position (0x607A) */
    int32 vel_off;    /* velocity offset (0x60B1) */
    int16 tau_off;    /* torque offset (0x60B2) */
} MotorOut;

/** EL3102 pressure sensor inputs to master */
typedef struct
{
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} PressureIn;

/** BEL slave class */
typedef struct
{
    int slave_id;
    /* Input/output interface */
    MotorIn *in_motor;
    MotorOut *out_motor;
    /* Motor attributes */
    int64 countsPerRad;
} BEL_slave;

/** EL3102 slave class */
typedef struct
{
    int slave_id;
    PressureIn *in_pressure;
} EL3102_slave;

typedef struct
{
    struct timespec ts;
    int64 delta_time;
} Run_time;

/** EtherCAT slave class */
typedef struct
{
    BEL_slave motor[2];
    EL3102_slave psensor;
    Run_time time;
} RDDA_slave;

RDDA_slave *rddaEcatConfig(void *ifnameptr);
void rdda_update(RDDA_slave *rddaSlave, JointStates *jointStates);
void rddaStop(RDDA_slave *rddaSlave);
void rdda_gettime(RDDA_slave *rddaSlave);
void rdda_sleep(RDDA_slave *rddaSlave, int cycletime);
void ecatcheck(void *ptr);

#endif //RDDA_ECAT_H