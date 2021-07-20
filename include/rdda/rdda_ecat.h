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

/** BEL drive CSP Mode inputs to master */
typedef struct PACKED
{
    /* PDO */
    uint16 stat_wd;   /* status word (0x6041) */
    int32 act_pos;    /* position actual value (0x6064) */
    int32 pos_err;    /* position error (0x60F4) */
    int32 act_vel;    /* actual velocity (0x606C) */
    int16 act_tau;    /* torque actual value (0x6077) */
    int32 load_vel;   /* load encoder velocity (0x2231) */
    int32 load_pos;   /* load encoder position (0x2242) */
    int16 error_code; /* load error code (0x603F)*/
    int16 analog_in;  /* general analog input (0x2200) */
} motor_input;

/** BEL drive CSP Mode outputs from master */
typedef struct PACKED
{
    uint16 ctrl_wd;   /* control word (0x6040) */
    int32 tg_pos;     /* target position (0x607A) */
    int32 vel_off;    /* velocity offset (0x60B1) */
    int16 tau_off;    /* torque offset (0x60B2) */
} motor_output;

/** EL3102 pressure sensor inputs to master */
typedef struct PACKED
{
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} analog_input;

/** BEL slave class */
typedef struct
{
    uint16 slave_id;
    /* Input/output interface */
    motor_input *in_motor;
    motor_output *out_motor;
    /* Motor attributes */
    int32 init_pos_cnts;
    double counts_per_rad;
    double counts_per_rad_sec;
    double load_counts_per_rad;
    double load_counts_per_rad_sec;
    double pascal_per_count;
    double nm_per_pascal;
    double units_per_nm;
} bel_slave;

/** EL3102 slave class */
typedef struct
{
    uint16 slave_id;
    analog_input *in_analog;
} el3102_slave;

/** EtherCAT slave class */
typedef struct
{
    bel_slave bel[2];
    el3102_slave el3102;
    struct timespec ts;
} ecat_slaves;

ecat_slaves *initEcatConfig(void *ifnameptr);
void add_timespec(struct timespec *ts, int64 addtime);
int64 ec_sync(int64 reftime, int64 cycletime);
int32 positionSDOread(uint16 slave_id);
void pivGainSDOwrite(uint16 slave_id, uint16 Pp, uint16 Vp);
int rddaDriverErrorSDOcheck(uint16 slave_id);
void ecatcheck(void *ptr);

#endif //RDDA_ECAT_H
