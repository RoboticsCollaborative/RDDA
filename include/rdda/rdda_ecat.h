#ifndef RDDA_ECAT_H
#define RDDA_ECAT_H

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
} MotorIn;

/** BEL drive CSP Mode outputs from master */
typedef struct PACKED
{
    uint16 ctrl_wd;   /* control word (0x6040) */
    int32 tg_pos;     /* target position (0x607A) */
    int32 vel_off;    /* velocity offset (0x60B1) */
    int16 tau_off;    /* torque offset (0x60B2) */
} MotorOut;

/** EL3102 pressure sensor inputs to master */
typedef struct PACKED
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
    int64 COUNTS_PER_RADIAN;
} BEL_slave;

/** EL3102 slave class */
typedef struct
{
    int slave_id;
    PressureIn *in_pressure;
} EL3102_slave;

/** EtherCAT slave class */
typedef struct
{
    BEL_slave *motor;
    EL3102_slave *psensor;
} RDDA_slave;

/** EtherCAT slave index class */
typedef struct
{
    int motor[2];
    int psensor;
} SlaveIndex;

SlaveIndex *rddaEcatConfig(void *ifnameptr);
void pdoUpdata(void *slave_id);
void ecatcheck(void *ptr);
void rddaStop();

#endif //RDDA_ECAT_H