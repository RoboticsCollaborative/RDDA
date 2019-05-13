#ifndef RDDA_SHM_DATA_H
#define RDDA_SHM_DATA_H

#include <stdint.h>
#include <pthread.h>

/** BEL drive CSP Mode inputs to master */
typedef struct
{
    double act_pos;
    double act_vel;
} MotorIn;

/** BEL drive CSP Mode outputs from master */
typedef struct
{
    double tg_pos;
    double vel_off;
    double tau_off;
} MotorOut;

/** EL3102 pressure sensor inputs to master */
typedef struct
{
    double val1;
    double val2;
} AnalogIn;

/** BEL slave class */
typedef struct
{
    MotorIn motorIn;
    MotorOut motorOut;
    int Pp;
    int Vp;
} BEL_slave;

/** EL3102 slave class */
typedef struct
{
    AnalogIn analogIn;
} EL3102_slave;

/** EtherCAT slave class */
typedef struct
{
    BEL_slave motor[2];
    EL3102_slave psensor;
    struct timespec ts;
    pthread_mutex_t mutex;
} RDDA_slave;

/*
typedef struct
{
    int64_t sec;
    int64_t nsec;
} Timestamp;

typedef struct
{
    pthread_mutex_t mutex;
    Timestamp timestamp;
    int ctrl_wd[2];
    double tg_pos[2];
    double vel_off[2];
    double tau_off[2];
} JointCommands;

typedef struct
{
    pthread_mutex_t mutex;
    Timestamp timestamp;
    int stat_wd[2];
    double act_pos[2];
    double act_vel[2];
    double act_tau[2];
} JointStates;
*/

#endif //RDDA_SHM_DATA_H
