#ifndef RDDA_SHM_DATA_H
#define RDDA_SHM_DATA_H

#include <stdint.h>
#include <pthread.h>

/** BEL drive CSP Mode inputs to master */
typedef struct {
    double act_pos;
    double act_vel;
    double act_tau;
} MotorIn;

/** BEL drive CSP Mode outputs from master */
typedef struct {
    double tg_pos;
    double vel_off;
    double tau_off;
} MotorOut;

/* Reference signals from ROS interface */
typedef struct {
    double pos_ref;
    double vel_sat;
    double tau_sat;
    double stiffness;
} RosOut;

/** EL3102 pressure sensor inputs to master */
typedef struct {
    double val1;
    double val2;
} AnalogIn;

/** BEL slave class */
typedef struct {
    MotorIn motorIn;
    MotorOut motorOut;
    RosOut rosOut;
    /* Constant */
    double tau_max;
    double init_pos;
    /* SDO */
    int Pp;
    int Vp;
} BEL_slave;

/** EL3102 slave class */
typedef struct {
    AnalogIn analogIn;
} EL3102_slave;

/** Timestamp */
typedef struct {
    int64_t sec;
    int64_t nsec;
} Timestamp;

/** EtherCAT slave class */
typedef struct {
    BEL_slave motor[2];
    EL3102_slave psensor;
    double freq_anti_alias;
    Timestamp ts;
    pthread_mutex_t mutex;
} Rdda;

#endif //RDDA_SHM_DATA_H
