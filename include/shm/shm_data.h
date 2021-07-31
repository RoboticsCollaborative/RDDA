#ifndef RDDA_SHM_DATA_H
#define RDDA_SHM_DATA_H

#include <stdint.h>
#include <pthread.h>

/*
+-----------------+         +------------------+          +-------------------+
|                 | MotorOut|                  | RDDAWrite|                   |
|                 <---------+                  <----------+                   |
|    Bel          |         |     RDDA         |          |        ROS        |
|                 +--------->                  +---------->                   |
|                 | MotorIn |                  | RDDARead |                   |
+-----------------+         +------------------+          +-------------------+
 */

/** BEL drive CSP Mode inputs to master */
typedef struct {
    double act_pos;
    double act_vel;
    double act_tau;
    double load_pos;
    double load_vel;
} MotorIn;

/** BEL drive CSP Mode outputs from master */
typedef struct {
    double tg_pos;
    double vel_off;
    double tau_off;
} MotorOut;

/** RDDAPacket transmitted using ROS */
typedef struct {
    double pos_in;
    double pos_out;
    double vel;
    double tau;
    double wave_in;
    double wave_out;
    int contact_flag;
} RDDAPacket;

/** EL3102 and EL3702 pressure sensor inputs to master */
typedef struct {
    double val1;
    double val2;
} AnalogIn;

/** BEL slave class */
typedef struct {
    MotorIn motorIn;
    MotorOut motorOut;
    RDDAPacket rddaPacket;
    /* Constant */
    double init_pos;
    /* Parameter */
    double vel_sat;
    double tau_sat;
    double stiffness;
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