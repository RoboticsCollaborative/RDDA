#ifndef RDDA_SHM_DATA_H
#define RDDA_SHM_DATA_H

#include <stdint.h>
#include <pthread.h>

#define MOTOR_COUNT 6

/** AEV drive CSP Mode inputs to master */
typedef struct {
    double act_pos;
    double act_vel;
    double act_tau;
    // double load_pos;
    // double load_vel;
    double act_pre;
} MotorIn;

/** AEV drive CSP Mode outputs from master */
typedef struct {
    double tg_pos;
    double vel_off;
    double tau_off;
} MotorOut;

/** RDDAPacket transmitted using ROS */
typedef struct {
    double pos_in;
    double pos_out;
    double vel_in;
    double vel_out;
    double pos_ref;  /* for homing */
    // double vel;
    double tau;
    double wave_in;
    double wave_out;
    double wave_in_aux;
    double wave_out_aux;
    int contact_flag;
    double test;
} RDDAPacket;

/** AEV slave class */
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
} AEV_slave;

/** Timestamp */
typedef struct {
    int64_t sec;
    int64_t nsec;
} Timestamp;

/** Error signal*/
typedef struct {
    int error_in;
    int error_out;
} Error_signal;

/* time stamp */
typedef struct {
    double remote_stamp;
    int delay_cycle;
} Stamp;

/** EtherCAT slave class */
typedef struct {
    AEV_slave motor[MOTOR_COUNT];
    double freq_anti_alias;
    Timestamp ts;
    pthread_mutex_t mutex;
    Error_signal error_signal;
    Stamp stamp;
} Rdda;

#endif //RDDA_SHM_DATA_H