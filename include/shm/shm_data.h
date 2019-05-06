#ifndef RDDA_SHM_DATA_H
#define RDDA_SHM_DATA_H

#include <stdint.h>
#include <pthread.h>

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

#endif //RDDA_SHM_DATA_H
