#ifndef RDDA_SHM_H
#define RDDA_SHM_H

#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "shm_data.h"

#define SHM_SIZE    4096*2


Rdda *initRdda(char* name);
int mutex_lock(pthread_mutex_t *mutex);
int mutex_unlock(pthread_mutex_t *mutex);

#endif //RDDA_SHM_H
