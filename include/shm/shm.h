#ifndef RDDA_SHM_H
#define RDDA_SHM_H

#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>

#define OPEN_FLAG   (O_RDWR|O_CREAT|O_TRUNC)
#define MODE_FLAG   0777
#define SHM_SIZE    4096

//char* SHARED_IN = (char*)"/shared_input";
//char* SHARED_OUT = (char*)"/shared_output";

int mutex_init(pthread_mutex_t *mutex);
int openSharedMemory(char *shm_name, void **p);
int mutex_lock(pthread_mutex_t *mutex);
int mutex_unlock(pthread_mutex_t *mutex);

#endif //RDDA_SHM_H
