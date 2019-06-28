#include "shm.h"

char* RDDA_DATA     =   (char *)"/rdda_data";

/** Acquire robust mutex
  *
  * @param[in] mutex     =       mutex lock
  * return 0 if owner died, return -1 on error, return 1 on success.
  */
int mutex_lock(pthread_mutex_t *mutex) {
    int err = pthread_mutex_lock(mutex);

    if (err == 0) {
        return 1; /* Acquire mutex success */
    }
    else if (err == EOWNERDEAD) {
        pthread_mutex_consistent(mutex);
        return 0; /* Mutex dead */
    }
    else {
        perror("pthread_mutex_lock");
        return 0;
    }
}

/** Release robust mutex
  *
  * @param[in] mutex     =       mutex lock
  * return -1 on error, return 0 on success.
 */
int mutex_unlock(pthread_mutex_t *mutex) {

    int err = pthread_mutex_unlock(mutex);

    if (!err) return 0; /* Mutex unlock success */

    perror("pthread_mutex_unlock");
    return -1;
}

/** Initialise mutex lock and condition variable.
  *
  * @param[in] mutex    =       mutex lock.
  * return 1 on success.
  */
static int
mutex_init(pthread_mutex_t *mutex) {
    /* Initialise mutex */
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
    pthread_mutexattr_setrobust(&mattr, PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(mutex, &mattr);
    pthread_mutexattr_destroy(&mattr);

    return 1;
}

/** Open and map a shared memory file.
 *
 * @param[in] shm_name =        Shared memory file name.
 * @param[in] p =       Intermediate pointer.
 * return 0 on success.
 */
static int
createSharedMemory(char *shm_name, void **p) {

    int fd = 0, ret = 0, err = 0; /* error detector*/

    /* Create or open a POSIX shared memory object */
    mode_t old_umask = umask(0);
    fd = shm_open(shm_name, O_RDWR | O_CREAT | O_TRUNC, 0777);  /* return 0 on success, -1 on error */
    err = fd < 0;

    /* Resize the shared memory file */
    if (!err) {
        ret = ftruncate(fd, SHM_SIZE);  /* return 0 on success, -1 on error */
        err = ret < 0;
    }

    /* Map shared memory to process virtual memory space */
    if (!err) {
        *p = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);   /* return pointer on success, (void*)-1 on error */
        err = (*p == (void *)-1);
    }

    /* Close the file descriptor */
    if (!err) {
        err = close(fd);
    }

    umask(old_umask);
    return err;
}

/** Initialize jointCommands to shared memory and robust mutex.
 *
 * @return jointCommands pointer.
 */
Rdda *initRdda() {

    Rdda *rdda;
    void *p;

    if (!createSharedMemory(RDDA_DATA, &p)) {
        rdda = (Rdda *) p;
    } else {
        fprintf(stderr, "open(RDDA_DATA)\n");
        return NULL;
    }

    /* initialise mutex lock */
    mutex_init(&rdda->mutex);

    return rdda;
}