/* rddaconfig.c */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ethercat.h"
#include "rdda/rddaconfig.h"
#include "rdda/init_BEL.h"
#include "rdda/rddauser.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define COUNTS_PER_RADIAN 52151.8917

/* SOEM global vars */
char IOmap[4096];
int expectedWKC;
volatile int wkc;
boolean inOP;
boolean needlf;
uint8 currentgroup = 0;

/** Locate and identify EtherCAT slaves.
 *
 * @param rdda_slave    = Slave index group.
 * @return 0 on success.
 */
static int slaveIdentify(rdda_slavet *rdda_slave)
{
    uint16 slaveIdx = 0;
    int buf = 0;
    for (slaveIdx = 1; slaveIdx <= ec_slavecount; slaveIdx++)
    {
        /* BEL motor drive */
        if ((ec_slave[slaveIdx].eep_man == 0x000000ab) && (ec_slave[slaveIdx].eep_id == 0x00001110))
        {
            uint32 serial_num;
            buf = sizeof(serial_num);
            ec_SDOread(slaveIdx, 0x1018, 4, FALSE, &buf, &serial_num, EC_TIMEOUTRXM);

            /* motor1 */
            if (serial_num == 0x2098302)
            {
                rdda_slave->motor1 = slaveIdx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[slaveIdx].name, slaveIdx);
                if (1 == mapMotorPDOs_callback(slaveIdx))
                {
                    fprintf(stderr, "Motor1 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor2 */
            if (serial_num == 0x2098303)
            {
                rdda_slave->motor2 = slaveIdx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[slaveIdx].name, slaveIdx);
                if (1 == mapMotorPDOs_callback(slaveIdx))
                {
                    fprintf(stderr, "Motor2 mapping failed!\n");
                    exit(1);
                }
            }
        }
        /* pressure sensor */
        if ((ec_slave[slaveIdx].eep_man == 0x00000002) && (ec_slave[slaveIdx].eep_id == 0x0c1e3052))
        {
            rdda_slave->psensor = slaveIdx;
        }
    }

    return 0;
}

/** Close socket */
void rddaStop()
{
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
void *rddaEcatConfig(void *ifnameptr)
{
    char *ifname  = (char *)ifnameptr;

    inOP = FALSE;
    needlf = FALSE;
    rdda_slavet *rdda_slave;
    rdda_slave = (rdda_slavet *)malloc(sizeof(rdda_slavet));
    if (rdda_slave == NULL)
    {
        return NULL;
    }
    rdda_slave->motor1 = 0;
    rdda_slave->motor2 = 0;
    rdda_slave->psensor = 0;
    
    printf("Begin network configuration\n");

    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
	    fprintf(stderr, "Socket connection on %s succeeded.\n", ifname);
    }
    else
    {
	    fprintf(stderr, "No socket connection on %s.\nExcecuted as root\n", ifname);
	    exit(1);
    }

    /* Find and configure slaves */
    if (ec_config_init(FALSE) > 0)
    {
	    fprintf(stderr, "%d slaves found and configured.\n", ec_slavecount);
    }
    else
    {
	    fprintf(stderr, "No slaves found!\n");
	    rddaStop();
	    exit(1);
    }

    /* Request for PRE-OP mode */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    /* Locate slaves */
    slaveIdentify(rdda_slave);
    if (rdda_slave->motor1 == 0 || rdda_slave->motor2 == 0 || rdda_slave->psensor == 0)
    {
        fprintf(stderr, "Slaves identification failure!");
        exit(1);
    }

    /* If Complete Access (CA) disabled => auto-mapping work */
    ec_config_map(&IOmap);

    /* Let DC off for the time being */
    //ec_configdc(); // DC should be launched for each identified slave

    printf("Slaves mapped, state to SAFE_OP\n");
    /* Wait for all salves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    /* Initialize motor params */
    if (initMotor(rdda_slave->motor1) || initMotor(rdda_slave->motor2))
    {
        rddaStop();
        fprintf(stderr, "Motor initialization failure!");
    }
    printf("Slaves initialized, state to OP\n");

    /* Check if all slaves are working properly */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /* Request for OP mode */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* Send one valid process data to make outputs in slave happy */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    //wkc = ec_receive_processdata(EC_TIMEOUTRET);
    /* Request OP state for all slaves */
    ec_writestate(0);
    /* Wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves\n");
    }
    else
    {
        rddaStop();
        fprintf(stderr, "Operational state failed\n");
        pthread_exit(NULL);
    }

    return rdda_slave;
}

/** Add ns to timespec.
 *
 * @param ts  =   Structure holding an interval broken down into seconds and nanoseconds
 * @param addtime   =   Elapsed time interval added to previous time.
 */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/** PI calculation to get linux time synced to DC time
 *
 * @param reftime   =   Reference DCtime.
 * @param cycletime    =   Cycle time of PDO transfer.
 * @param offsettime    =   Offset time.
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta/100)-(integral/20);
}

/** Thread for PDO cyclic transmission.
 *
 * @param[in] rdda_slave     =   Slave indices.
 */
void rddaCyclic(void *rdda_slave)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime, toff;
    int ctime;
//    rdda_slavet *slave;
    double theta1_rad;


//    slave = (rdda_slavet *)rdda_slave;

    /* Free the slave structure */
    free(rdda_slave);

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    ctime = 500; /* cycletime in us */
    cycletime = ctime * 1000; /* cycletime in ns */
    toff = 0;
    inOP = TRUE;
    ec_send_processdata();

    while(1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        /**
         *  Cylic part
         */
        theta1_rad = readMotor1Pos()/COUNTS_PER_RADIAN;
        printf("Motor1 Position: %lf\n", (double)theta1_rad);
        needlf = TRUE;

        /* Calulate toff to get linux time and DC synced */
        ec_sync(ec_DCtime, cycletime, &toff);
        ec_send_processdata();

    }
}

/** Error handling in OP mode.
 *
 * @param ptr   =   NULL;
 */
void ecatcheck(void *ptr)
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}