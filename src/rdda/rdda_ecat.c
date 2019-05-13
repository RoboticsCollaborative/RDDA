/** rdda_ecat.c */

#include "rdda_ecat.h"

/* SOEM global vars */
char IOmap[4096];
//boolean inOP;
//boolean needlf;
//uint8 currentgroup = 0;

void delete_rdda_slave(ecat_slave *slave)
{
    free(slave);
}

/** Close socket */
void rddaStop(ecat_slave *slave)
{
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    delete_rdda_slave(slave);
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}

/** Locate and identify EtherCAT slaves.
 *
 * @param rdda_slave    = Slave index group.
 * @return 0 on success.
 */
static int slaveIdentify(ecat_slave *slave)
{
    uint16 idx = 0;
    int buf = 0;
    for (idx = 1; idx <= ec_slavecount; idx++)
    {
        /* BEL motor drive */
        if ((ec_slave[idx].eep_man == 0x000000ab) && (ec_slave[idx].eep_id == 0x00001110))
        {
            uint32 serial_num;
            buf = sizeof(serial_num);
            ec_SDOread(idx, 0x1018, 4, FALSE, &buf, &serial_num, EC_TIMEOUTRXM);

            /* motor1 */
            if (serial_num == 0x2098302)
            {
                //network->motor[0] = idx;
                slave->motor[0].slave_id = idx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx))
                {
                    fprintf(stderr, "Motor1 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor2 */
            if (serial_num == 0x2098303)
            {
                slave->motor[1].slave_id = idx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx))
                {
                    fprintf(stderr, "Motor2 mapping failed!\n");
                    exit(1);
                }
            }
        }
        /* pressure sensor */
        if ((ec_slave[idx].eep_man == 0x00000002) && (ec_slave[idx].eep_id == 0x0c1e3052))
        {
            slave->slave.slave_id = idx;
        }
    }

    return 0;
}

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
ecat_slave *rddaEcatConfig(void *ifnameptr)
{
    char *ifname  = (char *)ifnameptr;

//    inOP = FALSE;
//    needlf = FALSE;
    int expectedWKC;
    int wkc;

    /* Initialize data structure */
    printf("Init data structure.\n");
    ecat_slave *ecatSlave;
    ecatSlave = (ecat_slave *)malloc(sizeof(ecat_slave));
    if (ecatSlave == NULL)
    {
        return NULL;
    }

    printf("Begin network configuration\n");
    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
	    printf("Socket connection on %s succeeded.\n", ifname);
    }
    else
    {
	    fprintf(stderr, "No socket connection on %s.\nExcecuted as root\n", ifname);
	    exit(1);
    }

    /* Find and configure slaves */
    if (ec_config_init(FALSE) > 0)
    {
	    printf("%d slaves found and configured.\n", ec_slavecount);
    }
    else
    {
	    fprintf(stderr, "No slaves found!\n");
	    ec_close();
	    exit(1);
    }

    /* Request for PRE-OP mode */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    /* Locate slaves */
    slaveIdentify(ecatSlave);
    printf("psensor_id: %d\n", ecatSlave->slave.slave_id);
    if (ecatSlave->motor[0].slave_id == 0 || ecatSlave->motor[1].slave_id == 0 || ecatSlave->slave.slave_id == 0)
    {
        fprintf(stderr, "Slaves identification failure!");
        exit(1);
    }

    /* If Complete Access (CA) disabled => auto-mapping work */
    ec_config_map(&IOmap);

    /* Let DC off for the time being */
    ec_configdc(); // DC should be launched for each identified slave

    printf("Slaves mapped, state to SAFE_OP\n");
    /* Wait for all salves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    initMotor(ecatSlave->motor[0].slave_id);
    initMotor(ecatSlave->motor[1].slave_id);
    printf("Slaves initialized, state to OP\n");

    /* Check if all slaves are working properly */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /* Request for OP mode */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* Send one valid process data to make outputs in slave happy */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc < expectedWKC)
    {
        fprintf(stderr, "WKC failure.\n");
        exit(1);
    }

    /* Request OP state for all slaves */
    ec_writestate(0);
    /* Wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    for (int mot_id = 0; mot_id < 2; mot_id ++)
    {
        ecatSlave->motor[mot_id].in_motor = (motor_input *)ec_slave[ecatSlave->motor[mot_id].slave_id].inputs;
    }
    ecatSlave->slave.in_analog = (analog_input *)ec_slave[ecatSlave->slave.slave_id].inputs;

    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves\n");
        return ecatSlave;
    }
    else
    {
        ec_close();
        fprintf(stderr, "Operational state failed\n");
        exit(1);
    }
}

/** Add ns to timespec.
 *
 * @param ts  =   Structure holding an interval broken down into seconds and nanoseconds
 * @param addtime   =   Elapsed time interval added to previous time.
 */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;
    int64 NSEC_PER_SEC = 1000000000;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;

    if (ts->tv_nsec > NSEC_PER_SEC) {
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
int64 ec_sync(int64 reftime, int64 cycletime)
{
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    return -(delta/100)-(integral/20);
}

/** Get system time in microseconds (us)
 *
 * @param rddaSlave     =   rdda structure.
 * @return system time at nearest us.
 */
int rdda_gettime(ecat_slave *rddaSlave)
{
    int64 nsec_per_sec = 1000000000;
    clock_gettime(CLOCK_MONOTONIC, &rddaSlave->ts);
    return (int)(rddaSlave->ts.tv_sec * nsec_per_sec + rddaSlave->ts.tv_nsec) / 1000 + 1;
}

/** Sleep and calibrate DC time.
 *
 * @param rddaSlave     =   rdda structure.
 * @param cycletime     =   sleep time.
 */
void rdda_sleep(ecat_slave *rddaSlave, int cycletime)
{
    int64 cycletime_ns = cycletime * 1000;
    int toff = 0;
    if (ec_slave[0].hasdc) {
        toff = ec_sync(ec_DCtime, cycletime);
    }
    add_timespec(&rddaSlave->ts, cycletime_ns + toff);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rddaSlave->ts, NULL);
}

/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(ecat_slave *ecatSlave, RDDA_slave *rddaSlave)
{
    ec_receive_processdata(EC_TIMEOUTRET);

    /* Update joint states */
    mutex_lock(&rddaSlave->mutex);

    for (int i = 0; i < 2; i++) {
        rddaSlave->motor[i].motorIn.act_pos = (double)(ecatSlave->motor[i].in_motor->act_pos/COUNTS_PER_RADIAN);
        rddaSlave->motor[i].motorIn.act_vel = (double)(ecatSlave->motor[i].in_motor->act_vel/COUNTS_PER_RADIAN/10.0);
    }
    rddaSlave->psensor.analogIn.val1 = (double)(ecatSlave->slave.in_analog->val1 * PASCAL_PER_COUNT * NM_PER_PASCAL);
    rddaSlave->psensor.analogIn.val2 = (double)(ecatSlave->slave.in_analog->val2 * PASCAL_PER_COUNT * NM_PER_PASCAL);

    mutex_unlock(&rddaSlave->mutex);

    ec_send_processdata();
}

//#define EC_TIMEOUTMON 500

/** Error handling in OP mode.
 *
 * @param ptr   =   NULL;
 */
/*
void ecatcheck(void *ptr)
{
    int slave;
    (void)ptr;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            // one ore more slaves are not responding
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
                        // re-check state
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
*/