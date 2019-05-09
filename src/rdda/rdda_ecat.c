/** rdda_ecat.c */

#include "rdda_ecat.h"

/* SOEM global vars */
char IOmap[4096];
//boolean inOP;
//boolean needlf;
//uint8 currentgroup = 0;

void delete_RDDA_slave(RDDA_slave *slave)
{
    /*
    for (int mot_id = 0; mot_id < 2; mot_id ++)
    {
        free(slave->motor[mot_id]);
    }
    free(slave->psensor);
    */
    free(slave);
}

/** Close socket */
void rddaStop(RDDA_slave *slave)
{
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    delete_RDDA_slave(slave);
    printf("Close socket\n");
    ec_close(); /* stop SOEM, close socket */
}

/** Locate and identify EtherCAT slaves.
 *
 * @param rdda_slave    = Slave index group.
 * @return 0 on success.
 */
static int slaveIdentify(RDDA_slave *slave)
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
            slave->psensor.slave_id = idx;
        }
    }

    return 0;
}

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
RDDA_slave *rddaEcatConfig(void *ifnameptr)
{
    char *ifname  = (char *)ifnameptr;

//    inOP = FALSE;
//    needlf = FALSE;
    int expectedWKC;
    int wkc;

/*
    SlaveIndex *slaveIndex;
    slaveIndex = (SlaveIndex *)malloc(sizeof(SlaveIndex));
    if ( slaveIndex == NULL)
    {
        return NULL;
    }
    memset(slaveIndex, 0, sizeof(SlaveIndex));
*/

    /* Initialize data structure */
    printf("Init data structure.\n");
    RDDA_slave *rddaSlave;
    rddaSlave = (RDDA_slave *)malloc(sizeof(RDDA_slave));
    if (rddaSlave == NULL)
    {
        return NULL;
    }

    /*
    rddaSlave->motor = (BEL_slave **)malloc(2 * sizeof(BEL_slave *));
    if (rddaSlave->motor == NULL)
    {
        free(rddaSlave);
        return NULL;
    }
    rddaSlave->psensor = (EL3102_slave *)malloc(sizeof(EL3102_slave));
    if (rddaSlave->psensor == NULL)
    {
        free(rddaSlave);
        return NULL;
    }
    */

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
    slaveIdentify(rddaSlave);
    printf("psensor_id: %d\n", rddaSlave->psensor.slave_id);
    if (rddaSlave->motor[0].slave_id == 0 || rddaSlave->motor[1].slave_id == 0 || rddaSlave->psensor.slave_id == 0)
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

    /* Initialize motor params */
    /*
    if (initMotor(rdda_slave->motor1) || initMotor(rdda_slave->motor2))
    {
        ec_close();
        fprintf(stderr, "Motor initialization failure!");
    }
     */
    initMotor(rddaSlave->motor[0].slave_id);
    initMotor(rddaSlave->motor[1].slave_id);
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
        rddaSlave->motor[mot_id].in_motor = (MotorIn *)ec_slave[rddaSlave->motor[mot_id].slave_id].inputs;
    }
    rddaSlave->psensor.in_pressure = (PressureIn *)ec_slave[rddaSlave->psensor.slave_id].inputs;

    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves\n");
        return rddaSlave;
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

/** Sync PDO data by layers
 *
 * @param rddaSlave
 * @param jointStates
 */
void rdda_update(RDDA_slave *rddaSlave, JointStates *jointStates)
{
    ec_receive_processdata(EC_TIMEOUTRET);

    /* Update joint states */
    mutex_lock(&jointStates->mutex);

    for (int i = 0; i < 2; i++) {
        jointStates->stat_wd[i] = (int)(rddaSlave->motor[i].in_motor->stat_wd);
        jointStates->act_pos[i] = (double)(rddaSlave->motor[i].in_motor->act_pos/COUNTS_PER_RADIAN);
        jointStates->act_vel[i] = (double)(rddaSlave->motor[i].in_motor->act_vel/COUNTS_PER_RADIAN/10.0);
    }
    jointStates->act_tau[0] = (double)(rddaSlave->psensor.in_pressure->val1 * PASCAL_PER_COUNT * NM_PER_PASCAL);
    jointStates->act_tau[1] = (double)(rddaSlave->psensor.in_pressure->val2 * PASCAL_PER_COUNT * NM_PER_PASCAL);

    mutex_unlock(&jointStates->mutex);

    /* Update joint commands */
/*
    mutex_lock(&jointCommands->mutex);

    for (int j = 0; j < 2; j++) {
        rddaSlave->motor[j].out_motor->ctrl_wd = (uint16)(jointCommands->ctrl_wd[j]);
        rddaSlave->motor[j].out_motor->tg_pos = (int32)(jointCommands->tg_pos[j] * (double)COUNTS_PER_RADIAN);
        rddaSlave->motor[j].out_motor->vel_off = (int32)(jointCommands->vel_off[j] * (double)COUNTS_PER_RADIAN * 10.0);
        rddaSlave->motor[j].out_motor->tau_off = (int16)(jointCommands->tau_off[j] * (double)UNITS_PER_NM);
    }

    mutex_unlock(&jointCommands->mutex);
*/

    ec_send_processdata();
}

void rdda_gettime(RDDA_slave *rddaSlave)
{
    int64 ht;
    int64 pre_time, current_time;
    int64 nsec_per_sec = 1000000000; int64 msec_per_nsec = 1000000;
    pre_time = rddaSlave->time.ts.tv_sec * nsec_per_sec + rddaSlave->time.ts.tv_nsec;
    clock_gettime(CLOCK_MONOTONIC, &rddaSlave->time.ts);
    ht = (rddaSlave->time.ts.tv_nsec / msec_per_nsec) + 1; /* round to nearest ms */
    rddaSlave->time.ts.tv_nsec = ht * msec_per_nsec;
    current_time = rddaSlave->time.ts.tv_sec * nsec_per_sec + rddaSlave->time.ts.tv_nsec;
    rddaSlave->time.delta_time = current_time - pre_time;
}

void rdda_sleep(RDDA_slave *rddaSlave, int cycletime)
{
    int64 cycletime_ns = cycletime * 1000;
//    int toff = 0;
//    if (ec_slave[0].hasdc) {
//        toff = ec_sync(ec_DCtime, cycletime);
//    }
//    add_timespec(&rddaSlave->time.ts, cycletime_ns - rddaSlave->time.delta_time + toff);
//    add_timespec(&rddaSlave->time.ts, cycletime_ns + toff);
    add_timespec(&rddaSlave->time.ts, cycletime_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rddaSlave->time.ts, NULL);
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