/** rdda_ecat.c */

#include "rdda_ecat.h"

/* SOEM global vars */
char IOmap[4096];
//boolean inOP;
//boolean needlf;
//uint8 currentgroup = 0;

/** Locate and identify EtherCAT slaves.
 *
 * @param rdda_slave    = Slave index group.
 * @return 0 on success.
 */
static int
slaveIdentify(ecat_slaves *slave) {
    uint16 idx = 0;
    int buf = 0;
    for (idx = 1; idx <= ec_slavecount; idx++) {
        /* BEL motor drive */
        if ((ec_slave[idx].eep_man == 0x000000ab) && (ec_slave[idx].eep_id == 0x00001110)) {
            uint32 serial_num;
            buf = sizeof(serial_num);
            ec_SDOread(idx, 0x1018, 4, FALSE, &buf, &serial_num, EC_TIMEOUTRXM);

            /* motor1 */
            if (serial_num == 0x2098302) {
                //network->motor[0] = idx;
                slave->bel[0].slave_id = idx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor1 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor2 */
            if (serial_num == 0x2098303) {
                slave->bel[1].slave_id = idx;
                /* CompleteAccess disabled for BEL drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor2 mapping failed!\n");
                    exit(1);
                }
            }
        }
        /* pressure sensor */
        if ((ec_slave[idx].eep_man == 0x00000002) && (ec_slave[idx].eep_id == 0x0c1e3052)) {
            slave->el3102.slave_id = idx;
        }
    }

    return 0;
}

/** Initialize ETherCAT slavs memory and internal constant parameters.
 *
 * @param ecatSlave     =   EtherCAT slave structure.
 */
static void
initEcatSlaves(ecat_slaves *ecatSlave) {
    for (int mot_id = 0; mot_id < 2; mot_id ++) {
        /* Input/output memory allocation */
        ecatSlave->bel[mot_id].in_motor = (motor_input *)ec_slave[ecatSlave->bel[mot_id].slave_id].inputs;
        ecatSlave->bel[mot_id].out_motor = (motor_output *)ec_slave[ecatSlave->bel[mot_id].slave_id].outputs;
        /* Constant parameters assignment */
        ecatSlave->bel[mot_id].counts_per_rad = 52151.8917;
        ecatSlave->bel[mot_id].counts_per_rad_sec = 52151.8917/10.0;
        ecatSlave->bel[mot_id].pascal_per_count = 21.04178;
        ecatSlave->bel[mot_id].nm_per_pascal = 2.822e-6;
        ecatSlave->bel[mot_id].units_per_nm = 5000.0;
    }
    ecatSlave->el3102.in_analog = (analog_input *)ec_slave[ecatSlave->el3102.slave_id].inputs;
}

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
ecat_slaves *initEcatConfig(void *ifnameptr) {
    char *ifname  = (char *)ifnameptr;

//    inOP = FALSE;
//    needlf = FALSE;
    int expectedWKC;
    int wkc;

    /* Initialize data structure */
    printf("Init data structure.\n");
    ecat_slaves *ecatSlaves;
    ecatSlaves = (ecat_slaves *)malloc(sizeof(ecat_slaves));
    if (ecatSlaves == NULL) {
        return NULL;
    }

    printf("Begin network configuration\n");
    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
	    printf("Socket connection on %s succeeded.\n", ifname);
    }
    else {
	    fprintf(stderr, "No socket connection on %s.\nExcecuted as root\n", ifname);
	    exit(1);
    }

    /* Find and configure slaves */
    if (ec_config_init(FALSE) > 0) {
	    printf("%d slaves found and configured.\n", ec_slavecount);
    }
    else {
	    fprintf(stderr, "No slaves found!\n");
	    ec_close();
	    exit(1);
    }

    /* Request for PRE-OP mode */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    /* Locate slaves */
    slaveIdentify(ecatSlaves);
    printf("psensor_id: %d\n", ecatSlaves->el3102.slave_id);
    if (ecatSlaves->bel[0].slave_id == 0 || ecatSlaves->bel[1].slave_id == 0 || ecatSlaves->el3102.slave_id == 0) {
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

    initMotor(ecatSlaves->bel[0].slave_id);
    initMotor(ecatSlaves->bel[1].slave_id);
    printf("Slaves initialized, state to OP\n");

    /* Check if all slaves are working properly */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /* Request for OP mode */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* Send one valid process data to make outputs in slave happy */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (wkc < expectedWKC) {
        fprintf(stderr, "WKC failure.\n");
        exit(1);
    }

    /* Request OP state for all slaves */
    ec_writestate(0);
    /* Wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    /* Initialize slaves before use */
    initEcatSlaves(ecatSlaves);

    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("Operational state reached for all slaves\n");
        return ecatSlaves;
    }
    else {
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
void add_timespec(struct timespec *ts, int64 addtime) {
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
int64 ec_sync(int64 reftime, int64 cycletime) {
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    return -(delta/100)-(integral/20);
}

/** Read actual position value via SDO
 *
 * @param slave_id  =   Slave index.
 * @return position value in counts.
 */
int32 positionSDOread(uint16 slave_id) {
    int32 initial_theta1_cnts = 0;
    int size = sizeof(initial_theta1_cnts);
    ec_SDOread(slave_id, 0x6064, 0, FALSE, &size, &initial_theta1_cnts, EC_TIMEOUTRXM);
    return initial_theta1_cnts;
}

/** Write PIV gain values via SDO
 *
 * @param slave_id  =   Slave index.
 * @param Pp        =   Pp gain.
 * @param Vp        =   Vp gain.
 */
void pivGainSDOwrite(uint16 slave_id, uint16 Pp, uint16 Vp) {
    SDO_write16(slave_id, 0x2382, 1, Pp);        /* position loop gain (Pp) */
    SDO_write16(slave_id, 0x2381, 1, Vp);        /* velocity loop gain (Vp) */
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