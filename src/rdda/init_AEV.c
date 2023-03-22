/** init_AEV.c */

#include "init_AEV.h"

/** SOEM wrapper for ec_SDOwrite().
 *
 * @param slave     = Slave index.
 * @param index     = SDO index.
 * @param subindex  = SDO sub-index.
 * @param value     = Output value.
 * @return working counter.
 */
int SDO_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

int SDO_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value) {
    int wkc;
    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
    return wkc;
}

/** Map AEV motor drive PDOs in CSP mode.
 *
 * @param[in] slaveIdx    = Slave index.
 * @return 0 on success, -1 on failure.
 */
int mapMotorPDOs(uint16 slaveIdx)
{
    int wkc = 0;

    /** Reset the sync manager SM2 (0x1C12:0) and SM3 (0x1C13:0) before they are configured,
     *  and then list the PDO in sub-indices (1, 2, etc), and finally set the number of PDOs
     *  mapped on sub-index 0, which we previously cleared.
     *  These mapping are documented in the slave ESI file.
     */

    /* AEV CSP RxPDOs (master outputs) */
    wkc += SDO_write8(slaveIdx, 0x1C12, 0, 0);           /* clear SM2 (slave RxPDOs) */
    wkc += SDO_write16(slaveIdx, 0x1C12, 1, 0x1700);     /* pre-mapped PDO */
    wkc += SDO_write8(slaveIdx, 0x1C12, 0, 1);           /* set # of mapped PDOs */

    /* AEV CSP TxPDOs (master inputs) */
    /* Use user-defined PDO to map the load encoder position and velocity, you
     * need to reference documentation on PDO mapping objects to understand the
     * values set here.  In a nutshell:  {object index,sub-index,size in bits}
     * These mapping defines user TxPDOs (inputs to master)
     */
    // wkc += SDO_write8(slaveIdx, 0x1A00, 0, 0);           /* clear the PDO first */
    // wkc += SDO_write32(slaveIdx, 0x1A00, 1, 0x22310020); /* Load encoder velocity */
    // wkc += SDO_write32(slaveIdx, 0x1A00, 2, 0x22420020); /* Load encoder position */
    // wkc += SDO_write8(slaveIdx, 0x1A00, 0, 2);           /* set number of objects mapped by PDO */

    wkc += SDO_write8(slaveIdx, 0x1A01, 0, 0);           /* clear the PDO first */
    wkc += SDO_write32(slaveIdx, 0x1A01, 1, 0x21830020); /* Load error code */
    wkc += SDO_write32(slaveIdx, 0x1A01, 2, 0x22000010); /* analog input */
    wkc += SDO_write8(slaveIdx, 0x1A01, 0, 2);           /* set number of objects mapped by PDO */

    /* pre-mapped PDOs that the slave sends to the master */
    wkc += SDO_write8(slaveIdx, 0x1C13, 0, 0);           /* clear SM3 (slave TxPDOs) */
    wkc += SDO_write16(slaveIdx, 0x1C13, 1, 0x1B00);     /* pre-mapped PDO */
    // wkc += SDO_write16(slaveIdx, 0x1C13, 2, 0x1A00);     /* user-PDO */
    wkc += SDO_write16(slaveIdx, 0x1C13, 2, 0x1A01);     /* user-PDO */
    wkc += SDO_write8(slaveIdx, 0x1C13, 0, 2);           /* set # of mapped PDOs */

    /* as specified in ESI file, set control word during PRE->SAFE transition */
    SDO_write16(slaveIdx, 0x6060, 0, 8);                 /* AEV set to CSP mode */

    if (wkc != 12)
        return 1;

    return 0;
}


/** Attach a callback function for PRE->SAFE transition
 *
 * @param slaveIdx      = Slave index.
 * @return 0.
 */
int mapMotorPDOs_callback(uint16 slaveIdx)
{
    ec_slave[slaveIdx].PO2SOconfig = mapMotorPDOs;

    return 0;
}

/** Initialize AEV/motor(ADR110-P-22-P) parameters via SDO
 *
 * @param slaveIdx      = Slave index.
 * @return 0.
 */
int initADRMotor(uint16 slaveIdx)
{
    printf("Motor drive %d init\n", slaveIdx);

    /* Clear latched faults */
    SDO_write16(slaveIdx, 0x6040, 0, 0x008f); /* set bit 7 low-to-high to clear latched fault*/
    SDO_write16(slaveIdx, 0x6040, 0, 0x000f); /* reset control word */

    /* Motor params */
    // SDO_write32(slaveIdx, 0x2383, 12, 22627);   /* motor torque constant */
    SDO_write32(slaveIdx, 0x2383, 12, 29500);   /* motor torque constant */
    SDO_write32(slaveIdx, 0x2383, 13, 580000);  /* motor peak torque */
    SDO_write32(slaveIdx, 0x2383, 14, 200000);   /* motor continuous torque */
    SDO_write32(slaveIdx, 0x6076, 0, 2000);      /* motor rated torque */

    /* Loop gains */
    SDO_write16(slaveIdx, 0x2382, 1, 0);        /* position loop gain (Pp) */
    SDO_write16(slaveIdx, 0x2381, 1, 0);        /* velocity loop gain (Vp) */

    /* Motor limits */
    SDO_write16(slaveIdx, 0x2110, 0, 2563);     /* peak current limit */
    SDO_write16(slaveIdx, 0x2111, 0, 840);      /* continuous current limit (units of 0.01A) */

    return 0;
}
