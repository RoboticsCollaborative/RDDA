#ifndef RDDAMAIN_H
#define RDDAMAIN_H

/** EtherCAT slave identifier */
typedef struct PACKED
{
    uint16 motor1;
    uint16 motor2;
    uint16 psensor;
}rdda_slavet;

/** Global struct to hold all slaves identified */
extern rdda_slavet *rdda_slave;

void rdda_ecat_config(void *ifnameptr);


#endif //RDDAMAIN_H