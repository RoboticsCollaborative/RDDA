#ifndef RDDAMAIN_H
#define RDDAMAIN_H

void *rddaEcatConfig(void *ifnameptr);
void rddaCyclic(void *rdda_slave);
void ecatcheck(void *ptr);
void rddaStop();

#endif //RDDAMAIN_H