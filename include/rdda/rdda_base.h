#ifndef RDDA_RDDA_BASE_H
#define RDDA_RDDA_BASE_H

RDDA_slave *init_RDDA_slave(SlaveIndex *slave_id);
void free_RDDA_slave(RDDA_slave *rdda_slave);

#endif //RDDA_RDDA_BASE_H
