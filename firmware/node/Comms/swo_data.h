#ifndef SWO_DATA_H
#define SWO_DATA_H

#include <stdint.h>

void swo_data_init(void);
void swo_send(const uint8_t *buf, uint16_t len);

#endif /* SWO_DATA_H */
