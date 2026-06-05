#ifndef UWB_TDMA_H
#define UWB_TDMA_H

#include <stdint.h>

void uwb_tdma_master_init(void);
void uwb_tdma_sensor_init(uint8_t node_id);
void uwb_tdma_tick(void);

#endif /* UWB_TDMA_H */
