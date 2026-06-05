#ifndef FUEL_GAUGE_H
#define FUEL_GAUGE_H

#include <stdint.h>

typedef struct { uint8_t soc_pct; uint16_t voltage_mv; } FuelGaugeData_t;

void fuel_gauge_init(void);
void fuel_gauge_read(FuelGaugeData_t *out);

#endif /* FUEL_GAUGE_H */
