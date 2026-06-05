/**
 * fuel_gauge.c — MAX17048 fuel gauge interface.
 *
 * Reports battery state-of-charge (%) and voltage over I2C.
 *
 * TODO:
 *  - Implement fuel_gauge_init() — configure alert thresholds.
 *  - Implement fuel_gauge_read() — fill FuelGaugeData_t (SOC %, voltage mV).
 */

#include "fuel_gauge.h"

void fuel_gauge_init(void) {}
void fuel_gauge_read(FuelGaugeData_t *out) { (void)out; }
