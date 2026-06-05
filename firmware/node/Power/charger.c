/**
 * charger.c — BQ25185 battery charger interface.
 *
 * Monitors charge status over I2C. USB-C VBUS → BQ25185 → LiPo.
 * The WBA55 has no native USB; USB-C is charging-only.
 *
 * TODO:
 *  - Implement charger_init() — configure charge current, termination voltage.
 *  - Implement charger_get_status() — read charge state register.
 */

#include "charger.h"

void charger_init(void) {}
ChargerStatus_t charger_get_status(void) { return CHARGER_UNKNOWN; }
