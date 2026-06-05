/**
 * mmc5983ma.c — driver for the MMC5983MA 3-axis magnetometer.
 *
 * Populated on all boards; enabled only if MAG_ENABLED in config.h.
 * Interface: I2C (TBD).
 *
 * TODO:
 *  - Implement I2C read/write helpers.
 *  - Implement mmc5983ma_init() — ODR, SET/RESET pulse.
 *  - Implement mmc5983ma_read() — raw 18-bit → int16 output.
 */

#include "mmc5983ma.h"

void mmc5983ma_init(void) {}
void mmc5983ma_read(MMC5983MA_Data_t *out) { (void)out; }
