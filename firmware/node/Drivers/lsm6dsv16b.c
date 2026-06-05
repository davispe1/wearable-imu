/**
 * lsm6dsv16b.c — driver for the LSM6DSV16B 6-axis IMU.
 *
 * Provides: init, read raw accel+gyro, read SFLP quaternion output.
 * Interface: SPI (or I2C — TBD, see config.h / hardware docs).
 *
 * TODO:
 *  - Implement SPI read/write helpers.
 *  - Implement lsm6dsv16b_init() — ODR, full-scale, FIFO config.
 *  - Implement lsm6dsv16b_read_raw() — accel + gyro raw int16.
 *  - Implement lsm6dsv16b_read_sflp() — quaternion from SFLP engine (option B).
 *  - Implement interrupt-driven DRDY path.
 */

#include "lsm6dsv16b.h"

void lsm6dsv16b_init(void) {}
void lsm6dsv16b_read_raw(LSM6DSV16B_RawData_t *out) { (void)out; }
void lsm6dsv16b_read_sflp(LSM6DSV16B_Quat_t *out)   { (void)out; }
