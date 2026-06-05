/**
 * imu.c — IMU sensor glue.
 *
 * Wraps the lsm6dsv16b driver. Converts raw counts to physical units,
 * applies static bias correction, and formats the sample for the ring buffer.
 *
 * TODO:
 *  - Implement imu_init() — call lsm6dsv16b_init, estimate gyro bias.
 *  - Implement imu_read() — read raw, apply scale + bias, fill ImuSample_t.
 *  - Apply DATA_FORMAT flag from config.h (raw vs SFLP).
 */

#include "imu.h"
#include "../Drivers/lsm6dsv16b.h"

void imu_init(void) { lsm6dsv16b_init(); }

void imu_read(ImuSample_t *out)
{
    (void)out;
    /* TODO: read raw, scale, bias-correct, fill out */
}
