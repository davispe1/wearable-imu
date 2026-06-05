/**
 * role_sensor.c — sensor-node state machine.
 *
 * Responsibilities:
 *  - Sample IMU (and optionally magnetometer) at IMU_SAMPLE_RATE_HZ.
 *  - Buffer samples in the RAM ring buffer.
 *  - Transmit samples to the master node over UWB TDMA in assigned slots.
 *  - Participate in UWB ranging rounds at RANGING_RATE_HZ.
 *  - Log samples to W25Q64 flash as session insurance.
 *
 * TODO:
 *  - Implement IMU sampling task.
 *  - Implement UWB TDMA transmit task.
 *  - Implement W25Q64 logging task.
 */

#include "role_sensor.h"

void role_sensor_init(void)
{
    /* TODO: init IMU, DWM3000, W25Q64, ring buffer */
}

void role_sensor_run(void)
{
    /* TODO: enter main loop / start RTOS tasks */
    for (;;) {}
}
