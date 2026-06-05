/**
 * role_master.c — master-node state machine.
 *
 * Responsibilities:
 *  - Receive IMU samples from sensor nodes over UWB TDMA.
 *  - Act as the UWB time reference (sync source).
 *  - Aggregate samples into a host-bound packet stream.
 *  - Forward aggregated data to the host over BLE serial (primary) or SWO (wired).
 *
 * TODO:
 *  - Implement TDMA frame receive loop.
 *  - Implement aggregation buffer.
 *  - Implement BLE uplink task.
 *  - Implement SWO data-out path.
 */

#include "role_master.h"

void role_master_init(void)
{
    /* TODO: init UWB, BLE, SWO, aggregation buffer */
}

void role_master_run(void)
{
    /* TODO: enter main loop / start RTOS tasks */
    for (;;) {}
}
