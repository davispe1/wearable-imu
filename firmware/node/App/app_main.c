/**
 * app_main.c — application entry point.
 *
 * Called from main() after CubeMX HAL init. Selects role (MASTER / SENSOR)
 * based on config.h and dispatches to the appropriate state machine.
 *
 * TODO:
 *  - Read NODE_ROLE from config.h (or a strapped GPIO) and call the right init.
 *  - Start the RTOS scheduler (or super-loop) with sensor + comms tasks.
 */

#include "app_main.h"
#include "role_master.h"
#include "role_sensor.h"
#include "../config.h"

void app_main(void)
{
#if NODE_ROLE == ROLE_MASTER
    role_master_init();
    role_master_run();   /* does not return */
#else
    role_sensor_init();
    role_sensor_run();   /* does not return */
#endif
}
