/**
 * mag.c — magnetometer glue.
 *
 * Enabled only when MAG_ENABLED == 1 in config.h.
 * Wraps mmc5983ma driver, applies hard/soft-iron calibration from flash.
 *
 * TODO:
 *  - Guard all code on MAG_ENABLED.
 *  - Implement mag_init() — call mmc5983ma_init, load calibration coefficients.
 *  - Implement mag_read() — read raw, apply ellipsoid correction, fill MagSample_t.
 */

#include "mag.h"
#include "../config.h"
#include "../Drivers/mmc5983ma.h"

void mag_init(void)
{
#if MAG_ENABLED
    mmc5983ma_init();
#endif
}

void mag_read(MagSample_t *out)
{
    (void)out;
#if MAG_ENABLED
    /* TODO: read, calibrate, fill out */
#endif
}
