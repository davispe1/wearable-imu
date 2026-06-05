/**
 * fusion.c — on-node fusion config / SFLP bridge.
 *
 * In DATA_FORMAT_RAW_9DOF mode this file is a pass-through; the host does all fusion.
 * In DATA_FORMAT_SFLP_QUAT mode this file reads the LSM6DSV16B SFLP quaternion output
 * and packages it for transmission.
 *
 * TODO:
 *  - Implement fusion_init() — configure SFLP engine if DATA_FORMAT == SFLP_QUAT.
 *  - Implement fusion_read() — fill FusionSample_t from appropriate source.
 */

#include "fusion.h"
#include "../config.h"

void fusion_init(void) {}

void fusion_read(FusionSample_t *out) { (void)out; }
