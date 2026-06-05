/**
 * swo_data.c — SWO (Serial Wire Output) data channel.
 *
 * Provides a one-way MCU → host data path through the ST-LINK via ITM/SWO.
 * Used for wired bring-up and as a data-recovery path.
 *
 * TODO:
 *  - Configure ITM stimulus port(s) for structured data output.
 *  - Implement swo_send() — write framed sample data to ITM port 0.
 *  - Document SWV viewer setup in docs/04-firmware.md.
 */

#include "swo_data.h"

void swo_data_init(void) {}
void swo_send(const uint8_t *buf, uint16_t len) { (void)buf; (void)len; }
