/**
 * dwm3000.c — driver for the DWM3000 UWB transceiver.
 *
 * Three roles from v1:
 *  1. Data transmission — node → master sample packets.
 *  2. Time sync — master is UWB time reference; nodes align before data leaves.
 *  3. Inter-node ranging — pairwise distances for phase-2 EKF.
 *
 * Built on top of Qorvo's DW3000 driver (add as a submodule or copy into Drivers/).
 * Interface: SPI (WBA55 SPI1 — TBD).
 *
 * TODO:
 *  - Integrate Qorvo DW3000 host driver.
 *  - Implement dwm3000_init() — channel, PRF, preamble config from config.h.
 *  - Implement dwm3000_tx() / dwm3000_rx() wrappers for TDMA use.
 *  - Implement TWR (two-way ranging) for inter-node distance measurement.
 *  - Implement sync-beacon transmit (master) / receive + clock-correction (sensor).
 */

#include "dwm3000.h"

void dwm3000_init(void) {}
int  dwm3000_tx(const uint8_t *buf, uint16_t len) { (void)buf; (void)len; return 0; }
int  dwm3000_rx(uint8_t *buf, uint16_t max_len)   { (void)buf; (void)max_len; return 0; }
