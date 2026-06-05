/**
 * uwb_tdma.c — UWB TDMA frame scheduler.
 *
 * Manages the on-body UWB time-division frame:
 *  - Data slots: each sensor node transmits one packet per frame.
 *  - Sync beacon: master broadcasts timestamp at frame start.
 *  - Ranging slots: pairwise TWR at RANGING_RATE_HZ (subset of IMU rate).
 *
 * TODO:
 *  - Define frame structure (slot count, durations).
 *  - Implement master frame controller (beacon + receive slots).
 *  - Implement sensor slot transmitter (await assigned slot, burst packet).
 *  - Implement ranging round scheduler (25–50 Hz, lower than data rate).
 */

#include "uwb_tdma.h"

void uwb_tdma_master_init(void) {}
void uwb_tdma_sensor_init(uint8_t node_id) { (void)node_id; }
void uwb_tdma_tick(void) {}
