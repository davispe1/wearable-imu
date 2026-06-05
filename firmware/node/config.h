/**
 * config.h — compile-time configuration for a single node.
 *
 * Edit before building. All open items from the README are represented here
 * so nothing is hard-coded elsewhere in the firmware.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ── Node identity ─────────────────────────────────────────────────────── */
#define NODE_ID          0          /* 0..N-1; set per-node before flash */

/* Role: exactly one node must be MASTER; all others are SENSOR.
 * Alternatively build two binaries from the same source with -DROLE_MASTER. */
#define ROLE_SENSOR      0
#define ROLE_MASTER      1
#define NODE_ROLE        ROLE_SENSOR   /* override per-node */

/* ── Sampling ───────────────────────────────────────────────────────────── */
#define IMU_SAMPLE_RATE_HZ    100   /* open item: 100 vs 200 */
#define MAG_SAMPLE_RATE_HZ     50   /* magnetometer can run slower */
#define RANGING_RATE_HZ        25   /* UWB inter-node ranging (power saving) */

/* ── Data format ────────────────────────────────────────────────────────── */
/* Open item: raw 9-DOF (OPTION_A) vs SFLP quaternion + raw mag (OPTION_B). */
#define DATA_FORMAT_RAW_9DOF   0
#define DATA_FORMAT_SFLP_QUAT  1
#define DATA_FORMAT            DATA_FORMAT_RAW_9DOF

/* ── Magnetometer ───────────────────────────────────────────────────────── */
/* Open item: enable once lab magnetic environment is characterised. */
#define MAG_ENABLED            0    /* 0 = disabled, 1 = enabled */

/* ── Node count (used for TDMA slot allocation) ─────────────────────────── */
#define MAX_NODES              8    /* open item: 3 → 6–8 */

/* ── UWB ────────────────────────────────────────────────────────────────── */
#define UWB_CHANNEL            5    /* DW3000 channel — TBD */

/* ── RAM ring buffer ────────────────────────────────────────────────────── */
#define RING_BUF_SIZE_SAMPLES  512  /* covers ~5 s @ 100 Hz */

#endif /* CONFIG_H */
