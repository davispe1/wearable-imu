/**
 * ble_serial.c — BLE serial-over-GATT uplink (master → host).
 *
 * Implements a NUS-equivalent transparent UART GATT service on the WBA55.
 * The host side connects via native Bluetooth or an nRF52840 USB dongle
 * presenting a COM port.
 *
 * TODO:
 *  - Set up BLE stack (STM32 BLE middleware / WBA55 stack).
 *  - Implement NUS-equivalent GATT service (TX characteristic notify).
 *  - Implement ble_serial_send() — fragment and notify up to MTU per call.
 *  - Implement connection / disconnection callbacks.
 */

#include "ble_serial.h"

void ble_serial_init(void) {}
int  ble_serial_send(const uint8_t *buf, uint16_t len) { (void)buf; (void)len; return 0; }
