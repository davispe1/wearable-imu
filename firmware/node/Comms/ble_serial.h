#ifndef BLE_SERIAL_H
#define BLE_SERIAL_H

#include <stdint.h>

void ble_serial_init(void);
int  ble_serial_send(const uint8_t *buf, uint16_t len);

#endif /* BLE_SERIAL_H */
