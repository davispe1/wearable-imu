#ifndef DWM3000_H
#define DWM3000_H

#include <stdint.h>

void dwm3000_init(void);
int  dwm3000_tx(const uint8_t *buf, uint16_t len);
int  dwm3000_rx(uint8_t *buf, uint16_t max_len);

#endif /* DWM3000_H */
