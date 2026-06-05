#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>

void w25q64_init(void);
int  w25q64_write(uint32_t addr, const uint8_t *buf, uint32_t len);
int  w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len);

#endif /* W25Q64_H */
