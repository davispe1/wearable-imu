#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>

typedef struct { int16_t mx, my, mz; } MMC5983MA_Data_t;

void mmc5983ma_init(void);
void mmc5983ma_read(MMC5983MA_Data_t *out);

#endif /* MMC5983MA_H */
