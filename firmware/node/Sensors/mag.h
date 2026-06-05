#ifndef MAG_H
#define MAG_H

#include <stdint.h>

typedef struct {
    float    mx, my, mz;   /* uT */
    uint32_t timestamp_us;
} MagSample_t;

void mag_init(void);
void mag_read(MagSample_t *out);

#endif /* MAG_H */
