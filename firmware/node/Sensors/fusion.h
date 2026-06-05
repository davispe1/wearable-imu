#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>

typedef struct {
    float    qw, qx, qy, qz;
    uint32_t timestamp_us;
} FusionSample_t;

void fusion_init(void);
void fusion_read(FusionSample_t *out);

#endif /* FUSION_H */
