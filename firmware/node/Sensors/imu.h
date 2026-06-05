#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float    ax, ay, az;   /* m/s^2 */
    float    gx, gy, gz;   /* rad/s */
    uint32_t timestamp_us;
} ImuSample_t;

void imu_init(void);
void imu_read(ImuSample_t *out);

#endif /* IMU_H */
