#ifndef LSM6DSV16B_H
#define LSM6DSV16B_H

#include <stdint.h>

typedef struct { int16_t ax, ay, az, gx, gy, gz; } LSM6DSV16B_RawData_t;
typedef struct { float qw, qx, qy, qz; }           LSM6DSV16B_Quat_t;

void lsm6dsv16b_init(void);
void lsm6dsv16b_read_raw(LSM6DSV16B_RawData_t *out);
void lsm6dsv16b_read_sflp(LSM6DSV16B_Quat_t *out);

#endif /* LSM6DSV16B_H */
