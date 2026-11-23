#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include <stdint.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

void Quaternion_Init(Quaternion *q);
Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2);
void Quaternion_Normalize(Quaternion *q);
void Quaternion_Integrate(Quaternion *q, float gyro_p, float gyro_q, float gyro_r, float dt);
Quaternion Euler_To_Quaternion(float roll, float pitch, float yaw);
void Quaternion_To_Euler(Quaternion *q, float *roll, float *pitch, float *yaw);
#endif /* QUATERNION_H */