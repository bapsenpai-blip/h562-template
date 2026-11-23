#include "quaternion.h"

#ifndef M_PI_F
#define M_PI_F  (3.141592653589793f)
#endif

void Quaternion_Init(Quaternion *q){
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2){
    Quaternion q;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return q;
}

void Quaternion_Normalize(Quaternion *q){
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if(norm < 1e-6f){
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }
    float inv_norm = 1.0f/norm;
    q->w *= inv_norm;
    q->x *= inv_norm;
    q->y *= inv_norm;
    q->z *= inv_norm;
}

void Quaternion_Integrate(Quaternion *q, float gyro_p, float gyro_q, float gyro_r, float dt){
    float dq_w = 0.5f * (-q->x*gyro_p - q->y*gyro_q - q->z*gyro_r);
    float dq_x = 0.5f * (q->w*gyro_p + q->y*gyro_r - q->z*gyro_q);
    float dq_y = 0.5f * (q->w*gyro_q - q->x*gyro_r + q->z*gyro_p);
    float dq_z = 0.5f * (q->w*gyro_r + q->x*gyro_q - q->y*gyro_p);

    q->w += dq_w*dt;
    q->x += dq_x*dt;
    q->y += dq_y*dt;
    q->z += dq_z*dt;
}

Quaternion Euler_To_Quaternion(float roll, float pitch, float yaw){
    Quaternion q;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = cy * sp * cr + sy * cp * sr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

void Quaternion_To_Euler(Quaternion *q, float *roll, float *pitch, float *yaw)
{
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI_F / 2.0f, sinp);   // dùng 90° nếu bị lock
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}



