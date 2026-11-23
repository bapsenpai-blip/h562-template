#include "math.h"
#include "stdint.h"
#include "quaternion.h"

typedef struct {
    float Kp;
    float Ki;
    Quaternion q;
    float IntegralX;
    float IntegralY;
    float IntegralZ;
    float roll, pitch, yaw;
    uint8_t angles_computed;
} Mahony_t;

void Mahony_Init(Mahony_t *m);
void Mahony_UpdateIMU(Mahony_t *m, float gyro_p, float gyro_q, float gyro_r, float ax, float ay, float az, float dt);
void Mahony_Update(Mahony_t *m, float gyro_p, float gyro_q, float gyro_r, 
                   float ax, float ay, float az, 
                   float mx, float my, float mz, float dt);
void Mahony_ComputeAngles(Mahony_t *m);
float Mahony_GetRoll(Mahony_t *m);
float Mahony_GetPitch(Mahony_t *m);
float Mahony_GetYaw(Mahony_t *m);
float Mahony_GetRollRadians(Mahony_t *m);
float Mahony_GetPitchRadians(Mahony_t *m);
float Mahony_GetYawRadians(Mahony_t *m);