#include "mahony.h"

#define DefKp 0.5f
#define DefKi 0.02f

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    // Sử dụng uint32_t thay vì long để đảm bảo luôn là 32-bit trên mọi hệ thống
    union { float f; uint32_t l; } i;
    i.f = x;
    i.l = 0x5f3759df - (i.l >> 1);
    float y = i.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(Mahony_t *m){
    m->Kp = DefKp;
    m->Ki = DefKi;
    Quaternion_Init(&m->q);
    m->IntegralX = 0.0f;
    m->IntegralY = 0.0f;
    m->IntegralZ = 0.0f;
    m->roll = 0.0f;
    m->pitch = 0.0f;
    m->yaw = 0.0f;
}

void Mahony_UpdateIMU(Mahony_t *m, float gyro_p, float gyro_q, float gyro_r, float ax, float ay, float az, float dt){
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
         ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        //Estimate direction of gravity
        vx = 2 * (m->q.x*m->q.z - m->q.w*m->q.y);
        vy = 2 * (m->q.w*m->q.x + m->q.y*m->q.z);
        vz = m->q.w*m->q.w - m->q.x*m->q.x - m->q.y*m->q.y + m->q.z*m->q.z;

        //Error: 
        // Cross product between measured direciton of accel and  estimated one.
        ex = ay*vz - vy*az;
        ey = az*vx - vz*ax;
        ez = ax*vy - vx*ay;

        //PI feedback for gyro
        if(m->Ki > 0.0f){
            m->IntegralX += m->Ki*ex*dt;
            m->IntegralY += m->Ki*ey*dt;
            m->IntegralZ += m->Ki*ez*dt;
        }else{
            m->IntegralX = 0.0f;
            m->IntegralY = 0.0f;
            m->IntegralZ = 0.0f;
        }
        gyro_p += m->Kp*ex + m->IntegralX;
        gyro_q += m->Kp*ey + m->IntegralY;       
        gyro_r += m->Kp*ez + m->IntegralZ;
    }
    //Intergrate and normalize quaternion
    Quaternion_Integrate(&m->q, gyro_p, gyro_q, gyro_r, dt);
    Quaternion_Normalize(&m->q);
    m->angles_computed = 0;
}
void Mahony_Update(Mahony_t *m, float gyro_p, float gyro_q, float gyro_r, 
                   float ax, float ay, float az, 
                   float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0 = m->q.w, q1 = m->q.x, q2 = m->q.y, q3 = m->q.z;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // Nếu magnetometer bằng 0 thì fallback về Mahony_UpdateIMU
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        Mahony_UpdateIMU(m, gyro_p, gyro_q, gyro_r, ax, ay, az, dt);
        return;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Chuẩn hóa Accel
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Chuẩn hóa Mag
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 1. Vector trọng lực ước lượng (v)
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 2. Chuyển vector Mag đo được sang Navigation frame (h)
        hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
        hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
        
        // Giữ nguyên tổng thành phần nằm ngang
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

        // 3. Vector từ trường ước lượng quay ngược lại Body frame (w)
        wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
        wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3));
        wz = 2.0f * (bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2));

        // 4. Tổng sai số = (a x v) + (m x w)
        ex = (ay * vz - vy * az) + (my * wz - wy * mz);
        ey = (az * vx - vz * ax) + (mz * wx - wz * mx);
        ez = (ax * vy - vx * ay) + (mx * wy - wx * my);

        // Bộ điều khiển PI cho gyro bias
        if (m->Ki > 0.0f) {
            m->IntegralX += m->Ki * ex * dt;
            m->IntegralY += m->Ki * ey * dt;
            m->IntegralZ += m->Ki * ez * dt;
        } else {
            m->IntegralX = 0.0f;
            m->IntegralY = 0.0f;
            m->IntegralZ = 0.0f;
        }

        gyro_p += m->Kp * ex + m->IntegralX;
        gyro_q += m->Kp * ey + m->IntegralY;
        gyro_r += m->Kp * ez + m->IntegralZ;
    }

    // Tích phân Quaternion
    Quaternion_Integrate(&m->q, gyro_p, gyro_q, gyro_r, dt);
    Quaternion_Normalize(&m->q);
    m->angles_computed = 0;
}
void Mahony_ComputeAngles(Mahony_t *m){
    Quaternion_To_Euler(&m->q, &m->roll, &m->pitch, &m->yaw);
    m->angles_computed = 1;
}
float Mahony_GetRoll(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
    return m->roll * 57.29578f;
}

float Mahony_GetPitch(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
    return m->pitch * 57.29578f;
}

float Mahony_GetYaw(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
     return m->yaw * 57.29578f + 180.0f;
}

float Mahony_GetRollRadians(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
    return m->roll;
}

float Mahony_GetPitchRadians(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
    return m->pitch;
}

float Mahony_GetYawRadians(Mahony_t *m) {
    if (!m->angles_computed) Mahony_ComputeAngles(m);
    return m->yaw;
}

    

