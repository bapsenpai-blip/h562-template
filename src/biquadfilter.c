#include "biquadfilter.h"

#ifndef M_2PI_F
#define M_2PI_F  (2.0f * 3.141592653589793f)
#endif
#ifndef M_PI_F
#define M_PI_F  (3.141592653589793f)
#endif

void Biquad_Init(Biquad *filter){
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;

    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;

    filter->w1 = 0.0f;
    filter->w2 = 0.0f;

    filter->out = 0.0f;
}

void Biquad_SetCoeff(Biquad *filter, float a1, float a2, float b0, float b1, float b2){
    filter->a1 = a1;
    filter->a2 = a2;

    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
}

void Biquad_LowPassSetup(Biquad *filter, float fc_hz, float Q, float fs_hz){
    float theta_c = M_2PI_F * fc_hz / fs_hz;
    float K = tanf(theta_c / 2);
    float W = K*K;
    float alpha = 1 + K / Q + W;
    Biquad_SetCoeff(filter, 2.0f * (W - 1.0f)/alpha, (1.0f - K/Q + W)/alpha, 
                            W/alpha, 2.0f * W/alpha, W/alpha);
}

void Biquad_NotchSetup(Biquad *filter, float fc_hz, float Q, float fs_hz){
    float omega_0 = 2.0 * atanf(M_PI_F * fc_hz / fs_hz);
    float alpha = sinf(omega_0)/(2.0f * Q);
    float c_omega0 = cosf(omega_0);
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * c_omega0 / a0;
    float a2 = (1.0f - alpha) / a0;
    float b0 = 1.0f / a0;
    float b1 = -2.0f * c_omega0 / a0;
    float b2 = 1.0f / a0;
    Biquad_SetCoeff(filter, a1, a2, b0, b1, b2);
}
float Biquad_Update(Biquad *filter, float in){
    filter->out = filter->b0 * in + filter->w1;
    filter->w1  = filter->b1 * in - filter->a1 * filter->out + filter->w2;
    filter->w2  = filter->b2 * in - filter->a2 * filter->out;
    return filter->out;
}