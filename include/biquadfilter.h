#include "math.h"
#include "stdint.h"

typedef struct{
    float a1, a2, b0, b1, b2;
    float w1, w2;
    float out;
} Biquad;

void Biquad_Init(Biquad *filter);
void Biquad_SetCoeff(Biquad *filter, float a1, float a2, float b0, float b1, float b2);
void Biquad_LowPassSetup(Biquad *filter, float fc_hz, float Q, float fs_hz);
void Biquad_NotchSetup(Biquad *filter, float fc_hz, float Q, float fs_hz);
float Biquad_Update(Biquad *filter, float x);