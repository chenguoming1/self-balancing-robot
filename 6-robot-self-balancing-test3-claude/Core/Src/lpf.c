#include "lpf.h"

float LPF_Update(LPF_t *lpf, float input, float dt)
{
    float alpha = dt / (lpf->Tf + dt);
    lpf->y += alpha * (input - lpf->y);
    return lpf->y;
}

void LPF_Reset(LPF_t *lpf)
{
    lpf->y = 0.0f;
}
