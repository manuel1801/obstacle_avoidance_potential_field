//#include <iostream>
#include <cmath>
#include "obstacle_avoidance_potential_field/potentialfield.h"

PotentialField::PotentialField(float aDelta, float aK_a, float aK_r, float aD0)
{
    delta = aDelta;
    K_a = aK_a;
    K_r = aK_r;
    d0 = aD0;
}

float PotentialField::calc_F_att(float q, float q_g, float K)
{
    return -K * (q - q_g);
}

float PotentialField::calc_F_rep(float q, float q_o, float K, float d, float d0, float radius)
{
    d -= radius;
    if (d < d0)
    {
        return K * (1.0 / d - 1.0 / d0) * (1.0 / (d * d)) * ((q - q_o) / d);
    }
    else
    {
        return 0;
    }
}

void PotentialField::calcPath(float *q, float *o, float *g, float radius)
{
    d_g = sqrt((g[0] - q[0]) * (g[0] - q[0]) + (g[1] - q[1]) * (g[1] - q[1]));
    if (d_g < 0.1)
        return;

    d_o = sqrt((o[0] - q[0]) * (o[0] - q[0]) + (o[1] - q[1]) * (o[1] - q[1]));

    f[0] = calc_F_att(q[0], g[0], K_a) + calc_F_rep(q[0], o[0], K_r, d_o, d0, radius);
    f[1] = calc_F_att(q[1], g[1], K_a) + calc_F_rep(q[1], o[1], K_r, d_o, d0, radius);

    f_abs = sqrt(f[0] * f[0] + f[1] * f[1]);
    q[0] += delta * (f[0] / f_abs);
    q[1] += delta * (f[1] / f_abs);
}
