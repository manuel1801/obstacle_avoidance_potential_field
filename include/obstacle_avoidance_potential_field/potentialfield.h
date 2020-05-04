
class PotentialField
{
private:
    float delta, K_a, K_r, d0;
    float f_abs, d_g, d_o;
    float f[2];

    float calc_F_att(float q, float q_g, float K);
    float calc_F_rep(float q, float q_o, float K, float d, float d0, float radius);

public:
    PotentialField(float aDelta, float aK_a, float aK_r, float aD0);
    void calcPath(float *q, float *o, float *g, float radius);
};