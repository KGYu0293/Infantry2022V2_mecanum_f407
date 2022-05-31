#ifndef _MRAC_H_
#define _MRAC_H_

typedef struct {
    float x1;
    float x2;
    float x;
    float r;
    float h;
    float aim;
    float dt;
} MTD_t;

typedef struct {
    float x1_ref;
    float x1_fdb;
    float x2_ref;
    float x2_fdb;
    float x1_err;
    float x2_err;
    float k1;
    float k2;
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
    float gamma1;
    float gamma2;
    float gamma3;
    float gamma4;
    float output;
    float output_max;
    float dt;
    MTD_t x1_td;
    MTD_t x2_td;

    struct
    {
        float ki;
        float sum_err;
        float sum_err_max;
    } integrator;
} mrac_2d;

void mrac_2d_calc(mrac_2d *mrac, float ref, float x1_fdb, float x2_fdb, unsigned char enable);
#endif