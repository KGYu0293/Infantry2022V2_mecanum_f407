#ifndef _MahonyAHRS_h
#define _MahonyAHRS_h

#pragma pack(1)
typedef struct _MahonyAHRS {
    float twoKp;
    float twoKi;
    float sampleFreq;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float euler[3];
} MahonyAHRS;
#pragma pack()

void MahonyAHRS_init(MahonyAHRS *obj, float _twoKp, float _twoKi,
                     float _sampleFreq);
void MahonyAHRS_clear(MahonyAHRS *obj);
void MahonyAHRS_update(MahonyAHRS *obj, float gx, float gy, float gz, float ax,
                       float ay, float az);

#endif