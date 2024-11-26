#ifndef __GY86_H
#define __GY86_H

#include "Acc.h"
#include "Quaternion.h"

typedef struct gy_t{
	float ax;//��λ�� g
	float ay;
	float az;
	float gx;//��λ ��ÿ�� 
	float gy;
	float gz;
	float mx;// ��λ ��˹gauss
	float my;
	float mz;	
}gy_t;

void GY86_Init(void);

void GY86_GetData(gy_t * gy86_data);

void IMU_Correct(Vector3f_t * offset_g, Vector3f_t *offset_a, Vector3f_t * offset_m,Vector3f_t * scale_a, Vector3f_t * scale_m);

void IMU_Task(gy_t *gy86_data,Vector3f_t *offset_a, Vector3f_t * offset_m,Vector3f_t * scale_a, Vector3f_t * scale_m,Quaternion * quaternion_data,float *a, float *b, float *c,Vector3f_t * offset_g);

#endif
