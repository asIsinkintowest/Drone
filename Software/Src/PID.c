#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include "PID.h"

#define PWM_IN_TO_OUT 0.05427f

//��������
#define MAX_INTEGRATE_NUM 10

/*
	����PID��
		���PID����λ�ã��Ƕȡ��߶�
		�ڲ�PID����λ�õ�λ���֣����ٶ�
		�ڿ�����
		
		----------------------------------------------------------
		|            |        �ڲ�        |         ���         |
		|---------------------------------------------------------
		|YAWƫ����   |         ��          |          ��           | 
		|---------------------------------------------------------
		|Roll������  |         ��          |          ��           |
		|---------------------------------------------------------
		|Pitch������ |         ��          |          ��           |
		|---------------------------------------------------------
		|Height�߶�  |         X          |          ��           |
		|--------------------------------------------------------|
		
		TODO_01: �޷����ƫ����Yaw�Ƕȣ���ʹֻ��Yawֻ�����ڲ�PID��
			�Ѹģ���YawҲ���봮��PID
*/

PID_t outerPitch,outerRoll,outerHeight;
PID_t innerPitch,innerRoll,innerYaw;
PID_t outerYaw;
/*
	Gyro      -> GY86��ý��ٶ�
	Angle     -> ��̬����õ��ĽǶ�
	expect    -> ң��������������Ƕ� 
	motor     -> �ĸ��������PWM
	PWM_IN[4] -> �����PWM��
	TODO_02: ��ͬ�ļ�֮���ת��
*/



/*
	TODO_03: ���׷��򵥿ɿ�����ֻ����Kp��������
*/

//ͳһ���ò���





////PWM����->�������

//void Get_T(float * T){
//	float last_t = 0; 
//	float now_t = 0;
//	
//	now_t = TIM1->CNT;
//	
//	* T = now_t / 84000000  - last_t / 84000000;
//	
//	last_t = now_t;
//}


void Single_PID_Init(PID_t* pid)
{
	pid->e = 0;
	pid->e_pre = 0;
	pid->e_sum = 0;
	pid->Kp = 0;
	pid->Ki = 0;
	pid->Kd = 0;
	pid->output = 0;
	pid->T = 0;
}

void PID_Init()
{
	Single_PID_Init(&outerHeight);
	outerHeight.Kp = 0.7;
	outerHeight.Kd = 0.1;
	
	Single_PID_Init(&outerPitch);
	outerPitch.Kp = 0.7;
	
	Single_PID_Init(&outerRoll);
	outerRoll.Kp = 0.1;
	
	Single_PID_Init(&innerPitch);
	innerPitch.Kp = 0.7;
	innerPitch.Ki = 0.1;
	innerPitch.Kd = 0.01;
	
	Single_PID_Init(&innerRoll);
	innerRoll.Kp = 0.7;
	innerRoll.Ki = 0.1;
	innerRoll.Kd = 0.01;
	
	Single_PID_Init(&innerYaw);
	innerYaw.Kp = 0.7;
	innerYaw.Ki = 0.1;
	innerYaw.Kd = 0.01;
	
	Single_PID_Init(&outerYaw);
	outerYaw.Kp = 0.7;
	outerYaw.Ki = 0.1;
	outerYaw.Kd = 0.01;
}


float PID_Calc(PID_t* inner, PID_t* outer, float outerError, float innerStatus){
	//�ı�����
	float Kp = inner->Kp;
	float Ki = inner->Ki;
	float Kd = inner->Kd;

	// �����������PID���㣺Roll Picth Yaw
	if((inner != 0) && (outer != 0))
	{
		//��������P��������
		outer->e = outerError;
		outer->output = outer->Kp * outer->e;
		outer->e_pre = outer->e;
		
		//��������Ϊ�ڲ�����
		inner->e = outer->output - innerStatus;
		
		//��e_sum���ֽ������� �������ıȽ�
		if(inner->e_sum - MAX_INTEGRATE_NUM <= 0.00001 && inner->e_sum + MAX_INTEGRATE_NUM >= 0.00001)
		{  
			inner->e_sum += inner->e;
		}
		
		inner->output = Kp * inner->e + Ki * inner->e_sum + Kd * (inner->e - inner->e_pre);
		inner->e_pre = inner->e;
		return inner->output;
	}
	
}

/*
	TODO4: ���ܷ�����Ҫ��� ���������ܷ���
*/
/*
	���ܷ��䣺
						    Motor2 ˳  O      ^      O Motor1 ��
													   \	  |    /
															 \ 	|  /                 ^
																 \|/                   |
										     <--------O---------    �� <---| ǰ
																 /|\                   |
															 /	|  \                 |
														 / 		|    \             
                 Motor3 �� O             O Motor4 ˳
	
          	|------------------------------------------------|
            |  ����    |   �ı��   |     +      |     -     |
          	|------------------------------------------------|
            |  ����    |   Roll     | Motor1+4   | Motor2+3  |
          	|------------------------------------------------|
            |  ǰ��    |   Pitch    | Motor3+4   | Motor1+2  |
          	|------------------------------------------------|
            |  ��ƫ    |   Yaw      | Motor2+ 3  | Motor1+4  |
          	|------------------------------------------------|
						|  �Ϸ�    |     -      |Motor1+2+3+4|    -      |
						|------------------------------------------------|

	���㡢ǰ������ƫʱ��PID���ؽ��roll pitch yawΪ����
	pwm_out1 = -roll + pitch + yaw + thr
	pwm_out2 = +roll + pitch - yaw + thr 
	pwm_out3 = +roll - pitch + yaw + thr
	pwm_out4 = -roll - pitch - yaw + thr
*/

void Motor_Calc(gy_t gy86_data, float pitch , float roll ,float yaw , float expectRoll, float expectPicth, float expectYaw, float expectThr, float *motor1, float * motor2, float *motor3, float *motor4){
	
	float GyroRoll = gy86_data.gx;
    float GyroPitch = gy86_data.gy; 
	float GyroYaw = gy86_data.gz;
	
	float AngleYaw = yaw;
	float AngleRoll = roll;
	float AnglePitch = pitch;	
	
	
	//�洢pid������
	float pid_yaw, pid_roll, pid_pitch, pid_thr;
	
	
	pid_pitch = PID_Calc(&innerPitch, &outerPitch, expectPicth - AnglePitch, GyroPitch);
	pid_roll  = PID_Calc(&innerRoll, &outerRoll, expectRoll - AngleRoll, GyroRoll);
	pid_yaw   = PID_Calc(&innerYaw, &outerYaw, expectYaw - AngleYaw, GyroYaw);
	pid_thr = expectThr;
	
	//pid_thr   = PID_Calc(0,&outerHeight, expectThr, 0);
	
	//����õ������ĵ���ٶ�motor = �������ɶ� + ���ŵ���(pid_thr)
	*motor1 = -(-pid_roll + pid_pitch + pid_yaw + pid_thr) * PWM_IN_TO_OUT;
	*motor2 = -(+pid_roll + pid_pitch - pid_yaw + pid_thr) * PWM_IN_TO_OUT;
	*motor3 = -(-pid_roll - pid_pitch + pid_yaw + pid_thr) * PWM_IN_TO_OUT;
	*motor4 = -(+pid_roll - pid_pitch - pid_yaw + pid_thr) * PWM_IN_TO_OUT;	
}



//ͨ��PWM���񵽵�ռ�ձ��������û�ң���������������Ƕ�ֵ
void Expect_Angle_Calc(float *PWM_IN , float * roll, float *pitch, float *yaw, float *thr){
	  //TODO6: �ĸ�ͨ���Ƿ�ƥ���������ɶ��Լ����� ��ʽ��Ȼ��Ҫ����
	
    * roll = (float)((PWM_IN[3] - 1500) * 0.03f);  //roll
    *pitch = (float)((PWM_IN[1] - 1500) * 0.04f); //pitch
    *yaw = (float)((PWM_IN[0] - 1500) * 0.02f); 
    *thr = PWM_IN[2];
	
}

//TODO7: ������ΰ��� 
/*
	���ڲ���ͨ��PID������PWM���ڵı��� Ӱ��Ki��Kd
*/
