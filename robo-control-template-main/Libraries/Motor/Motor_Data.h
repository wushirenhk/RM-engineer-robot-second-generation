#ifndef __MOTOR_DATA_H__
#define __MOTOR_DATA_H__

#include "Task.h"

#define RoboMaster_GM6020     0
#define RoboMaster_3508       1
#define RoboMaster_2006       2
#define RMD_X8                3
#define DM_J4310              4
#define BM_M15                5
#define MG8016                6

#define MOTOR_CW              1    // ����������ת�����������ֶ����ж�����ͬ
#define MOTOR_CCW            -1    // ����������ת�����������ֶ����ж����෴

typedef struct _Motor_Linear_Movement_t
{
  float offset; // �߻�׼��m��
  float displacement; // ��λ�ƣ�m��
  float relative_displacement; // ����߻�׼����λ�ƣ�m��
  float velocity; // ���ٶȣ�m/s��
  // float acceleration; // ���Լ��ٶ�

} Motor_Linear_Movement_t;

typedef struct _Motor_Angular_Movement_t
{
  float angle; // ��λ�ƣ�rad��
  float polar_angle; // ����(-PI~PI)
  float angular_velocity; // ���ٶȣ�rad/s�����ܾ��Ȳ�����
  float angular_velocity_imu; // ��IMU��������Ľ��ٶȣ�rad/s��
  // float acceleration; // �Ǽ��ٶȣ�rad/s^2��

	
	
} Motor_Angular_Movement_t;

typedef struct _Motor_Common_Measurement_t
{
  timeus_t timestamp; // ����ʱ�����uint32_t��

  Motor_Angular_Movement_t rotator, output; // ת�ӣ������
  Motor_Linear_Movement_t linear; // ����

} Motor_Common_Measurement_t;

#endif
