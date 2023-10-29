#ifdef ENGINEER

#ifndef CHASSIS_CONTROL_TASK_H
#define CHASSIS_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"
#include "Filters.h"
#include "/Robot/Params.h"

//#define CAN_CHASSIS_MOTOR_SUPERCAP_CTRL_ID      0x300
//#define CAN_CHASSIS_CTRL_ID                     0x200
//#define CAN_3508_MOTOR1_ID                      0x201
//#define CAN_3508_MOTOR2_ID                      0x202
//#define CAN_3508_MOTOR3_ID                      0x203
//#define CAN_3508_MOTOR4_ID                      0x204

#define INITIAL_HALF_WHEEL_BASE .42f// �����[m]
#define INITIAL_HALF_TREAD .42// ���־�[m]
#define INITIAL_ROTATE_RATIO_X .0f// ��ת������x�����ϵĹ�һ������
#define INITIAL_ROTATE_RATIO_Y .0f// ��ת������y�����ϵĹ�һ������
#define INITIAL_WHEEL_VEL_RATIO 1.0f// ��������
#define INITIAL_XY_Chassis_Slow 1.0f// //ƽ����������

struct suercap_RX_data_pack
{
	float supercap_voltage;
	float supercap_energy_percent;
};

typedef enum
{
  Chassis_OFF = 0, // ����ʧ��
  Chassis_FAST = 1, // ������ʻ����·�ã�
  Chassis_SLOW = 2, // ������ʻ��΢���ã�
} Chassis_Control_Mode;

typedef enum
{
  Chassis_NORMAL = 0, // ����
  Chassis_ORE = 1 // ����ʯ

} Chassis_direction_Mode;

class ChassisControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class MainControlTask;
  friend class RemoteControlTask;
  friend class RefereeSystemTask;

  ChassisControlTask(
    Robot &robot0,
    Motor_RM_Params_t *m1, PID_Params_t *m1_ang, PID_Params_t *m1_ang_vel,
    Motor_RM_Params_t *m2, PID_Params_t *m2_ang, PID_Params_t *m2_ang_vel,
    Motor_RM_Params_t *m3, PID_Params_t *m3_ang, PID_Params_t *m3_ang_vel,
    Motor_RM_Params_t *m4, PID_Params_t *m4_ang, PID_Params_t *m4_ang_vel,
    timeus_t interval_tick_us0 = 0
  ) ;

  virtual ~ChassisControlTask(void) {};

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

	void setControlMode(Chassis_Control_Mode mode)
  {
    chassis_mode = mode;
  }
  
	Chassis_Control_Mode getControlMode()
  {
    return chassis_mode;
  }
	
	float getexp_vel(void)
	{
		return exp_vel[2];
	}
	
	Motor_RM_PIDControlTask *getMotor_RM_PIDControlTaskPointer(void)
	{
		return motor_3_pid_task;
	}	
	
	uint8_t *get_can_tx_data()
	{
		return can_tx_data;
	}
	
	LowPassFilter<float> get_lowpass_filter()
	{
		return lowpass_filter;
	}
//	PIDControlTask getChassis_follow_PIDTask()
//	{
//		return chassis_follow_pid_task;
//	}	
	
	Chassis_direction_Mode chassis_direction_mode;
	
protected:
  // uint8_t can_tx_data[8];
  Motor_RM_PIDControlTask *motor_1_pid_task, *motor_2_pid_task, *motor_3_pid_task, *motor_4_pid_task;
	Chassis_Control_Mode chassis_mode;

	LowPassFilter<float> lowpass_filter;
	
	uint8_t can_tx_data[8];
  float vx; // ����x�����ٶ�[m/s]
  float vy; // y�����ٶ�[m/s]
  float vw; // ��z����ٶ�[rad/s]
  float vx_filter;//�˲���ĵ���x�����ٶ�[m/s]
  float vy_filter;//�˲���ĵ���x�����ٶ�[m/s]
	
  float half_wheel_base; // �����[m]
  float half_tread; // ���־�[m]

  float rotate_ratio_x; // ��ת������x�����ϵĹ�һ������[-1.0, 1.0]��������Ϊ1.0��-1.0
  float rotate_ratio_y; // ��ת������y�����ϵĹ�һ������[-1.0, 1.0]��������Ϊ1.0��-1.0

  float wheel_vel_ratio; // ��ת��������
	float xy_chassis_slow;//ƽ����������
  float exp_vel[4] = {0}; // �ĸ�����������ٶ�[m/s]
	float rotate_ratio[4];
  int16_t motor_send[4] = {0}; // ���շ�������ĵ���ֵ
  /*     forward
            X              1.0
           /|\
       M2   |   M1
    Y<------|------        0.0
       M3   |   M4
            |
            |             -1.0
  1.0      0.0       -1.0
  */
  
	float max_vx = 3.5f;
	float min_vx = -3.5f;
	float max_vy = 3.5f;
	float min_vy = -3.5f;
	float max_vw = 3.5f;
  float min_vw = -3.5f;
	
//	float max_vx = 4.8f;
//	float min_vx = -4.8f;
//	float max_vy = 4.8f;
//	float min_vy = -4.8f;
//	float max_vw = 4.5f;
//  float min_vw = -4.5f;
  
  float max_vx_slow = 0.3f;
	float min_vx_slow = -0.3f;
	float max_vy_slow = 0.3f;
	float min_vy_slow = -0.3f;
	float max_vw_slow = 0.3f;
  float min_vw_slow = -0.3f;
	
};
#endif

#endif