#ifdef ENGINEER

#ifndef GIMBAL_CONTROL_TASK_H
#define GIMBAL_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"


typedef enum
{
  GIMBAL_FREE = 0, // ���ɿ���ģʽ
  GIMBAL_LOCK = 1, // ȡ���ģʽ
 
} Gimbal_Control_Mode;

typedef enum
{
  Overturn_OFF = 0, // �µ�ģʽ
  Overturn_CW  = 1, // ˳ʱ��
  Overturn_CCW  = 2, // ˳ʱ��  
  Overturn_IN =  3, // �����ʯ
  Overturn_OUT = 4  // �ó���ʯ
} Overturn_Control_Mode;


class GimbalControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  friend class MainControlTask;
  friend class RefereeSystemTask;
  friend class Arm_ControlTask;
  friend class ChassisControlTask;

  GimbalControlTask(
    Robot &robot0,
    Motor_RM_Params_t *Overturn1, PID_Params_t *Overturn1_ang, PID_Params_t *Overturn1_ang_vel,
    Motor_RM_Params_t *Overturn2, PID_Params_t *Overturn2_ang, PID_Params_t *Overturn2_ang_vel,
    Motor_RM_Params_t *gimbal_6020,PID_Params_t *gimbal_6020_ang, PID_Params_t *gimbal_6020_ang_vel,

    timeus_t interval_tick_us0 = 0
  );

  virtual ~GimbalControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);
	  
  Motor_RM_PIDControlTask* getOverturn1PIDControlTaskPointer(void)
  {
    return motor_Overturn1_pid_task;
  }
  Motor_RM_PIDControlTask* getOverturn2PIDControlTaskPointer(void)
  {
    return motor_Overturn2_pid_task;
  }
  
protected:
	
  Motor_RM_PIDControlTask *motor_Overturn1_pid_task, *motor_Overturn2_pid_task, *motor_gimbal_6020_pid_task;
  Gimbal_Control_Mode gimbal_mode;
  Overturn_Control_Mode overturn_mode;

  float exp_angular_velocity_CW;
  float exp_angular_velocity_IN;
  float overturn1_angle;
  float overturn2_angle;

  // ��̨�����ƽǶȣ�
  float gimbal_yaw_angle = -2.64f;
  float gimbal_pitch_angle = 140;
  float angle_glod_yaw = -1.0f;
  float angle_glod_pitch = 105;  
  float angle_yaw_Orebin = -1.0f;
  float angle_pitch_Orebin = 115;

   //���ٶ�
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;
  //�޷�
  uint8_t gimbal_max_yaw_vel = 7;

 //�������޷�
  float overturn1_angular_velocity_max = 5.0f;
  float overturn1_angular_velocity_min = -5.0f;
  float overturn2_angular_velocity_max = 5.0f;
  float overturn2_angular_velocity_min = -5.0f;

};


#endif
#endif
