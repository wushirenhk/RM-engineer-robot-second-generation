#ifdef ENGINEER
#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_MC_Tasks.h"
#include "Motor_DM_Task.h"
#include "Motor_RM_Tasks.h"
#include "../Robot/Params.h"

typedef enum
{
  Arm_OFF = 0 , 
  Arm_normal = 1,       // ��е���������� �Լ�hold
  Arm_AirConnection = 2,//��е�ۿս�
  Arm_SilveryOre = 3,   // ��е��ȡ����ʯ
  Arm_GroundOre = 4,    // ��е��ȡ���ʯ ����
  Arm_OrePlacement = 5, //��е�۷��ÿ�ʯ
  Arm_HoldOre = 6       //��е��ȡ��ǶȽ��ʯ
} Arm_Control_Mode;

typedef enum
{
  state0 = 0,
  state1 = 1,
  state2 = 2, 
  state3 = 3,
  state4 = 4,
	state5 = 5
} Arm_Mode_Check;


class Arm_ControlTask : public Task_Base  
{
   friend class Robot;
	 friend class RemoteControlTask;
	 friend class RefereeSystemTask;
	
public:		
	Arm_ControlTask(
    Robot &robot0,
    Motor_RM_Params_t *shift_3508,  PID_Params_t *shift_3508_ang,   PID_Params_t *shift_3508_ang_vel,
		Motor_RM_Params_t *uplift1_3508,PID_Params_t *uplift1_3508_ang, PID_Params_t *uplift1_3508_ang_vel,
		Motor_RM_Params_t *uplift2_3508,PID_Params_t *uplift2_3508_ang, PID_Params_t *uplift2_3508_ang_vel,
		Motor_RM_Params_t *extend1_3508,PID_Params_t *extend1_3508_ang, PID_Params_t *extend1_3508_ang_vel, 
		Motor_RM_Params_t *extend2_3508,PID_Params_t *extend2_3508_ang, PID_Params_t *extend2_3508_ang_vel,
    Motor_DM_Params_t *arm_dm_1,
    Motor_DM_Params_t *arm_dm_2,
		Motor_DM_Params_t *arm_dm_3,
	 timeus_t interval_tick_us0 = 0
   );
	
  virtual ~Arm_ControlTask(void) {}
  //�������Ҫ�����ĺ���
  virtual void init(void);

  virtual void update(timeus_t dT_us);
  
  virtual void uninit(void);	
	  
  uint8_t airpump_flag[3];
	  
  void arm_mode_switch();	  
	  
  int arm_time_delay(uint16_t time);
	  
protected:
		Motor_RM_PIDControlTask *shift_3508_pid_task;
		Motor_RM_PIDControlTask *uplift1_3508_pid_task;
		Motor_RM_PIDControlTask *uplift2_3508_pid_task;
		Motor_RM_PIDControlTask *extend1_3508_pid_task;
		Motor_RM_PIDControlTask *extend2_3508_pid_task;
		Motor_DM_PSControlTask 	*motor_dm1_task, *motor_dm2_task, *motor_dm3_task;
   
   uint8_t switch_lock;
   uint8_t airpump_lock[2];
   Arm_Control_Mode arm_mode;
   Arm_Control_Mode last_arm_mode;
   //�Զ�����������Ʊ���
   uint8_t custom_lock;
   uint8_t custom_flag;
   float Custom_pitch_save;
   float Custom_roll_save;
   float Custom_yaw_save;
   float position_rad_save[3];
   float Custom_position_delt[3];

   Arm_Mode_Check normal;
   Arm_Mode_Check AirConnection;
   Arm_Mode_Check SilveryOre;
   Arm_Mode_Check GroundOre;
   Arm_Mode_Check OrePlacement;
   Arm_Mode_Check HoldOre;
   uint16_t timecnt;
   
  //RM�������λ�ÿ��ƣ�������
  
  float exp_angle;
  float shift_3508_angle;
  float shift_3508_angular_velocity_max;
  float shift_3508_angular_velocity_min;
	
  float uplift1_3508_angle;
  float uplift1_3508_angular_velocity_max;
  float uplift1_3508_angular_velocity_min;
	
  float uplift2_3508_angle;
  float uplift2_3508_angular_velocity_max;
  float uplift2_3508_angular_velocity_min;
	
  float extend1_3508_angle;
  float extend1_3508_angular_velocity_max;
  float extend1_3508_angular_velocity_min;
	
  float extend2_3508_angle;
  float extend2_3508_angular_velocity_max;
  float extend2_3508_angular_velocity_min;

  // ��̨��ң��������ת�٣�
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;
  

 //��ͬ״̬�¸�������ĽǶȿ��ƣ���������״̬�����״̬��
 //״̬1��������ʻ״̬
 float dm1_state_normal_angle;float dm1_state_normal_speed;
 float dm2_state_normal_angle;float dm2_state_normal_speed;
 float dm3_state_normal_angle;float dm3_state_normal_speed;
 float shift_3508_state_normal_angle;
 float uplift1_3508_state_normal_angle;
 float uplift2_3508_state_normal_angle;
 float extend1_3508_state_normal_angle;
 float extend2_3508_state_normal_angle;
 
 //״̬2���ս�(���)״̬
 float dm1_state_AirConnection_angle;float dm1_state_AirConnection_speed;
 float dm2_state_AirConnection_angle;float dm2_state_AirConnection_speed;
 float dm3_state_AirConnection_angle;float dm3_state_AirConnection_speed;
 float shift_3508_state_AirConnection_angle;
 float uplift1_3508_state_AirConnection_angle;
 float uplift2_3508_state_AirConnection_angle;
 float extend1_3508_state_AirConnection_angle;
 float extend2_3508_state_AirConnection_angle;
 
 //״̬3��ȡ����ʯ
 float dm1_state_SilveryOre_angle;float dm1_state_SilveryOre_speed;
 float dm2_state_SilveryOre_angle;float dm2_state_SilveryOre_speed;
 float dm3_state_SilveryOre_angle;float dm3_state_SilveryOre_speed;
 float shift_3508_state_SilveryOre_angle;
 float uplift1_3508_state_SilveryOre_angle;
 float uplift2_3508_state_SilveryOre_angle;
 float extend1_3508_state_SilveryOre_angle;
 float extend2_3508_state_SilveryOre_angle;
 
 //״̬4��ȡ�����ʯ
 float dm1_state_GroundOre_angle;float dm1_state_GroundOre_speed;
 float dm2_state_GroundOre_angle;float dm2_state_GroundOre_speed;
 float dm3_state_GroundOre_angle;float dm3_state_GroundOre_speed;
 float shift_3508_state_GroundOre_angle;
 float uplift1_3508_state_GroundOre_angle;
 float uplift2_3508_state_GroundOre_angle;
 float extend1_3508_state_GroundOre_angle;
 float extend2_3508_state_GroundOre_angle;
 
 //״̬5�����ÿ�ʯ�ڷ����
 float dm1_state_OrePlacement_angle;float dm1_state_OrePlacement_speed;
 float dm2_state_OrePlacement_angle;float dm2_state_OrePlacement_speed;
 float dm3_state_OrePlacement_angle;float dm3_state_OrePlacement_speed;
 float shift_3508_state_OrePlacement_angle;
 float uplift1_3508_state_OrePlacement_angle;
 float uplift2_3508_state_OrePlacement_angle;
 float extend1_3508_state_OrePlacement_angle;
 float extend2_3508_state_OrePlacement_angle;

 //״̬6��ץס��ʯǰ��
 float dm1_state_HoldOre_angle;float dm1_state_HoldOre_speed;
 float dm2_state_HoldOre_angle;float dm2_state_HoldOre_speed;
 float dm3_state_HoldOre_angle;float dm3_state_HoldOre_speed;
 float shift_3508_state_HoldOre_angle;
 float uplift1_3508_state_HoldOre_angle;
 float uplift2_3508_state_HoldOre_angle;
 float extend1_3508_state_HoldOre_angle;
 float extend2_3508_state_HoldOre_angle;
};


#endif

#endif
