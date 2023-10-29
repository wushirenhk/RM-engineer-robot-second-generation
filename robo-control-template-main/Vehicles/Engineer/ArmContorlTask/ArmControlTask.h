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
  Arm_normal = 1,       // 机械臂正常运行 以及hold
  Arm_AirConnection = 2,//机械臂空接
  Arm_SilveryOre = 3,   // 机械臂取银矿石
  Arm_GroundOre = 4,    // 机械臂取金矿石 正放
  Arm_OrePlacement = 5, //机械臂放置矿石
  Arm_HoldOre = 6       //机械臂取多角度金矿石
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
  //任务基类要求必须的函数
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
   //自定义控制器控制变量
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
   
  //RM电机绝对位置控制（增量）
  
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

  // 云台（遥控器控制转速）
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;
  

 //不同状态下各个电机的角度控制（包括保持状态与过渡状态）
 //状态1：正常行驶状态
 float dm1_state_normal_angle;float dm1_state_normal_speed;
 float dm2_state_normal_angle;float dm2_state_normal_speed;
 float dm3_state_normal_angle;float dm3_state_normal_speed;
 float shift_3508_state_normal_angle;
 float uplift1_3508_state_normal_angle;
 float uplift2_3508_state_normal_angle;
 float extend1_3508_state_normal_angle;
 float extend2_3508_state_normal_angle;
 
 //状态2：空接(金矿)状态
 float dm1_state_AirConnection_angle;float dm1_state_AirConnection_speed;
 float dm2_state_AirConnection_angle;float dm2_state_AirConnection_speed;
 float dm3_state_AirConnection_angle;float dm3_state_AirConnection_speed;
 float shift_3508_state_AirConnection_angle;
 float uplift1_3508_state_AirConnection_angle;
 float uplift2_3508_state_AirConnection_angle;
 float extend1_3508_state_AirConnection_angle;
 float extend2_3508_state_AirConnection_angle;
 
 //状态3：取银矿石
 float dm1_state_SilveryOre_angle;float dm1_state_SilveryOre_speed;
 float dm2_state_SilveryOre_angle;float dm2_state_SilveryOre_speed;
 float dm3_state_SilveryOre_angle;float dm3_state_SilveryOre_speed;
 float shift_3508_state_SilveryOre_angle;
 float uplift1_3508_state_SilveryOre_angle;
 float uplift2_3508_state_SilveryOre_angle;
 float extend1_3508_state_SilveryOre_angle;
 float extend2_3508_state_SilveryOre_angle;
 
 //状态4：取地面矿石
 float dm1_state_GroundOre_angle;float dm1_state_GroundOre_speed;
 float dm2_state_GroundOre_angle;float dm2_state_GroundOre_speed;
 float dm3_state_GroundOre_angle;float dm3_state_GroundOre_speed;
 float shift_3508_state_GroundOre_angle;
 float uplift1_3508_state_GroundOre_angle;
 float uplift2_3508_state_GroundOre_angle;
 float extend1_3508_state_GroundOre_angle;
 float extend2_3508_state_GroundOre_angle;
 
 //状态5：放置矿石于翻矿机
 float dm1_state_OrePlacement_angle;float dm1_state_OrePlacement_speed;
 float dm2_state_OrePlacement_angle;float dm2_state_OrePlacement_speed;
 float dm3_state_OrePlacement_angle;float dm3_state_OrePlacement_speed;
 float shift_3508_state_OrePlacement_angle;
 float uplift1_3508_state_OrePlacement_angle;
 float uplift2_3508_state_OrePlacement_angle;
 float extend1_3508_state_OrePlacement_angle;
 float extend2_3508_state_OrePlacement_angle;

 //状态6：抓住矿石前进
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
