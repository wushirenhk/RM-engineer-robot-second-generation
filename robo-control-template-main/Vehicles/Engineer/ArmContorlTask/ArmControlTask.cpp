/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   Engineer
** 文 件 名：   ArmControlTask.cpp
** 文件说明：   脉塔电机控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						       孔明骏     	        2023-02-11

***************************************************************************/
#include "ArmControlTask.h"
#include "../Robot/Robot.h"
#include "Helper.h"
#include "../Robot/Params.h"
#include "/Gimbal/GimbalControlTask.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

double s2=0;//dm2电机整体角度调节

/***********************************************************************
** 函 数 名： Arm_ControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定yaw和pitch轴电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、射击摩擦轮电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
***********************************************************************/

Arm_ControlTask::Arm_ControlTask(
    Robot &robot0,
		Motor_RM_Params_t *shift_3508,  PID_Params_t *shift_3508_ang,   PID_Params_t *shift_3508_ang_vel,
		Motor_RM_Params_t *uplift1_3508,PID_Params_t *uplift1_3508_ang, PID_Params_t *uplift1_3508_ang_vel,
		Motor_RM_Params_t *uplift2_3508,PID_Params_t *uplift2_3508_ang, PID_Params_t *uplift2_3508_ang_vel,
		Motor_RM_Params_t *extend1_3508,PID_Params_t *extend1_3508_ang, PID_Params_t *extend1_3508_ang_vel,
		Motor_RM_Params_t *extend2_3508,PID_Params_t *extend2_3508_ang, PID_Params_t *extend2_3508_ang_vel,
      Motor_DM_Params_t *arm_dm_1,
      Motor_DM_Params_t *arm_dm_2,
		Motor_DM_Params_t *arm_dm_3,
	  timeus_t interval_tick_us0
	 ):Task_Base(robot0)
{
		this->interval_tick_us = interval_tick_us0;
	
	/********************new控制任务********************/
	/*****************************↓***************************/
  //RM电机//
	//平移电机 1个
	  if(shift_3508 != NULL)
	  {
    if(shift_3508->canx == 1)
    {
          this->shift_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          shift_3508->can_rx_id,
          shift_3508->can_tx_id,
          shift_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          shift_3508->interval);
    }
	 else  if(shift_3508->canx == 2)
    {
          this->shift_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          shift_3508->can_rx_id,
          shift_3508->can_tx_id,
          shift_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          shift_3508->interval);
    }
      this->shift_3508_pid_task->motor_backend_p->setParams(*shift_3508);
		}
		
	  //抬升电机1号
	  if(uplift1_3508 != NULL)
	  {
    if(uplift1_3508->canx == 1)
    {
          this->uplift1_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          uplift1_3508->can_rx_id,
          uplift1_3508->can_tx_id,
          uplift1_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          uplift1_3508->interval);
    }
	 else  if(uplift1_3508->canx == 2)
    {
          this->uplift1_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          uplift1_3508->can_rx_id,
          uplift1_3508->can_tx_id,
          uplift1_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          uplift1_3508->interval);
    }
      this->uplift1_3508_pid_task->motor_backend_p->setParams(*uplift1_3508);
		}
		
		//抬升电机2号
		if(uplift2_3508 != NULL)
	  {
    if(uplift2_3508->canx == 1)
    {
          this->uplift2_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          uplift2_3508->can_rx_id,
          uplift2_3508->can_tx_id,
          uplift2_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          uplift2_3508->interval);
    }
	 else  if(uplift2_3508->canx == 2)
    {
          this->uplift2_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          uplift2_3508->can_rx_id,
          uplift2_3508->can_tx_id,
          uplift2_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          uplift2_3508->interval);
    }
      this->uplift2_3508_pid_task->motor_backend_p->setParams(*uplift2_3508);
		}
		
		//伸出电机1号
		if(extend1_3508 != NULL)
	  {
    if(extend1_3508->canx == 1)
    {
          this->extend1_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          extend1_3508->can_rx_id,
          extend1_3508->can_tx_id,
          extend1_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          extend1_3508->interval);
    }
	 else  if(extend1_3508->canx == 2)
    {
          this->extend1_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          extend1_3508->can_rx_id,
          extend1_3508->can_tx_id,
          extend1_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          extend1_3508->interval);
    }
      this->extend1_3508_pid_task->motor_backend_p->setParams(*extend1_3508);
		}
		
		//伸出电机2号
		if(extend2_3508 != NULL)
	  {
    if(extend2_3508->canx == 1)
    {
          this->extend2_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          extend2_3508->can_rx_id,
          extend2_3508->can_tx_id,
          extend2_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          extend2_3508->interval);
    }
	 else  if(extend2_3508->canx == 2)
    {
          this->extend2_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          extend2_3508->can_rx_id,
          extend2_3508->can_tx_id,
          extend2_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          extend2_3508->interval);
    }
      this->extend2_3508_pid_task->motor_backend_p->setParams(*extend2_3508);
		}
		
  //DM一号电机
	    if(arm_dm_1 != NULL)
  {
    if(arm_dm_1->canx == 1)
    {
          this->motor_dm1_task = new Motor_DM_PSControlTask(robot0,
          robot0.can1_device,
          arm_dm_1->can_rx_id,
          arm_dm_1->can_tx_id,
          arm_dm_1->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_1->interval);
    }
	 else  if(arm_dm_1->canx == 2)
    {
          this->motor_dm1_task = new Motor_DM_PSControlTask(robot0,
          robot0.can2_device,
          arm_dm_1->can_rx_id,
          arm_dm_1->can_tx_id,
          arm_dm_1->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_1->interval);
    }
	  this->motor_dm1_task->motor_backend_p->setParams(*arm_dm_1);
  }
	
  //DM二号电机
  	    if(arm_dm_2 != NULL)
  {
    if(arm_dm_2->canx == 1)
    {
          this->motor_dm2_task = new Motor_DM_PSControlTask(robot0,
          robot0.can1_device,
          arm_dm_2->can_rx_id,
          arm_dm_2->can_tx_id,
          arm_dm_2->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_2->interval);
    }
	 else  if(arm_dm_2->canx == 2)
    {
          this->motor_dm2_task = new Motor_DM_PSControlTask(robot0,
          robot0.can2_device,
          arm_dm_2->can_rx_id,
          arm_dm_2->can_tx_id,
          arm_dm_2->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_2->interval);
    }
	  this->motor_dm2_task->motor_backend_p->setParams(*arm_dm_2);
  }
	
  //DM三号电机
  	    if(arm_dm_3 != NULL)
  {
    if(arm_dm_3->canx == 1)
    {
          this->motor_dm3_task = new Motor_DM_PSControlTask(robot0,
          robot0.can1_device,
          arm_dm_3->can_rx_id,
          arm_dm_3->can_tx_id,
          arm_dm_3->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_3->interval);
    }
	 else  if(arm_dm_3->canx == 2)
    {
          this->motor_dm3_task = new Motor_DM_PSControlTask(robot0,
          robot0.can2_device,
          arm_dm_3->can_rx_id,
          arm_dm_3->can_tx_id,
          arm_dm_3->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_3->interval);
    }
	  this->motor_dm3_task->motor_backend_p->setParams(*arm_dm_3);
  }
		
	//平移电机
    if(shift_3508_ang_vel != NULL)
  {
    this->shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*shift_3508_ang_vel);
    this->shift_3508_pid_task->angular_velocity_control_task_p->setInterval((*shift_3508_ang_vel).interval);
  }

  if(shift_3508_ang != NULL)
  {
    this->shift_3508_pid_task->angle_control_task_p->setPIDControllerParams(*shift_3508_ang);
    this->shift_3508_pid_task->angle_control_task_p->setInterval((*shift_3508_ang).interval);
  }  
	
	//抬升电机1号
	if(uplift1_3508_ang_vel != NULL)
  {
    this->uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*uplift1_3508_ang_vel);
    this->uplift1_3508_pid_task->angular_velocity_control_task_p->setInterval((*uplift1_3508_ang_vel).interval);
  }

  if(uplift1_3508_ang != NULL)
  {
    this->uplift1_3508_pid_task->angle_control_task_p->setPIDControllerParams(*uplift1_3508_ang);
    this->uplift1_3508_pid_task->angle_control_task_p->setInterval((*uplift1_3508_ang).interval);
  }  
	
	//抬升电机2号
	if(uplift2_3508_ang_vel != NULL)
  {
    this->uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*uplift2_3508_ang_vel);
    this->uplift2_3508_pid_task->angular_velocity_control_task_p->setInterval((*uplift2_3508_ang_vel).interval);
  }

  if(uplift2_3508_ang != NULL)
  {
    this->uplift2_3508_pid_task->angle_control_task_p->setPIDControllerParams(*uplift2_3508_ang);
    this->uplift2_3508_pid_task->angle_control_task_p->setInterval((*uplift2_3508_ang).interval);
  }  
	
	//伸出电机1号
	if(extend1_3508_ang_vel != NULL)
  {
    this->extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*extend1_3508_ang_vel);
    this->extend1_3508_pid_task->angular_velocity_control_task_p->setInterval((*extend1_3508_ang_vel).interval);
  }

  if(extend1_3508_ang != NULL)
  {
    this->extend1_3508_pid_task->angle_control_task_p->setPIDControllerParams(*extend1_3508_ang);
    this->extend1_3508_pid_task->angle_control_task_p->setInterval((*extend1_3508_ang).interval);
  }  
	
	//伸出电机2号
	if(extend2_3508_ang_vel != NULL)
  {
    this->extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*extend2_3508_ang_vel);
    this->extend2_3508_pid_task->angular_velocity_control_task_p->setInterval((*extend2_3508_ang_vel).interval);
  }

  if(extend2_3508_ang != NULL)
  {
    this->extend2_3508_pid_task->angle_control_task_p->setPIDControllerParams(*extend2_3508_ang);
    this->extend2_3508_pid_task->angle_control_task_p->setInterval((*extend2_3508_ang).interval);
  }  
}



/***********************************************************************
** 函 数 名： Arm_ControlTask::init()
** 函数说明： 注册各个摩擦轮发射电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::init(void)
{
	   inited = true;
		
	
	  //电机的数据获取任务注册
		if(shift_3508_pid_task != NULL)
     robot.scheduler.registerTask(shift_3508_pid_task);
		if(uplift1_3508_pid_task != NULL)
     robot.scheduler.registerTask(uplift1_3508_pid_task);
		if(uplift2_3508_pid_task != NULL)
     robot.scheduler.registerTask(uplift2_3508_pid_task);
		if(extend1_3508_pid_task != NULL)
     robot.scheduler.registerTask(extend1_3508_pid_task);
		if(extend2_3508_pid_task != NULL)
     robot.scheduler.registerTask(extend2_3508_pid_task);
	  if(motor_dm1_task != NULL)
     robot.scheduler.registerTask(motor_dm1_task);
	  if(motor_dm2_task != NULL)
     robot.scheduler.registerTask(motor_dm2_task);
	  if(motor_dm3_task != NULL)
     robot.scheduler.registerTask(motor_dm3_task);		
		
	   /* PID任务注册 */
         /*↓*/

      if(shift_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(shift_3508_pid_task->getAngleTaskPointer());
      if(shift_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(shift_3508_pid_task->getAngularVelocityTaskPointer());
			
      if(uplift1_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(uplift1_3508_pid_task->getAngleTaskPointer());
      if(uplift1_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(uplift1_3508_pid_task->getAngularVelocityTaskPointer());
			
		  if(uplift2_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(uplift2_3508_pid_task->getAngleTaskPointer());
      if(uplift2_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(uplift2_3508_pid_task->getAngularVelocityTaskPointer());
			
			if(extend1_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(extend1_3508_pid_task->getAngleTaskPointer());
      if(extend1_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(extend1_3508_pid_task->getAngularVelocityTaskPointer());
			
			if(extend2_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(extend2_3508_pid_task->getAngleTaskPointer());
      if(extend2_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(extend2_3508_pid_task->getAngularVelocityTaskPointer());
		
	       /*↑*/
 	   /* PID任务注册 */		
		  
		  
		arm_mode = Arm_OFF;
		exp_angle = 0;
	
		shift_3508_angle = -0.1;  //弧度		
		uplift1_3508_angle = 0; //注意两者的角度相反，负号不要忘了 uplift1_3508_angle取负为上
		uplift2_3508_angle = -uplift1_3508_angle;
		extend1_3508_angle = -0.1;//注意两者的角度相反，负号不要忘了 extend1_3508_angle取负为上
		extend2_3508_angle = -extend1_3508_angle;
			
		shift_3508_angular_velocity_max = 3;
    shift_3508_angular_velocity_min = -3;	
		
		uplift1_3508_angular_velocity_max = 2;
    uplift1_3508_angular_velocity_min = -2;
		
		uplift2_3508_angular_velocity_max = 2;
    uplift2_3508_angular_velocity_min = -2;	
	 
		extend1_3508_angular_velocity_max = 3;
    extend1_3508_angular_velocity_min = -3;	
	 
		extend2_3508_angular_velocity_max = 3;
    extend2_3508_angular_velocity_min = -3;	
			
		motor_dm1_task->motor_backend_p->params.position_rad = -1.72;      //picth轴
		motor_dm2_task->motor_backend_p->params.position_rad = -0.766; 
		motor_dm3_task->motor_backend_p->params.position_rad =  -2.44;      //弧度
	  motor_dm1_task->motor_backend_p->params.speed_rad = 0;      //弧度/sz
		motor_dm2_task->motor_backend_p->params.speed_rad = 0;	
		motor_dm3_task->motor_backend_p->params.speed_rad = 0;	
}

/***********************************************************************
** 函 数 名： ArmRM_ControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::uninit(void) 
{

}

/***********************************************************************
** 函 数 名： Arm_ControlTask::arm_time_delay(uint16_t time)
** 函数说明： arm任务延时（无中断式），单位ms
**---------------------------------------------------------------------
** 输入参数： time，延时时间，单位ms
** 返回参数： 无
***********************************************************************/
int Arm_ControlTask::arm_time_delay(uint16_t time) 
{
	timecnt++;
	
	if(timecnt == time * 4 / 10)
	{
		timecnt = 0;
	   return 1;
	}
	else{return 0;}
}

/***********************************************************************
** 函 数 名： Arm_ControlTask::arm_mode_switch()
** 函数说明： 机械臂模式切换
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::arm_mode_switch()
{
  /*判断此时状态以使用不同控制方式*/
           /*↓*/
		 switch (arm_mode){
		 case Arm_OFF:	  
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = 0;
		 motor_dm2_task->motor_backend_p->params.speed_rad = 0; 
		 motor_dm3_task->motor_backend_p->params.speed_rad = 0; 

		 break;
		 /****************************normal状态开始转换************************************/
		 case Arm_normal:	

		    switch (last_arm_mode)
			 {
				 case Arm_OFF:
					 if(normal == state0)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.1; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.759;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = -2.429;  
						 
				   dm1_state_normal_speed = 2;      
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state1;
					  switch_lock = 0;
					 }
					 }	
				 break;
					 
				 case Arm_normal:
					 
				 break;
				 
				 case Arm_AirConnection:				 
					 
					 if(normal == state0)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -6; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -2.50; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = 0.720;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					 }
					 }
					 
					 if(normal == state2)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state3;
					 }
					 }
					 
					 if(normal == state3)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = 0.900; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state4;
						switch_lock = 0;
					 }
					 }
					 
				 break;
				 
				 case Arm_SilveryOre:
					 if(normal == state0)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -4.789; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -2.50; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = 2.321; 
				 	 dm3_state_normal_angle = 0.720;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state1;
					 }
					 }	
					 
					 if(normal == state1)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -4.789; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -2.50; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = 0.720;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					 }
					 }
					 
					 if(normal == state2)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state3;
					 }
					 }
					 
					 if(normal == state3)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = 0.900; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state4;
						switch_lock = 0;
					 }
					 }
					 
				 break;
				 
				 case Arm_GroundOre:
					
				 break;
				 
				 case Arm_OrePlacement:
					if(normal == state0)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -5; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -2.50; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = 0.720;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state1;
					 }
					 }
					 
					 if(normal == state1)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					 }
					 }
					 
					 if(normal == state2)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = 0.900; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state3;
						switch_lock = 0;
					 }
					 }
					 
					 break;
				 
				 case Arm_HoldOre:
					 if(normal == state0)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.769; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -2.50; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = -0.766; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state1;
					 }
					 }
					 
					 if(normal == state1)
					 {	 
					 shift_3508_state_normal_angle = 0;
					 uplift1_3508_state_normal_angle = -1.169; 
					 uplift2_3508_state_normal_angle = -uplift1_3508_state_normal_angle; 
					 extend1_3508_state_normal_angle = -0.10; 
					 extend2_3508_state_normal_angle = -extend1_3508_state_normal_angle;						 
						 
				   dm1_state_normal_angle = -1.691;      
				   dm2_state_normal_angle = 0.900; 
				 	 dm3_state_normal_angle = -2.500;  
						 
				   dm1_state_normal_speed = 2;
				   dm2_state_normal_speed = 1;
					 dm3_state_normal_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state2;
						switch_lock = 0;
					 }
					 }
					 
				 break;
				 
			 }
		 if(switch_lock != 0)
		 {
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;
			 
	   shift_3508_angle = shift_3508_state_normal_angle;
		 uplift1_3508_angle = uplift1_3508_state_normal_angle; 
		 uplift2_3508_angle = uplift2_3508_state_normal_angle; 
		 extend1_3508_angle = extend1_3508_state_normal_angle; 
		 extend2_3508_angle = extend2_3508_state_normal_angle;
			 
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_normal_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_normal_angle;
		 motor_dm3_task->motor_backend_p->params.position_rad = dm3_state_normal_angle;
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_normal_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_normal_speed;
		 motor_dm3_task->motor_backend_p->params.speed_rad = dm3_state_normal_speed;
			 
		 }
		 
		 break;
			 /****************************AirConnection状态开始转换************************************/
	 	 case Arm_AirConnection:	
		    switch (last_arm_mode)
			 {
				 
				 case Arm_OFF:
				    
				 break;
					 
				 case Arm_normal:
					 
				   if(AirConnection == state0)
					 {	 
					 shift_3508_state_AirConnection_angle = 0;
					 uplift1_3508_state_AirConnection_angle = -6; 
					 uplift2_3508_state_AirConnection_angle = -uplift1_3508_state_AirConnection_angle; 
					 extend1_3508_state_AirConnection_angle = -2.50; 
					 extend2_3508_state_AirConnection_angle = -extend1_3508_state_AirConnection_angle;									 
						 
				   dm1_state_AirConnection_angle = -1.691;      
				   dm2_state_AirConnection_angle = -0.766; 
					 dm3_state_AirConnection_angle = 0.720; 
						 
				   dm1_state_AirConnection_speed = 2;      
				   dm2_state_AirConnection_speed = 1;
					 dm3_state_AirConnection_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;	
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  AirConnection = state1;
					  switch_lock = 0;
					 }
					 }						 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:
		 
				 break;
				 
				 case Arm_GroundOre:
	 
				 break;
				 
				 case Arm_OrePlacement:
		 
				 break;
				 
				 case Arm_HoldOre:
				 break;
				 		 
			 }
		 if(switch_lock != 0)
		 {			 
	   shift_3508_angle = shift_3508_state_AirConnection_angle;
		 uplift1_3508_angle = uplift1_3508_state_AirConnection_angle; 
		 uplift2_3508_angle = uplift2_3508_state_AirConnection_angle; 
		 extend1_3508_angle = extend1_3508_state_AirConnection_angle; 
		 extend2_3508_angle = extend2_3508_state_AirConnection_angle;
			 
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_AirConnection_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_AirConnection_angle;
		 motor_dm3_task->motor_backend_p->params.position_rad = dm3_state_AirConnection_angle;
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_AirConnection_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_AirConnection_speed;
		 motor_dm3_task->motor_backend_p->params.speed_rad = dm3_state_AirConnection_speed;
	
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;
		 robot.arm_task_p->custom_flag = 0;
		 }
		 
		 break;
			 /****************************Arm_SilveryOre状态开始转换************************************/
		 case Arm_SilveryOre:	
		    switch (last_arm_mode)
			 {

				 case Arm_OFF:

				 break;
					 
				 case Arm_normal:

					 if(SilveryOre == state0)
					 {	 
					 shift_3508_state_SilveryOre_angle = 0;
					 uplift1_3508_state_SilveryOre_angle = -4.789; 
					 uplift2_3508_state_SilveryOre_angle = -uplift1_3508_state_SilveryOre_angle; 
					 extend1_3508_state_SilveryOre_angle = -2.50; 
					 extend2_3508_state_SilveryOre_angle = -extend1_3508_state_SilveryOre_angle;	
						 
				   dm1_state_SilveryOre_angle = -1.759;
				   dm2_state_SilveryOre_angle = -0.766;
					 dm3_state_SilveryOre_angle = -2.429;
						 
				   dm1_state_SilveryOre_speed = 2;
				   dm2_state_SilveryOre_speed = 1;
					 dm3_state_SilveryOre_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					 SilveryOre = state1;
					 }
					 }		 
					 if(SilveryOre == state1)
					 {	 
					 shift_3508_state_SilveryOre_angle = 0;
					 uplift1_3508_state_SilveryOre_angle = -4.789; 
					 uplift2_3508_state_SilveryOre_angle = -uplift1_3508_state_SilveryOre_angle; 
					 extend1_3508_state_SilveryOre_angle = -2.50; 
					 extend2_3508_state_SilveryOre_angle = -extend1_3508_state_SilveryOre_angle;	
						 
				   dm1_state_SilveryOre_angle = -1.691;
				   dm2_state_SilveryOre_angle = 2.321;
					 dm3_state_SilveryOre_angle = 0.720;
						 
				   dm1_state_SilveryOre_speed = 2;
				   dm2_state_SilveryOre_speed = 1;
					 dm3_state_SilveryOre_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;	
           robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  SilveryOre = state2;
					  switch_lock = 0;
					 }
					 }						 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:		 
				 break;
				 
				 case Arm_GroundOre:

				 break;
				 
				 case Arm_OrePlacement:
		 
				 break;
				 
				 case Arm_HoldOre:

             break;				 
			 }		
		 if(switch_lock != 0)
		 {		 
	   shift_3508_angle = shift_3508_state_SilveryOre_angle;
		 uplift1_3508_angle = uplift1_3508_state_SilveryOre_angle; 
		 uplift2_3508_angle = uplift2_3508_state_SilveryOre_angle; 
		 extend1_3508_angle = extend1_3508_state_SilveryOre_angle; 
		 extend2_3508_angle = extend2_3508_state_SilveryOre_angle;
			 
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_SilveryOre_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_SilveryOre_angle;
		 motor_dm3_task->motor_backend_p->params.position_rad = dm3_state_SilveryOre_angle;
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_SilveryOre_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_SilveryOre_speed;
		 motor_dm3_task->motor_backend_p->params.speed_rad = dm3_state_SilveryOre_speed;
		 
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;
		 robot.arm_task_p->custom_flag = 0;
		 }
		 break;
			 			 
		/****************************Arm_OrePlacement状态开始转换************************************/
		 case Arm_OrePlacement:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:

				    
				 break;
					 
				 case Arm_normal:
					 if(OrePlacement == state0)
					 {
					 shift_3508_state_OrePlacement_angle = 0;
					 uplift1_3508_state_OrePlacement_angle = -5; 
					 uplift2_3508_state_OrePlacement_angle = -uplift1_3508_state_OrePlacement_angle; 
					 extend1_3508_state_OrePlacement_angle = -2.5; 
					 extend2_3508_state_OrePlacement_angle = -extend1_3508_state_OrePlacement_angle;	
						 
				   dm1_state_OrePlacement_angle = -1.691;    
				   dm2_state_OrePlacement_angle = -0.766; 
					 dm3_state_OrePlacement_angle = 0.720;    
						 
				   dm1_state_OrePlacement_speed = 2;      
				   dm2_state_OrePlacement_speed = 1;
					 dm3_state_OrePlacement_speed = 2;
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
				    if(arm_time_delay(1000) == 1)
					 {
					  OrePlacement = state1;
					  switch_lock = 0;
					 }
					 }			 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:
		 
				 break;
				 
				 case Arm_GroundOre:

				 break;
				 
				 case Arm_OrePlacement:
	 
				 break;
				 
				 case Arm_HoldOre:
					 
         break;					 
			 }	
		 if(switch_lock != 0)
		 {			 
	   shift_3508_angle = shift_3508_state_OrePlacement_angle;
		 uplift1_3508_angle = uplift1_3508_state_OrePlacement_angle; 
		 uplift2_3508_angle = uplift2_3508_state_OrePlacement_angle; 
		 extend1_3508_angle = extend1_3508_state_OrePlacement_angle; 
		 extend2_3508_angle = extend2_3508_state_OrePlacement_angle;
			 
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_OrePlacement_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_OrePlacement_angle;
		 motor_dm3_task->motor_backend_p->params.position_rad = dm3_state_OrePlacement_angle;
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_OrePlacement_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_OrePlacement_speed;
		 motor_dm3_task->motor_backend_p->params.speed_rad = dm3_state_OrePlacement_speed;
		 	
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;
		 robot.arm_task_p->custom_flag = 0;
		 }
		 
		 break;
			 			 
		/****************************Arm_HoldOre状态开始转换************************************/
		 case Arm_HoldOre:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:
			    
				 break;
					 
				 case Arm_normal:
					 if(HoldOre == state0)
					 {	 
					 shift_3508_state_HoldOre_angle = 0;
					 uplift1_3508_state_HoldOre_angle = -1.769; 
					 uplift2_3508_state_HoldOre_angle = -uplift1_3508_state_HoldOre_angle; 
					 extend1_3508_state_HoldOre_angle = -3.00; 
					 extend2_3508_state_HoldOre_angle = -extend1_3508_state_HoldOre_angle;							 
						 
				   dm1_state_HoldOre_angle = -1.759;      
				   dm2_state_HoldOre_angle = -0.766; 
					 dm3_state_HoldOre_angle = -2.429;    
						 
				   dm1_state_HoldOre_speed = 2;     
				   dm2_state_HoldOre_speed = 1;
					 dm3_state_HoldOre_speed = 2;     
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					 HoldOre = state1;
					 }
					 }
					 if(HoldOre == state1)
					 {	 
					 shift_3508_state_HoldOre_angle = 0;
					 uplift1_3508_state_HoldOre_angle = -1.769; 
					 uplift2_3508_state_HoldOre_angle = -uplift1_3508_state_HoldOre_angle; 
					 extend1_3508_state_HoldOre_angle = -3.00; 
					 extend2_3508_state_HoldOre_angle = -extend1_3508_state_HoldOre_angle;		
						 
				   dm1_state_HoldOre_angle = -1.720;      
				   dm2_state_HoldOre_angle = -0.766; 
					 dm3_state_HoldOre_angle = -2.44;   
						 
				   dm1_state_HoldOre_speed = 2;     
				   dm2_state_HoldOre_speed = 1;
					 dm3_state_HoldOre_speed = 2;  
						 
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state2;
					 switch_lock = 0;
					 }
					 } 
					 
				 break;
				 
				 case Arm_AirConnection:
					 
				 break;
				 
				 case Arm_SilveryOre:
					 
				 break;
				 
				 case Arm_GroundOre:
					 
				 break;
				 
				 case Arm_OrePlacement:
					 
				 break;
				 
				 case Arm_HoldOre:

         break;					 
			 }		
		 if(switch_lock != 0)
		 {			 
		 shift_3508_angle = shift_3508_state_HoldOre_angle;
		 uplift1_3508_angle = uplift1_3508_state_HoldOre_angle; 
		 uplift2_3508_angle = uplift2_3508_state_HoldOre_angle; 
		 extend1_3508_angle = extend1_3508_state_HoldOre_angle; 
		 extend2_3508_angle = extend2_3508_state_HoldOre_angle;
		 
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_HoldOre_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_HoldOre_angle;
		 motor_dm3_task->motor_backend_p->params.position_rad = dm3_state_HoldOre_angle;
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_HoldOre_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_HoldOre_speed;
		 motor_dm3_task->motor_backend_p->params.speed_rad = dm3_state_HoldOre_speed;
		 
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;
		 robot.arm_task_p->custom_flag = 0;
		 }
		 
		 break;		 
}
	//输出角度与速度限幅

//    if(shift_3508_angle >=  5)    {shift_3508_angle =  5;}
    if(shift_3508_angle <= 0.1)   {shift_3508_angle =  0.1;}	 //左限位
		
    if(uplift1_3508_angle <=  -6 )  {uplift1_3508_angle =  -6;}
    if(uplift1_3508_angle >= -0.2)  {uplift1_3508_angle = -0.2;}	 
    if(uplift2_3508_angle <=  0.2)  {uplift2_3508_angle =  0.2;}
    if(uplift2_3508_angle >= 6)  {uplift2_3508_angle = 6;}	 
		
    if(extend1_3508_angle <= -8.4)  {extend1_3508_angle = -8.4;}
    if(extend1_3508_angle >= -0.1) {extend1_3508_angle = -0.1;}	 
    if(extend2_3508_angle >= 8.4)  {extend2_3508_angle = 8.4;}
    if(extend2_3508_angle <= 0.1) {extend2_3508_angle = 0.1;}	 		
		
	 
    if(motor_dm1_task->motor_backend_p->params.position_rad >= -0.15f) { motor_dm1_task->motor_backend_p->params.position_rad = -0.15f;}
    if(motor_dm1_task->motor_backend_p->params.position_rad <= -3.1f) { motor_dm1_task->motor_backend_p->params.position_rad = -3.1f;}
    if(motor_dm1_task->motor_backend_p->params.speed_rad >= 5.0f)    { motor_dm1_task->motor_backend_p->params.speed_rad = 5.0f;}
    if(motor_dm1_task->motor_backend_p->params.speed_rad <= 0)       { motor_dm1_task->motor_backend_p->params.speed_rad = 0;}

		if(motor_dm2_task->motor_backend_p->params.position_rad >= 2.5f) { motor_dm2_task->motor_backend_p->params.position_rad = 2.5f;}
    if(motor_dm2_task->motor_backend_p->params.position_rad <= -0.77f)  { motor_dm2_task->motor_backend_p->params.position_rad = -0.77f;}
    if(motor_dm2_task->motor_backend_p->params.speed_rad >= 5.0f)        { motor_dm2_task->motor_backend_p->params.speed_rad = 5.0f;}
    if(motor_dm2_task->motor_backend_p->params.speed_rad <= 0)           { motor_dm2_task->motor_backend_p->params.speed_rad = 0;}
		
		if(motor_dm3_task->motor_backend_p->params.position_rad >= 1.0f) { motor_dm3_task->motor_backend_p->params.position_rad = 1.0f;}
    if(motor_dm3_task->motor_backend_p->params.position_rad <= -2.5f) { motor_dm3_task->motor_backend_p->params.position_rad = -2.5f;}
    if(motor_dm3_task->motor_backend_p->params.speed_rad >= 5.0f)    { motor_dm3_task->motor_backend_p->params.speed_rad = 5.0f;}
    if(motor_dm3_task->motor_backend_p->params.speed_rad <= 0)       { motor_dm3_task->motor_backend_p->params.speed_rad = 0;}
		
		if(custom_flag == 1)
		{
			if(arm_mode == Arm_OrePlacement)
			{
			    if(motor_dm1_task->motor_backend_p->params.position_rad >= -0.15f) { motor_dm1_task->motor_backend_p->params.position_rad = -0.15f;}
          if(motor_dm1_task->motor_backend_p->params.position_rad <= -3.1f) { motor_dm1_task->motor_backend_p->params.position_rad = -3.1f;}
					
					if(motor_dm2_task->motor_backend_p->params.position_rad >= 2.5f) { motor_dm2_task->motor_backend_p->params.position_rad = 2.5f;}
					if(motor_dm2_task->motor_backend_p->params.position_rad <= -0.77f)  { motor_dm2_task->motor_backend_p->params.position_rad = -0.77f;}
					
					if(motor_dm3_task->motor_backend_p->params.position_rad >= 1.0f) { motor_dm3_task->motor_backend_p->params.position_rad = 1.0f;}
					if(motor_dm3_task->motor_backend_p->params.position_rad <= -0.557f) { motor_dm3_task->motor_backend_p->params.position_rad = -0.557f;}					
	 		}
		}
}


/***********************************************************************
** 函 数 名： Arm_ControlTask::update(timeus_t dT_us)
** 函数说明：
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::update(timeus_t dT_us)
{
   float dT_s = dT_us / 1e6f;
	  //机械臂微调

	 if(custom_flag == 1)
	 {
	 	 Custom_position_delt[0] = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_pitch - robot.arm_task_p->Custom_pitch_save / 57.3; //注意此处为弧度值
		 Custom_position_delt[1] = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_roll - robot.arm_task_p->Custom_roll_save / 57.3;
		 Custom_position_delt[2] = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_yaw - robot.arm_task_p->Custom_yaw_save / 57.3;
		 
		 motor_dm1_task->motor_backend_p->params.position_rad = robot.arm_task_p->position_rad_save[0] + Custom_position_delt[0];
		 motor_dm2_task->motor_backend_p->params.position_rad = robot.arm_task_p->position_rad_save[1] + Custom_position_delt[1];
		 motor_dm3_task->motor_backend_p->params.position_rad = robot.arm_task_p->position_rad_save[2] + Custom_position_delt[2];		
	 }
	
	  motor_dm1_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm1_delta;
	  motor_dm2_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm2_delta;	
	  motor_dm3_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm3_delta;	
	  shift_3508_angle += robot.remote_control_task_p->shift_3508_delta;
	  uplift1_3508_angle -= robot.remote_control_task_p->uplift_3508_delta;
	  uplift2_3508_angle  = -uplift1_3508_angle;
	  extend1_3508_angle += robot.remote_control_task_p->extend_3508_delta;
	  extend2_3508_angle  = -extend1_3508_angle;	
     
    //机械臂模式切换
     arm_mode_switch();
	
	/*RM电机控制 */
         /*↓*/	
   // 获取电机角度返回值
    shift_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(shift_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	
		uplift1_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(uplift1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		uplift2_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(uplift2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		extend1_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(extend1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		extend2_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(extend2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
	 
	 // 获取电机角速度返回值
    shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(shift_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(uplift1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(uplift2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);	
	  extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(extend1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(extend2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
		
	 //设置电机角度期望值(外环)
		shift_3508_pid_task->angle_control_task_p->setPIDControllerExpect(shift_3508_angle);
		uplift1_3508_pid_task->angle_control_task_p->setPIDControllerExpect(uplift1_3508_angle);
		uplift2_3508_pid_task->angle_control_task_p->setPIDControllerExpect(uplift2_3508_angle);
		extend1_3508_pid_task->angle_control_task_p->setPIDControllerExpect(extend1_3508_angle);
		extend2_3508_pid_task->angle_control_task_p->setPIDControllerExpect(extend2_3508_angle);

    //限制3058内环角速度输出值
	  // 设置电机角速度期望值（内环）
	  // 平移电机
		if(shift_3508_pid_task->angle_control_task_p->getOutput() <= shift_3508_angular_velocity_max && shift_3508_pid_task->angle_control_task_p->getOutput() >= shift_3508_angular_velocity_min)
		{
      shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(shift_3508_pid_task->angle_control_task_p->getOutput());
		}
		
		if(shift_3508_pid_task->angle_control_task_p->getOutput() > shift_3508_angular_velocity_max) 
		{
      shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(shift_3508_angular_velocity_max);
		}	
		
		if(shift_3508_pid_task->angle_control_task_p->getOutput() < shift_3508_angular_velocity_min)
		{
      shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(shift_3508_angular_velocity_min);
		}
		
		// 抬升电机1号
		if(uplift1_3508_pid_task->angle_control_task_p->getOutput() <= uplift1_3508_angular_velocity_max && uplift1_3508_pid_task->angle_control_task_p->getOutput() >= uplift1_3508_angular_velocity_min)
		{
      uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift1_3508_pid_task->angle_control_task_p->getOutput());
		}
		
	  if(uplift1_3508_pid_task->angle_control_task_p->getOutput() > uplift1_3508_angular_velocity_max) 
		{
      uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift1_3508_angular_velocity_max);
		}	
		
		if(uplift1_3508_pid_task->angle_control_task_p->getOutput() < uplift1_3508_angular_velocity_min)
		{
      uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift1_3508_angular_velocity_min);
		}
		
	  // 抬升电机2号
		if(uplift2_3508_pid_task->angle_control_task_p->getOutput() <= uplift2_3508_angular_velocity_max && uplift2_3508_pid_task->angle_control_task_p->getOutput() >= uplift2_3508_angular_velocity_min)
		{
    uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift2_3508_pid_task->angle_control_task_p->getOutput());
		}
		
		if(uplift2_3508_pid_task->angle_control_task_p->getOutput() > uplift2_3508_angular_velocity_max) 
		{
      uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift2_3508_angular_velocity_max);
		}	
		
		if(uplift2_3508_pid_task->angle_control_task_p->getOutput() < uplift2_3508_angular_velocity_min)
		{
      uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(uplift2_3508_angular_velocity_min);
		}
		
		// 伸出电机1号
		if(extend1_3508_pid_task->angle_control_task_p->getOutput() <= extend1_3508_angular_velocity_max && extend1_3508_pid_task->angle_control_task_p->getOutput() >= extend1_3508_angular_velocity_min)
		{
    extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend1_3508_pid_task->angle_control_task_p->getOutput());
		}
		
	  if(extend1_3508_pid_task->angle_control_task_p->getOutput() > extend1_3508_angular_velocity_max) 
		{
      extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend1_3508_angular_velocity_max);
		}	
		
		if(extend1_3508_pid_task->angle_control_task_p->getOutput() < extend1_3508_angular_velocity_min)
		{
      extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend1_3508_angular_velocity_min);
		}
		
	  // 伸出电机2号
		if(extend2_3508_pid_task->angle_control_task_p->getOutput() <= extend2_3508_angular_velocity_max && extend2_3508_pid_task->angle_control_task_p->getOutput() >= extend2_3508_angular_velocity_min)
		{
    extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend2_3508_pid_task->angle_control_task_p->getOutput());
		}
				
	  if(extend2_3508_pid_task->angle_control_task_p->getOutput() > extend2_3508_angular_velocity_max) 
		{
      extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend2_3508_angular_velocity_max);
		}	
		
		if(extend2_3508_pid_task->angle_control_task_p->getOutput() < extend2_3508_angular_velocity_min)
		{
      extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(extend2_3508_angular_velocity_min);
		}

/////////////////////////////////////////////////////////			
//限制3058内环角速度输出值//

  // 电机正常输出
	// RM 电机
	  shift_3508_pid_task->motor_backend_p->setMotorInput(shift_3508_pid_task->angular_velocity_control_task_p->getOutput());
		uplift1_3508_pid_task->motor_backend_p->setMotorInput(uplift1_3508_pid_task->angular_velocity_control_task_p->getOutput());
		uplift2_3508_pid_task->motor_backend_p->setMotorInput(uplift2_3508_pid_task->angular_velocity_control_task_p->getOutput());
		extend1_3508_pid_task->motor_backend_p->setMotorInput(extend1_3508_pid_task->angular_velocity_control_task_p->getOutput());
		extend2_3508_pid_task->motor_backend_p->setMotorInput(extend2_3508_pid_task->angular_velocity_control_task_p->getOutput());
	 
	// DM 电机
	  motor_dm1_task->motor_backend_p->setMotorPositon(motor_dm1_task->motor_backend_p->params.position_rad ,motor_dm1_task->motor_backend_p->params.speed_rad);	
	  motor_dm2_task->motor_backend_p->setMotorPositon(motor_dm2_task->motor_backend_p->params.position_rad ,motor_dm2_task->motor_backend_p->params.speed_rad);	
	  motor_dm3_task->motor_backend_p->setMotorPositon(motor_dm3_task->motor_backend_p->params.position_rad ,motor_dm3_task->motor_backend_p->params.speed_rad);		
}

