/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   Engineer
** �� �� ����   ArmControlTask.cpp
** �ļ�˵����   ���������������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						       ������     	        2023-02-11

***************************************************************************/
#include "ArmControlTask.h"
#include "../Robot/Robot.h"
#include "Helper.h"
#include "../Robot/Params.h"
#include "/Gimbal/GimbalControlTask.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

double s2=0;//dm2�������Ƕȵ���

/***********************************************************************
** �� �� ���� Arm_ControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ��yaw��pitch����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á����Ħ���ֵ��CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
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
	
	/********************new��������********************/
	/*****************************��***************************/
  //RM���//
	//ƽ�Ƶ�� 1��
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
		
	  //̧�����1��
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
		
		//̧�����2��
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
		
		//������1��
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
		
		//������2��
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
		
  //DMһ�ŵ��
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
	
  //DM���ŵ��
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
	
  //DM���ŵ��
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
		
	//ƽ�Ƶ��
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
	
	//̧�����1��
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
	
	//̧�����2��
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
	
	//������1��
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
	
	//������2��
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
** �� �� ���� Arm_ControlTask::init()
** ����˵���� ע�����Ħ���ַ�����Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::init(void)
{
	   inited = true;
		
	
	  //��������ݻ�ȡ����ע��
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
		
	   /* PID����ע�� */
         /*��*/

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
		
	       /*��*/
 	   /* PID����ע�� */		
		  
		  
		arm_mode = Arm_OFF;
		exp_angle = 0;
	
		shift_3508_angle = -0.1;  //����		
		uplift1_3508_angle = 0; //ע�����ߵĽǶ��෴�����Ų�Ҫ���� uplift1_3508_angleȡ��Ϊ��
		uplift2_3508_angle = -uplift1_3508_angle;
		extend1_3508_angle = -0.1;//ע�����ߵĽǶ��෴�����Ų�Ҫ���� extend1_3508_angleȡ��Ϊ��
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
			
		motor_dm1_task->motor_backend_p->params.position_rad = -1.72;      //picth��
		motor_dm2_task->motor_backend_p->params.position_rad = -0.766; 
		motor_dm3_task->motor_backend_p->params.position_rad =  -2.44;      //����
	  motor_dm1_task->motor_backend_p->params.speed_rad = 0;      //����/sz
		motor_dm2_task->motor_backend_p->params.speed_rad = 0;	
		motor_dm3_task->motor_backend_p->params.speed_rad = 0;	
}

/***********************************************************************
** �� �� ���� ArmRM_ControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::uninit(void) 
{

}

/***********************************************************************
** �� �� ���� Arm_ControlTask::arm_time_delay(uint16_t time)
** ����˵���� arm������ʱ�����ж�ʽ������λms
**---------------------------------------------------------------------
** ��������� time����ʱʱ�䣬��λms
** ���ز����� ��
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
** �� �� ���� Arm_ControlTask::arm_mode_switch()
** ����˵���� ��е��ģʽ�л�
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::arm_mode_switch()
{
  /*�жϴ�ʱ״̬��ʹ�ò�ͬ���Ʒ�ʽ*/
           /*��*/
		 switch (arm_mode){
		 case Arm_OFF:	  
			 
		 motor_dm1_task->motor_backend_p->params.speed_rad = 0;
		 motor_dm2_task->motor_backend_p->params.speed_rad = 0; 
		 motor_dm3_task->motor_backend_p->params.speed_rad = 0; 

		 break;
		 /****************************normal״̬��ʼת��************************************/
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
			 /****************************AirConnection״̬��ʼת��************************************/
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
			 /****************************Arm_SilveryOre״̬��ʼת��************************************/
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
			 			 
		/****************************Arm_OrePlacement״̬��ʼת��************************************/
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
			 			 
		/****************************Arm_HoldOre״̬��ʼת��************************************/
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
	//����Ƕ����ٶ��޷�

//    if(shift_3508_angle >=  5)    {shift_3508_angle =  5;}
    if(shift_3508_angle <= 0.1)   {shift_3508_angle =  0.1;}	 //����λ
		
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
** �� �� ���� Arm_ControlTask::update(timeus_t dT_us)
** ����˵����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::update(timeus_t dT_us)
{
   float dT_s = dT_us / 1e6f;
	  //��е��΢��

	 if(custom_flag == 1)
	 {
	 	 Custom_position_delt[0] = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_pitch - robot.arm_task_p->Custom_pitch_save / 57.3; //ע��˴�Ϊ����ֵ
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
     
    //��е��ģʽ�л�
     arm_mode_switch();
	
	/*RM������� */
         /*��*/	
   // ��ȡ����Ƕȷ���ֵ
    shift_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(shift_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	
		uplift1_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(uplift1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		uplift2_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(uplift2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		extend1_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(extend1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
		extend2_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(extend2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);
	 
	 // ��ȡ������ٶȷ���ֵ
    shift_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(shift_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  uplift1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(uplift1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  uplift2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(uplift2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);	
	  extend1_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(extend1_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	  extend2_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(extend2_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
		
	 //���õ���Ƕ�����ֵ(�⻷)
		shift_3508_pid_task->angle_control_task_p->setPIDControllerExpect(shift_3508_angle);
		uplift1_3508_pid_task->angle_control_task_p->setPIDControllerExpect(uplift1_3508_angle);
		uplift2_3508_pid_task->angle_control_task_p->setPIDControllerExpect(uplift2_3508_angle);
		extend1_3508_pid_task->angle_control_task_p->setPIDControllerExpect(extend1_3508_angle);
		extend2_3508_pid_task->angle_control_task_p->setPIDControllerExpect(extend2_3508_angle);

    //����3058�ڻ����ٶ����ֵ
	  // ���õ�����ٶ�����ֵ���ڻ���
	  // ƽ�Ƶ��
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
		
		// ̧�����1��
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
		
	  // ̧�����2��
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
		
		// ������1��
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
		
	  // ������2��
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
//����3058�ڻ����ٶ����ֵ//

  // ����������
	// RM ���
	  shift_3508_pid_task->motor_backend_p->setMotorInput(shift_3508_pid_task->angular_velocity_control_task_p->getOutput());
		uplift1_3508_pid_task->motor_backend_p->setMotorInput(uplift1_3508_pid_task->angular_velocity_control_task_p->getOutput());
		uplift2_3508_pid_task->motor_backend_p->setMotorInput(uplift2_3508_pid_task->angular_velocity_control_task_p->getOutput());
		extend1_3508_pid_task->motor_backend_p->setMotorInput(extend1_3508_pid_task->angular_velocity_control_task_p->getOutput());
		extend2_3508_pid_task->motor_backend_p->setMotorInput(extend2_3508_pid_task->angular_velocity_control_task_p->getOutput());
	 
	// DM ���
	  motor_dm1_task->motor_backend_p->setMotorPositon(motor_dm1_task->motor_backend_p->params.position_rad ,motor_dm1_task->motor_backend_p->params.speed_rad);	
	  motor_dm2_task->motor_backend_p->setMotorPositon(motor_dm2_task->motor_backend_p->params.position_rad ,motor_dm2_task->motor_backend_p->params.speed_rad);	
	  motor_dm3_task->motor_backend_p->setMotorPositon(motor_dm3_task->motor_backend_p->params.position_rad ,motor_dm3_task->motor_backend_p->params.speed_rad);		
}

