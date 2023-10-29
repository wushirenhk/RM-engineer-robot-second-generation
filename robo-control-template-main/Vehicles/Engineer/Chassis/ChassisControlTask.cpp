/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   ChassisControlTask.cpp
** �ļ�˵����   ��̨��������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-26
***************************************************************************/
#include "/Chassis/ChassisControlTask.h"
#include "/Robot/Robot.h"
#include "/Robot/Params.h"
#include "UARTDriver.h"
#include "math.h"

#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MID               ((uint16_t)3)
#define SWITCH_DWN               ((uint16_t)2)

/***********************************************************************
** �� �� ���� ChassisControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ�����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������ͬʱ
**            �����ע��Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á�yaw��pitch����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
***********************************************************************/
ChassisControlTask::ChassisControlTask(
  Robot &robot0,
  Motor_RM_Params_t *m1, PID_Params_t *m1_ang, PID_Params_t *m1_ang_vel,
  Motor_RM_Params_t *m2, PID_Params_t *m2_ang, PID_Params_t *m2_ang_vel,
  Motor_RM_Params_t *m3, PID_Params_t *m3_ang, PID_Params_t *m3_ang_vel,
  Motor_RM_Params_t *m4, PID_Params_t *m4_ang, PID_Params_t *m4_ang_vel,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;

  if(m1 != NULL)
  {
    if(m1->canx == 1)
    {
      this->motor_1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m1->can_rx_id,
          m1->can_tx_id,
          m1->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    else if(m1->canx == 2)
    {
      this->motor_1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m1->can_rx_id,
          m1->can_tx_id,
          m1->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    this->motor_1_pid_task->motor_backend_p->setParams(*m1);
  }

  if(m2 != NULL)
  {
    if(m2->canx == 1)
    {
      this->motor_2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m2->can_rx_id,
          m2->can_tx_id,
          m2->can_tx_data_start_pos,
          RoboMaster_3508,
          m2->interval);
    }
    else if(m2->canx == 2)
    {
      this->motor_2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m2->can_rx_id,
          m2->can_tx_id,
          m2->can_tx_data_start_pos,
          RoboMaster_3508,
          m2->interval);
    }
    this->motor_2_pid_task->motor_backend_p->setParams(*m2);

  }

  if(m3 != NULL)
  {
    if(m3->canx == 1)
    {
      this->motor_3_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m3->can_rx_id,
          m3->can_tx_id,
          m3->can_tx_data_start_pos,
          RoboMaster_3508,
          m3->interval);
    }
    else if(m3->canx == 2)
    {
      this->motor_3_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m3->can_rx_id,
          m3->can_tx_id,
          m3->can_tx_data_start_pos,
          RoboMaster_3508,
          m3->interval);
    }
    this->motor_3_pid_task->motor_backend_p->setParams(*m3);

  }
  if(m4 != NULL)
  {
    if(m4->canx == 1)
    {
      this->motor_4_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m4->can_rx_id,
          m4->can_tx_id,
          m4->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    else if(m4->canx == 2)
    {
      this->motor_4_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m4->can_rx_id,
          m4->can_tx_id,
          m4->can_tx_data_start_pos,
          RoboMaster_3508,
          m4->interval);
    }
    this->motor_4_pid_task->motor_backend_p->setParams(*m4);
  }

  if(m1_ang_vel != NULL)
  {
    this->motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m1_ang_vel);
    this->motor_1_pid_task->angular_velocity_control_task_p->setInterval((*m1_ang_vel).interval);
  }

  if(m1_ang != NULL)
  {
    this->motor_1_pid_task->angle_control_task_p->setPIDControllerParams(*m1_ang);
    this->motor_1_pid_task->angle_control_task_p->setInterval((*m1_ang).interval);
  }


  if(m2_ang_vel != NULL)
  {
    this->motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m2_ang_vel);
    this->motor_2_pid_task->angular_velocity_control_task_p->setInterval((*m2_ang_vel).interval);
  }

  if(m2_ang != NULL)
  {
    this->motor_2_pid_task->angle_control_task_p->setPIDControllerParams(*m2_ang);
    this->motor_2_pid_task->angle_control_task_p->setInterval((*m2_ang).interval);
  }

  if(m3_ang_vel != NULL)
  {
    this->motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m3_ang_vel);
    this->motor_3_pid_task->angular_velocity_control_task_p->setInterval((*m3_ang_vel).interval);
  }

  if(m3_ang != NULL)
  {
    this->motor_3_pid_task->angle_control_task_p->setPIDControllerParams(*m3_ang);
    this->motor_3_pid_task->angle_control_task_p->setInterval((*m3_ang).interval);
  }

  if(m4_ang_vel != NULL)
  {
    this->motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m4_ang_vel);
    this->motor_4_pid_task->angular_velocity_control_task_p->setInterval((*m4_ang_vel).interval);
  }

  if(m4_ang != NULL)
  {
    this->motor_4_pid_task->angle_control_task_p->setPIDControllerParams(*m4_ang);
    this->motor_4_pid_task->angle_control_task_p->setInterval((*m4_ang).interval);
  }

}

/***********************************************************************
** �� �� ���� ChassisControlTask::init()
** ����˵���� ע����̨�������Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void ChassisControlTask::init(void)
{
  inited = true;
	/* ������������ݻ�ȡ����ע�� */
  if(motor_1_pid_task != NULL)
    robot.scheduler.registerTask(motor_1_pid_task);
  if(motor_2_pid_task != NULL)
    robot.scheduler.registerTask(motor_2_pid_task);
  if(motor_3_pid_task != NULL)
    robot.scheduler.registerTask(motor_3_pid_task);
  if(motor_4_pid_task != NULL)
    robot.scheduler.registerTask(motor_4_pid_task);
	/* ������������ݻ�ȡ����ע�� */
	
	/* PID����ע�� */
        /*��*/
  if(motor_1_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_1_pid_task->angular_velocity_control_task_p);

  if(motor_2_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_2_pid_task->angular_velocity_control_task_p);

  if(motor_3_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_3_pid_task->angular_velocity_control_task_p);

  if(motor_4_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_4_pid_task->angular_velocity_control_task_p);
        /*��*/
 	/* PID����ע�� */	
  
   half_wheel_base = INITIAL_HALF_WHEEL_BASE;
   half_tread = INITIAL_HALF_TREAD;
   rotate_ratio_x = INITIAL_ROTATE_RATIO_X;
   rotate_ratio_y = INITIAL_ROTATE_RATIO_Y;
   wheel_vel_ratio = INITIAL_WHEEL_VEL_RATIO;
	xy_chassis_slow = INITIAL_XY_Chassis_Slow;
	chassis_mode = Chassis_FAST;
	
	rotate_ratio[0]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f-rotate_ratio_x)) + cmt::sq(half_tread*(1.0f+rotate_ratio_y)));
	rotate_ratio[1]=std::sqrtf(cmt::sq(half_wheel_base*(1.0-rotate_ratio_x)) + cmt::sq(half_tread*(1.0f-rotate_ratio_y)));
	rotate_ratio[2]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f+rotate_ratio_x)) + cmt::sq(half_tread*(1.0f-rotate_ratio_y)));
	rotate_ratio[3]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f+rotate_ratio_x)) + cmt::sq(half_tread*(1.0f+rotate_ratio_y)));
}

/***********************************************************************
** �� �� ���� ChassisControlTask::update(timeus_t dT_us)
** ����˵���� ͨ����̨�����������IMU������̨Yaw�������������
**            �Ȳ��ֵĿռ���̬��������̨��������������񣬲�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void ChassisControlTask::update(timeus_t dT_us)
{
     
	if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().sw_left == SWITCH_DWN )//�������
	{
		
	//��ȡ��ģʽ����������
	  if(robot.chassis_task_p->chassis_direction_mode == Chassis_NORMAL && robot.gimbal_task_p->gimbal_mode != GIMBAL_LOCK)
	  {
			if(robot.rc_protocol.getRCData().keyboard.key_bit.W ||robot.rc_protocol.getRCData().keyboard.key_bit.S)
			{			
				
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vx = robot.chassis_task_p->vx + (robot.rc_protocol.getRCData().keyboard.key_bit.W - robot.rc_protocol.getRCData().keyboard.key_bit.S)*0.01f;//������ƻ���
				 if(robot.chassis_task_p->vx > max_vx){robot.chassis_task_p->vx = max_vx;}
				 else if(robot.chassis_task_p->vx < min_vx){robot.chassis_task_p->vx = min_vx;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vx += (robot.rc_protocol.getRCData().keyboard.key_bit.W - robot.rc_protocol.getRCData().keyboard.key_bit.S)*0.002f;//������ƻ���
				 if(robot.chassis_task_p->vx > max_vx_slow){robot.chassis_task_p->vx = max_vx_slow;}
				 else if(robot.chassis_task_p->vx < min_vx_slow){robot.chassis_task_p->vx = min_vx_slow;}
				}
			}
				 
			else{
						if(chassis_mode == Chassis_FAST)
						{
							if(robot.chassis_task_p->vx > 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx - 0.008f;
									 if(robot.chassis_task_p->vx < 0.1){robot.chassis_task_p->vx = 0;}
									 
								 }
							if(robot.chassis_task_p->vx < 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx + 0.008f;
									 if(robot.chassis_task_p->vx > -0.1){robot.chassis_task_p->vx = 0;}
								 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
							if(robot.chassis_task_p->vx > 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx - 0.004f;
									 if(robot.chassis_task_p->vx < 0.1){robot.chassis_task_p->vx = 0;}
									 
								 }
							if(robot.chassis_task_p->vx < 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx + 0.004f;
									 if(robot.chassis_task_p->vx > -0.1){robot.chassis_task_p->vx = 0;}
								 }	
						}			 
				  }
			
			if(robot.rc_protocol.getRCData().keyboard.key_bit.A ||robot.rc_protocol.getRCData().keyboard.key_bit.D)
			{		
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vy += (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*0.01f;//������ƻ���
				 if(robot.chassis_task_p->vy > max_vy){robot.chassis_task_p->vy = max_vy;}
				 else if(robot.chassis_task_p->vy < min_vy){robot.chassis_task_p->vy = min_vy;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vy += (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*0.002f;//������ƻ���
				 if(robot.chassis_task_p->vy > max_vy_slow){robot.chassis_task_p->vy = max_vy_slow;}
				 else if(robot.chassis_task_p->vy < min_vy_slow){robot.chassis_task_p->vy = min_vy_slow;}
				}
			}
			else{
						if(chassis_mode == Chassis_FAST)
						{
						if(robot.chassis_task_p->vy > 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy - 0.008f;
								 if(robot.chassis_task_p->vy < 0.1){robot.chassis_task_p->vy = 0;}
								 
							 }
						if(robot.chassis_task_p->vy < 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy + 0.008f;
								 if(robot.chassis_task_p->vy > -0.1){robot.chassis_task_p->vy = 0;}
							 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
						if(robot.chassis_task_p->vy > 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy - 0.004f;
								 if(robot.chassis_task_p->vy < 0.1){robot.chassis_task_p->vy = 0;}
								 
							 }
						if(robot.chassis_task_p->vy < 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy + 0.004f;
								 if(robot.chassis_task_p->vy > -0.1){robot.chassis_task_p->vy = 0;}
							 }	
						}				 
				  }
		
			if(robot.rc_protocol.getRCData().keyboard.key_bit.Q ||robot.rc_protocol.getRCData().keyboard.key_bit.E)
			{			
				
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vw += (robot.rc_protocol.getRCData().keyboard.key_bit.Q - robot.rc_protocol.getRCData().keyboard.key_bit.E)*0.01f;//������ƻ���
				 if(robot.chassis_task_p->vw > max_vw){robot.chassis_task_p->vw = max_vw;}
				 else if(robot.chassis_task_p->vw < min_vw){robot.chassis_task_p->vw = min_vw;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vw += (robot.rc_protocol.getRCData().keyboard.key_bit.Q - robot.rc_protocol.getRCData().keyboard.key_bit.E)*0.002f;//������ƻ���
				 if(robot.chassis_task_p->vw > max_vw_slow){robot.chassis_task_p->vw = max_vw_slow;}
				 else if(robot.chassis_task_p->vw < min_vw_slow){robot.chassis_task_p->vw = min_vw_slow;}
				}
				
			}
			else{
						if(chassis_mode == Chassis_FAST)
						{
						if(robot.chassis_task_p->vw > 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw - 0.008f;
								 if(robot.chassis_task_p->vw < 0.1){robot.chassis_task_p->vw = 0;}
								 
							 }
						if(robot.chassis_task_p->vw < 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw + 0.008f;
								 if(robot.chassis_task_p->vw > -0.1){robot.chassis_task_p->vw = 0;}
							 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
						if(robot.chassis_task_p->vw > 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw - 0.004f;
								 if(robot.chassis_task_p->vw < 0.1){robot.chassis_task_p->vw = 0;}
								 
							 }
						if(robot.chassis_task_p->vw < 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw + 0.004f;
								 if(robot.chassis_task_p->vw > -0.1){robot.chassis_task_p->vw = 0;}
							 }	
						}			 
				  }
		 }
	  
		//ȡ��ģʽ����������ת90��
	  if(robot.chassis_task_p->chassis_direction_mode == Chassis_ORE || robot.gimbal_task_p->gimbal_mode == GIMBAL_LOCK)
	  {
			if(robot.rc_protocol.getRCData().keyboard.key_bit.A ||robot.rc_protocol.getRCData().keyboard.key_bit.D)
			{			
				
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vx = robot.chassis_task_p->vx + (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*0.01f;//������ƻ���
				 if(robot.chassis_task_p->vx > max_vx){robot.chassis_task_p->vx = max_vx;}
				 else if(robot.chassis_task_p->vx < min_vx){robot.chassis_task_p->vx = min_vx;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vx += (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*0.003f;//������ƻ���
				 if(robot.chassis_task_p->vx > max_vx_slow){robot.chassis_task_p->vx = max_vx_slow;}
				 else if(robot.chassis_task_p->vx < min_vx_slow){robot.chassis_task_p->vx = min_vx_slow;}
				}
			}
				 
			else{
						if(chassis_mode == Chassis_FAST)
						{
							if(robot.chassis_task_p->vx > 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx - 0.01f;
									 if(robot.chassis_task_p->vx < 0.1){robot.chassis_task_p->vx = 0;}
									 
								 }
							if(robot.chassis_task_p->vx < 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx + 0.01f;
									 if(robot.chassis_task_p->vx > -0.1){robot.chassis_task_p->vx = 0;}
								 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
							if(robot.chassis_task_p->vx > 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx - 0.004f;
									 if(robot.chassis_task_p->vx < 0.1){robot.chassis_task_p->vx = 0;}
									 
								 }
							if(robot.chassis_task_p->vx < 0) 
								 {
									 robot.chassis_task_p->vx = robot.chassis_task_p->vx + 0.004f;
									 if(robot.chassis_task_p->vx > -0.1){robot.chassis_task_p->vx = 0;}
								 }	
						}				 
				  }
			
			if(robot.rc_protocol.getRCData().keyboard.key_bit.W ||robot.rc_protocol.getRCData().keyboard.key_bit.S)
			{		
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vy += (robot.rc_protocol.getRCData().keyboard.key_bit.S - robot.rc_protocol.getRCData().keyboard.key_bit.W)*0.01f;//������ƻ���
				 if(robot.chassis_task_p->vy > max_vy){robot.chassis_task_p->vy = max_vy;}
				 else if(robot.chassis_task_p->vy < min_vy){robot.chassis_task_p->vy = min_vy;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vy += (robot.rc_protocol.getRCData().keyboard.key_bit.S - robot.rc_protocol.getRCData().keyboard.key_bit.W)*0.003f;//������ƻ���
				 if(robot.chassis_task_p->vy > max_vy_slow){robot.chassis_task_p->vy = max_vy_slow;}
				 else if(robot.chassis_task_p->vy < min_vy_slow){robot.chassis_task_p->vy = min_vy_slow;}
				}
			}
			else{
						if(chassis_mode == Chassis_FAST)
						{
						if(robot.chassis_task_p->vy > 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy - 0.01f;
								 if(robot.chassis_task_p->vy < 0.1){robot.chassis_task_p->vy = 0;}
								 
							 }
						if(robot.chassis_task_p->vy < 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy + 0.01f;
								 if(robot.chassis_task_p->vy > -0.1){robot.chassis_task_p->vy = 0;}
							 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
						if(robot.chassis_task_p->vy > 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy - 0.004f;
								 if(robot.chassis_task_p->vy < 0.1){robot.chassis_task_p->vy = 0;}
								 
							 }
						if(robot.chassis_task_p->vy < 0) 
							 {
								 robot.chassis_task_p->vy = robot.chassis_task_p->vy + 0.004f;
								 if(robot.chassis_task_p->vy > -0.1){robot.chassis_task_p->vy = 0;}
							 }	
						}			 
				  }
		
			if(robot.rc_protocol.getRCData().keyboard.key_bit.Q ||robot.rc_protocol.getRCData().keyboard.key_bit.E)
			{			
				
				 if(chassis_mode == Chassis_FAST)
				{
				 robot.chassis_task_p->vw += (robot.rc_protocol.getRCData().keyboard.key_bit.Q - robot.rc_protocol.getRCData().keyboard.key_bit.E)*0.005f;//������ƻ���
				 if(robot.chassis_task_p->vw > max_vw){robot.chassis_task_p->vw = max_vw;}
				 else if(robot.chassis_task_p->vw < min_vw){robot.chassis_task_p->vw = min_vw;}
				}
				 if(chassis_mode == Chassis_SLOW)
				{
				 robot.chassis_task_p->vw += (robot.rc_protocol.getRCData().keyboard.key_bit.Q - robot.rc_protocol.getRCData().keyboard.key_bit.E)*0.003f;//������ƻ���
				 if(robot.chassis_task_p->vw > max_vw_slow){robot.chassis_task_p->vw = max_vw_slow;}
				 else if(robot.chassis_task_p->vw < min_vw_slow){robot.chassis_task_p->vw = min_vw_slow;}
				}
				
			}
			else{
						if(chassis_mode == Chassis_FAST)
						{
						if(robot.chassis_task_p->vw > 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw - 0.02f;
								 if(robot.chassis_task_p->vw < 0.1){robot.chassis_task_p->vw = 0;}
								 
							 }
						if(robot.chassis_task_p->vw < 0) 
							 {
								 robot.chassis_task_p->vw = robot.chassis_task_p->vw + 0.02f;
								 if(robot.chassis_task_p->vw > -0.1){robot.chassis_task_p->vw = 0;}
							 }	
						 }

						 if(chassis_mode == Chassis_SLOW)
						{
							if(robot.chassis_task_p->vw > 0) 
								 {
									 robot.chassis_task_p->vw = robot.chassis_task_p->vw - 0.004f;
									 if(robot.chassis_task_p->vw < 0.1){robot.chassis_task_p->vw = 0;}
									 
								 }
							if(robot.chassis_task_p->vw < 0) 
								 {
									 robot.chassis_task_p->vw = robot.chassis_task_p->vw + 0.004f;
									 if(robot.chassis_task_p->vw > -0.1){robot.chassis_task_p->vw = 0;}
								 }	
						}					
				  }		  
		  
		  
		  
		  
		  
	  }
	}
	
	
	//�˲�

	if(chassis_mode == Chassis_OFF)
	{
		vx = 0;
		vy = 0;
		vw = 0;
	}
  
		
  exp_vel[0] = (-vx - vy)*xy_chassis_slow - vw * rotate_ratio[0] * wheel_vel_ratio;
  exp_vel[1] = (vx - vy)*xy_chassis_slow - vw * rotate_ratio[1] * wheel_vel_ratio;
  exp_vel[2] = (vx + vy)*xy_chassis_slow - vw * rotate_ratio[2] * wheel_vel_ratio;
  exp_vel[3] = (-vx + vy)*xy_chassis_slow - vw * rotate_ratio[3] * wheel_vel_ratio;
  
  motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[0]);
  motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[1]);
  motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[2]);
  motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[3]);
	
  motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_1_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_2_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_3_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_4_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  
  motor_1_pid_task->motor_backend_p->setMotorInput(motor_1_pid_task->angular_velocity_control_task_p->getOutput());
  motor_2_pid_task->motor_backend_p->setMotorInput(motor_2_pid_task->angular_velocity_control_task_p->getOutput());
  motor_3_pid_task->motor_backend_p->setMotorInput(motor_3_pid_task->angular_velocity_control_task_p->getOutput());
  motor_4_pid_task->motor_backend_p->setMotorInput(motor_4_pid_task->angular_velocity_control_task_p->getOutput());
  
}

/***********************************************************************
** �� �� ���� ChassisControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void ChassisControlTask::uninit(void) {}
