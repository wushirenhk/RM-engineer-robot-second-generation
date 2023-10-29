/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   GimbalControlTask.cpp
** �ļ�˵����   ��̨�ͷ����������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-26
**  1.1                  �����˾����������                ����Դ            2023-02-12
**  1.2                  ���̳��ľ���ʹ��                ������            2023-03-28
***************************************************************************/
#include "/Gimbal/GimbalControlTask.h"
#include "/Robot/Robot.h"

/***********************************************************************
** �� �� ���� GimbalControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ��yaw��pitch����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������ͬʱ
**            ��yaw��pitch����ע��Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á�yaw��pitch����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
***********************************************************************/
GimbalControlTask::GimbalControlTask(
  Robot &robot0,
  Motor_RM_Params_t *Overturn1, PID_Params_t *Overturn1_ang, PID_Params_t *Overturn1_ang_vel,
  Motor_RM_Params_t *Overturn2, PID_Params_t *Overturn2_ang, PID_Params_t *Overturn2_ang_vel,
  Motor_RM_Params_t *gimbal_6020,PID_Params_t *gimbal_6020_ang, PID_Params_t *gimbal_6020_ang_vel,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;		
	
	  if(Overturn1 != NULL)
  {
    if(Overturn1->canx == 1)
    {
      this->motor_Overturn1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          Overturn1->can_rx_id,
          Overturn1->can_tx_id,
          Overturn1->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn1->interval);
    }
    else if(Overturn1->canx == 2)
    {
      this->motor_Overturn1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          Overturn1->can_rx_id,
          Overturn1->can_tx_id,
          Overturn1->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn1->interval);
    }
    this->motor_Overturn1_pid_task->motor_backend_p->setParams(*Overturn1);
  }

  if(Overturn2 != NULL)
  {
    if(Overturn2->canx == 1)
    {
      this->motor_Overturn2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          Overturn2->can_rx_id,
          Overturn2->can_tx_id,
          Overturn2->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn2->interval);
    }
    else if(Overturn2->canx == 2)
    {
      this->motor_Overturn2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          Overturn2->can_rx_id,
          Overturn2->can_tx_id,
          Overturn2->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn2->interval);
    }
    this->motor_Overturn2_pid_task->motor_backend_p->setParams(*Overturn2);
  }
  
    if(gimbal_6020 != NULL)
  {
    if(gimbal_6020->canx == 1)
    {
      this->motor_gimbal_6020_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          gimbal_6020->can_rx_id,
          gimbal_6020->can_tx_id,
          gimbal_6020->can_tx_data_start_pos,
          RoboMaster_GM6020,
          gimbal_6020->interval);
    }
    else if(gimbal_6020->canx == 2)
    {
      this->motor_gimbal_6020_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          gimbal_6020->can_rx_id,
          gimbal_6020->can_tx_id,
          gimbal_6020->can_tx_data_start_pos,
          RoboMaster_GM6020,
          gimbal_6020->interval);
    }
    this->motor_gimbal_6020_pid_task->motor_backend_p->setParams(*gimbal_6020);
  }

  if(Overturn1_ang_vel != NULL)
  {
    this->motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*Overturn1_ang_vel);
    this->motor_Overturn1_pid_task->angular_velocity_control_task_p->setInterval((*Overturn1_ang_vel).interval);
  }

  if(Overturn1_ang != NULL)
  {
    this->motor_Overturn1_pid_task->angle_control_task_p->setPIDControllerParams(*Overturn1_ang);
    this->motor_Overturn1_pid_task->angle_control_task_p->setInterval((*Overturn1_ang).interval);
  }


  if(Overturn2_ang_vel != NULL)
  {
    this->motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*Overturn2_ang_vel);
    this->motor_Overturn2_pid_task->angular_velocity_control_task_p->setInterval((*Overturn2_ang_vel).interval);
  }

  if(Overturn2_ang != NULL)
  {
    this->motor_Overturn2_pid_task->angle_control_task_p->setPIDControllerParams(*Overturn2_ang);
    this->motor_Overturn2_pid_task->angle_control_task_p->setInterval((*Overturn2_ang).interval);
  }

  if(gimbal_6020_ang_vel != NULL)
  {
    this->motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*gimbal_6020_ang_vel);
    this->motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setInterval((*gimbal_6020_ang_vel).interval);
  }

  if(gimbal_6020_ang != NULL)
  {
    this->motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerParams(*gimbal_6020_ang);
    this->motor_gimbal_6020_pid_task->angle_control_task_p->setInterval((*gimbal_6020_ang).interval);
  }	
	
	
	
}




/***********************************************************************
** �� �� ���� GimbalControlTask::init()
** ����˵���� ע����̨�������Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::init(void)
{
  inited = true;
  if(motor_Overturn1_pid_task != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task);
  if(motor_Overturn2_pid_task != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task);
  if(motor_gimbal_6020_pid_task != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task);

  if(motor_Overturn1_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task->angle_control_task_p );
  if(motor_Overturn1_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task->angular_velocity_control_task_p );

  if(motor_Overturn2_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task->angle_control_task_p );
  if(motor_Overturn2_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task->angular_velocity_control_task_p );

  if(motor_gimbal_6020_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task->angle_control_task_p );
  if(motor_gimbal_6020_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task->angular_velocity_control_task_p );
  
  overturn_mode = Overturn_OFF;
  gimbal_mode = GIMBAL_FREE;
}

/***********************************************************************
** �� �� ���� GimbalControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::uninit(void) {}
	
/***********************************************************************
** �� �� ���� GimbalControlTask::update(timeus_t dT_us)
** ����˵���� ͨ����̨�����������IMU������̨Yaw�������������
**            �Ȳ��ֵĿռ���̬��������̨��������������񣬲�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::update(timeus_t dT_us)
{
	  float dT_s = dT_us / 1e6f;
   //   USART1_DMA_Debug_Printf("1=%-16f2=%-16f\r\n",motor_Overturn2_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity , motor_Overturn2_pid_task->angular_velocity_control_task_p->getOutput());
	/**************************�������************************************************/
   /*********************************************************************************/	
   if(overturn_mode == Overturn_CW)
	{

	 	 // ����1������ٶ�����ֵ���ڻ���
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_max
	    );

		
	 	 // ����2������ٶ�����ֵ���ڻ���
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_min
	    );

	}
	   if(overturn_mode == Overturn_CCW)
	{

	 	 // ����1������ٶ�����ֵ���ڻ���
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_min
	    );

		
	 	 // ����2������ٶ�����ֵ���ڻ���
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_max
	    );

	}
	
	   if(overturn_mode == Overturn_OUT)
	{

	 	 // ����1������ٶ�����ֵ���ڻ���
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_min
	    );
	
	 	 // ����2������ٶ�����ֵ���ڻ���
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_min
	    );
		
	}
		   
	   if(overturn_mode == Overturn_IN)
	{

	 	 // ����1������ٶ�����ֵ���ڻ���
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_max
	    );
	
	 	 // ����2������ٶ�����ֵ���ڻ���
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_max
	    );
		
	}	

	if(overturn_mode == Overturn_OFF)
	{
		 // 1������ֵ
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	     0
	    );
	
	    // 2������ֵ
	   motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
        0
	    );
	}
	
	 // ��ȡ������ٶȷ���ֵ
    motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_Overturn1_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	 motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_Overturn2_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
			
	    // 1������ֵ
	    motor_Overturn1_pid_task->motor_backend_p->setMotorInput(
       motor_Overturn1_pid_task->angular_velocity_control_task_p->getOutput()
	    );  
	
	    // 2������ֵ
	    motor_Overturn2_pid_task->motor_backend_p->setMotorInput(
       motor_Overturn2_pid_task->angular_velocity_control_task_p->getOutput()
	    ); 

 
	

	
	
	/********************************6020����******************************************/
   /*********************************************************************************/	
	
	/* gimbal_6020������� */
         /*��*/	
switch (gimbal_mode)
{	
	case(0):
    gimbal_yaw_angle += gimbal_yaw_vel * dT_s;
	break;
	
	case(1):
	 gimbal_yaw_angle = motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + angle_glod_yaw;
	 gimbal_pitch_angle = angle_glod_pitch;
	
}
    // ��ȡ����Ƕȷ���ֵ
    motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerFeedback(motor_gimbal_6020_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	
    
	 // ��ȡ������ٶȷ���ֵ
    motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_gimbal_6020_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);	
	
	
		// ���õ���Ƕ�����ֵ(�⻷)
		motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerExpect(
			gimbal_yaw_angle
			);
			
	   motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   motor_gimbal_6020_pid_task->angle_control_task_p->getOutput()
		);
		
		// gimbal������ֵ
	   motor_gimbal_6020_pid_task->motor_backend_p->setMotorInput(
      motor_gimbal_6020_pid_task->angular_velocity_control_task_p->getOutput()
	    ); 
	
			/* ������� */
	
	  robot.helper.setServo(3, gimbal_pitch_angle);
	
}







