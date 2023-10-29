/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Robot.cpp
** �ļ�˵����   �����˻���
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-10
***************************************************************************/
#include "Robot.h"
#include "arm_math.h"
#include "Motor_RM_Tasks.h"
#include "Params.h"
#include "tim.h"
#include "UARTDriver.h"
/***********************************************************************
** �� �� ���� Robot::init()
** ����˵���� ������ϵͳ��ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Robot::init()
{
  // ��ʼ���������
  params.initMotorsParams();                                  //��������
  attitude_solution_task_p = new AttitudeSolutionTask(*this); // ��̬����, ���ⲿ�ж��н���
  led_control_task_p = new LEDControlTask(*this);             // LED����
  referee_system_task_p = new RefereeSystemTask(*this);       // ����ϵͳ
  remote_control_task_p = new RemoteControlTask(*this);       //ң��
  bmi088_id = inertial_sensors.addBackend(new InertialSensor_BMI088(inertial_sensors, ROTATION_YAW_180)); // BMI088 �Ʊ���Z����ת180��

  rc_protocol.init();
  scheduler.init();
  can1_device.init(&hcan1);
  can2_device.init(&hcan2);

	//arm_control����new
	 arm_task_p = new Arm_ControlTask(
		*this,
		&params.motor_params.shift_3508,
      &params.motor_params.shift_3508_ang,
      &params.motor_params.shift_3508_ang_vel,
		
		&params.motor_params.uplift1_3508,
      &params.motor_params.uplift1_3508_ang,
      &params.motor_params.uplift1_3508_ang_vel,
		
		&params.motor_params.uplift2_3508,
      &params.motor_params.uplift2_3508_ang,
      &params.motor_params.uplift2_3508_ang_vel,
		
		&params.motor_params.extend1_3508,
      &params.motor_params.extend1_3508_ang,
      &params.motor_params.extend1_3508_ang_vel,
		
		&params.motor_params.extend2_3508,
      &params.motor_params.extend2_3508_ang,
      &params.motor_params.extend2_3508_ang_vel,
		
		&params.motor_params.arm_dm_1,
		&params.motor_params.arm_dm_2,
		&params.motor_params.arm_dm_3
	);
		
	//chassis ����new
  chassis_task_p = new ChassisControlTask(
    *this,

    &params.motor_params.chassis_motor_1,
    NULL,
    &params.motor_params.chassis_motor_1_ang_vel,

    &params.motor_params.chassis_motor_2,
    NULL,
    &params.motor_params.chassis_motor_2_ang_vel,

    &params.motor_params.chassis_motor_3,
    NULL,
    &params.motor_params.chassis_motor_3_ang_vel,

    &params.motor_params.chassis_motor_4,
    NULL,
    &params.motor_params.chassis_motor_4_ang_vel
  );
	 
		//gimbal_control����new
	 gimbal_task_p = new GimbalControlTask(
		*this,
    &params.motor_params.overturn1_3508,
    &params.motor_params.overturn1_3508_ang,
    &params.motor_params.overturn1_3508_ang_vel,

    &params.motor_params.overturn2_3508,
    &params.motor_params.overturn2_3508_ang,
    &params.motor_params.overturn2_3508_ang_vel,
		
	 &params.motor_params.gimbal_6020,
	 &params.motor_params.gimbal_6020_ang,		
	 &params.motor_params.gimbal_6020_ang_vel
		
	);
	 
	 
	 can1_send_task_0x101_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x101,
    params.motor_params.control_tasks_interval.can1_send_0x101_task_interval
  );
	 
	 can1_send_task_0x102_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x102,
    params.motor_params.control_tasks_interval.can1_send_0x102_task_interval
  );
	
	 can1_send_task_0x103_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x103,
    params.motor_params.control_tasks_interval.can1_send_0x103_task_interval
  );	 
//	 can1_send_task_0x145_p = new CANSendTask(
//    *this,
//    &this->can1_device,
//    0x145,
//    params.motor_params.control_tasks_interval.can1_send_0x145_task_interval
//  );
//	 can1_send_task_0x146_p = new CANSendTask(
//    *this,
//    &this->can1_device,
//    0x146,
//    params.motor_params.control_tasks_interval.can1_send_0x146_task_interval
//  );	 
//	 
	  //can2_0x200���ͳ�ʼ��	 
	 can2_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x200,
    params.motor_params.control_tasks_interval.can2_send_0x200_task_interval
  );
	 
	 //can1_0x200���ͳ�ʼ��	 
	 can1_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x200,
    params.motor_params.control_tasks_interval.can1_send_0x200_task_interval
  );
	 
	 
	  //can2 0x1FF���ͳ�ʼ��	 
	 can2_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can2_send_0x1ff_task_interval
  );
	 
	 //can1 0x1FF���ͳ�ʼ��	 
	 can1_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can1_send_0x1ff_task_interval
  );
	 
	 
	 /* �����ʼ�� */
	 	/* �������ڳ�ʼ�� */
    arm_task_p->setInterval(
    params.motor_params.control_tasks_interval.arm_task_interval
  );//��е��
	 
	  chassis_task_p->setInterval(
    params.motor_params.control_tasks_interval.chassis_task_interval
  );//����
	 
	  gimbal_task_p->setInterval(
    params.motor_params.control_tasks_interval.gimbal_task_interval
  );//����,��̨
	 
	  led_control_task_p->setInterval(
    params.motor_params.control_tasks_interval.led_task_interval
  );
	
	  referee_system_task_p->setInterval(
    params.motor_params.control_tasks_interval.referee_system_task_interval
  );
	
   	//ע������
    arm_task_p->init();
	  chassis_task_p->init();
	  gimbal_task_p->init();
	  attitude_solution_task_p->inited = true;
    attitude_solution_task_p->init();	
    led_control_task_p->init();	 
	 
	 	   // ��ʱ�������ʼ��
	   HAL_TIM_Base_Start_IT(&htim2);
      HAL_TIM_Base_Start_IT(&htim5);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//���1
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//���2
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);//���3
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//������
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);//IMU���ȵ���
	   
	 
		/* ע������ */ 
	 scheduler.registerTask(referee_system_task_p);
	 scheduler.registerTask(arm_task_p);
	 scheduler.registerTask(chassis_task_p);
	 scheduler.registerTask(gimbal_task_p);
//	 scheduler.registerTask(can1_send_task_0x32_p);
	 scheduler.registerTask(can1_send_task_0x101_p);
	 scheduler.registerTask(can1_send_task_0x102_p);
    scheduler.registerTask(can1_send_task_0x103_p);	 
//	 scheduler.registerTask(can1_send_task_0x145_p);
//	 scheduler.registerTask(can1_send_task_0x146_p);
	 scheduler.registerTask(can2_send_task_0x200_p);
	 scheduler.registerTask(can1_send_task_0x200_p);
	 scheduler.registerTask(can2_send_task_0x1ff_p);
	 scheduler.registerTask(can1_send_task_0x1ff_p);
	 	/* ע������ */
}

/***********************************************************************
** �� �� ���� Robot::run()
** ����˵���� ������ϵͳ���к������˺�����������ѭ����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Robot::run()
{
  scheduler.run();
}

void Robot::setParams()
{

}
