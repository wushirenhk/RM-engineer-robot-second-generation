/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   MotorRM_ControlTask.cpp
** �ļ�˵����   �����������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "Motor_RM_Tasks.h"
#include "/Robot/Robot.h"


/***********************************************************************
** �� �� ���� Motor_RM_PIDControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ�����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬��������ͣ�
**            ʵ������ע�����ǶȻ����ٶȻ�PID_MotorRM_ControlTask����
**            ʵ����Motor_RM���࣬����robot0���������Backend
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������
** ���ز����� ��
***********************************************************************/
Motor_RM_PIDControlTask::Motor_RM_PIDControlTask(
  Robot &robot0,
  CanDevice &_can_device,
  uint16_t _can_rx_id, uint16_t _can_tx_id,
  uint8_t _can_tx_data_start_pos,
  int8_t _motor_type,
  timeus_t interval_tick_us0
)
  : Task_Base(robot0), can_device(_can_device)
{

  this->interval_tick_us = interval_tick_us0; // �������������Ƶ��

  angle_control_task_p =
    new PIDControlTask(robot, 0);
  angular_velocity_control_task_p =
    new PIDControlTask(robot, 0);

//  robot.scheduler.registerTask(angle_control_task_p);
//  robot.scheduler.registerTask(angular_velocity_control_task_p);

  can_tx_data_start_pos = _can_tx_data_start_pos;

  motor_backend_p = new Motor_RM(robot0.motors, _can_device, _can_tx_id, _can_rx_id, _can_tx_data_start_pos, _motor_type);
  robot0.motors.addBackend(motor_backend_p);

}

/***********************************************************************
** �� �� ���� Motor_RM_PIDControlTask::init()
** ����˵���� �����ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_RM_PIDControlTask::init(void)
{

}

/***********************************************************************
** �� �� ���� Motor_RM_PIDControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_RM_PIDControlTask::uninit(void)
{

}




/***********************************************************************
** �� �� ���� PIDControlTask::update(timeus_t dT_us)
** ����˵���� ������£�����motor_backend_p->update()
**---------------------------------------------------------------------
** ��������� ����ʱ������us��
** ���ز����� ��
***********************************************************************/
void Motor_RM_PIDControlTask::update(timeus_t dT_us)
{
  if(motor_backend_p != NULL)
  {
    motor_backend_p->update(dT_us);
  }

//  pid_motor_angular_velocity_control_task_p->calculatePID(dT_us, )
//  send();
}
