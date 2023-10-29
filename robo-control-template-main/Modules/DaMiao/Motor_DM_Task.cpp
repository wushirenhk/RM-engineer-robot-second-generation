/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Motor_DM_Task.cpp
** �ļ�˵����   �����������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						       ������     	        2022-03-12
***************************************************************************/

#include  "Motor_DM_Task.h"
#include "/Robot/Robot.h"

/***********************************************************************
** �� �� ���� Motor_DM_PSControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ�����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬��������ͣ�
**            ʵ������ע�����ǶȻ����ٶȻ�PID_MotorRM_ControlTask����
**            ʵ����Motor_RM���࣬����robot0���������Backend
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������
** ���ز����� ��
***********************************************************************/
Motor_DM_PSControlTask::Motor_DM_PSControlTask(
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


  can_tx_data_start_pos = _can_tx_data_start_pos;

  motor_backend_p = new Motor_DM(robot0.motors, _can_device, _can_tx_id, _can_rx_id, _can_tx_data_start_pos, _motor_type);
  robot0.motors.addBackend(motor_backend_p);

}


/***********************************************************************
** �� �� ���� Motor_DM_PSControlTask::init()
** ����˵���� �����ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_DM_PSControlTask::init(void)
{

}

/***********************************************************************
** �� �� ���� Motor_DM_PSControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_DM_PSControlTask::uninit(void)
{

}

/***********************************************************************
** �� �� ���� Motor_DM_PSControlTask::update(timeus_t dT_us)
** ����˵���� ������£�����motor_backend_p->update()
**---------------------------------------------------------------------
** ��������� ����ʱ������us��
** ���ز����� ��
***********************************************************************/
void Motor_DM_PSControlTask::update(timeus_t dT_us)
{
  if(motor_backend_p != NULL)
  {
    motor_backend_p->update(dT_us);
  }

//  pid_motor_angular_velocity_control_task_p->calculatePID(dT_us, )
//  send();
}



