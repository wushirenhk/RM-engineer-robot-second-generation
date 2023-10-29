/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   MotorMG_ControlTask.cpp
** �ļ�˵����   �����������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						       �����     	     2022-12-22
**  1.1                                                      ������             
***************************************************************************/

#include "Motor_MG_Tasks.h"
#include "/Robot/Robot.h"
#include "Motor_MG.h"

/***********************************************************************
** �� �� ���� Motor_MG_PIDControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ�����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬��������ͣ�
**            ʵ������ע�����ǶȻ����ٶȻ�PID_Motor_MG_ControlTask����
**            ʵ����Motor_MG���࣬����robot0���������Backend
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������
** ���ز����� ��
***********************************************************************/
Motor_MG_ControlTask::Motor_MG_ControlTask(
  Robot &robot0,
  CanDevice &_can_device,
  uint16_t _can_rx_id, uint16_t _can_tx_id,
  uint8_t  _can_tx_data_start_pos,
  int8_t   _motor_type,
  timeus_t interval_tick_us0
)
  : Task_Base(robot0),can_device(_can_device)
{

  this->interval_tick_us = interval_tick_us0; // �������������Ƶ��
  can_tx_data_start_pos = _can_tx_data_start_pos;
  motor_backend_p = new Motor_MG(robot0.motors, _can_device, _can_tx_id, _can_rx_id, _can_tx_data_start_pos, _motor_type);
  robot0.motors.addBackend(motor_backend_p);
}




/***********************************************************************
** �� �� ���� Motor_MC_ControlTask::init()
** ����˵���� �����ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_MG_ControlTask::init(void)
{
//		motor_backend_p->cmdMotorStop();
//	   motor_backend_p->cmdMotorShutOff();
}

/***********************************************************************
** �� �� ���� Motor_MC_ControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Motor_MG_ControlTask::uninit(void)
{

}




/***********************************************************************
** �� �� ���� Motor_MC_ControlTask::update(timeus_t dT_us)
** ����˵���� ������£�����motor_backend_p->update()
**---------------------------------------------------------------------
** ��������� ����ʱ������us��
** ���ز����� ��
***********************************************************************/
void Motor_MG_ControlTask::update(timeus_t dT_us)
{
  if(motor_backend_p != NULL)
  {
    motor_backend_p->update(dT_us);
  }

  
//  //�޸�PIDֵ
//  //д��RAM���粻����
//    if(motor_backend_p->PIDchange_flag == 1)
//  {
//     motor_backend_p->setPIDParams_RAM(motor_backend_p->pid_params);
//  }
//    if(motor_backend_p->can_rx_data.data[0] == 0x31)
//    {
//	  motor_backend_p->MG_motor_reply();
//	  motor_backend_p->PIDchange_flag = 0;
//    }
//  
//  //д��ROM���籣��
//  if(motor_backend_p->PIDchange_flag == 2)
//  {
//     motor_backend_p->setPIDParams_ROM(motor_backend_p->pid_params);
//  }
//    if(motor_backend_p->can_rx_data.data[0] == 0x32)
//    {
//	  motor_backend_p->MG_motor_reply();
//	  motor_backend_p->PIDchange_flag = 0;
//    }
  //��ȡ��Ȧ����������
  	 //�رյ�������ͬʱ����������״̬��֮ǰ���յĿ���ָ��(0x80)
	 
	 
//����Ҫ��
   if(motor_backend_p->can_rx_data.data[0] == 0x80)
   {
      motor_backend_p->MG_motor_reply();
   }
	
	 if(motor_backend_p->can_rx_data.data[0] == 0x81)
   {
      motor_backend_p->MG_motor_reply();
   }
	
      motor_backend_p->getMultiLoopAngle();
      if(motor_backend_p->can_rx_data.data[0] == 0x92)
  {
      motor_backend_p->MG_motor_reply();
  }
	   motor_backend_p->getSingleLoopAngle();
	if(motor_backend_p->can_rx_data.data[0] == 0x94)
  {
		motor_backend_p->MG_motor_reply();
  }
  
 		if(motor_backend_p->can_update_flag == 0)
		{
      motor_backend_p->cmdAngleControl(motor_backend_p->params.max_speed,motor_backend_p->params.angle_control);
		motor_backend_p->angle_update_flag = 0;	
		} 
	  if(motor_backend_p->can_rx_data.data[0] == 0xA4)
   {
      motor_backend_p->MG_motor_reply();
    }
	
	 
	
}
/***********************************************************************
** �� �� ���� Motor_MG_ControlTask::setHomePosition() 
** ����˵���� �ϵ縴λ
**---------------------------------------------------------------------
** ��������� ����ʱ������us��
** ���ز����� ��
***********************************************************************/
void Motor_MG_ControlTask::setHomePosition()
{
	motor_backend_p->params.angle_control = 0;
}
