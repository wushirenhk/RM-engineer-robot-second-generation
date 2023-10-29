/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   Engineer
** �� �� ����   Motor_DM.cpp
** �ļ�˵����   DM������ݶ�ȡ����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						       ������     	     2022-03-14
***************************************************************************/

#include "Motor_M15.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"

/***********************************************************************
** �� �� ���� Motor_M15::Motor_M15()
** ����˵���� ������ĵ�������CAN�Ľ����뷢�����ݵ�ַ��ӵ�can_device��
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
Motor_M15::Motor_M15(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 2);
  motor_type = _motor_type;
}

/***********************************************************************
** �� �� ���� updateMotorMeasurement()0
** ����˵���� ��CAN���ݸ��µ��������Ϣ
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
/***********************************************************************
** �� �� ���� updateMotorMeasurement()0
** ����˵���� ��CAN���ݸ��µ��������Ϣ
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
void Motor_M15::updateMotorMeasurement()
{
  static bool first_update = 1;
  // ʱ���
  com_msr.timestamp = msr.timestamp = micros();
 // CAN����ת��Ϊ�����������
 msr.speed_rpm = (uint16_t)((can_rx_data.data)[0] << 8 | (can_rx_data.data)[1]);
 msr.given_current = (uint16_t)((can_rx_data.data)[2] << 8 | (can_rx_data.data)[3]);
 msr.ecd = (uint16_t)((can_rx_data.data)[4] << 8 | (can_rx_data.data)[5]);
 msr.err = (uint8_t)(can_rx_data.data)[6];
 msr.mode = (uint8_t)(can_rx_data.data)[7];

}


void Motor_M15::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_M15::init(void)
{

}

bool Motor_M15::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_M15::uninit(void) {}




