/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Motor_RM.cpp
** �ļ�˵����   RoboMaster������ݶ�ȡ����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "Motor_RM.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"

/***********************************************************************
** �� �� ���� Motor_RM::Motor_RM()
** ����˵���� ������ĵ�������CAN�Ľ����뷢�����ݵ�ַ��ӵ�can_device��
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
Motor_RM::Motor_RM(Motor &motor0,
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
void Motor_RM::updateMotorMeasurement()
{
  static bool first_update = 1;


  // ʱ���
  com_msr.timestamp = msr.timestamp = micros();

  // CAN����ת��Ϊ�����������
  msr.ecd = (uint16_t)((can_rx_data.data)[0] << 8 | (can_rx_data.data)[1]);
  msr.speed_rpm = (uint16_t)((can_rx_data.data)[2] << 8 | (can_rx_data.data)[3]);
  msr.given_current = (uint16_t)((can_rx_data.data)[4] << 8 | (can_rx_data.data)[5]);
  msr.temperate = (can_rx_data.data)[6];

  // ��ӳ�������ֵ
  if(msr.ecd >= 0 && msr.ecd < params.offset_ecd + params.half_max_value_ecd)
    msr.ecd = msr.ecd - params.offset_ecd;
  else if(msr.ecd >= params.offset_ecd + params.half_max_value_ecd && msr.ecd < params.max_value_ecd)
    msr.ecd = msr.ecd - params.max_value_ecd - params.offset_ecd;

  // �������������ֵ
  msr.delta_ecd = msr.ecd - msr.last_ecd;
  
  // ��������ֵ���䴦��
  if (msr.delta_ecd < -params.half_max_value_ecd)
  {
    msr.round_cnt++;
    msr.delta_ecd += params.max_value_ecd;
	 msr.total_ecd = msr.total_ecd + 8192;
  }
  else if (msr.delta_ecd > params.half_max_value_ecd)
  {
    msr.round_cnt--;
    msr.delta_ecd -= params.max_value_ecd;
	 msr.total_ecd = msr.total_ecd - 8192;
  }
  
  // ����Ȧ������ֵ
  msr.delta_round_cnt = msr.round_cnt - msr.last_round_cnt;
  
  // ת�ӽ�λ��
  com_msr.rotator.angle = 2 * params.direction * PI *
                          (msr.round_cnt +
                           1.0f * msr.ecd / params.max_value_ecd);



  

//  /*  TODO: ת��ת�٣�����ֱ����speed_rpm���㣬����̫�
  com_msr.rotator.angular_velocity = msr.speed_rpm / 60.0f * 2 * PI * params.direction;
//  static float delta_T_us;
//  static float last_T_us;
//  static float last_displacement;

//  if((msr.round_cnt != msr.last_round_cnt) || (msr.ecd != msr.last_ecd))
//  {
//    delta_T_us = com_msr.timestamp - last_T_us;
//    com_msr.rotator.angular_velocity = (com_msr.rotator.angle - last_displacement)*1e6f/delta_T_us;

//    last_displacement = com_msr.rotator.angle;
//    last_T_us = com_msr.timestamp;
//  }
//  */

  // ������ԭʼ����תΪ�����������
  com_msr.rotator.polar_angle = msr.ecd * PI * params.direction / params.half_max_value_ecd;

  /*  TODO: ���������ļ���
  */
  com_msr.output.angle = com_msr.rotator.angle / params.reduction_ratio;
  com_msr.output.angular_velocity = com_msr.rotator.angular_velocity / params.reduction_ratio;
  com_msr.linear.velocity = com_msr.output.angular_velocity * params.output_radius;
  com_msr.linear.displacement = com_msr.output.angle * params.output_radius;
  com_msr.linear.relative_displacement = com_msr.linear.displacement - com_msr.linear.offset;
  
  // �����ϴε�ֵ
  msr.last_ecd = msr.ecd;
  msr.last_round_cnt = msr.round_cnt;
}

//void Motor_RM::updateMotorAngle(Motor_RM_Measurement_t *msr_p,
//                                Motor_RM_Params_t *params,
//                                Motor_Common_Measurement_t *com_p)
//{


////  int16_t tmp = msr.ecd - params.offset_ecd;
////  if(tmp >= - params.offset_ecd && tmp < params.half_max_value_ecd)
////    com_msr.rotator.polar_angle = tmp * PI / params.half_max_value_ecd;
////  else if(tmp >= params.half_max_value_ecd && tmp < - params.offset_ecd)
////    com_msr.rotator.polar_angle = (params.max_value_ecd - tmp) * PI / params.half_max_value_ecd;

//  // ������ԭʼ����תΪ�����λ������
////  tmp += params.max_value_ecd;
////  com_msr.rotator.polar_angle = ((tmp >= params.max_value_ecd/2) ? (tmp - params.max_value_ecd) : tmp) / (params.max_value_ecd*1.0f) * 2 * PI * params.direction;

//  //ptr->spd = ptr_ex->status->speed_rpm * ptr->dir / 60.0f * 2 * PI;
//  //ptr->spd = UpdateGimbalAng_Freq * ptr_ex->delt_angle / 8192.0f * 2 * PI * ptr->dir;
//}

void Motor_RM::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_RM::init(void)
{
  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_RM::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_RM::uninit(void) {}
