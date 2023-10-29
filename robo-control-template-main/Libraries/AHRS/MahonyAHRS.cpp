/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   MahonyAHRS.cpp
** �ļ�˵����   ��̬����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-18
**   1.1              �޸��㷨�·�����           ���Ӻ�          2022-08-01
**	 1.2							   ����ע��						     ����Դ     	   2022-12-10
***************************************************************************/

#include "MahonyAHRS.h"
#include "Quaternion.h"
#include "UARTDriver.h"
#include "arm_math.h"
#include "CmsisCompatibility.h"

/***********************************************************************
** �� �� ���� MahonyAHRS::MahonyAHRS()
** ����˵���� ���캯��
**---------------------------------------------------------------------
** ��������� ϵ��two_kp0��two_ki0
** ���ز����� ��
***********************************************************************/
MahonyAHRS::MahonyAHRS(float two_kp0, float two_ki0):sample_freq(SAMPLE_FREQ_DEF)
{
  two_ki = two_ki0;
  two_kp = two_kp0;

  integral_fb.zero();

  imu_quat.w = 1.0f;
  imu_quat.x = .0f;
  imu_quat.y = .0f;
  imu_quat.z = .0f;
  
}

void MahonyAHRS::init(void)
{

}
/***********************************************************************
** �� �� ���� MahonyAHRS::update()
** ����˵���� Mahony�㷨�������Ԫ��
**---------------------------------------------------------------------
** ��������� ������ٶȣ�������ٶ�
** ���ز����� ��
***********************************************************************/
void MahonyAHRS::update(Vector3f accel, Vector3f gyro)
{
  float recipNorm;
  Vector3f halfv, halfe, q_temp;
	// accel Ϊ��������ʱ���ɽ���
  if(!accel.isZero())
  {
    accel.normalize(); // ��׼��
		
    // ������������ʹ�ֱ�ڴ�ͨ����ʸ��
    halfv.x = imu_quat.x * imu_quat.z - imu_quat.w * imu_quat.y;
    halfv.y = imu_quat.w * imu_quat.x + imu_quat.y * imu_quat.z;
    halfv.z = imu_quat.w * imu_quat.w - 0.5f + imu_quat.z * imu_quat.z;

		// ����ǹ��Ƶ�������������������������������֮��
    halfe.x = accel.y * halfv.z - accel.z * halfv.y;
    halfe.y = accel.z * halfv.x - accel.x * halfv.z;
    halfe.z = accel.x * halfv.y - accel.y * halfv.x;

    if(two_ki > 0.0f)
    {
      integral_fb.x += two_ki * halfe.x * (1.0f / sample_freq);
      integral_fb.y += two_ki * halfe.y * (1.0f / sample_freq);
      integral_fb.z += two_ki * halfe.z * (1.0f / sample_freq);
      gyro.x += integral_fb.x;
      gyro.y += integral_fb.y;
      gyro.z += integral_fb.z;
    }
    else
    {
      integral_fb.zero();
    }
    gyro.x += two_kp * halfe.x;
    gyro.y += two_kp * halfe.y;
    gyro.z += two_kp * halfe.z;
  }

  gyro.x *= (0.5f * (1.0f / sample_freq));
  gyro.y *= (0.5f * (1.0f / sample_freq));
  gyro.z *= (0.5f * (1.0f / sample_freq));

  q_temp.x = imu_quat.w;
  q_temp.y = imu_quat.x;
  q_temp.z = imu_quat.y;

  imu_quat.w += (-q_temp.y * gyro.x - q_temp.z * gyro.y - imu_quat.z * gyro.z);
  imu_quat.x += ( q_temp.x * gyro.x + q_temp.z * gyro.z - imu_quat.z * gyro.y);
  imu_quat.y += ( q_temp.x * gyro.y - q_temp.y * gyro.z + imu_quat.z * gyro.x);
  imu_quat.z += ( q_temp.x * gyro.z + q_temp.y * gyro.y - q_temp.z * gyro.x);

  imu_quat.normalize();
  
  // pitch
  imu_euler.x = asinf(-2.0f * (imu_quat.x * imu_quat.z - imu_quat.w * imu_quat.y));
  
  // roll
  imu_euler.y = atan2f(2.0f * (imu_quat.w * imu_quat.x + imu_quat.y * imu_quat.z), 2.0f * (imu_quat.w * imu_quat.w + imu_quat.z * imu_quat.z) - 1.0f); 
  
  // yaw
  imu_euler.z = atan2f(2.0f * (imu_quat.w * imu_quat.z + imu_quat.x * imu_quat.y), 2.0f * (imu_quat.w * imu_quat.w + imu_quat.x * imu_quat.x) - 1.0f); 

  imu_coordinate_system = imu_quat.toRotationMatrix();

}
