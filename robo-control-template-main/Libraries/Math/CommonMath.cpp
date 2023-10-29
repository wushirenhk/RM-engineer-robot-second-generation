/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   CommonMath.cpp
** �ļ�˵����   ��ѧ��������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-10
***************************************************************************/

#include "CommonMath.h"
#include "arm_math.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Matrix3.h"

namespace cmt
{
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float sq(float x)
{
  return x * x;
}

SimpleAdd<float> simple_adder_instance_f;
PolarAdd<float> polar_adder_instance_f;

}
