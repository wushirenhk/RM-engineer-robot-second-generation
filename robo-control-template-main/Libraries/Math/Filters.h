#ifndef _FILTERS_H_
#define _FILTERS_H_

#include "stdint.h"
#include "CommonMath.h"

// ��ͨ�˲�
template<class T> // ģ��ͷ��template�ؼ��ָ��߱�������ʼ���ͱ��
class LowPassFilter
{
public:
  LowPassFilter<T>() {}
  LowPassFilter<T>(float _dT_us, float _cut_off_freq);
	
	// һ�����ֵ�ͨ�˲�����
  T calc(float _dT_us, T _xn)
  {
    dT_us = _dT_us;
    if(cmt::isZero(dT_us))
      return yn * 0;

    xn = _xn;
		
    updateCoef(); // �����˲�ϵ��
    limitCoef(); // ����coef��С
		
    update(); // ���ֵ�ͨ�˲�����

    return yn;
  }
	
	// �����˲����yn
  T getOutput(void)
  {
    return yn;
  }
	
	// ���ò���Ƶ��
  void setSamplingCycle(float t)
  {
    dT_us = t;
  }
	
	// �����˲�����Ƶ��
  void setCutOffFreq(float freq)
  {
    cut_off_freq = freq;
  }

protected:
  T xn; // ��n�β���������
  T yn; // ��n���˲������ֵ
  T yn_1; // ��n-1���˲������ֵ
  float coef; // �˲�ϵ��
  float cut_off_freq; // �˲�����ֹƵ��
  float dT_us; // �������ڣ�us��

	// ����coef��С
  void limitCoef()
  {
    coef = limit(coef, 0.0f, 1.0f);
  }
	
	// ���������˲�ϵ����ȡ���ڽ�ֹƵ��
  void updateCoef()
  {
    coef = cut_off_freq * 2 * PI * dT_us / 1e6f; // ����ϵ��=��������/ʱ�䳣��=�������� * ��ֹ��Ƶ��
  }
  void update();
};


#endif
