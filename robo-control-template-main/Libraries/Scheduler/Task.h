#ifndef __TASK_H__
#define __TASK_H__

#include "stdint.h"
#include "Scheduler_Common.h"

class Robot;
// ������࣬���е�������̳��Դ���
class Task_Base//������
{
  // ���õ�������Ϊ��Ԫ�࣬ʹ����������Է����Լ��ķǹ�����Ա
  friend class Scheduler;

public:
  Task_Base(Robot &robot0) : robot(robot0)//���캯���������ò���&robot0��ֵ������robot
  {
  }

  virtual ~Task_Base(void) {};

  // ���麯�����������ʵ��
  virtual void update(timeus_t dT_us) = 0; // ���interval_tick_usʱ�������ѯ���������dT_usΪʵ�����м��ʱ��
  virtual void init(void) = 0; // �����ʼ��ʱִ�д˺���
  virtual void uninit(void) = 0; // ��������ʱִ�д˺���
  
		//�Ƿ��Ѿ���ʼ��
		bool isInited(void)
  {
    return inited;
  }
	
	//�Ƿ���ɱ
  bool isKilledBySelf(void)
  {
    return self_kill;
  }
	//��ɱ
  void selfKill(void)
  {
    self_kill = true;
  }
	//����ʱ����
  void setInterval(timeus_t interval)
  {
    interval_tick_us = interval;
  }
	//����ʱ����
  timeus_t getInterval(void)
  {
    return interval_tick_us;
  }
//  void setInitState(bool state)
//  {
//    inited = state;
//  }
protected:
  bool inited;
#ifdef DEBUG_MODE
  float update_freq;
#endif
  timeus_t interval_tick_us; // �������ʱ�䣬�����ó�0���򲻻���Scheduler�����и�����
  timeus_t real_interval_tick_us; // ��ʵ�����м��
  timeus_t last_tick_us;	// �ϴ����е�ʱ���
  timeus_t end_tick_us; // �˴����н���ʱ���
  Robot &robot; // �����Ļ����˶���
  bool self_kill;
};

#endif
