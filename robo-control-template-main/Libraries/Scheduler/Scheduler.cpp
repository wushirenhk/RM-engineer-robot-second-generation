/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Scheduler.cpp
** �ļ�˵����   ���������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-08
***************************************************************************/


#include "Scheduler.h"

Scheduler::Scheduler(void)
{

}

/***********************************************************************
** �� �� ���� Scheduler::getTaskNum()
** ����˵���� ��ȡ��ǰ��������
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��������
***********************************************************************/
uint16_t Scheduler::getTaskNum(void)
{
  return task_num;
}

/***********************************************************************
** �� �� ���� Scheduler::registerTask()
** ����˵���� �ڵ�������ע�������
**---------------------------------------------------------------------
** ��������� ����ָ��
** ���ز����� ע���Ƿ�ɹ�
***********************************************************************/
bool Scheduler::registerTask(Task_Base *task_p)
{
  uint8_t i;
  if(task_p == NULL) return false;

  /* ��ֹ�ظ�������� */
  for(i = 0; i < task_num; i++)
  {
    if(task_p == tasks[i])
    {
      return false;
    }
  }
  /* ��ֹ�ظ�������� */

  if(task_num >= MAX_TASK_NUM)
  {
    return false;
  }

  tasks[task_num++] = task_p;
  return true;
}

//  TODO ע������ unRegisterTask(Task_Base *task_p)
/***********************************************************************
** �� �� ���� Scheduler::unregisterTask()
** ����˵���� �ڵ�������ע��������
**---------------------------------------------------------------------
** ��������� ����ָ��
** ���ز����� ע���Ƿ�ɹ�
***********************************************************************/
bool Scheduler::unregisterTask(Task_Base **task_p_p)
{
  if(*task_p_p == NULL)
  {
    return false;
  }

  (*task_p_p)->uninit();

  uint16_t i = 0;
  bool found_flag = false;

  for(i = 0; i < task_num; i += 1)//��ѯ�����ע������
  {
    if(tasks[i] == *task_p_p)
    {
      delete *task_p_p;
      tasks[i] = NULL;
      *task_p_p = NULL;
      found_flag = true;
      break;
    }
  }

  if(found_flag)//ע������󣬺�������ָ��ǰ��
  {
    for(; i < task_num - 1; i ++)
    {
      tasks[i] = tasks[i + 1];
    }
    tasks[i] = NULL;
  }

  task_num --;

  return true;
}


/***********************************************************************
** �� �� ���� Scheduler::getSysTimeUs()
** ����˵���� ��ȡϵͳʱ�䣬��λus
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
timeus_t Scheduler::getSysTimeUs(void)
{
  return micros();
}

/***********************************************************************
** �� �� ���� Scheduler::init()
** ����˵���� ��������ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Scheduler::init(void)
{

}

/***********************************************************************
** �� �� ���� Scheduler::run()
** ����˵���� ���������к������˺�����������ѭ����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Scheduler::run(void)
{
  for(uint16_t i = 0; i < async_num; i ++)
	{
    timeus_t time_now_ms = micros() / 1000;
		if(asyncs[i]->run != true && asyncs[i]->register_ms > 0 && time_now_ms - asyncs[i]->register_ms > asyncs[i]->delay_ms && time_now_ms - asyncs[i]->delay_ms < 0xffffffff)
		{
			(*asyncs[i])();
			//asyncs[i]->run = true;
		}
	}
  for(uint16_t i = 0; i < task_num; i ++)
  {
    timeus_t time_now_us = micros();
    timeus_t delta_time_us = (timeus_t)(time_now_us - tasks[i]->last_tick_us);

    // �жϵ�ǰʱ������ϴ�ִ�е�ʱ���Ƿ�ﵽ�趨�ļ������ʱ��
    if((delta_time_us >= tasks[i]->interval_tick_us) && (tasks[i]->interval_tick_us))
    {
      // ��ֹdelta_time_us�����������Ҫ����ʧ��������������
      if(delta_time_us >= 0xfffffff)
      {
        tasks[i]->last_tick_us = time_now_us;
        continue;
      }
#ifdef DEBUG_MODE
      tasks[i]->update_freq = 1e6f / delta_time_us;
#endif
      // ���������ִ��ʱ��
      tasks[i]->last_tick_us = micros();

      // �ж������Ƿ��ѳ�ʼ��
      if(tasks[i]->inited)
      {
        tasks[i]->update(delta_time_us);//����ÿ�������е�updata����
      }
      else
      {
        tasks[i]->inited = true;
        tasks[i]->init();
      }
      //�ж������Ƿ���Ҫע��
      if(tasks[i]->self_kill == true)
      {
        tasks[i]->self_kill = false;
        unregisterTask(&tasks[i]);
      }
      tasks[i]->end_tick_us = micros();
      tasks[i]->real_interval_tick_us = tasks[i]->end_tick_us - time_now_us;
    }

  }
}
