/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   SongPlayerTask.cpp
** �ļ�˵����   ����������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "SongPlayerTask.h"

#include "/Robot/Robot.h"

/***********************************************************************
** �� �� ���� SongPlayerTask::init
** ����˵���� ��ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void SongPlayerTask::init(void)
{
  bzply_n = 0;
  bzply_count = 1;
  song_end = false;
  buzzer_state = PLAYING_STOP;

  robot.helper.setBuzzerOff();
}
/***********************************************************************
** �� �� ���� SongPlayerTask::update
** ����˵���� ���������ע������
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void SongPlayerTask::update(timeus_t dT_us)
{
  song_end = playSong();
  if(song_end)
  {
    selfKill();
  }
}
/***********************************************************************
** �� �� ���� SongPlayerTask::playSong
** ����˵���� ���Ÿ���
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ���ųɹ�����true������false
***********************************************************************/
bool SongPlayerTask::playSong(void)
{
  uint8_t off_delay, on_delay, level, step;

  off_delay = ((song[bzply_n] & 0xF000) >> 12);
  on_delay = ((song[bzply_n] & 0x0F00) >> 8) * 6;
  level = ((song[bzply_n] & 0x00F0) >> 4);
  step = (song[bzply_n] & 0x000F) - 1;

  if(bzply_count < on_delay)
  {
    bzply_count++;
    robot.helper.setBuzzerFrequence(music_steps[level][step]);//��������������
  }
  else if(bzply_count < (on_delay + off_delay))
  {
    bzply_count++;
    robot.helper.setBuzzerOff();
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= song_length)
    {
      return true;
    }
  }
  return false;
}

void SongPlayerTask::uninit(void)
{

}
