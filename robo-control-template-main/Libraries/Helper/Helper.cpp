/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Helper.cpp
** �ļ�˵����   ����LED��5v�ӿڡ����������������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-08
**	 1.1							   ����ע��						     ����Դ     	   2022-12-10
***************************************************************************/

#include "Helper.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim4;

/***********************************************************************
** �� �� ���� Helper::setBuzzerOff
** ����˵���� �رշ�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::setBuzzerOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}

/***********************************************************************
** �� �� ���� Helper::setBuzzerFrequence
** ����˵���� ���÷�����Ƶ��
**---------------------------------------------------------------------
** ��������� freqƵ��
** ���ز����� ��
***********************************************************************/
void Helper::setBuzzerFrequence(uint16_t freq)
{
  uint16_t period = 1000000 / freq - 1;

  __HAL_TIM_SET_AUTORELOAD(&htim4, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 20);
}

/***********************************************************************
** �� �� ���� Helper::setBuzzerState
** ����˵���� ���÷�����״̬
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
int8_t Helper::setBuzzerState(uint8_t state)
{
	//PLAYING_INIT_MUSIC���ܱ���ϣ�����ERROR
  if((state != PLAYING_ERROR_SOUND) && (buzzer_state == PLAYING_INIT_MUSIC))
  {
    return -1;
  }
  
  buzzer_state = state;
  if(state == PLAYING_STOP)
  {
    setBuzzerOff();
  }

  bzply_n = 0;
  bzply_count = 1;

  return 0;
}

/***********************************************************************
** �� �� ���� Helper::playingSound
** ����˵���� 
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::playingSound(uint8_t *sound, uint16_t len)
{
  uint8_t delay, freq;

  delay = ((sound[bzply_n] & 0xF0) >> 4) * 2;
  freq = (sound[bzply_n] & 0x0F);

  if(bzply_count < delay)
  {
    bzply_count++;
    if(freq == 0x00)
    {
      setBuzzerOff();
    }
    else if(freq == 0x0B)
    {
      setBuzzerFrequence(640);
    }
    else if(freq == 0x0D)
    {
      setBuzzerFrequence(256);
    }
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= len)
    {
      bzply_n = 0;
    }
  }
}

/***********************************************************************
** �� �� ���� Helper::playingSong
** ����˵���� ���Ÿ���
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
bool Helper::playingSong(uint16_t *song, uint16_t len)
{
  uint8_t off_delay, on_delay, level, step;

  off_delay = ((song[bzply_n] & 0xF000) >> 12);
  on_delay = ((song[bzply_n] & 0x0F00) >> 8) * 6;
  level = ((song[bzply_n] & 0x00F0) >> 4);
  step = (song[bzply_n] & 0x000F) - 1;

  if(bzply_count < on_delay)
  {
    bzply_count++;
    setBuzzerFrequence(music_steps[level][step]);
  }
  else if(bzply_count < (on_delay + off_delay))
  {
    bzply_count++;
    setBuzzerOff();
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= len)
    {
      bzply_n = 0;
      buzzer_state = PLAYING_STOP;
      setBuzzerOff();
      return true;
    }
  }
  return false;
}

/***********************************************************************
** �� �� ���� Helper::setLED
** ����˵���� ����LED����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::setLED(uint8_t R, uint8_t G, uint8_t B)
{
  if (R == 1)
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  }
  else if(R == 0)
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  }
  if (G == 1)
  {
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  }
  else if(G == 0)
  {
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  }
  if (B == 1)
  {
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  }
  else if(B == 0)
  {
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  }
}
/***********************************************************************
** �� �� ���� Helper::toggleLED
** ����˵���� ��תLED
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::toggleLED(uint8_t R, uint8_t G, uint8_t B)
{
  if (R == 1)
  {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  }
  if (G == 1)
  {
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  }
  if (B == 1)
  {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  }
}
/***********************************************************************
** �� �� ���� Helper::toggleAllLED
** ����˵���� ��ת����LED
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::toggleAllLED(void)
{
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}
/***********************************************************************
** �� �� ���� Helper::setLaser
** ����˵���� ����������
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::setLaser(void)
{
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}
/***********************************************************************
** �� �� ���� Helper::resetLaser
** ����˵���� �رռ�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::resetLaser(void)
{
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
/***********************************************************************
** �� �� ���� Helper::toggleLaser
** ����˵���� ���ؼ�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::toggleLaser(void)
{
  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
}
/***********************************************************************
** �� �� ���� Helper::setServo
** ����˵���� ���ƶ��
**---------------------------------------------------------------------
** ��������� number���id,degree��ת�Ƕ�
** ���ز����� ��
***********************************************************************/
void Helper::setServo(uint8_t number, float degree)
{
  uint16_t serp;

  switch(number)//
  {
  case(1):
	 serp = 500 + ((2000 * degree) / 135); //�ǶȻ����ռ�ձ�
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, serp);
    break;
  case(2):
	  serp = 500 + ((2000 * degree) / 270); //�ǶȻ����ռ�ձ�
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, serp);
    break;
  
    case(3):
	  serp = 500 + ((2000 * degree) / 270); //�ǶȻ����ռ�ձ�
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, serp);
    break;
  }
}
/***********************************************************************
** �� �� ���� Helper::heat
** ����˵���� PWM����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Helper::heat(uint16_t out)
{
  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 2000);
}
