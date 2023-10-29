#ifndef __MOTOR_M15_H__
#define __MOTOR_M15_H__

#include "Motor_Backend.h"
#include "CanDevice.h"


//DM�������
typedef struct _Motor_M15_Params_t
{
	  uint16_t can_tx_id; // CAN���߷��͸�����õ�ID
     uint16_t can_rx_id; // CAN���ߵ�����͸��������õ�ID
     uint8_t can_tx_data_start_pos; // CAN���߷���������ʼ�±�
	  uint8_t canx; // CAN���߱��
	
	  uint16_t set_position;
	  float interval; // ���������ݽ���Ƶ��
	
}Motor_M15_Params_t;

typedef struct _Motor_M15_Measurement_t
{
  // ʱ���
  timeus_t timestamp; // ����ʱ���
  
	int16_t given_current;
	int16_t speed_rpm;
   int16_t ecd;
   uint8_t err;
	uint8_t mode;

}Motor_M15_Measurement_t;

class Motor_M15 : public Motor_Backend
{
public:	
  friend class Motor_Backend;
  friend class Arm_ControlTask;

  Motor_M15(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);

  // ͨ���������������������Ƕ�ֵ
  void updateMotorMeasurement();
  virtual bool update(timeus_t dT_us); // ���µ�����
  virtual void init(void); // ��ʼ������
  virtual void uninit(void); // ����ʼ��

  virtual void updateMeasurement(void);




protected:

  void syncCANData()
  {
    can_tx_data[0] = set_positon >> 8;
    can_tx_data[1] = set_positon;
  }

  Motor_M15_Measurement_t msr;
  Motor_M15_Params_t params;//���������ֵ
  uint8_t can_tx_data[2];
  uint16_t set_positon;

  uint8_t can_tx_data_start_pos;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;

};







#endif
