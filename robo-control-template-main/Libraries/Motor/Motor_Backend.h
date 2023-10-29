#ifndef __MOTOR_BACKEND_H__
#define __MOTOR_BACKEND_H__

#include "stdint.h"
#include "Motor.h"
#include "Motor_Data.h"

class Motor_Backend
{
public:
  friend class Motor;
  friend class Motor_RM;
  Motor_Backend(Motor &motor0);
  virtual void init(void) = 0; // ��ʼ������
  virtual bool update(timeus_t dT_us) = 0;
  virtual void uninit(void) = 0; // ��ʼ������

  // ��ȡ���ͨ�ò�������
  Motor_Common_Measurement_t getCommonMeasurement(){
    return com_msr;
  }
  
  // ���õ�����˶���׼ֵ
  void setLinearOffset(float offset)
  {
    this->com_msr.linear.offset = offset;
  }
  
//  Motor_Backend(Motor &motor0,
//                uint16_t _id,
//                uint16_t _CAN_Ctrl_Id,
//                uint16_t _CAN_Feedback_Id,
//                float _Reduction_Ratio,
//                int8_t _Direction
//               );
protected:
  Motor &motor;
  virtual void updateMeasurement(void) = 0; // ��ȡԭʼ����
  // virtual void processMeasurement(timeus_t dT_us) = 0; // ��һ����������
  virtual void publishMeasurement(void);
  uint8_t motor_type;
  uint16_t id;
  uint16_t getId()
  {
    return id;
  }
  Motor_Common_Measurement_t com_msr;

};

#endif