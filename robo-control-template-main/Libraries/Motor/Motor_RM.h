#ifndef __MOTOR_RM_H__
#define __MOTOR_RM_H__

#include "stdint.h"
#include "Motor_Backend.h"
#include "Vector3.h"
#include "main.h"
#include "CanDevice.h"
//#include "Task.h"
#include "PID_Controller.h"
//#include "Scheduler.h"


class Robot;

//typedef struct _CAN_Data_RM_Motor_t
//{
//  CAN_Rx_Data_Pack_t data;
//  uint8_t bytes_group; //0 = [0:1], 1 = [2:3], 2 = [4:5], 3 = [6:7]
//} CAN_Data_RM_Motor_t;

// RM�������
typedef struct _Motor_RM_Params_t
{
  float reduction_ratio; // ���ٱ� = ת��ת��/�����ת��
  float output_radius; // ��������������ͬ���ְ뾶��m�������ڼ������ٶ�
  int8_t direction; // ������ʵ��������ͬ��Ϊ1������Ϊ-1
  int16_t max_value_ecd; // ���������������ֵ
  int16_t half_max_value_ecd; // ���������������ֵ/2
  int16_t offset_ecd; // ���������ƫ��ֵ��ԭʼ��ֵ��
  uint16_t can_tx_id; // CAN���߷��͸�����õ�ID
  uint16_t can_rx_id; // CAN���ߵ�����͸��������õ�ID
  uint8_t can_tx_data_start_pos; // CAN���߷���������ʼ�±�
  uint8_t canx; // CAN���߱��
  float interval; // ���������ݽ���Ƶ��
} Motor_RM_Params_t;

// RM�������ֵ
typedef struct _Motor_RM_Measurement_t
{
  // ʱ���
  timeus_t timestamp; // ����ʱ���

  // ԭʼ�������
  int16_t ecd; // ��ǰ��������ֵ
  int16_t speed_rpm; // ��ǰת�٣�rpm��
  int16_t given_current; // ����
  uint8_t temperate; // �¶�
  int16_t last_ecd; // �ϴα�������ֵ

  // ��������ĵ������
  int16_t delta_ecd; // ����������
  int32_t round_cnt; // Ȧ��
  int32_t last_round_cnt; // �ϴ�Ȧ��
  int32_t delta_round_cnt; // Ȧ������
  int32_t total_ecd; // �ܼƵı�������ֵ�����ܻ������

} Motor_RM_Measurement_t;


class Motor_RM : public Motor_Backend
{
public:
  friend class Motor_Backend;
  friend class GimbalControlTask;
  friend class Arm_ControlTask;

  Motor_RM(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);



  // ͨ���������������������Ƕ�ֵ
  void updateMotorMeasurement();
//  // �������Ƕ�ֵ
//  void updateMotorAngle(Motor_RM_Measurement_t *msr_p,
//                        Motor_RM_Params_t *params_p,
//                        Motor_Common_Measurement_t *com_p);

  //virtual void process(void); // �������������
  virtual bool update(timeus_t dT_us); // ���µ�����
  virtual void init(void); // ��ʼ������
  virtual void uninit(void); // ����ʼ��

  virtual void updateMeasurement(void);

  void setParams(Motor_RM_Params_t _params)
  {
    params = _params;
    params.half_max_value_ecd = params.max_value_ecd / 2;
  }
  Motor_RM_Measurement_t getMeasurement()
  {
    return msr;
  }
  void setMotorInput(int16_t input)
  {
    motor_input = input * params.direction;
    syncCANData();
  }
    
  void setAngularVelocityIMU(float ang_v)
  {
    com_msr.rotator.angular_velocity_imu = ang_v;
  }
    
// virtual void updateMovement(void);

//  virtual void processMeasurement(timeus_t dT_us);

//  bool registerPIDTask(Scheduler &scheduler0){
//    bool flg;
//    flg = scheduler0.registerTask(pid_angle_task_p);
//    flg &= scheduler0.registerTask(pid_velocity_task_p);
//    return flg;
//  }

protected:
    
  void syncCANData()
  {
    can_tx_data[0] = motor_input >> 8;
    can_tx_data[1] = motor_input;
  }

  Motor_RM_Measurement_t msr;
  Motor_RM_Params_t params;

  uint8_t can_tx_data[2];
  int16_t motor_input;
  uint8_t can_tx_data_start_pos;

  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;
};

#endif
