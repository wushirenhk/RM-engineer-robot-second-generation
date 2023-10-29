#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "Motor_Backend.h"
#include "CanDevice.h"
#include "PID_Controller.h"

class Robot;

//DM�������
typedef struct _Motor_DM_Params_t
{
	  float reduction_ratio; // ���ٱ� = ת��ת��/�����ת��
     float output_radius; // ��������������ͬ���ְ뾶��m�������ڼ������ٶ�
	  int8_t direction; // ������ʵ��������ͬ��Ϊ1������Ϊ-1
	  int16_t max_value_ecd; // ���������������ֵ
	  int16_t half_max_value_ecd; // ���������������ֵ
	  uint16_t can_tx_id; // CAN���߷��͸�����õ�ID
     uint16_t can_rx_id; // CAN���ߵ�����͸��������õ�ID
     uint8_t can_tx_data_start_pos; // CAN���߷���������ʼ�±�
	  uint8_t canx; // CAN���߱��
	
	  float position_rad;
	  float speed_rad;
    float interval; // ���������ݽ���Ƶ��
}Motor_DM_Params_t;

//DM�������ֵ
typedef struct _Motor_DM_Measurement_t
{
  // ʱ���
  timeus_t timestamp; // ����ʱ���
	  // ԭʼ�������
  uint8_t CAN_ID = 0;
  uint8_t ERR;
  uint16_t ecd_int; // ��ǰ��������ֵ 
  uint16_t speed_rad_int; // ��ǰת�� 
  uint16_t torque_int; // ת��
  float ecd;//�����λ�ã���λrad
  float speed_rad;//�����ת�٣���λrad/s 
  float torque;
  float t_mos;
  float t_rotor;
  int16_t last_ecd; // �ϴα�������ֵ

  // ��������ĵ������
  int16_t delta_ecd; // ����������
  int32_t round_cnt; // Ȧ��
  int32_t last_round_cnt; // �ϴ�Ȧ��
  int32_t delta_round_cnt; // Ȧ������
  // int32_t total_ecd; // �ܼƵı�������ֵ�����ܻ������
	
}Motor_DM_Measurement_t;


class Motor_DM : public Motor_Backend
{
public:	
  friend class Motor_Backend;
  friend class Arm_ControlTask;
	friend class RemoteControlTask;
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -10.0f
 #define T_MAX 10.0f

  Motor_DM(Motor &motor0,
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


  void setParams(Motor_DM_Params_t _params)
  {
    params = _params;
    params.half_max_value_ecd = params.max_value_ecd / 2;
  }
  
//  void setMotorInput(int16_t input)
//  {
//    motor_input = input * params.direction;
//  }	
  
  void setMotorPositon(float position,float speed)
{
	 uint8_t *pbuf,*vbuf;
  	 pbuf=(uint8_t*)&position;
	 vbuf=(uint8_t*)&speed;
	 can_tx_data[0] = *pbuf;
	 can_tx_data[1] = *(pbuf+1);
	 can_tx_data[2] = *(pbuf+2);
	 can_tx_data[3] = *(pbuf+3);
	 can_tx_data[4] = *vbuf;
	 can_tx_data[5] = *(vbuf+1);
	 can_tx_data[6] = *(vbuf+2);
	 can_tx_data[7] = *(vbuf+3);
	 
    start_flag++;
	 if(start_flag >= 200)
   {
	  start_motor();
	  start_flag = 0;
   }

}
  
 void start_motor()
{ 
	 can_tx_data[0] = 0xFF;
	 can_tx_data[1] = 0xFF;
	 can_tx_data[2] = 0xFF;
	 can_tx_data[3] = 0xFF;
	 can_tx_data[4] = 0xFF;
	 can_tx_data[5] = 0xFF;
	 can_tx_data[6] = 0xFF;
	 can_tx_data[7] = 0xFC;  
	  
}
  
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
	//������ת��Ϊ������
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
	//������ת��Ϊ������
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
  
protected:
  
  Motor_DM_Measurement_t msr;
  Motor_DM_Params_t params;//���������ֵ
  
  uint8_t can_tx_data[8];
//  uint32_t motor_position_int;
//  uint32_t motor_speed_int;
//  float motor_position;
//  float motor_speed;
  uint8_t can_tx_data_start_pos;
  uint16_t start_flag=0;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;
  
};
























#endif
