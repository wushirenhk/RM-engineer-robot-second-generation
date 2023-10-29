#ifndef __MOTOR_MG_H__
#define __MOTOR_MG_H__

#include "stdint.h"
#include "Motor_Backend.h"
#include "Vector3.h"
#include "main.h"
#include "CanDevice.h"
//#include "Task.h"
#include "PID_Controller.h"
//#include "Scheduler.h"


class Robot;

//typedef struct _CAN_Data_MC_Motor_t
//{
//  CAN_Rx_Data_Pack_t data;
//  uint8_t bytes_group; //0 = [0:1], 1 = [2:3], 2 = [4:5], 3 = [6:7]
//} CAN_Data_MC_Motor_t;

enum MGState
{
  MGSTATE_INITING = 0,
  MGSTATE_NORMAL =  1,
  MGSTATE_LOCK =    2,
  MGSTATE_DISABLE = 3
};

typedef struct _Motor_MG_PID_Params_t
{
	uint8_t PosKp_change;
	uint8_t PosKi_change;
	uint8_t SpdKp_change;
	uint8_t SpdKi_change;
	uint8_t TqKp_change;
	uint8_t TqKi_change;

	uint8_t PosKp_now;
	uint8_t PosKi_now;
	uint8_t SpdKp_now;
	uint8_t SpdKi_now;
	uint8_t TqKp_now;
	uint8_t TqKi_now;
}Motor_MG_PID_Params_t;


typedef struct _Motor_MG_Params_t
{
	/***MG�������***/
	/***��***/
	int16_t ecd_offset;		//�����������Ȧֵƫ��ֵ������ȷ�����
	int16_t tq_control;		//ת�ص������ ��λ��0.01A/LSB��
	int32_t speed_control;	//�ٶȱջ�  ���������ٶ� ��λ��0.01dps/LSB��
	int32_t angle_control;  //����λ�ñջ����� ��λ��0.01degree/LSB��
	int32_t delta_angle_control;//����λ�ñջ������Ƕ� ��λ��0.01degree/LSB��
	uint16_t max_speed;      //�ٶ�����
	
	uint16_t singleturn_angle; //��Ȧ�Ƕ� ��λ��0.01degree/LSB��
	uint16_t multiturn_position;//��Ȧ������λ�ã���ȥ��ƫ
	uint16_t multiturn_position_raw;//��Ȧ������λ�ã�������ƫ
	int64_t multiturn_angle;//��Ȧ�Ƕȣ�������ƫ
	
	int8_t temperate;	//�¶� ��λ(1degree/LSB)
	uint16_t voltage; //��ѹ ��λ(0.1v/LSB)
	uint8_t errorState;//errorStateλ      0         1
	/***��***/         //     0        ��ѹ����   ��ѹ����
	/***MG�������***/ //     3        �¶�����   ���±���                        
	
	
  float reduction_ratio; // ���ٱ� = ת��ת��/�����ת��
  float output_radius; // ��������������ͬ���ְ뾶�����ڼ������ٶ�
  int8_t direction; // ������ʵ��������ͬ��Ϊ1������Ϊ-1
  int64_t max_value_ecd; // ���������������ֵ
  int16_t half_max_value_ecd; // ���������������ֵ
  int16_t offset_ecd; // ���������ƫ��ֵ��ԭʼ��ֵ��
  uint16_t can_tx_id; // CAN���߷��͸�����õ�ID
  uint16_t can_rx_id; // CAN���ߵ�����͸��������õ�ID
  uint8_t can_tx_data_start_pos; // CAN���߷���������ʼ�±�
  uint8_t canx; // CAN���߱��
  float interval; // ���������ݽ���Ƶ��
} Motor_MG_Params_t;

typedef struct _Motor_MG_Measurement_t
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
  // int32_t total_ecd; // �ܼƵı�������ֵ�����ܻ������

} Motor_MG_Measurement_t;


// BMI088 backend��
class Motor_MG : public Motor_Backend
{
public:
   friend class Motor_Backend;
	friend class Motor_MG_ControlTask;
   friend class Arm_ControlTask;

  Motor_MG(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);
	
/********************겿ؿƼ�MG�������****************/
/***********************CAN����ָ��**********************/
	/*Get��ȡ����*/
	void getPIDParams();	//��ȡ PID �������0x30)
	void getAccel();			//��ȡ���ٶ�(0x33)
	void getMultiLoopEcd();			//��ȡ��������ǰλ�ã�0x90��
	void getMultiLoopAngle();//��ȡ��ǰ����Ķ�Ȧ���ԽǶ�ֵ(��ֵΪ˳ʱ����۽Ƕ�ֵ)(0x92)
	void getSingleLoopAngle();//��ȡ��Ȧ�Ƕ����0x94��
	void getState1andErrorState();//��ȡ�����״̬1���¶ȡ���ѹ���ʹ���״̬��־(0x9A)
	void getState2();							//��ȡ���״̬2���¶ȡ�ת�١�������λ�ã���0x9C��
	void getState3();							//��ȡ���״̬3���¶ȡ����������0x9D��
//	void getRunMode();//��ȡ��ǰ�������ģʽ(0x70)1.������2.�ٶȻ�3.λ�û�
//	void getMotorPower();//��ȡ��ǰ������ʣ�0x71��/��λ��0.1w/LSB��
//	void getMotorTimestamp();//��ȡϵͳ����ʱ��(0xB1)��λ(ms)
//	void getMotorCANID();//ͨѶID 0x300����ָ�0x79����1��CAN ID����ֵ��0x240 + ID��
	
	/*Setд������*/
	void setPIDParams_RAM(Motor_MG_PID_Params_t _PIDParams);		//д��PID������RAM��0x31��
	void setPIDParams_ROM(Motor_MG_PID_Params_t _PIDParams);		//д��PID������ROM(0x32)
	void setStartPosition_ROM(int32_t _ecdOffset);//д���������Ȧֵ�� ROM ��Ϊ���������0x63��
	void setCurruntEcdasStartPosition();          //д���������ǰ��Ȧλ�õ� ROM ��Ϊ���������0x64��
//	void setMotorsCANID(uint16_t _CANID);//ͨѶID 0x300����ָ�0x79����0дCAN ID��1~32��
	/*Command��������*/
	
	void cmdMotorShutOff();	//�رյ�������ͬʱ����������״̬�������καջ�ģʽ��(0x80)
	void cmdMotorStop();		//ֹͣ�����������ٶ�ͣ��������ʹ������ֲ���(0x81)
	void cmdMotorRun();     //�����������ָ����ֹͣ����ǰ�Ŀ��Ʒ�ʽ(0x88)
	void cmdTqControl(int16_t _tqControl);//ת�رջ��������0xA1��/��λ��0.01A/LSB��
	void cmdSpeedControl(int32_t _speedControl);//�ٶȱջ����ƣ�0xA2��/��λ��0.01dps/LSB��
	void cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl);//����λ�ñջ����ƣ�0xA4��/��λ��1dps/LSB,0.01degree/LSB��
	void cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _angleControl);//����λ�ñջ����ƣ�0xA8��/��λ��1dps/LSB,0.01degree/LSB��
	
/***********************����ظ����**********************/
  void  MG_motor_reply();//�ظ�����
  uint8_t can_update_flag = 0;//CAN���±�ʶ
  uint8_t angle_update_flag = 1;//�ײ�����Ƕȸ��±�ʶ
  uint8_t reset_flag = 0;
  uint8_t PIDchange_flag = 0;  

  void updateMotorMeasurement();

  virtual bool update(timeus_t dT_us); // ���µ�����
  virtual void init(void); // ��ʼ������
  virtual void uninit(void); // ����ʼ��

  virtual void updateMeasurement(void);

  void setParams(Motor_MG_Params_t _params)
  {
    params = _params;
    params.half_max_value_ecd = params.max_value_ecd / 2;
  }
  Motor_MG_Measurement_t getMeasurement()
  {
    return msr;
  }
	
  

int64_t move_left64(int64_t a, int len)
{
 int32_t *p = (int32_t*) &a;
 if (len <32)
 {
 
 *(p+1) <<= len;
 int32_t tmp = (*p) >> (32-len);
 *(p+1) |= tmp;
 *p <<= len;
 }
 else
 {
  *(p+1) = *p;
  *p = 0x00000000;
  *(p+1) <<= (len-32);
 }
 return a;
}



protected:
    
  void syncCANData()
  {
    can_tx_data[0] = motor_input >> 8;
    can_tx_data[1] = motor_input;
  }
  Motor_MG_PID_Params_t pid_params;
  Motor_MG_Measurement_t msr;
  Motor_MG_Params_t params;
  uint8_t can_tx_data[8];
  int16_t motor_input;
  uint8_t can_tx_data_start_pos;
	uint8_t reset_switch;//��λ����
   uint8_t crossline_flag = 0;
  bool isHomed;
  bool first_update = 1;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;
};

#endif
