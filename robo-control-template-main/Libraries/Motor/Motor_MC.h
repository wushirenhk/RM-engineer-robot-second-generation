#ifndef __MOTOR_MC_H__
#define __MOTOR_MC_H__

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

enum MCState
{
  MCSTATE_INITING = 0,
  MCSTATE_NORMAL =  1,
  MCSTATE_LOCK =    2,
  MCSTATE_DISABLE = 3
};

typedef struct _Motor_MC_PID_Params_t
{
	uint8_t CurKp_change;
	uint8_t CurKi_change;
	uint8_t SpdKp_change;
	uint8_t SpdKi_change;
	uint8_t PosKp_change;
	uint8_t PosKi_change;
   uint8_t CurKp_now;
	uint8_t CurKi_now;
	uint8_t SpdKp_now;
	uint8_t SpdKi_now;
	uint8_t PosKp_now;
	uint8_t PosKi_now;
}Motor_MC_PID_Params_t;


typedef struct _Motor_MC_Params_t
{
	/***MC�������***/
	/***��***/
	int32_t ecd_offset;		//�����������Ȧֵƫ��ֵ������ȷ�����
	int16_t tq_control;		//ת�ص������ ��λ��0.01A/LSB��
	int32_t speed_control;	//�ٶȱջ�  ���������ٶ� ��λ��0.01dps/LSB��
	int32_t angle_control;  //����λ�ñջ����� ��λ��0.01degree/LSB��
	int32_t delta_angle_control;//����λ�ñջ������Ƕ� ��λ��0.01degree/LSB��
	int16_t max_speed;      //�ٶ�����
	
	int32_t multiturn_position;//��Ȧ������λ�ã���ȥ��ƫ
	int32_t multiturn_position_raw;//��Ȧ������λ�ã�������ƫ
	int32_t multiturn_angle;//��Ȧ�Ƕȣ�������ƫ
	
	 uint8_t temperate; // �¶�
	 int32_t last_angle;
	/***��***/
	/***MC�������***/
	
	
  float reduction_ratio; // ���ٱ� = ת��ת��/�����ת��
  float output_radius; // ��������������ͬ���ְ뾶�����ڼ������ٶ�
  int8_t direction; // ������ʵ��������ͬ��Ϊ1������Ϊ-1
  int16_t max_value_ecd; // ���������������ֵ
  int16_t half_max_value_ecd; // ���������������ֵ
  int16_t offset_ecd; // ���������ƫ��ֵ��ԭʼ��ֵ��
  uint16_t can_tx_id; // CAN���߷��͸�����õ�ID
  uint16_t can_rx_id; // CAN���ߵ�����͸��������õ�ID
  uint8_t can_tx_data_start_pos; // CAN���߷���������ʼ�±�
  uint8_t canx; // CAN���߱��
  float interval; // ���������ݽ���Ƶ��
} Motor_MC_Params_t;

typedef struct _Motor_MC_Measurement_t
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

} Motor_MC_Measurement_t;


// BMI088 backend��
class Motor_MC : public Motor_Backend
{
public:
   friend class Motor_Backend;
	friend class Motor_MC_ControlTask;
   friend class Arm_ControlTask;

  Motor_MC(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);
	
/********************���������ŷ��������****************/
/***********************CAN����ָ��**********************/
	/*Get��ȡ����*/
	void getPIDParams();	//��ȡ PID �������0x30)
	void getAccel();			//��ȡ���ٶ�
	void getMultiLoopEcdPosition();			//��ȡ��Ȧ������λ��ָ�0x60��
	void getMultiLoopEcdRawPosition();	//��ȡ��Ȧ������ԭʼλ��ָ�0x61��
	void getMultiLoopEcdOffset();	//��ȡ��Ȧ��������ƫ�������0x62��
	void getMultiLoopAngle();			//��ȡ��ǰ����Ķ�Ȧ���ԽǶ�ֵ(0x92)
	void getState1andErrorState();//��ȡ�����״̬1���¶ȡ���ѹ���ʹ���״̬��־(0x9A)
	void getState2();							//��ȡ���״̬2���¶ȡ�ת�١�������λ�ã���0x9C��
	void getState3();							//��ȡ���״̬3���¶ȡ����������0x9D��
	void getRunMode();//��ȡ��ǰ�������ģʽ(0x70)1.������2.�ٶȻ�3.λ�û�
	void getMotorPower();//��ȡ��ǰ������ʣ�0x71��/��λ��0.1w/LSB��
	void getMotorTimestamp();//��ȡϵͳ����ʱ��(0xB1)��λ(ms)
	void getMotorCANID();//ͨѶID 0x300����ָ�0x79����1��CAN ID����ֵ��0x240 + ID��
	
	/*Setд������*/
	void setPIDParams_RAM(Motor_MC_PID_Params_t _PIDParams);		//д��PID������RAM��0x31��
	void setPIDParams_ROM(Motor_MC_PID_Params_t _PIDParams);		//д��PID������ROM(0x32)
	void setStartPosition_ROM(int32_t _ecdOffset);//д���������Ȧֵ�� ROM ��Ϊ���������0x63��
	void setCurruntEcdasStartPosition();//д���������ǰ��Ȧλ�õ� ROM ��Ϊ���������0x64��
	void setMotorsCANID(uint16_t _CANID);//ͨѶID 0x300����ָ�0x79����0дCAN ID��1~32��
	/*Command��������*/
	
	void cmdMotorShutOff();	//�رյ�������ͬʱ����������״̬�������καջ�ģʽ��(0x80)
	void cmdMotorStop();		//ֹͣ�����������ٶ�ͣ��������ʹ������ֲ���(0x81)
	void cmdTqControl(int16_t _tqControl);//ת�رջ��������0xA1��/��λ��0.01A/LSB��
	void cmdSpeedControl(int32_t _speedControl);//�ٶȱջ����ƣ�0xA2��/��λ��0.01dps/LSB��
	void cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl);//����λ�ñջ����ƣ�0xA4��/��λ��1dps/LSB,0.01degree/LSB��
	void cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _angleControl);//����λ�ñջ����ƣ�0xA8��/��λ��1dps/LSB,0.01degree/LSB��
 	void cmdSystemReset();//��λϵͳ����(0x76)
	
/***********************����ظ����**********************/
  void  MC_motor_reply();//�ظ�����
  uint8_t can_update_flag = 0;//CAN���±�ʶ
  uint8_t angle_update_flag = 1;//�ײ�����Ƕȸ��±�ʶ
  uint8_t reset_flag = 0;
  uint8_t PIDchange_flag = 0;  

  void updateMotorMeasurement();

  virtual bool update(timeus_t dT_us); // ���µ�����
  virtual void init(void); // ��ʼ������
  virtual void uninit(void); // ����ʼ��

  virtual void updateMeasurement(void);

  void setParams(Motor_MC_Params_t _params)
  {
    params = _params;
    params.half_max_value_ecd = params.max_value_ecd / 2;
  }
  Motor_MC_Measurement_t getMeasurement()
  {
    return msr;
  }
	
protected:
    
  void syncCANData()
  {
    can_tx_data[0] = motor_input >> 8;
    can_tx_data[1] = motor_input;
  }
  Motor_MC_PID_Params_t pid_params;
  Motor_MC_Measurement_t msr;
  Motor_MC_Params_t params;
  uint8_t can_tx_data[8];
  int16_t motor_input;
  uint8_t can_tx_data_start_pos;
	uint8_t reset_switch;//��λ����
	bool isHomed;
	bool first_update = 1;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;
};

#endif
