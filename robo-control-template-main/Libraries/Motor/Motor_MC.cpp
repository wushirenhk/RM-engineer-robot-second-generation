/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   truck-arm
** �� �� ����   Motor_MC.cpp
** �ļ�˵����   �������ܵ�����ݶ�ȡ����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     �����     	     2022-12-22
***************************************************************************/

#include "Motor_MC.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"


Motor_MC::Motor_MC(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;//���������������data��Ĭ��Ϊ0
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 8);
  motor_type = _motor_type;
}
/********************************************************************************/
/**********************************Get��ȡ����***********************************/
/********************************************************************************/
////��ȡ��Ȧ������λ��ָ�0x60��
///*�ظ�������
//DATA[4] ������λ�õ��ֽ� 1 DATA[4] = (uint8_t)(encoder)
//DATA[5] ������λ���ֽ� 2 DATA[5] = (uint8_t)(encoder>>8)
//DATA[6] ������λ���ֽ� 3 DATA[6] = (uint8_t)(encoder>>16)
//DATA[7] ������λ���ֽ� 4 DATA[7] = (uint8_t)(encoder>>24)*/
void Motor_MC::getMultiLoopEcdPosition()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
  can_tx_data[0] = 0x60;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}
////��ȡ��Ȧ������ԭʼλ��ָ�0x61��
///*�ظ�������
//DATA[4] ������ԭʼλ���ֽ� 1 DATA[4] = (uint8_t)(encoderRaw)
//DATA[5] ������ԭʼλ���ֽ� 2 DATA[5] = (uint8_t)(encoderRaw>>8)
//DATA[6] ������ԭʼλ���ֽ� 3 DATA[6] = (uint8_t)(encoderRaw>>16)
//DATA[7] ������ԭʼλ���ֽ� 4 DATA[7] = (uint8_t)(encoderRaw>>24)*/
void Motor_MC::getMultiLoopEcdRawPosition()
{
	if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
  can_tx_data[0] = 0x61;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}
////��ȡ��Ȧ��������ƫ�������0x62��
///*�ظ�������
//DATA[4] ��������ƫ�ֽ� 1 DATA[4] = (uint8_t)(encoderOffset)
//DATA[5] ��������ƫ�ֽ� 2 DATA[5] = (uint8_t)(encoderOffset>>8)
//DATA[6] ��������ƫ�ֽ� 3 DATA[6] = (uint8_t)(encoderOffset>>16)
//DATA[7] ��������ƫ�ֽ� 4 DATA[7] = (uint8_t)(encoderOffset>>24)*/
void Motor_MC::getMultiLoopEcdOffset()
{
//	can_device.can_tx_data->flag = 0;//����ģʽ
	if(can_update_flag == 0)
	{
  can_tx_data[0] = 0x62;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
  can_update_flag = can_tx_data[0];
	}
}
////��ȡ��Ȧ�Ƕ����0x92��
///*�ظ�������
//DATA[4] �Ƕȵ��ֽ� 1 DATA[4] = (uint8_t)(motorAngle)
//DATA[5] �Ƕ��ֽ� 2 DATA[5] = (uint8_t)(motorAngle>>8)
//DATA[6] �Ƕ��ֽ� 3 DATA[6] = (uint8_t)(motorAngle>>16)
//DATA[7] �Ƕ��ֽ� 4 DATA[7] = (uint8_t)(motorAngle>>24)
void Motor_MC::getMultiLoopAngle()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
	can_tx_data[0] = 0x92;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0;
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}
//��ȡ�����״̬1���¶ȡ���ѹ���ʹ���״̬��־(0x9A)
///*�ظ�������
//DATA[0] �����ֽ� 0x9A
//DATA[1] ����¶� DATA[1] = (uint8_t)(temperature)
//DATA[2] NULL 0x00
//DATA[3] ��բ�ͷ�ָ�� DATA[3] = (uint8_t)(RlyCtrlRslt)
//DATA[4] ��ѹ���ֽ� DATA[4] = (uint8_t)(voltage)
//DATA[5] ��ѹ���ֽ� DATA[5] = (uint8_t)(voltage>>8)
//DATA[6] ����״̬���ֽ� 1 DATA[6] = (uint8_t)(errorState)
//DATA[7] ����״̬�ֽ� 2 DATA[7] = (uint8_t)(errorState>>8)*/
void Motor_MC::getState1andErrorState()
{
	if(can_update_flag == 0)
	{

//	can_device.can_tx_data->flag = 0;//����ģʽ
	can_tx_data[0] = 0x9A;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	can_update_flag = can_tx_data[0];
	}

}
//��ȡ���״̬2���¶ȡ�ת�١�������λ�ã���0x9C��
///*�ظ�������
//DATA[0] �����ֽ� 0x9C
//DATA[1] ����¶� DATA[1] = (uint8_t)(temperature)
//DATA[2] ת�ص������ֽ� DATA[2] = (uint8_t)(iq)
//DATA[3] ת�ص������ֽ� DATA[3] = (uint8_t)(iq>>8)
//DATA[4] ����ٶȵ��ֽ� DATA[4] = (uint8_t)(speed)
//DATA[5] ����ٶȸ��ֽ� DATA[5] = (uint8_t)(speed>>8)
//DATA[6] ����Ƕȵ��ֽ� DATA[6] = (uint8_t)(degree)
//DATA[7] ����Ƕȸ��ֽ� DATA[7] = (uint8_t)(degree>>8)*/
void Motor_MC::getState2()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x9C;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
  can_update_flag = can_tx_data[0];
	}
}
//��ȡ���״̬3���¶ȡ����������0x9D��
///*�ظ�������
//DATA[0] �����ֽ� 0x9D
//DATA[1] ����¶� DATA[1] = (uint8_t)(temperature)
//DATA[2] A ��������ֽ� DATA[2] = (uint8_t)(iA)
//DATA[3] A ��������ֽ� DATA[3] = (uint8_t)(iA>>8)
//DATA[4] B ��������ֽ� DATA[4] = (uint8_t)(iB)
//DATA[5] B ��������ֽ� DATA[5] = (uint8_t)(iB>>8)
//DATA[6] C ��������ֽ� DATA[6] = (uint8_t)(iC)
//DATA[7] C ��������ֽ� DATA[7] = (uint8_t)(iC>>8)*/
void Motor_MC::getState3()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x9D;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//��ȡ��ǰ�������ģʽ(0x70)
///*�ظ�������
// ������ģʽ(0x01)
// �ٶȻ�ģʽ(0x02)
// λ�û�ģʽ(0x03)
//DATA[0] �����ֽ� 0x70
//DATA[7] �������ģʽ DATA[7] = (uint8_t)(runmode)*/
void Motor_MC::getRunMode()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x70;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//��ȡ��ǰ������ʣ�0x71��/��λ��0.1w/LSB��
///*�ظ������� 
//DATA[6] ������й��ʵ��ֽ� DATA[6] = (uint8_t)(motorpower)
//DATA[7] ������й��ʸ��ֽ� DATA[7] = (uint8_t)(motorpower>>8))*/
void Motor_MC::getMotorPower()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x71;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//��ȡϵͳ����ʱ��(0xB1)��λ(ms)
///*�ظ������� ��λ��0.1w/LSB��
//DATA[6] ������й��ʵ��ֽ� DATA[6] = (uint8_t)(motorpower)
//DATA[7] ������й��ʸ��ֽ� DATA[7] = (uint8_t)(motorpower>>8))*/
void Motor_MC::getMotorTimestamp()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0xB1;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//ͨѶID 0x300����ָ�0x79����1��CAN ID��0x240 + ID��
void Motor_MC::getMotorCANID()
{
	if(can_update_flag == 0)
	{
	bool wReadWriteFlag = 1;
	can_tx_data[0] = 0x79;
  can_tx_data[1] = 0;
  can_tx_data[2] = wReadWriteFlag;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}

//��ȡPIDֵ��0x30��
void Motor_MC::getPIDParams()
{
		if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x30;
	can_tx_data[1] = 0;
	can_tx_data[2] = 0;
	can_tx_data[3] = 0;
	can_tx_data[4] = 0; 
	can_tx_data[5] = 0;
	can_tx_data[6] = 0;
	can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}


/*Setд������*/
//д��PID������RAM��0x31��
void Motor_MC::setPIDParams_RAM(Motor_MC_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x31;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.CurKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.CurKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKp_change);
  can_tx_data[6] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.PosKi_change);
		can_update_flag = can_tx_data[0];
	}
}
//д��PID������ROM��0x32��
void Motor_MC::setPIDParams_ROM(Motor_MC_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x32;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.CurKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.CurKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.PosKi_change);
   can_update_flag = can_tx_data[0];
	}
}
//д���������Ȧֵ�� ROM ��Ϊ���������0x63�������
void Motor_MC::setStartPosition_ROM(int32_t _ecdOffset)
{
	if(can_update_flag == 0)
	{
	params.ecd_offset = _ecdOffset;
	can_tx_data[0] = 0x63;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = (uint8_t)(params.ecd_offset); 
  can_tx_data[5] = (uint8_t)(params.ecd_offset>>8);
  can_tx_data[6] = (uint8_t)(params.ecd_offset>>16);
  can_tx_data[7] = (uint8_t)(params.ecd_offset>>24);
		can_update_flag = can_tx_data[0];
	}
}
//д���������ǰ��Ȧλ�õ� ROM ��Ϊ���������0x64�������
void Motor_MC::setCurruntEcdasStartPosition()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x64;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//ͨѶID 0x300����ָ�0x79����0дCAN ID��1~32��
void Motor_MC::setMotorsCANID(uint16_t _CANID)
{
	if(can_update_flag == 0)
	{
	params.can_tx_id = _CANID;
	bool wReadWriteFlag = 0;
	can_tx_data[0] = 0x79;
  can_tx_data[1] = 0;
  can_tx_data[2] = wReadWriteFlag;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = (uint8_t)(_CANID);
		can_update_flag = can_tx_data[0];
	}
}
/*******************************************************/
/********************Command��������*********************/
/*******************************************************/

//�رյ�������ͬʱ����������״̬�������καջ�ģʽ��(0x80)
void Motor_MC::cmdMotorShutOff()
{
		if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x80;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//ֹͣ�����������ٶ�ͣ��������ʹ������ֲ���(0x81)
void Motor_MC::cmdMotorStop()
{
			if(can_update_flag == 0)
	{
  can_tx_data[0] = 0x81;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
			  can_update_flag = can_tx_data[0];
	}
}
//ת�رջ��������0xA1��/��λ��0.01A/LSB��
void Motor_MC::cmdTqControl(int16_t _tqControl)
{
	if(can_update_flag == 0)
	{
	params.tq_control = _tqControl;
	can_tx_data[0] = 0xA1;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = (uint8_t)(params.tq_control); 		//ת�ص�������ֵ���ֽ�
  can_tx_data[5] = (uint8_t)(params.tq_control>>8);	//ת�ص�������ֵ���ֽ�
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//�ٶȱջ����ƣ�0xA2��/��λ��0.01dps/LSB��
void Motor_MC::cmdSpeedControl(int32_t _speedControl)
{
	if(can_update_flag == 0)
	{
	params.speed_control = _speedControl;
	can_tx_data[0] = 0xA2;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = * (uint8_t *)(&params.speed_control); //λ�ÿ��Ƶ��ֽ�
  can_tx_data[5] = *((uint8_t *)(&params.speed_control)+1);
  can_tx_data[6] = *((uint8_t *)(&params.speed_control)+2);
  can_tx_data[7] = *((uint8_t *)(&params.speed_control)+3);//λ�ÿ��Ƹ��ֽ�
		can_update_flag = can_tx_data[0];
	}
}
//����λ�ñջ����ƣ�0xA4��/��λ��1dps/LSB,0.01degree/LSB��
void Motor_MC::cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl)
{
//	can_device.can_tx_data->flag = 0;//����ģʽ
	  if(can_update_flag == 0)
	{
	params.max_speed = _maxSpeed;
	params.angle_control = _angleControl;
	can_tx_data[0] = 0xA4;
  can_tx_data[1] = 0x00;
  can_tx_data[2] = (uint8_t)(params.max_speed);		//�ٶ����Ƶ��ֽ�
  can_tx_data[3] = (uint8_t)(params.max_speed>>8);//�ٶ����Ƹ��ֽ�
  can_tx_data[4] = (uint8_t)(params.angle_control);		//λ�ÿ��Ƶ��ֽ�
  can_tx_data[5] = (uint8_t)(params.angle_control>>8);	//λ�ÿ���
  can_tx_data[6] = (uint8_t)(params.angle_control>>16);//λ�ÿ���
  can_tx_data[7] = (uint8_t)(params.angle_control>>24);//λ�ÿ��Ƹ��ֽ�
  can_update_flag = can_tx_data[0];
	}
}
//����λ�ñջ����ƣ�0xA8��/��λ��1dps/LSB,0.01degree/LSB��
void Motor_MC::cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _delta_angleControl)
{
	if(can_update_flag == 0)
	{
	params.max_speed = _maxSpeed;
	params.delta_angle_control = _delta_angleControl;
	can_tx_data[0] = 0xA8;
  can_tx_data[1] = 0x00;
  can_tx_data[2] = (uint8_t)(params.max_speed);		//�ٶ����Ƶ��ֽ�
  can_tx_data[3] = (uint8_t)(params.max_speed>>8);//�ٶ����Ƹ��ֽ�
  can_tx_data[4] = (uint8_t)(params.delta_angle_control);		//λ�ÿ��Ƶ��ֽ�
  can_tx_data[5] = (uint8_t)(params.delta_angle_control>>8);	//λ�ÿ���
  can_tx_data[6] = (uint8_t)(params.delta_angle_control>>16);//λ�ÿ���
  can_tx_data[7] = (uint8_t)(params.delta_angle_control>>24);//λ�ÿ��Ƹ��ֽ�
		can_update_flag = can_tx_data[0];
	}
}
//��λϵͳ����(0x76)
void Motor_MC::cmdSystemReset()
{

	can_tx_data[0] = 0x76;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;

}

void  Motor_MC::MC_motor_reply()
{
	if(can_rx_data.data[0] == can_update_flag)
 {
	 can_update_flag = 0;
	 
    switch(can_rx_data.data[0]){
	  case 0x30://��ȡPID����
	  pid_params.CurKp_now = ((int32_t)(can_rx_data.data[2]));//*1/256
	  pid_params.CurKi_now = ((int32_t)(can_rx_data.data[3]));//*0.5/256
	  pid_params.SpdKp_now = ((int32_t)(can_rx_data.data[4]));//*0.05/256
	  pid_params.SpdKi_now = ((int32_t)(can_rx_data.data[5]));//*0.005/256
	  pid_params.PosKp_now = ((int32_t)(can_rx_data.data[6]));//*0.5/256
	  pid_params.PosKi_now = ((int32_t)(can_rx_data.data[7]));//*0.0005/256
	  
	  break;
 
	  case 0x60://��ȡ��Ȧ������λ�ã�Ϊ������ԭʼλ�ü�ȥ��������Ȧ��ƫ����ʼλ�ã����ֵ
	  params.multiturn_position = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;
	  
	  case 0x61://��ȡ��Ȧ������λ�ã�������ƫ
	  params.multiturn_position_raw = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;
	  
	  case 0x62://��ȡ��Ȧ��������ƫλ��
	  params.ecd_offset = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;	  

	  case 0x92://��ȡ��Ȧ�Ƕ�����
	  params.multiturn_angle = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24))/100;
	  break;
	  
	  case 0x9C://��ȡ�����״̬1���¶ȡ���ѹ���ʹ���״̬��־
	  params.temperate  = ((uint8_t)(can_rx_data.data[1]));
	  params.tq_control =((int16_t)(can_rx_data.data[2] | can_rx_data.data[3] << 8 ));
	  params.speed_control = ((int16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 )); 
	  
	  break;
		  
     }
	}
}


/***********************************************************************
** �� �� ���� updateMotorMeasurement()0
** ����˵���� ��CAN���ݸ��µ��������Ϣ
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
void Motor_MC::updateMotorMeasurement()
{
  // ʱ���
	  static bool first_update = 1;
  // ʱ���
  com_msr.timestamp = msr.timestamp = micros();

}



void Motor_MC::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_MC::init(void)
{
//  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_MC::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_MC::uninit(void) {}
