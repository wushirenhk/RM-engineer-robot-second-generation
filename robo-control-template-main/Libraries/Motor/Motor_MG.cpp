/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Motor_MG.cpp
** �ļ�˵����   겿ؿƼ�MG������ݶ�ȡ����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     �����     	     2023-12-22
***************************************************************************/

#include "Motor_MG.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"


Motor_MG::Motor_MG(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;//MG�����������data��Ĭ��Ϊ0
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 8);
  motor_type = _motor_type;
}

/********************************************************************************/
/**********************************Get��ȡ����***********************************/
/********************************************************************************/
//��ȡPIDֵ��0x30��
///*�����ظ�
//DATA[2] λ�û�P����     DATA[2]= anglePidKp
//DATA[3] λ�û�I����     DATA[3]= anglePidKi
//DATA[4] �ٶȻ�P����     DATA[4]= speedPidKp
//DATA[5] �ٶȻ�I����     DATA[5]= speedPidKi
//DATA[6] ת�ػ�P����     DATA[6]= iqPidKp
//DATA[7] ת�ػ�I����     DATA[7]= iqPidKi*/
void Motor_MG::getPIDParams()
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
//��ȡ���ٶȣ�0x33��
///*�����ظ�
//DATA[4] ���ٶȵ��ֽ� 1   DATA[4]= (uint8_t)Accel
//DATA[5] ���ٶ��ֽ� 2     DATA[5]= (uint8_t)Accel+1
//DATA[6] ���ٶ��ֽ� 3     DATA[6]= (uint8_t)Accel+2
//DATA[7] ���ٶ��ֽ� 4     DATA[7]= (uint8_t)Accel+3*/
void Motor_MG::getAccel()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
  can_tx_data[0] = 0x33;
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

//��ȡ�������������0x90��
///*�����ظ�
//DATA[2] ������λ�õ��ֽ�     DATA[2]= *(uint8_t *) (&encoder)
//DATA[3] ������λ�ø��ֽ�     DATA[3]= *((uint8_t *) (&encoder)+1)
//DATA[4] ������ԭʼλ�õ��ֽ� DATA[4]= *(uint8_t *) (&encoderRaw)
//DATA[5] ������ԭʼλ�ø��ֽ� DATA[5]= *((uint8_t *) (&encoderRaw)+1)
//DATA[6] ��������ƫ���ֽ�     DATA[6]= *(uint8_t *)(&encoderOffset)
//DATA[7] ��������ƫ���ֽ�     DATA[7]= *((uint8_t *)(&encoderOffset)+1)*/
void Motor_MG::getMultiLoopEcd()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
  can_tx_data[0] = 0x90;
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

//��ȡ��Ȧ�Ƕ����0x92��
///*�����ظ�
//DATA[1] �Ƕȵ��ֽ� 1  DATA[1]= *(uint_8_t *) (&motorAngle)
//DATA[2] �Ƕ��ֽ� 2    DATA[2]= *((uint8_t *) (&motorAngle)+1)
//DATA[3] �Ƕ��ֽ� 3    DATA[3]= *((uint8_t *) (&motorAngle)+2)
//DATA[4] �Ƕ��ֽ� 4    DATA[4]= *((uint8_t *) (&motorAngle)+3)
//DATA[5] �Ƕ��ֽ� 5    DATA[5]= *((uint8_t *) (&motorAngle)+4)
//DATA[6] �Ƕ��ֽ� 6    DATA[6]= *((uint8_t *) (&motorAngle)+5)
//DATA[7] �Ƕ��ֽ� 7    DATA[7]= *((uint8_t *) (&motorAngle)+6)*/
void Motor_MG::getMultiLoopAngle()
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

//��ȡ��Ȧ�Ƕ����0x94��
///*�����ظ�
//DATA[6] �Ƕȵ��ֽ� 1  DATA[6]= *(uint_8_t *) (&circleAngle)
//DATA[7] �Ƕȸ��ֽ� 7  DATA[7]= *((uint8_t *) (&circleAngle)+1)*/
void Motor_MG::getSingleLoopAngle()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//����ģʽ
	can_tx_data[0] = 0x94;
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
//DATA[1] ����¶�     DATA[1] = *(uint8_t *)(temperature)
//DATA[2] NULL 0x00
//DATA[3] ��ѹ���ֽ� 	 DATA[3] = *(uint8_t *)(&voltage)
//DATA[4] ��ѹ���ֽ�   DATA[4] = *((uint8_t *)(&voltage)+1)
//DATA[5] NULL 0x00
//DATA[6] NULL 0x00
//DATA[7] ����״̬�ֽ� DATA[7] = *(uint8_t *)(errorState)*/
void Motor_MG::getState1andErrorState()
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

//��ȡ���״̬2���¶ȡ���ѹ��ת�١�������λ�ã���0x9C��
///*�ظ�������
//DATA[1] ����¶� DATA[1] = (uint8_t)(temperature)
//DATA[2] ת�ص������ֽ� DATA[2] = (uint8_t)(iq)
//DATA[3] ת�ص������ֽ� DATA[3] = ((uint8_t)(iq)+1
//DATA[4] ����ٶȵ��ֽ� DATA[4] = (uint8_t)(speed)
//DATA[5] ����ٶȸ��ֽ� DATA[5] = (uint8_t)(speed>>8)
//DATA[6] ������λ�õ��ֽ� DATA[6] = (uint8_t)(encoder)
//DATA[7] ������λ�ø��ֽ� DATA[7] = (uint8_t)(encoder)+1*/
void Motor_MG::getState2()
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
//DATA[1] ����¶�       DATA[1] = (uint8_t)(temperature)
//DATA[2] A ��������ֽ� DATA[2] = (uint8_t)(iA)
//DATA[3] A ��������ֽ� DATA[3] = (uint8_t)(iA)+1
//DATA[4] B ��������ֽ� DATA[4] = (uint8_t)(iB)
//DATA[5] B ��������ֽ� DATA[5] = (uint8_t)(iB)+1
//DATA[6] C ��������ֽ� DATA[6] = (uint8_t)(iC)
//DATA[7] C ��������ֽ� DATA[7] = (uint8_t)(iC)+1*/
void Motor_MG::getState3()
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

/********************************************************************************/
/**********************************Setд������***********************************/
/********************************************************************************/
//д��PID������RAM��0x31��
void Motor_MG::setPIDParams_RAM(Motor_MG_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x31;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.PosKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.TqKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.TqKi_change);
		can_update_flag = can_tx_data[0];
	}
}
//д��PID������ROM��0x32��
void Motor_MG::setPIDParams_ROM(Motor_MG_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x32;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.PosKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.TqKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.TqKi_change);
   can_update_flag = can_tx_data[0];
	}
}
//д����ٶȵ�RAM(0x34)

//д���������Ȧֵ�� ROM ��Ϊ���������0x91��
void Motor_MG::setStartPosition_ROM(int32_t _ecdOffset)
{
	if(can_update_flag == 0)
	{
	params.ecd_offset = _ecdOffset;
	can_tx_data[0] = 0x91;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0;
  can_tx_data[5] = 0;
  can_tx_data[6] = (uint8_t)(params.ecd_offset);
  can_tx_data[7] = (uint8_t)(params.ecd_offset>>8);
		can_update_flag = can_tx_data[0];
	}
}
//д���������ǰ��Ȧλ�õ� ROM ��Ϊ���������0x19�������(Ƶ��д��Ӱ������)
void Motor_MG::setCurruntEcdasStartPosition()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x19;
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

/*******************************************************/
/********************Command��������********************/
/*******************************************************/
//�رյ�������ͬʱ����������״̬��֮ǰ���յĿ���ָ��(0x80)
void Motor_MG::cmdMotorShutOff()
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
//ֹͣ������������������״̬��֮ǰ���յĿ���ָ��(0x81)
void Motor_MG::cmdMotorStop()
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
//�����������ָ����ֹͣ����ǰ�Ŀ��Ʒ�ʽ(0x88)
void Motor_MG::cmdMotorRun()
{
  if(can_update_flag == 0)
	{
    can_tx_data[0] = 0x88;
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
void Motor_MG::cmdTqControl(int16_t _tqControl)
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
void Motor_MG::cmdSpeedControl(int32_t _speedControl)
{
	if(can_update_flag == 0)
	{
	params.speed_control = _speedControl;
	can_tx_data[0] = 0xA2;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = * (uint8_t *)(&params.speed_control); //�ٶȿ��Ƶ��ֽ�
  can_tx_data[5] = *((uint8_t *)(&params.speed_control)+1);
  can_tx_data[6] = *((uint8_t *)(&params.speed_control)+2);
  can_tx_data[7] = *((uint8_t *)(&params.speed_control)+3);//�ٶȿ��Ƹ��ֽ�
		can_update_flag = can_tx_data[0];
	}
}
//����λ�ñջ����ƣ�0xA4��/��λ��1dps/LSB,0.01degree/LSB��
void Motor_MG::cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl)
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
//λ�ñջ��������� 6 ����ʽ��0xA8��/��λ��1dps/LSB,0.01degree/LSB��
void Motor_MG::cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _delta_angleControl)
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

void  Motor_MG::MG_motor_reply()
{
	if(can_rx_data.data[0] == can_update_flag)
 {
	 can_update_flag = 0;
	 
    switch(can_rx_data.data[0]){
	  case 0x30://��ȡPID����
	  pid_params.PosKi_now = can_rx_data.data[2];//*1/256
	  pid_params.PosKi_now = can_rx_data.data[3];//*0.5/256
	  pid_params.SpdKp_now = can_rx_data.data[4];//*0.05/256
	  pid_params.SpdKi_now = can_rx_data.data[5];//*0.005/256
	  pid_params.TqKi_now =  can_rx_data.data[6];//*0.5/256
	  pid_params.TqKi_now =  can_rx_data.data[7];//*0.0005/256
	  
	  break;
 
	  case 0x90://��ȡ��Ȧ������λ�ã�Ϊ������ԭʼλ�ü�ȥ��������Ȧ��ƫ����ʼλ�ã����ֵ
	  params.multiturn_position = ((uint16_t)(can_rx_data.data[2] | can_rx_data.data[3] << 8));	  
	  //��ȡ��Ȧ������λ�ã�������ƫ
	  params.multiturn_position_raw = ((uint16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8));
	  //��ȡ��Ȧ��������ƫλ��
	  params.ecd_offset = ((uint16_t)(can_rx_data.data[6] | can_rx_data.data[7] << 8));
	  break;	  

	  case 0x92://��ȡ��Ȧ�Ƕ�����
	  params.multiturn_angle = ((int64_t)(can_rx_data.data[1] | can_rx_data.data[2] << 8 
	                                     | can_rx_data.data[3] << 16 | can_rx_data.data[4] << 24 
		                                  |  (int64_t)can_rx_data.data[5] << 32 | (int64_t)can_rx_data.data[6] << 40 
		                                  |  (int64_t)can_rx_data.data[7] << 48 ));
//		 *(uint8_t*)((&params.multiturn_angle)) = can_rx_data.data[1];
//	    *(uint8_t*)((&params.multiturn_angle)+1) = can_rx_data.data[2];
//		 *(uint8_t*)((&params.multiturn_angle)+2) = can_rx_data.data[3];
//		 *(uint8_t*)((&params.multiturn_angle)+3) = can_rx_data.data[4];
//		 *(uint8_t*)((&params.multiturn_angle)+4) = can_rx_data.data[5];
//		 *(uint8_t*)((&params.multiturn_angle)+5) = can_rx_data.data[6];
//		 *(uint8_t*)((&params.multiturn_angle)+6) = can_rx_data.data[7];
//	  if(params.multiturn_angle < 0)
//	  {
//		  crossline_flag = 1;	  
//	  }
	  break;
	  
		case 0x94:
			params.singleturn_angle=((uint16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8));
		
		break;
	  
		case 0x9C://��ȡ�����״̬2
	  params.temperate = can_rx_data.data[1];
//		params.errorState = can_rx_data.data[7];
//DATA[3] ��ѹ���ֽ� 	 DATA[3] = *(uint8_t *)(&voltage)
//DATA[4] ��ѹ���ֽ�   DATA[4] = *((uint8_t *)(&voltage)+1)
		break;
		  
     }
	}
}

/***********************************************************************
** �� �� ���� updateMotorMeasurement()
** ����˵���� ��CAN���ݸ��µ��������Ϣ
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
void Motor_MG::updateMotorMeasurement()
{
  // ʱ���
	static bool first_update = 1;
  // ʱ���
  com_msr.timestamp = msr.timestamp = micros();
}

void Motor_MG::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_MG::init(void)
{
//  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_MG::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_MG::uninit(void) {}





















