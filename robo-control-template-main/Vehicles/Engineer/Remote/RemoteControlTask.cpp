#include "/Remote/RemoteControlTask.h"
#include "/Robot/Robot.h"

#define JOYSTICK_DEAD_ZONE 5
#define JOYSTICK_MAX 700

/* ----------------------- RC Switch Definition----------------------------- */
#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MID               ((uint16_t)3)
#define SWITCH_DWN               ((uint16_t)2)

/* ----------------------- ROBOT Mode Definition---------------------------- */
#define MOT	OR_DISABLE										((uint16_t)1)
#define CHASSIS_FOLLOW_GIMBAL						((uint16_t)2)
#define CHASSIS_GYRO_ROTATION						((uint16_t)3)
#define CHASSIS_FREE										((uint16_t)4)

/***********************************************************************
** �� �� ���� isBeyondDeadZone()
** ����˵���� �ж��Ƿ�Խ������
**---------------------------------------------------------------------
** ��������� rc_data
** ���ز����� ��
***********************************************************************/
bool isBeyondDeadZone(int16_t rc_data)
{
  if(rc_data >= JOYSTICK_DEAD_ZONE || rc_data <= -JOYSTICK_DEAD_ZONE)
    return true;
  else
    return false;
}

/***********************************************************************
** �� �� ���� RemoteControlTask::init()
** ����˵���� �ж��Ƿ���ɳ�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void RemoteControlTask::init(void)
{
  inited = true;
}

/***********************************************************************
** �� �� ���� RemoteControlTask::update()
** ����˵���� ������յ���rc_data[]�е����ݣ����ң��������
**---------------------------------------------------------------------
** ��������� ��������
** ���ز����� ��
***********************************************************************/
void RemoteControlTask::update(timeus_t dT_us)
{	
	//ˢ��UI  ���ˢ��UI
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().mouse.press_l)
		{
			robot.referee_system_task_p->set_num_0();
			robot.referee_system_task_p->set_t_0();
		}	
		
		/* ���������� ң����΢�� Begin */
	         /* �� */
	//����ʱ�򣬶���������
	if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().sw_left == SWITCH_DWN )
{	
      //arm_mormal -- Z
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.arm_task_p->arm_mode != Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->normal = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_normal;
			robot.arm_task_p->switch_lock = 1;
		}
		//arm_AirConnection -- C
 		if(robot.rc_protocol.getRCData().keyboard.key_bit.C && robot.arm_task_p->arm_mode == Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->AirConnection = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_AirConnection;
			robot.arm_task_p->switch_lock = 1;
		}
		//arm_SilveryOre -- X	
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.X && robot.arm_task_p->arm_mode == Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->SilveryOre = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_SilveryOre;
			robot.arm_task_p->switch_lock = 1;
		}
		//arm_HoldOre -- V
		if(robot.rc_protocol.getRCData().keyboard.key_bit.V && robot.arm_task_p->arm_mode != Arm_OFF && robot.arm_task_p->arm_mode != Arm_HoldOre  && robot.arm_task_p->switch_lock == 0)//KMJ�иĶ�
		{
			robot.arm_task_p->HoldOre = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_HoldOre;
			robot.arm_task_p->switch_lock = 1;
		}
      //arm_OrePlacement---CTRL + Z
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1 && (robot.arm_task_p->arm_mode == Arm_normal || robot.arm_task_p->arm_mode == Arm_HoldOre) && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->OrePlacement = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_OrePlacement;
			robot.arm_task_p->switch_lock = 1;
		}
		
		
		//�ر��Զ�������� B
		if(robot.rc_protocol.getRCData().keyboard.key_bit.B && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.arm_task_p->custom_lock == 0 && robot.arm_task_p->switch_lock == 0)//KMJ�иĶ� ֹͣ�Զ��������
		{
      robot.arm_task_p->custom_lock = 1;
			robot.arm_task_p->custom_flag = 0;
			
		}
		//�����Զ�������� B + ctrl
		if(robot.rc_protocol.getRCData().keyboard.key_bit.B && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1 
			&& robot.arm_task_p->custom_lock == 0 && robot.arm_task_p->switch_lock == 0 && robot.arm_task_p->arm_mode == Arm_normal)//KMJ�иĶ� �����Զ��������
		{
       robot.arm_task_p->custom_lock = 1;      
			 robot.arm_task_p->custom_flag = 1;			
			 robot.arm_task_p->Custom_pitch_save = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_pitch;
			 robot.arm_task_p->Custom_roll_save  = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_roll;
			 robot.arm_task_p->Custom_yaw_save   = robot.referee_system_task_p->robot_referee_status.custom_robot_data.Custom_yaw;
			 robot.arm_task_p-> motor_dm1_task->motor_backend_p->params.position_rad =  robot.arm_task_p->position_rad_save[0];
	     robot.arm_task_p-> motor_dm2_task->motor_backend_p->params.position_rad =  robot.arm_task_p->position_rad_save[1];
	     robot.arm_task_p-> motor_dm3_task->motor_backend_p->params.position_rad =  robot.arm_task_p->position_rad_save[2];	
		}

		if(robot.rc_protocol.getRCData().keyboard.key_bit.B == 0 && custom_time_cnt < 210 && robot.arm_task_p->custom_lock ==  1) //2������ֶ��л�һ��
		{
			custom_time_cnt ++;
		}
		if(custom_time_cnt == 210)
		{
			custom_time_cnt = 0;
			robot.arm_task_p->custom_lock = 0;
		}
		
		

		
		//F--������ת
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CW;
		}
		//Ctrl+F--����ת
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CCW;			
		}
		//G -- �����
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OUT;			
		}
		//ctrl + G -- ������
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_IN;			
		}
		else
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OFF;
		}


		
		/*������������*/
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 &&robot.arm_task_p->airpump_flag[0] == 0)//R��������
		{
          robot.arm_task_p->airpump_flag[0] = 1;
			 robot.arm_task_p->airpump_lock[0] = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 && robot.arm_task_p->airpump_flag[0] == 1)//�ٰ�R�ر�����
		{
          robot.arm_task_p->airpump_flag[0] = 0;
			 robot.arm_task_p->airpump_lock[0]=  1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R == 0 && time_cnt[0] < 70 && robot.arm_task_p->airpump_lock[0] ==  1) //1������ֶ��л�һ��
		{
			time_cnt[0] ++;
		}
		if(time_cnt[0] ==50)
		{
			time_cnt[0] = 0;
			robot.arm_task_p->airpump_lock[0] = 0;
		}
		
		
		//�����ٶ�ģʽ�л�
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT && robot.chassis_task_p->chassis_mode == Chassis_SLOW && shift_lock == 0)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FAST;
			shift_lock = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT && robot.chassis_task_p->chassis_mode == Chassis_FAST && shift_lock == 0)
		{
			robot.chassis_task_p->chassis_mode = Chassis_SLOW;
			shift_lock = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT == 0 && shift_tim < 70 && shift_lock == 1) //2������ֶ��л�һ��
		{
			shift_tim ++;
		}
		if(shift_tim == 30)
		{
			shift_tim = 0;
			shift_lock = 0;
		}

//��̨����ģʽ0�������ƶ�
if(robot.gimbal_task_p->gimbal_mode == GIMBAL_FREE)		
{
	 //���X�����ͼ����yaw��6020���
	 if(robot.rc_protocol.getRCData().mouse.vx != 0)
    {
      robot.gimbal_task_p->gimbal_yaw_vel = -robot.rc_protocol.getRCData().mouse.vx / 15.0f;
      if(robot.gimbal_task_p->gimbal_yaw_vel > robot.gimbal_task_p->gimbal_max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = robot.gimbal_task_p->gimbal_max_yaw_vel;
      }
      else if(robot.gimbal_task_p->gimbal_yaw_vel < -robot.gimbal_task_p->gimbal_max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = -robot.gimbal_task_p->gimbal_max_yaw_vel;
      }
    }
    else // ����������
    {
      robot.gimbal_task_p->gimbal_yaw_vel = 0;
    }
	 
	//���Y�����ͼ����pitcn����		
	robot.gimbal_task_p->gimbal_pitch_angle += robot.rc_protocol.getRCData().mouse.vy/30.0f;
	if(robot.gimbal_task_p->gimbal_pitch_angle >= 185)
   {
		robot.gimbal_task_p->gimbal_pitch_angle = 160;
	}
	if(robot.gimbal_task_p->gimbal_pitch_angle <= 90)
	{
		robot.gimbal_task_p->gimbal_pitch_angle = 90;
	}	
}

//����Ҽ�������̨ģʽ�л�
	if(robot.rc_protocol.getRCData().mouse.press_r && press_r_lock == 0)
	{
	  if(robot.gimbal_task_p->gimbal_mode == GIMBAL_LOCK){robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;press_r_lock = 1;}
	  else {robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;press_r_lock = 1;}	  
	}
	if(robot.rc_protocol.getRCData().mouse.press_r == 0 && press_r_tim < 30 && press_r_lock == 1) //1������ֶ��л�һ��
	{
	    press_r_tim ++;
		 if(press_r_tim == 30)
		{
			press_r_tim = 0;
			press_r_lock = 0;
		}
	}




  //ң����΢���Ƕ�
	if(robot.arm_task_p->switch_lock == 0)
	{
			if(robot.rc_protocol.getRCData().ch3 >=300)         {extend_3508_delta =  -0.01;}
			else if (robot.rc_protocol.getRCData().ch3 <=- 300) {extend_3508_delta = 0.01;}
			else{extend_3508_delta = 0;}
		//��ǰ��extend_3508_delta
		
			if(robot.rc_protocol.getRCData().ch2 >=300)         {shift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {shift_3508_delta = -0.01;}
			else{shift_3508_delta = 0;}	
		//�����ң�shift_3508_delta
		
			if(robot.rc_protocol.getRCData().ch1 >=300)         {uplift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch1 <=- 300) {uplift_3508_delta =  -0.01;}
			else{uplift_3508_delta = 0;}
		//��ǰ�󣬿���uplift_3508_delta
		
		if(robot.arm_task_p->custom_flag == 1)
		{
			if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_fowrwardback >= 200){ extend_3508_delta  =  -0.03;}
			else if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_fowrwardback <= 100){ extend_3508_delta  =   0.03;}
			else{extend_3508_delta  =  0;}
		
			if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_updown >= 200){ uplift_3508_delta  =  -0.03;}
			else if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_updown <= 100){ uplift_3508_delta  =   0.03;}
			else{uplift_3508_delta  =  0;}
			
			if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_rightleft >= 200){ shift_3508_delta  =  -0.03;}
			else if(robot.referee_system_task_p->robot_referee_status.custom_robot_data.custom_robot_data_rightleft <= 100){ shift_3508_delta  =   0.03;}		
			else{shift_3508_delta  =  0;}
		}	
	}
	
}

	         /* �� */	
	  /* ���������� ң����΢�� End */


else{	
	/* ң�������� Begin
	ң��������,�ֱ���̣���е�ۣ��ҿ��ظ������ÿ��ģ��ľ���ģʽ */
	
	/*ң����˵��
	���� + arm_off + ���� == �������µ�
	���� + ���� == ��������
	���� + chassis_off + ���� == ��е���л�ģʽ
	*/
	
 if(robot.rc_protocol.getRCData().sw_left != SWITCH_UP || robot.rc_protocol.getRCData().sw_right != SWITCH_DWN)
 {		
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP && robot.arm_task_p->arm_mode == Arm_OFF)  //��������	
	{
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FAST;//��������+����=�����ϵ�
		}
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel> 500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_OFF;//��������+����=�����µ�
		}
	}
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)  //��������	
	{
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel <-500)
		{
			robot.arm_task_p->airpump_flag[0] = 1;//��������+����=��������
		}
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel > 500)
		{
			robot.arm_task_p->airpump_flag[0] = 0;//��������+����=�ر�����
		}

	}		
	
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID )//������
	{	
			if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.chassis_task_p->chassis_mode == Chassis_OFF)
			{
				if(robot.rc_protocol.getRCData().wheel < -500) //���� ���� ��������  �����µ�  off��normal
				{
			  robot.arm_task_p->normal = state0;
			  robot.arm_task_p->last_arm_mode = Arm_OFF;
				robot.arm_task_p->arm_mode = Arm_normal;
				robot.arm_task_p->switch_lock = 1;
				}
				if(robot.rc_protocol.getRCData().wheel > 500)  //����  ��������  normal��off   
				{
			  robot.arm_task_p->last_arm_mode = Arm_normal;
				robot.arm_task_p->arm_mode = Arm_OFF;				
				}
			}
	}
		
		
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)		
	 {		   
			//�����ٶ�
     	if(robot.chassis_task_p->chassis_mode == Chassis_FAST)	
		{
		  robot.chassis_task_p->vx = robot.rc_protocol.getRCData().ch3 / 600.0f;

        robot.chassis_task_p->vy = -robot.rc_protocol.getRCData().ch2 / 600.0f;

	     robot.chassis_task_p->vw = -robot.rc_protocol.getRCData().ch0 / 600.0f;
		}
	}
}	


	if(robot.arm_task_p->switch_lock == 0)
	{
 
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID)		
	  {	
		  	if(robot.rc_protocol.getRCData().ch3 >=300)         {extend_3508_delta =  -0.01;}
			else if (robot.rc_protocol.getRCData().ch3 <=- 300) {extend_3508_delta =  0.01;}
			else{extend_3508_delta = 0;}	
		//�� ǰ�󣬿���extend_3508_delta
			
			if(robot.rc_protocol.getRCData().ch2 >=300)         {shift_3508_delta = 0.01;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {shift_3508_delta = -0.01;}
			else{shift_3508_delta = 0;}
		//�� ���ң�shift_3508_delta
		
			if(robot.rc_protocol.getRCData().ch1 >=300)         {uplift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch1 <=- 300) {uplift_3508_delta =  -0.01;}
			else{uplift_3508_delta = 0;}
		//�� ǰ�󣬿���uplift_3508_delta
			dm1_delta = 0;
			dm2_delta = 0;
			dm3_delta = 0;
			
			robot.chassis_task_p->vx = 0;

      robot.chassis_task_p->vy = 0;

	    robot.chassis_task_p->vw = 0;	
	  }
		
	   if(robot.rc_protocol.getRCData().sw_left == SWITCH_DWN)		
	  {	
		  if(robot.rc_protocol.getRCData().ch3 >=300)         {dm2_delta = -0.002;}
			else if (robot.rc_protocol.getRCData().ch3 <=- 300) {dm2_delta = 0.002;}
			else{dm2_delta = 0;}
				//��-ǰ��dm2_delta-pitch  
			
		  if(robot.rc_protocol.getRCData().ch2 >=300)         {dm1_delta = -0.002;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {dm1_delta = 0.002;}
			else{dm1_delta = 0;}			
	      	//�� ���ң�����dm1_delta-roll		  
		  
		  if(robot.rc_protocol.getRCData().ch0 >=300)         {dm3_delta = -0.002;}
			else if (robot.rc_protocol.getRCData().ch0 <=- 300) {dm3_delta = 0.002;}
			else{dm3_delta = 0;}			
	      	//�� ���ң�����dm3_delta-yaw		
			extend_3508_delta = 0;
			shift_3508_delta = 0;
			uplift_3508_delta = 0;	
			
			robot.chassis_task_p->vx = 0;

      robot.chassis_task_p->vy = 0;

	    robot.chassis_task_p->vw = 0;			
	  }
 }
	

      if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP && robot.rc_protocol.getRCData().sw_right == SWITCH_DWN)
  { 		
			  robot.arm_task_p->last_arm_mode = Arm_normal;
	
				robot.arm_task_p->arm_mode = Arm_OFF;	
	
        robot.chassis_task_p->chassis_mode = Chassis_OFF;
  }
}

	/* ң�������� End*/
	
 

}

void RemoteControlTask::uninit(void)
{

}
