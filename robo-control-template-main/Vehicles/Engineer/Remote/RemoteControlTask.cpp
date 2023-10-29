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
** 函 数 名： isBeyondDeadZone()
** 函数说明： 判断是否越过死区
**---------------------------------------------------------------------
** 输入参数： rc_data
** 返回参数： 无
***********************************************************************/
bool isBeyondDeadZone(int16_t rc_data)
{
  if(rc_data >= JOYSTICK_DEAD_ZONE || rc_data <= -JOYSTICK_DEAD_ZONE)
    return true;
  else
    return false;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::init()
** 函数说明： 判断是否完成初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::init(void)
{
  inited = true;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::update()
** 函数说明： 处理接收到的rc_data[]中的数据，完成遥控器控制
**---------------------------------------------------------------------
** 输入参数： 更新周期
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::update(timeus_t dT_us)
{	
	//刷新UI  左键刷新UI
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().mouse.press_l)
		{
			robot.referee_system_task_p->set_num_0();
			robot.referee_system_task_p->set_t_0();
		}	
		
		/* 键盘鼠标控制 遥控器微调 Begin */
	         /* ↓ */
	//比赛时候，都拨到下面
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
		if(robot.rc_protocol.getRCData().keyboard.key_bit.V && robot.arm_task_p->arm_mode != Arm_OFF && robot.arm_task_p->arm_mode != Arm_HoldOre  && robot.arm_task_p->switch_lock == 0)//KMJ有改动
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
		
		
		//关闭自定义控制器 B
		if(robot.rc_protocol.getRCData().keyboard.key_bit.B && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.arm_task_p->custom_lock == 0 && robot.arm_task_p->switch_lock == 0)//KMJ有改动 停止自定义控制器
		{
      robot.arm_task_p->custom_lock = 1;
			robot.arm_task_p->custom_flag = 0;
			
		}
		//开启自定义控制器 B + ctrl
		if(robot.rc_protocol.getRCData().keyboard.key_bit.B && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1 
			&& robot.arm_task_p->custom_lock == 0 && robot.arm_task_p->switch_lock == 0 && robot.arm_task_p->arm_mode == Arm_normal)//KMJ有改动 启动自定义控制器
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

		if(robot.rc_protocol.getRCData().keyboard.key_bit.B == 0 && custom_time_cnt < 210 && robot.arm_task_p->custom_lock ==  1) //2秒最多手动切换一次
		{
			custom_time_cnt ++;
		}
		if(custom_time_cnt == 210)
		{
			custom_time_cnt = 0;
			robot.arm_task_p->custom_lock = 0;
		}
		
		

		
		//F--翻矿正转
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CW;
		}
		//Ctrl+F--翻矿反转
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CCW;			
		}
		//G -- 翻矿出
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OUT;			
		}
		//ctrl + G -- 翻矿收
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_IN;			
		}
		else
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OFF;
		}


		
		/*控制六个气泵*/
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 &&robot.arm_task_p->airpump_flag[0] == 0)//R启动气泵
		{
          robot.arm_task_p->airpump_flag[0] = 1;
			 robot.arm_task_p->airpump_lock[0] = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 && robot.arm_task_p->airpump_flag[0] == 1)//再按R关闭气泵
		{
          robot.arm_task_p->airpump_flag[0] = 0;
			 robot.arm_task_p->airpump_lock[0]=  1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R == 0 && time_cnt[0] < 70 && robot.arm_task_p->airpump_lock[0] ==  1) //1秒最多手动切换一次
		{
			time_cnt[0] ++;
		}
		if(time_cnt[0] ==50)
		{
			time_cnt[0] = 0;
			robot.arm_task_p->airpump_lock[0] = 0;
		}
		
		
		//底盘速度模式切换
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
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT == 0 && shift_tim < 70 && shift_lock == 1) //2秒最多手动切换一次
		{
			shift_tim ++;
		}
		if(shift_tim == 30)
		{
			shift_tim = 0;
			shift_lock = 0;
		}

//云台控制模式0：自由移动
if(robot.gimbal_task_p->gimbal_mode == GIMBAL_FREE)		
{
	 //鼠标X轴控制图传的yaw轴6020电机
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
    else // 无输入置零
    {
      robot.gimbal_task_p->gimbal_yaw_vel = 0;
    }
	 
	//鼠标Y轴控制图传的pitcn轴舵机		
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

//鼠标右键控制云台模式切换
	if(robot.rc_protocol.getRCData().mouse.press_r && press_r_lock == 0)
	{
	  if(robot.gimbal_task_p->gimbal_mode == GIMBAL_LOCK){robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;press_r_lock = 1;}
	  else {robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;press_r_lock = 1;}	  
	}
	if(robot.rc_protocol.getRCData().mouse.press_r == 0 && press_r_tim < 30 && press_r_lock == 1) //1秒最多手动切换一次
	{
	    press_r_tim ++;
		 if(press_r_tim == 30)
		{
			press_r_tim = 0;
			press_r_lock = 0;
		}
	}




  //遥控器微调角度
	if(robot.arm_task_p->switch_lock == 0)
	{
			if(robot.rc_protocol.getRCData().ch3 >=300)         {extend_3508_delta =  -0.01;}
			else if (robot.rc_protocol.getRCData().ch3 <=- 300) {extend_3508_delta = 0.01;}
			else{extend_3508_delta = 0;}
		//左前后，extend_3508_delta
		
			if(robot.rc_protocol.getRCData().ch2 >=300)         {shift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {shift_3508_delta = -0.01;}
			else{shift_3508_delta = 0;}	
		//左左右，shift_3508_delta
		
			if(robot.rc_protocol.getRCData().ch1 >=300)         {uplift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch1 <=- 300) {uplift_3508_delta =  -0.01;}
			else{uplift_3508_delta = 0;}
		//右前后，控制uplift_3508_delta
		
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

	         /* ↑ */	
	  /* 键盘鼠标控制 遥控器微调 End */


else{	
	/* 遥控器控制 Begin
	遥控器左上,分别底盘，机械臂，右开关负责控制每个模块的具体模式 */
	
	/*遥控器说明
	左上 + arm_off + 右上 == 底盘上下电
	左上 + 右中 == 开关气泵
	左中 + chassis_off + 右上 == 机械臂切换模式
	*/
	
 if(robot.rc_protocol.getRCData().sw_left != SWITCH_UP || robot.rc_protocol.getRCData().sw_right != SWITCH_DWN)
 {		
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP && robot.arm_task_p->arm_mode == Arm_OFF)  //左上右上	
	{
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FAST;//左上右上+滑轮=底盘上电
		}
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel> 500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_OFF;//左上右中+滑轮=底盘下电
		}
	}
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)  //左上右上	
	{
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel <-500)
		{
			robot.arm_task_p->airpump_flag[0] = 1;//左上右中+滑轮=启动气泵
		}
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel > 500)
		{
			robot.arm_task_p->airpump_flag[0] = 0;//左上右中+滑轮=关闭气泵
		}

	}		
	
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID )//左中右
	{	
			if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.chassis_task_p->chassis_mode == Chassis_OFF)
			{
				if(robot.rc_protocol.getRCData().wheel < -500) //左中 右上 波轮向下  底盘下电  off变normal
				{
			  robot.arm_task_p->normal = state0;
			  robot.arm_task_p->last_arm_mode = Arm_OFF;
				robot.arm_task_p->arm_mode = Arm_normal;
				robot.arm_task_p->switch_lock = 1;
				}
				if(robot.rc_protocol.getRCData().wheel > 500)  //左中  波轮向上  normal变off   
				{
			  robot.arm_task_p->last_arm_mode = Arm_normal;
				robot.arm_task_p->arm_mode = Arm_OFF;				
				}
			}
	}
		
		
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)		
	 {		   
			//底盘速度
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
		//左 前后，控制extend_3508_delta
			
			if(robot.rc_protocol.getRCData().ch2 >=300)         {shift_3508_delta = 0.01;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {shift_3508_delta = -0.01;}
			else{shift_3508_delta = 0;}
		//左 左右，shift_3508_delta
		
			if(robot.rc_protocol.getRCData().ch1 >=300)         {uplift_3508_delta =  0.01;}
			else if (robot.rc_protocol.getRCData().ch1 <=- 300) {uplift_3508_delta =  -0.01;}
			else{uplift_3508_delta = 0;}
		//右 前后，控制uplift_3508_delta
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
				//左-前后，dm2_delta-pitch  
			
		  if(robot.rc_protocol.getRCData().ch2 >=300)         {dm1_delta = -0.002;}
			else if (robot.rc_protocol.getRCData().ch2 <=- 300) {dm1_delta = 0.002;}
			else{dm1_delta = 0;}			
	      	//左 左右，控制dm1_delta-roll		  
		  
		  if(robot.rc_protocol.getRCData().ch0 >=300)         {dm3_delta = -0.002;}
			else if (robot.rc_protocol.getRCData().ch0 <=- 300) {dm3_delta = 0.002;}
			else{dm3_delta = 0;}			
	      	//右 左右，控制dm3_delta-yaw		
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

	/* 遥控器控制 End*/
	
 

}

void RemoteControlTask::uninit(void)
{

}
