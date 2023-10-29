	#include "Params.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/***********************************************************************
** 函 数 名： Params::initMotorsParams()
** 函数说明： 初始化各个电机参数，PID参数和任务时间周期参数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Params::initMotorsParams()
{
	/************** 电机CAN总线地址 **************/ 
	
  /************** RM 电机 CAN总线地址 **************/

  motor_params.chassis_motor_1 = {.can_tx_id = 0x200, .can_rx_id = 0x201, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.chassis_motor_2 = {.can_tx_id = 0x200, .can_rx_id = 0x202, .can_tx_data_start_pos = 2, .canx = 2};
  motor_params.chassis_motor_3 = {.can_tx_id = 0x200, .can_rx_id = 0x203, .can_tx_data_start_pos = 4, .canx = 2};
  motor_params.chassis_motor_4 = {.can_tx_id = 0x200, .can_rx_id = 0x204, .can_tx_data_start_pos = 6, .canx = 2};
 
  motor_params.gimbal_6020 =     {.can_tx_id = 0x1FF, .can_rx_id = 0x205, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.shift_3508 =      {.can_tx_id = 0x1FF, .can_rx_id = 0x206, .can_tx_data_start_pos = 2, .canx = 2};
  motor_params.overturn1_3508 =  {.can_tx_id = 0x1FF, .can_rx_id = 0x207, .can_tx_data_start_pos = 4, .canx = 2};
  motor_params.overturn2_3508 =  {.can_tx_id = 0x1FF, .can_rx_id = 0x208, .can_tx_data_start_pos = 6, .canx = 2};
  
  /************** DM 电机 CAN总线地址 **************/  
	
  motor_params.arm_dm_1 = {.can_tx_id = 0x101, .can_rx_id = 0x01, .can_tx_data_start_pos = 0, .canx = 1};
  motor_params.arm_dm_2 = {.can_tx_id = 0x102, .can_rx_id = 0x02, .can_tx_data_start_pos = 0, .canx = 1};  
	motor_params.arm_dm_3 = {.can_tx_id = 0x103, .can_rx_id = 0x03, .can_tx_data_start_pos = 0, .canx = 1};
	
  motor_params.uplift1_3508 =    {.can_tx_id = 0x1FF, .can_rx_id = 0x205, .can_tx_data_start_pos = 0, .canx = 1};
  motor_params.uplift2_3508 =    {.can_tx_id = 0x1FF, .can_rx_id = 0x206, .can_tx_data_start_pos = 2, .canx = 1};
  motor_params.extend1_3508 =    {.can_tx_id = 0x1FF, .can_rx_id = 0x207, .can_tx_data_start_pos = 4, .canx = 1};
  motor_params.extend2_3508 =    {.can_tx_id = 0x1FF, .can_rx_id = 0x208, .can_tx_data_start_pos = 6, .canx = 1};
 
	
  /**************  电机参数  **************/
	
	/************** 3508 电机参数 **************/ 
  // 底盘3508电机参数（4个）
  motor_params.chassis_motor_1.reduction_ratio = 19.0f;
  motor_params.chassis_motor_1.output_radius = .075f;
  motor_params.chassis_motor_1.direction = MOTOR_CW;
  motor_params.chassis_motor_1.max_value_ecd = 8192;
  motor_params.chassis_motor_1.offset_ecd = 0;

  motor_params.chassis_motor_2.reduction_ratio = 19.0f;
  motor_params.chassis_motor_2.output_radius = .075f;
  motor_params.chassis_motor_2.direction = MOTOR_CW;
  motor_params.chassis_motor_2.max_value_ecd = 8192;
  motor_params.chassis_motor_2.offset_ecd = 0;

  motor_params.chassis_motor_3.reduction_ratio = 19.0f;
  motor_params.chassis_motor_3.output_radius = .075f;
  motor_params.chassis_motor_3.direction = MOTOR_CW;
  motor_params.chassis_motor_3.max_value_ecd = 8192;
  motor_params.chassis_motor_3.offset_ecd = 0;

  motor_params.chassis_motor_4.reduction_ratio = 19.0f;
  motor_params.chassis_motor_4.output_radius = .075f;
  motor_params.chassis_motor_4.direction = MOTOR_CW;
  motor_params.chassis_motor_4.max_value_ecd = 8192;
  motor_params.chassis_motor_4.offset_ecd = 0;
    
  //云台6020电机参数（1个）
  motor_params.gimbal_6020.reduction_ratio = 1.0f;
  motor_params.gimbal_6020.output_radius = 1.0f;
  motor_params.gimbal_6020.direction = MOTOR_CW;
  motor_params.gimbal_6020.max_value_ecd = 8192;
  motor_params.gimbal_6020.offset_ecd = 0;
  
  //平移3508电机参数（1个）
  motor_params.shift_3508.reduction_ratio = 19.0f;
  motor_params.shift_3508.output_radius = .075f;
  motor_params.shift_3508.direction = MOTOR_CW;
  motor_params.shift_3508.max_value_ecd = 8192;
  motor_params.shift_3508.offset_ecd = 0;
	 
  //抬升3508电机参数（2个）
  motor_params.uplift1_3508.reduction_ratio = 260.0f;
  motor_params.uplift1_3508.output_radius = .075f;
  motor_params.uplift1_3508.direction = MOTOR_CW;
  motor_params.uplift1_3508.max_value_ecd = 8192;
  motor_params.uplift1_3508.offset_ecd = 0;

  motor_params.uplift2_3508.reduction_ratio = 200.0f;//右侧抬升
  motor_params.uplift2_3508.output_radius = .075f;
  motor_params.uplift2_3508.direction = MOTOR_CW;
  motor_params.uplift2_3508.max_value_ecd = 8192;
  motor_params.uplift2_3508.offset_ecd = 0;
	
	//伸出3508电机参数（2个）
  motor_params.extend1_3508.reduction_ratio = 19.0f;
  motor_params.extend1_3508.output_radius = .075f;
  motor_params.extend1_3508.direction = MOTOR_CW;
  motor_params.extend1_3508.max_value_ecd = 8192;
  motor_params.extend1_3508.offset_ecd = 0;

  motor_params.extend2_3508.reduction_ratio = 19.0f;
  motor_params.extend2_3508.output_radius = .075f;
  motor_params.extend2_3508.direction = MOTOR_CW;
  motor_params.extend2_3508.max_value_ecd = 8192;
  motor_params.extend2_3508.offset_ecd = 0;
	
	//翻矿3508电机参数（2个）
  motor_params.overturn1_3508.reduction_ratio = 19.0f;
  motor_params.overturn1_3508.output_radius = .075f;
  motor_params.overturn1_3508.direction = MOTOR_CW;
  motor_params.overturn1_3508.max_value_ecd = 8192;
  motor_params.overturn1_3508.offset_ecd = 0;

  motor_params.overturn2_3508.reduction_ratio = 19.0f;
  motor_params.overturn2_3508.output_radius = .075f;
  motor_params.overturn2_3508.direction = MOTOR_CW;
  motor_params.overturn2_3508.max_value_ecd = 8192;
  motor_params.overturn2_3508.offset_ecd = 0;
	
	
  /************** DM 电机参数 **************/ 
  //DM电机参数（3个）
  motor_params.arm_dm_1.reduction_ratio = 10;
  motor_params.arm_dm_1.output_radius = 1;  
  motor_params.arm_dm_1.direction = MOTOR_CW;
  motor_params.arm_dm_1.max_value_ecd = 8192;
  
  motor_params.arm_dm_2.reduction_ratio = 10;
  motor_params.arm_dm_2.output_radius = 1;  
  motor_params.arm_dm_2.direction = MOTOR_CW;
  motor_params.arm_dm_2.max_value_ecd = 8192;
	
	motor_params.arm_dm_3.reduction_ratio = 10;
  motor_params.arm_dm_3.output_radius = 1;  
  motor_params.arm_dm_3.direction = MOTOR_CW;
  motor_params.arm_dm_3.max_value_ecd = 8192;

  /************** 电机PID参数 **************/

	/************** 3508 PID参数 **************/

  //底盘3508（4个） PID参数 角速度环 单环
  motor_params.chassis_motor_1_ang_vel.type_selection = PID_DELTA;
  motor_params.chassis_motor_1_ang_vel.kp = 10000;
  motor_params.chassis_motor_1_ang_vel.ki = 200000;
  motor_params.chassis_motor_1_ang_vel.kd_fb = 0;
  motor_params.chassis_motor_1_ang_vel.kd_ex = 0;
  motor_params.chassis_motor_1_ang_vel.k_ff = 0;
  motor_params.chassis_motor_1_ang_vel.max_out_value = 10000;
  motor_params.chassis_motor_1_ang_vel.min_out_value = -10000;
  motor_params.chassis_motor_1_ang_vel.limit_output = true;
  motor_params.chassis_motor_1_ang_vel.max_integral = 0;
  motor_params.chassis_motor_1_ang_vel.min_integral = -0;
  motor_params.chassis_motor_1_ang_vel.limit_integral = true;
  motor_params.chassis_motor_1_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.chassis_motor_1_ang_vel.kd_able_error_range = 0;
  motor_params.chassis_motor_1_ang_vel.ki_able_error_range = 0;

  motor_params.chassis_motor_2_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_3_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_4_ang_vel = motor_params.chassis_motor_1_ang_vel;

  //平移3508（1个） PID参数 角速度环 内环
  motor_params.shift_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.shift_3508_ang_vel.kp = 1000;
  motor_params.shift_3508_ang_vel.ki = 10000;
  motor_params.shift_3508_ang_vel.kd_fb = 0;
  motor_params.shift_3508_ang_vel.kd_ex = 0;
  motor_params.shift_3508_ang_vel.k_ff = 0;
  motor_params.shift_3508_ang_vel.max_out_value = 10000;
  motor_params.shift_3508_ang_vel.min_out_value = -10000;
  motor_params.shift_3508_ang_vel.limit_output = true;
  motor_params.shift_3508_ang_vel.max_integral = 0.1;
  motor_params.shift_3508_ang_vel.min_integral = -0.1;
  motor_params.shift_3508_ang_vel.limit_integral = true;
  motor_params.shift_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.shift_3508_ang_vel.kd_able_error_range = 0;
  motor_params.shift_3508_ang_vel.ki_able_error_range = 0;
  
  //平移3508（1个） PID参数 角度环 外环
  motor_params.shift_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.shift_3508_ang.kp = 5;
  motor_params.shift_3508_ang.ki = 0;
  motor_params.shift_3508_ang.kd_fb = 0;
  motor_params.shift_3508_ang.kd_ex = 0;
  motor_params.shift_3508_ang.k_ff = 0;
  motor_params.shift_3508_ang.max_out_value = 10000;
  motor_params.shift_3508_ang.min_out_value = -10000;
  motor_params.shift_3508_ang.limit_output = true;
  motor_params.shift_3508_ang.max_integral = 0.1;
  motor_params.shift_3508_ang.min_integral = -0.1;
  motor_params.shift_3508_ang.limit_integral = true;
  motor_params.shift_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.shift_3508_ang.kd_able_error_range = 0;
  motor_params.shift_3508_ang.ki_able_error_range = 0;    
	
	//抬升3508（2个） PID参数 角速度环 内环
  motor_params.uplift1_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.uplift1_3508_ang_vel.kp = 6000;
  motor_params.uplift1_3508_ang_vel.ki = 20000;
  motor_params.uplift1_3508_ang_vel.kd_fb = 0;
  motor_params.uplift1_3508_ang_vel.kd_ex = 0;
  motor_params.uplift1_3508_ang_vel.k_ff = 0;
  motor_params.uplift1_3508_ang_vel.max_out_value = 16000;
  motor_params.uplift1_3508_ang_vel.min_out_value = -16000;
  motor_params.uplift1_3508_ang_vel.limit_output = true;
  motor_params.uplift1_3508_ang_vel.max_integral = 0.1;
  motor_params.uplift1_3508_ang_vel.min_integral = -0.1;
  motor_params.uplift1_3508_ang_vel.limit_integral = true;
  motor_params.uplift1_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.uplift1_3508_ang_vel.kd_able_error_range = 0;
  motor_params.uplift1_3508_ang_vel.ki_able_error_range = 0;
  
  //抬升3508（2个） PID参数 角度环 外环
  motor_params.uplift1_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.uplift1_3508_ang.kp = 10;
  motor_params.uplift1_3508_ang.ki = 0;
  motor_params.uplift1_3508_ang.kd_fb = 0;
  motor_params.uplift1_3508_ang.kd_ex = 0;
  motor_params.uplift1_3508_ang.k_ff = 0;
  motor_params.uplift1_3508_ang.max_out_value = 16000;
  motor_params.uplift1_3508_ang.min_out_value = -16000;
  motor_params.uplift1_3508_ang.limit_output = true;
  motor_params.uplift1_3508_ang.max_integral = 0.1;
  motor_params.uplift1_3508_ang.min_integral = -0.1;
  motor_params.uplift1_3508_ang.limit_integral = true;
  motor_params.uplift1_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.uplift1_3508_ang.kd_able_error_range = 0;
  motor_params.uplift1_3508_ang.ki_able_error_range = 0;  

	//抬升3508（2个） PID参数 角速度环 内环
  motor_params.uplift2_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.uplift2_3508_ang_vel.kp = 3000;
  motor_params.uplift2_3508_ang_vel.ki = 20000;
  motor_params.uplift2_3508_ang_vel.kd_fb = 0;
  motor_params.uplift2_3508_ang_vel.kd_ex = 0;
  motor_params.uplift2_3508_ang_vel.k_ff = 0;
  motor_params.uplift2_3508_ang_vel.max_out_value = 16000;
  motor_params.uplift2_3508_ang_vel.min_out_value = -16000;
  motor_params.uplift2_3508_ang_vel.limit_output = true;
  motor_params.uplift2_3508_ang_vel.max_integral = 0.1;
  motor_params.uplift2_3508_ang_vel.min_integral = -0.1;
  motor_params.uplift2_3508_ang_vel.limit_integral = true;
  motor_params.uplift2_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.uplift2_3508_ang_vel.kd_able_error_range = 0;
  motor_params.uplift2_3508_ang_vel.ki_able_error_range = 0;
  
  //抬升3508（2个） PID参数 角度环 外环
  motor_params.uplift2_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.uplift2_3508_ang.kp = 8;
  motor_params.uplift2_3508_ang.ki = 0;
  motor_params.uplift2_3508_ang.kd_fb = 0;
  motor_params.uplift2_3508_ang.kd_ex = 0;
  motor_params.uplift2_3508_ang.k_ff = 0;
  motor_params.uplift2_3508_ang.max_out_value = 16000;
  motor_params.uplift2_3508_ang.min_out_value = -16000;
  motor_params.uplift2_3508_ang.limit_output = true;
  motor_params.uplift2_3508_ang.max_integral = 0.1;
  motor_params.uplift2_3508_ang.min_integral = -0.1;
  motor_params.uplift2_3508_ang.limit_integral = true;
  motor_params.uplift2_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.uplift2_3508_ang.kd_able_error_range = 0;
  motor_params.uplift2_3508_ang.ki_able_error_range = 0;  

	
	//伸出3508（2个） PID参数 角速度环 内环
  motor_params.extend1_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.extend1_3508_ang_vel.kp = 1000;
  motor_params.extend1_3508_ang_vel.ki = 10000;
  motor_params.extend1_3508_ang_vel.kd_fb = 0;
  motor_params.extend1_3508_ang_vel.kd_ex = 0;
  motor_params.extend1_3508_ang_vel.k_ff = 0;
  motor_params.extend1_3508_ang_vel.max_out_value = 10000;
  motor_params.extend1_3508_ang_vel.min_out_value = -10000;
  motor_params.extend1_3508_ang_vel.limit_output = true;
  motor_params.extend1_3508_ang_vel.max_integral = 0.1;
  motor_params.extend1_3508_ang_vel.min_integral = -0.1;
  motor_params.extend1_3508_ang_vel.limit_integral = true;
  motor_params.extend1_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.extend1_3508_ang_vel.kd_able_error_range = 0;
  motor_params.extend1_3508_ang_vel.ki_able_error_range = 0;
	
	motor_params.extend2_3508_ang_vel = motor_params.extend1_3508_ang_vel;
  
  //伸出3508（2个） PID参数 角度环 外环
  motor_params.extend1_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.extend1_3508_ang.kp = 5;
  motor_params.extend1_3508_ang.ki = 0;
  motor_params.extend1_3508_ang.kd_fb = 0;
  motor_params.extend1_3508_ang.kd_ex = 0;
  motor_params.extend1_3508_ang.k_ff = 0;
  motor_params.extend1_3508_ang.max_out_value = 10000;
  motor_params.extend1_3508_ang.min_out_value = -10000;
  motor_params.extend1_3508_ang.limit_output = true;
  motor_params.extend1_3508_ang.max_integral = 0.1;
  motor_params.extend1_3508_ang.min_integral = -0.1;
  motor_params.extend1_3508_ang.limit_integral = true;
  motor_params.extend1_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.extend1_3508_ang.kd_able_error_range = 0;
  motor_params.extend1_3508_ang.ki_able_error_range = 0;  

	motor_params.extend2_3508_ang = motor_params.extend1_3508_ang;
	
  //翻矿3508（2个） PID参数 角速度环 内环
  motor_params.overturn1_3508_ang_vel.type_selection = PID_DELTA;
  motor_params.overturn1_3508_ang_vel.kp = 500;
  motor_params.overturn1_3508_ang_vel.ki = 10000;
  motor_params.overturn1_3508_ang_vel.kd_fb = 0;
  motor_params.overturn1_3508_ang_vel.kd_ex = 0;
  motor_params.overturn1_3508_ang_vel.k_ff = 0;
  motor_params.overturn1_3508_ang_vel.max_out_value = 16000;
  motor_params.overturn1_3508_ang_vel.min_out_value = -16000;
  motor_params.overturn1_3508_ang_vel.limit_output = true;
  motor_params.overturn1_3508_ang_vel.max_integral = 0.1;
  motor_params.overturn1_3508_ang_vel.min_integral = -0.1;
  motor_params.overturn1_3508_ang_vel.limit_integral = true;
  motor_params.overturn1_3508_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.overturn1_3508_ang_vel.kd_able_error_range = 0;
  motor_params.overturn1_3508_ang_vel.ki_able_error_range = 0;
	
	motor_params.overturn2_3508_ang_vel = motor_params.overturn1_3508_ang_vel;
  
  //翻矿3508（2个） PID参数 角度环 外环
  motor_params.overturn1_3508_ang.type_selection = PID_ABSOLUTE;
  motor_params.overturn1_3508_ang.kp = 5;
  motor_params.overturn1_3508_ang.ki = 10;
  motor_params.overturn1_3508_ang.kd_fb = 0;
  motor_params.overturn1_3508_ang.kd_ex = 0;
  motor_params.overturn1_3508_ang.k_ff = 0;
  motor_params.overturn1_3508_ang.max_out_value = 16000;
  motor_params.overturn1_3508_ang.min_out_value = -16000;
  motor_params.overturn1_3508_ang.limit_output = true;
  motor_params.overturn1_3508_ang.max_integral = 0.1;
  motor_params.overturn1_3508_ang.min_integral = -0.1;
  motor_params.overturn1_3508_ang.limit_integral = true;
  motor_params.overturn1_3508_ang.add = &cmt::simple_adder_instance_f;
  motor_params.overturn1_3508_ang.kd_able_error_range = 0;
  motor_params.overturn1_3508_ang.ki_able_error_range = 0;   

	motor_params.overturn2_3508_ang =  motor_params.overturn1_3508_ang;
	
	/************** 6020 PID参数 **************/
  //角速度环PID参数
  motor_params.gimbal_6020_ang_vel.type_selection = PID_DELTA;
  motor_params.gimbal_6020_ang_vel.kp = 200;
  motor_params.gimbal_6020_ang_vel.ki = 20000;
  motor_params.gimbal_6020_ang_vel.kd_fb = 0;
  motor_params.gimbal_6020_ang_vel.kd_ex = 0;
  motor_params.gimbal_6020_ang_vel.k_ff = 0;
  motor_params.gimbal_6020_ang_vel.max_out_value = 10000;
  motor_params.gimbal_6020_ang_vel.min_out_value = -10000;
  motor_params.gimbal_6020_ang_vel.limit_output = true;
  motor_params.gimbal_6020_ang_vel.max_integral = 0.1;
  motor_params.gimbal_6020_ang_vel.min_integral = -0.1;
  motor_params.gimbal_6020_ang_vel.limit_integral = true;
  motor_params.gimbal_6020_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_6020_ang_vel.kd_able_error_range = 0;
  motor_params.gimbal_6020_ang_vel.ki_able_error_range = 0;
  
  //角度环PID参数
  motor_params.gimbal_6020_ang.type_selection = PID_ABSOLUTE;
  motor_params.gimbal_6020_ang.kp = 20;
  motor_params.gimbal_6020_ang.ki = 0;
  motor_params.gimbal_6020_ang.kd_fb = 0;
  motor_params.gimbal_6020_ang.kd_ex = 0;
  motor_params.gimbal_6020_ang.k_ff = 0;
  motor_params.gimbal_6020_ang.max_out_value = 10000;
  motor_params.gimbal_6020_ang.min_out_value = -10000;
  motor_params.gimbal_6020_ang.limit_output = true;
  motor_params.gimbal_6020_ang.max_integral = 0.1;
  motor_params.gimbal_6020_ang.min_integral = -0.1;
  motor_params.gimbal_6020_ang.limit_integral = true;
  motor_params.gimbal_6020_ang.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_6020_ang.kd_able_error_range = 0;
  motor_params.gimbal_6020_ang.ki_able_error_range = 0;  
  

  
  /************** 电机PID参数 **************/


  /************** 任务频率 **************/


  // 四个底盘电机传感器数据获取任务与PID运算任务
  motor_params.chassis_motor_1.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_1_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_2.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_2_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_3.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_3_ang_vel.interval = 1e6f / 200.0f;

  motor_params.chassis_motor_4.interval = 1e6f / 200.0f;
  motor_params.chassis_motor_4_ang_vel.interval = 1e6f / 200.0f;
  
  //四个机械臂电机传感器数据获取
  
  
  //平移3508电机获取频率
   motor_params.shift_3508.interval =  1e6f / 200.0f;
   motor_params.shift_3508_ang.interval = 1e6f / 200.0f;
   motor_params.shift_3508_ang_vel.interval = 1e6f / 200.0f;
	 
	   
   //抬升3508电机获取频率
   motor_params.uplift1_3508.interval =  1e6f / 200.0f;
   motor_params.uplift1_3508_ang.interval = 1e6f / 200.0f;
   motor_params.uplift1_3508_ang_vel.interval = 1e6f / 200.0f;	
	
	 motor_params.uplift2_3508.interval =  1e6f / 200.0f;
   motor_params.uplift2_3508_ang.interval = 1e6f / 200.0f;
   motor_params.uplift2_3508_ang_vel.interval = 1e6f / 200.0f;
	 
	   
   //伸出3508电机获取频率
   motor_params.extend1_3508.interval =  1e6f / 200.0f;
   motor_params.extend1_3508_ang.interval = 1e6f / 200.0f;
   motor_params.extend1_3508_ang_vel.interval = 1e6f / 200.0f;	
	
	 motor_params.extend2_3508.interval =  1e6f / 200.0f;
   motor_params.extend2_3508_ang.interval = 1e6f / 200.0f;
   motor_params.extend2_3508_ang_vel.interval = 1e6f / 200.0f;
	
   //机械臂6020电机获取频率
   motor_params.gimbal_6020.interval =  1e6f / 200.0f;
   motor_params.gimbal_6020_ang.interval = 1e6f / 200.0f;
   motor_params.gimbal_6020_ang_vel.interval = 1e6f / 200.0f;
  
   //翻矿3508电机获取频率
   motor_params.overturn1_3508.interval =  1e6f / 200.0f;
   motor_params.overturn1_3508_ang.interval = 1e6f / 200.0f;
   motor_params.overturn1_3508_ang_vel.interval = 1e6f / 200.0f;	
	
	 motor_params.overturn2_3508.interval =  1e6f / 200.0f;
   motor_params.overturn2_3508_ang.interval = 1e6f / 200.0f;
   motor_params.overturn2_3508_ang_vel.interval = 1e6f / 200.0f;
	
  //DM电机获取频率
   motor_params.arm_dm_1.interval =  1e6f / 100.0f;
   motor_params.arm_dm_2.interval =  1e6f / 100.0f; 
	 motor_params.arm_dm_3.interval =  1e6f / 100.0f;  	  
  
  // 底盘任务
  motor_params.control_tasks_interval.chassis_task_interval = 1e6f / 200.0f;
  
  //机械臂任务
  motor_params.control_tasks_interval.arm_task_interval = 1e6f / 200.0f;
  
    //云台任务
  motor_params.control_tasks_interval.gimbal_task_interval = 1e6f / 200.0f;
  
  // LED 任务
  motor_params.control_tasks_interval.led_task_interval = 1e6f / 200.0f;

  // 裁判系统数据处理任务
  motor_params.control_tasks_interval.referee_system_task_interval = 1e6f / 100.0f;

  // CAN1 0x1FF地址发送任务
  motor_params.control_tasks_interval.can1_send_0x1ff_task_interval = 1e6f / 200.0f;

  // CAN2 0x1FF地址发送任务
  motor_params.control_tasks_interval.can2_send_0x1ff_task_interval = 1e6f / 200.0f;
  
  // CAN1 0x200地址发送任务
  motor_params.control_tasks_interval.can1_send_0x200_task_interval = 1e6f / 200.0f;

  // CAN2 0x200地址发送任务
  motor_params.control_tasks_interval.can2_send_0x200_task_interval = 1e6f / 200.0f;
	
  
     // CAN1 0x101地址发送任务
  motor_params.control_tasks_interval.can1_send_0x101_task_interval = 1e6f / 400.0f;
  
     // CAN1 0x102地址发送任务
  motor_params.control_tasks_interval.can1_send_0x102_task_interval = 1e6f / 400.0f;
	
     // CAN1 0x103地址发送任务
  motor_params.control_tasks_interval.can1_send_0x103_task_interval = 1e6f / 400.0f;    
	 
     // CAN1 0x32地址发送任务
  motor_params.control_tasks_interval.can1_send_0x32_task_interval = 1e6f / 100.0f;

		 // CAN1 0x145地址发送任务
  motor_params.control_tasks_interval.can1_send_0x145_task_interval = 1e6f / 100.0f;
  
  /************** 任务频率 **************/


}

