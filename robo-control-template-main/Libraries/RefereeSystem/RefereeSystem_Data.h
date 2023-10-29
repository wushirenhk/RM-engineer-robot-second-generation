#ifndef __REFERENCESYSTEM_DATA_H__
#define __REFERENCESYSTEM_DATA_H__

#include <stdio.h>
#include <stdarg.h>
#include "stdint.h"
#include "main.h"
#include "Scheduler_Common.h"

#define REFEREE_BUF_NUM 1024
#define REFEREE_BUF_LEN 1024
#define REFEREE_FIFO_BUF_NUM				8
#define REFEREE_FIFO_BUF_LEN 				1024



/* ����֡֡ͷ�ṹ�� */
typedef __PACKED_STRUCT
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

/* ----------------------------------��������֡�����ݶο��--------------------------------------- */
/**
  @brief  ����״̬���ݣ�0x0001��1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t game_type: 4;
  uint8_t game_progress: 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
  timeus_t timestamp;
} ext_game_status_t;

/**
   @brief ����������ݣ�0x0002���ڱ�����������
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t winner;
  timeus_t timestamp;
} ext_game_result_t;

/**
   @brief ������Ѫ�����ݣ�0x0003��1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
  timeus_t timestamp;
} ext_game_robot_HP_t;

/**
   @brief �����¼����ݣ�0x0101���¼��ı䷢��
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint32_t event_type;
  timeus_t timestamp;
} ext_event_data_t;

/**
   @brief ����վ������ʶ��0x0102	�����ı����
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
  timeus_t timestamp;
} ext_supply_projectile_action_t;

/**
   @brief ���о�����Ϣ��0x0104	���淢������
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t level;
  uint8_t foul_robot_id;
  timeus_t timestamp;
} ext_referee_warning_t;

/**
   @brief ��ǰ����������״̬��0x0201��10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;
  uint16_t shooter_id1_17mm_cooling_rate;
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output: 1;
  uint8_t mains_power_chassis_output: 1;
  uint8_t mains_power_shooter_output: 1;
  timeus_t timestamp;
} ext_game_robot_status_t;

/**
   @brief ʵʱ�����������ݣ�0x0202��50Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
  timeus_t timestamp;
} ext_power_heat_data_t;

/**
   @brief ������λ�ã�0x0203,10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  float x;
  float y;
  float z;
  float yaw;
  timeus_t timestamp;
} ext_game_robot_pos_t;

/**
   @brief ���������棺0x0204��״̬�ı����
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t power_rune_buff;
  timeus_t timestamp;
} ext_buff_t;

/**
   @brief �˺�״̬��0x0206���ܵ��˺�����
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint8_t armor_id: 4;
  uint8_t hurt_type: 4;
  timeus_t timestamp;
} ext_robot_hurt_t;

/**
   @brief ʵʱ�����Ϣ��0x0207���������
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt, recv_cnt_last;
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
  float bullet_speed_last;
  timeus_t timestamp;
} ext_shoot_data_t;

/**
   @brief �ӵ�ʣ�෢������0x0208��10Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
  timeus_t timestamp;
} ext_bullet_remaining_t;

/**
   @brief ������RFID״̬��0x0209��1Hz
*/
typedef __PACKED_STRUCT
{
  uint32_t recv_cnt;
  uint32_t rfid_status;
  timeus_t timestamp;
} ext_rfid_status_t;

/**
	 @brief �����ֿ�ʹ���Զ��������ͨ��ͼ����·���Ӧ�Ļ����˷�������:0x0302,30HZ
*/
typedef __PACKED_STRUCT
{
	uint8_t custom_robot_data_pitch[4]; //4bytes
	uint8_t custom_robot_data_roll[4];  //4bytes
	uint8_t custom_robot_data_yaw[4];   //4bytes
	uint8_t custom_robot_data_fowrwardback;    //1bytes
	uint8_t custom_robot_data_updown;  //1bytes
	uint8_t custom_robot_data_rightleft; //1bytes
	uint8_t custom_robot_data_left;  //1bytes

	uint8_t custom_robot_data_reserved[14]; //14bytes
	float Custom_yaw;
   float Custom_pitch;
   float Custom_roll;
}  ext_custom_robot_data_t;

/**
   @brief �����˲���ϵͳ��ȡ��״̬����
*/
typedef __PACKED_STRUCT
{
  ext_game_status_t game_status; //0x0001
  ext_game_result_t game_result; //0x0002
  ext_game_robot_HP_t game_robot_HP; //0x0003

  ext_event_data_t event_data; //0x0101
  ext_supply_projectile_action_t supply_projectile_action; //0x0102
  ext_referee_warning_t referee_warning; //0x0104

  ext_game_robot_status_t game_robot_status; //0x0201
  ext_power_heat_data_t power_heat_data; //0x0202
  ext_game_robot_pos_t game_robot_pos; //0x0203
  ext_buff_t buff; //0x0204

  ext_robot_hurt_t robot_hurt; //0x0206
  ext_shoot_data_t shoot_data; //0x0207
  ext_bullet_remaining_t bullet_remaining; //0x0208
  ext_rfid_status_t rfid_status; //0x0209
  ext_custom_robot_data_t custom_robot_data;//0x302

  uint32_t cmd_error_count[6]; //0-seq_num error; 1-frame_header & length error; 2-head_crc8 error; 3-package length error; 4-package_crc16 error; 5-cmd error
} RobotRefereeStatus_t;

/* ----------------------------------����ϵͳ�ͻ��˽�������--------------------------------------- */
/**
	 @brief �����˼佻������:0x0301,10HZ
*/
typedef __PACKED_STRUCT
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
	 @brief �����˼佻������:0x0301,10HZ
	 @brief �ͻ���ɾ��ͼ�Σ���������ID:0x0101
*/ 
typedef __PACKED_STRUCT
{
	uint8_t operate_type;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

/**
	 @brief ͼ�����ݣ�15�ֽ�
*/
typedef __PACKED_STRUCT
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	__packed union{
		__packed struct{
			uint32_t radius:10;
			uint32_t end_x:11;
			uint32_t end_y:11;
		} value;
		float float_value;
		int32_t int32_value;
	} graphic_config_3;
} graphic_data_struct_t;

/**
	 @brief �����˼佻������:0x0301,10HZ
	 @brief �ͻ��˻����ַ�����������ID��0x0110
*/
typedef __PACKED_STRUCT
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

/**
	 @brief �ͻ���UI��������
*/
typedef __PACKED_STRUCT
{
	
}ClientUIStatus_t;




/*---------------------DECLARES----------------------*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
