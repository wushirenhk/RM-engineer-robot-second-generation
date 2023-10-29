/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   CanDevice.cpp
** 文件说明：   CAN设备
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "CanDevice.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"
#include "stdlib.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CanDevice::CanDevice()
{

}
/***********************************************************************
** 函 数 名： CanDevice::init
** 函数说明： CAN_filter初始化
**---------------------------------------------------------------------
** 输入参数： CAN_HandleTypeDef *_hcanx句柄指针
** 返回参数： 无
***********************************************************************/
void CanDevice::init(CAN_HandleTypeDef *_hcanx)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码滤波
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; // 32位一对滤波
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  hcanx = _hcanx;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  if(_hcanx == &hcan1)
  {

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  else if(_hcanx == &hcan2)
  {
		
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
}

/***********************************************************************
** 函 数 名： CanDevice::send
** 函数说明： CAN发送函数（header自定义）
**---------------------------------------------------------------------
** 输入参数： CAN_TxHeaderTypeDef *header, uint8_t *data（需要发送的数据）
** 返回参数： 无
***********************************************************************/
void CanDevice::send(CAN_TxHeaderTypeDef *header, uint8_t *data)
{
  uint32_t send_mail_box;

  HAL_CAN_AddTxMessage(hcanx, header, data, &send_mail_box);
}

/***********************************************************************
** 函 数 名： CanDevice::send
** 函数说明： can发送函数（使用了standard_tx_header）
**---------------------------------------------------------------------
** 输入参数： uint16_t std_id（发送对象的ID）, uint8_t *data（需要发送的数据）
** 返回参数： 无
***********************************************************************/
void CanDevice::send(uint16_t std_id, uint8_t *data)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header = standard_tx_header;
  tx_header.StdId = std_id;

  HAL_CAN_AddTxMessage(hcanx, &tx_header, data, &send_mail_box);
}

/***********************************************************************
** 函 数 名： CanDevice::addRxLink
** 函数说明： 添加新的CAN_Rx_Data_Pack_t指针
**---------------------------------------------------------------------
** 输入参数： CAN_Rx_Data_Pack_t* addr
** 返回参数： 添加成功返回true，添加失败返回false
***********************************************************************/
bool CanDevice::addRxLink(CAN_Rx_Data_Pack_t* addr)
{
	//如果数目已经大于MAX_CAN_RX_ID_NUM或者空指针，报错
  if(!addr || can_rx_link_count >= MAX_CAN_RX_ID_NUM)
  {
#ifdef DEBUG_MODE
    error.error_code.can_tx_id_num_out_of_range = 1;
    error.Error_Handler();
#endif
    return false;
  }
  can_rx_data_p[can_rx_link_count++] = addr;
  return true;
}

/***********************************************************************
** 函 数 名： CanDevice::addTxLink
** 函数说明： 添加新的can_tx_data指针
**---------------------------------------------------------------------
** 输入参数： 
** 返回参数： 添加成功返回true，添加失败返回false
***********************************************************************/
bool CanDevice::addTxLink(
  uint16_t can_tx_id,
  uint8_t can_tx_dlc,
  uint8_t start_pos,
  uint8_t* data_p,
  uint8_t size
)
{
  // 如果已经大于MAX_CAN_TX_ID_NUM数目，报错
  if(can_tx_link_count >= MAX_CAN_TX_ID_NUM)
  {
#ifdef DEBUG_MODE
    error.error_code.can_rx_id_num_out_of_range = 1;
    error.Error_Handler();
#endif
    return false;
  }
	
	
  for(int i = 0; i < can_tx_link_count; i++)
  {
    // 如果已有的tx_link中已有相同id的
    if(can_tx_data[i].std_id == can_tx_id)
    {
      // dlc不同，报错
      if(can_tx_data[i].dlc != can_tx_dlc)
      {
#ifdef DEBUG_MODE
        error.error_code.can_tx_dlc_mismatch = 1;
        error.Error_Handler();
#endif
      }

      for(int j = 0; j < size; j++)
      {
        // 数组越界，报错
        if(start_pos + j >= can_tx_dlc)
        {
#ifdef DEBUG_MODE
          error.error_code.can_tx_data_out_of_range = 1;
          error.Error_Handler();
#endif
        }
        // id中data指针重复绑定，报错
        else if(can_tx_data[i].data[start_pos + j] != NULL)
        {
#ifdef DEBUG_MODE
          error.error_code.can_tx_data_conflict.error = 1;
          error.error_code.can_tx_data_conflict.error_can_addr = can_tx_data[i].std_id;
          error.error_code.can_tx_data_conflict.error_data_pos = start_pos + j;
          if(this->hcanx == &hcan1)
            error.error_code.can_tx_data_conflict.can_device_id = 1;
          else if(this->hcanx == &hcan2)
            error.error_code.can_tx_data_conflict.can_device_id = 2;
          error.Error_Handler();
#endif
        }
        // 添加tx_link
        else
        {
          can_tx_data[i].data[start_pos + j] = data_p + j;

        }
      }
      return true;
    }
  }

  // 新增tx_link
  can_tx_data[can_tx_link_count].std_id = can_tx_id;
  can_tx_data[can_tx_link_count].dlc = can_tx_dlc;
  for(int j = 0; j < size; j++)
  {
    // 数组越界，报错
    if(start_pos + j >= can_tx_dlc)
    {
#ifdef DEBUG_MODE
      error.error_code.can_tx_data_out_of_range = 1;
      error.Error_Handler();
#endif
    }
    // id冲突，报错
    else if(can_tx_data[can_tx_link_count].data[start_pos + j] != NULL)
    {
#ifdef DEBUG_MODE
      error.error_code.can_tx_data_conflict.error = 1;
      error.error_code.can_tx_data_conflict.error_can_addr = can_tx_data[can_tx_link_count].std_id;
      error.error_code.can_tx_data_conflict.error_data_pos = start_pos + j;
      if(this->hcanx == &hcan1)
        error.error_code.can_tx_data_conflict.can_device_id = 1;
      else if(this->hcanx == &hcan2)
        error.error_code.can_tx_data_conflict.can_device_id = 2;
      error.Error_Handler();
#endif
    }
    // 为新的tx_link添加数据
    else
    {
      can_tx_data[can_tx_link_count].data[start_pos + j] = data_p + j;
//      can_tx_data[can_tx_link_count].dlc = can_tx_dlc;
    }
  }
  can_tx_link_count++;
  return true;
}
