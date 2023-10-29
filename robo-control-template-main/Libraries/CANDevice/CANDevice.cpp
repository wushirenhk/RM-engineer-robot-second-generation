/***************************************************************************
**   					             ����������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   CanDevice.cpp
** �ļ�˵����   CAN�豸
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
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
** �� �� ���� CanDevice::init
** ����˵���� CAN_filter��ʼ��
**---------------------------------------------------------------------
** ��������� CAN_HandleTypeDef *_hcanx���ָ��
** ���ز����� ��
***********************************************************************/
void CanDevice::init(CAN_HandleTypeDef *_hcanx)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; // �����˲�
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; // 32λһ���˲�
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
** �� �� ���� CanDevice::send
** ����˵���� CAN���ͺ�����header�Զ��壩
**---------------------------------------------------------------------
** ��������� CAN_TxHeaderTypeDef *header, uint8_t *data����Ҫ���͵����ݣ�
** ���ز����� ��
***********************************************************************/
void CanDevice::send(CAN_TxHeaderTypeDef *header, uint8_t *data)
{
  uint32_t send_mail_box;

  HAL_CAN_AddTxMessage(hcanx, header, data, &send_mail_box);
}

/***********************************************************************
** �� �� ���� CanDevice::send
** ����˵���� can���ͺ�����ʹ����standard_tx_header��
**---------------------------------------------------------------------
** ��������� uint16_t std_id�����Ͷ����ID��, uint8_t *data����Ҫ���͵����ݣ�
** ���ز����� ��
***********************************************************************/
void CanDevice::send(uint16_t std_id, uint8_t *data)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header = standard_tx_header;
  tx_header.StdId = std_id;

  HAL_CAN_AddTxMessage(hcanx, &tx_header, data, &send_mail_box);
}

/***********************************************************************
** �� �� ���� CanDevice::addRxLink
** ����˵���� �����µ�CAN_Rx_Data_Pack_tָ��
**---------------------------------------------------------------------
** ��������� CAN_Rx_Data_Pack_t* addr
** ���ز����� ���ӳɹ�����true������ʧ�ܷ���false
***********************************************************************/
bool CanDevice::addRxLink(CAN_Rx_Data_Pack_t* addr)
{
	//�����Ŀ�Ѿ�����MAX_CAN_RX_ID_NUM���߿�ָ�룬����
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
** �� �� ���� CanDevice::addTxLink
** ����˵���� �����µ�can_tx_dataָ��
**---------------------------------------------------------------------
** ��������� 
** ���ز����� ���ӳɹ�����true������ʧ�ܷ���false
***********************************************************************/
bool CanDevice::addTxLink(
  uint16_t can_tx_id,
  uint8_t can_tx_dlc,
  uint8_t start_pos,
  uint8_t* data_p,
  uint8_t size
)
{
  // ����Ѿ�����MAX_CAN_TX_ID_NUM��Ŀ������
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
    // ������е�tx_link��������ͬid��
    if(can_tx_data[i].std_id == can_tx_id)
    {
      // dlc��ͬ������
      if(can_tx_data[i].dlc != can_tx_dlc)
      {
#ifdef DEBUG_MODE
        error.error_code.can_tx_dlc_mismatch = 1;
        error.Error_Handler();
#endif
      }

      for(int j = 0; j < size; j++)
      {
        // ����Խ�磬����
        if(start_pos + j >= can_tx_dlc)
        {
#ifdef DEBUG_MODE
          error.error_code.can_tx_data_out_of_range = 1;
          error.Error_Handler();
#endif
        }
        // id��dataָ���ظ��󶨣�����
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
        // ����tx_link
        else
        {
          can_tx_data[i].data[start_pos + j] = data_p + j;

        }
      }
      return true;
    }
  }

  // ����tx_link
  can_tx_data[can_tx_link_count].std_id = can_tx_id;
  can_tx_data[can_tx_link_count].dlc = can_tx_dlc;
  for(int j = 0; j < size; j++)
  {
    // ����Խ�磬����
    if(start_pos + j >= can_tx_dlc)
    {
#ifdef DEBUG_MODE
      error.error_code.can_tx_data_out_of_range = 1;
      error.Error_Handler();
#endif
    }
    // id��ͻ������
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
    // Ϊ�µ�tx_link��������
    else
    {
      can_tx_data[can_tx_link_count].data[start_pos + j] = data_p + j;
//      can_tx_data[can_tx_link_count].dlc = can_tx_dlc;
    }
  }
  can_tx_link_count++;
  return true;
}