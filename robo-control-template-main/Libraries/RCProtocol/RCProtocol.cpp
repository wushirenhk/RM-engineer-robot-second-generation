/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   RCProtocol.cpp
** �ļ�˵����   ң������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "RCProtocol.h"
#include "RCProtocol_Backend.h"
#include "RCProtocol_DBUS.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/***********************************************************************
** �� �� ���� RCProtocol::init
** ����˵���� huart3��DMA��ʼ����ʵ������̨�������
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void RCProtocol::init(void)
{
  addBackend(new RCProtocol_DBUS(*this));//��DBUS���Ϊ�º�̨
  backend->init();//��ʼ��DBUS(��)
  //enable the DMA transfer for the receiver request
  //ʹ��DMA���ڽ���
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

  //enalbe idle interrupt
  //ʹ�ܿ����ж�
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  //disable DMA
  //ʧЧDMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }

  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
  //memory buffer 1

  //memory buffer 2

  //�ڴ滺����1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
  //�ڴ滺����2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
  //data length
  //���ݳ���
  hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
  //enable double memory buffer
  //ʹ��˫������
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

  //enable DMA
  //ʹ��DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

/***********************************************************************
** �� �� ���� RCProtocol::processByte
** ����˵���� ���ú�̨��processByte����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void RCProtocol::processByte(volatile const uint8_t *buf)
{
  if(backend != NULL)
  {
    backend->processByte(buf);
  }
}

/***********************************************************************
** �� �� ���� RCProtocol::addBackend
** ����˵���� ��ǰ̨����º�̨
**---------------------------------------------------------------------
** ��������� ��̨��ָ��backend0
** ���ز����� ��
***********************************************************************/
void RCProtocol::addBackend(RCProtocol_Backend *backend0)
{
  backend = backend0;
}

/***********************************************************************
** �� �� ���� RCProtocol::uninit
** ����˵���� ��ǰ̨���ú�̨��ȥ��ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void RCProtocol::uninit(void)
{
  backend->uninit();
}
