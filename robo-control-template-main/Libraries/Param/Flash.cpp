/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Flash.cpp
** �ļ�˵����   STM32�ڲ�Flash��ȡ����
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "Flash.h"

/***********************************************************************
** �� �� ���� flash_ger_sector()
** ����˵���� ��ȡflash��ַ���ڵ�������
**---------------------------------------------------------------------
** ��������� flash��ַ
** ���ز����� ��
***********************************************************************/
uint32_t flash_getSector(uint32_t address)
{
  if(address < ADDR_FLASH_SECTOR_1)
  {
    return FLASH_SECTOR_0;
  }
  else if(address < ADDR_FLASH_SECTOR_2)
  {
    return FLASH_SECTOR_1;
  }
  else if(address < ADDR_FLASH_SECTOR_3)
  {
    return FLASH_SECTOR_2;
  }
  else if(address < ADDR_FLASH_SECTOR_4)
  {
    return FLASH_SECTOR_3;
  }
  else if(address < ADDR_FLASH_SECTOR_5)
  {
    return FLASH_SECTOR_4;
  }
  else if(address < ADDR_FLASH_SECTOR_6)
  {
    return FLASH_SECTOR_5;
  }
  else if(address < ADDR_FLASH_SECTOR_7)
  {
    return FLASH_SECTOR_6;
  }
  else if(address < ADDR_FLASH_SECTOR_8)
  {
    return FLASH_SECTOR_7;
  }
  else if(address < ADDR_FLASH_SECTOR_9)
  {
    return FLASH_SECTOR_8;
  }
  else if(address < ADDR_FLASH_SECTOR_10)
  {
    return FLASH_SECTOR_9;
  }
  else if(address < ADDR_FLASH_SECTOR_11)
  {
    return FLASH_SECTOR_10;
  }
  else
  {
    return FLASH_SECTOR_11;
  }
}

/***********************************************************************
** �� �� ���� flash_erase_address()
** ����˵���� ����flash
**---------------------------------------------------------------------
** ��������� flash��ʼ��ַ,����������
** ���ز����� ��
***********************************************************************/
void flash_eraseAddress(uint32_t address, uint16_t num)
{
  FLASH_EraseInitTypeDef flash_erase;
  uint32_t error;

  flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  flash_erase.Sector = flash_getSector(address);
  flash_erase.NbSectors = num;
  flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&flash_erase, &error);
  HAL_FLASH_Lock();
}

/***********************************************************************
** �� �� ���� flash_write_address()
** ����˵���� ��flashд����
**---------------------------------------------------------------------
** ��������� д��flah����ʼ��ַ,д�����ݵ�Դ��ַ,д��32λ���ݵĸ���
** ���ز����� ��
***********************************************************************/
void flash_writeAddress(uint32_t address, uint32_t *buf, uint16_t num)
{
  uint16_t i;

  HAL_FLASH_Unlock();

  for(i = 0; i < num; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 4 * i, buf[i]);
  }

  HAL_FLASH_Lock();
}

/***********************************************************************
** �� �� ���� flash_read_address()
** ����˵���� ��flash������
**---------------------------------------------------------------------
** ��������� ��ȡflash����ʼ��ַ,�����Ŀ���ַ,��ȡ32λ���ݵĸ���
** ���ز����� ��
***********************************************************************/
void flash_readAddress(uint32_t address, uint32_t *buf, uint16_t num)
{
  memcpy(buf, (void*)address, num * 4);
}
