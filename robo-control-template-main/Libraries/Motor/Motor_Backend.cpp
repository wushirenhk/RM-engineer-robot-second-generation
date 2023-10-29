/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Motor_Backend.cpp
** 文件说明：   Motor Backend 基类
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "Motor_Backend.h"

/***********************************************************************
** 函 数 名： Motor_Backend::Motor_Backend()
** 函数说明： 构造函数，初始化引用变量motor=motor0
**---------------------------------------------------------------------
** 输入参数： 引用成员变量 &motor0
** 返回参数： 无
***********************************************************************/
Motor_Backend::Motor_Backend(Motor &motor0) : motor(motor0)
{
}

/***********************************************************************
** 函 数 名： Motor_Backend::publishMeasurement()
** 函数说明： 将测量结果com_msr publish到motor结构体的数组中
**---------------------------------------------------------------------
** 输入参数： Motor &motor0
** 返回参数： 无
***********************************************************************/
void Motor_Backend::publishMeasurement(void)
{
  motor.measurement[id] = com_msr;
}

