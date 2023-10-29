#ifndef PIDCONTROLTASK_H
#define PIDCONTROLTASK_H

#include "Task.h"
#include "stdint.h"
#include "PID_Controller.h"

class Robot;

class PIDControlTask : public Task_Base
{
public:
  friend class PIDMotorControlTask;
  friend class Motor_RM_PIDControlTask;
  friend class Motor_MC_ControlTask;
  friend class MainControlTask;
  friend class Robot;
  PIDControlTask(Robot &robot0, timeus_t interval_tick_us0 = 0) : Task_Base(robot0)
  {
    this->interval_tick_us = interval_tick_us0;
  }

  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

  // ����PID�����ṹ������
  void setPIDControllerParams(const PID_Params_t &params)
  {
    pid_controller.setParams(params);
  }
  // ����PID����ֵ
  void setPIDControllerExpect(float expect)
  {
    pid_controller.setExpect(expect);
  }
  // ����PID����ֵ
  void setPIDControllerFeedback(float feedback)
  {
    pid_controller.setFeedback(feedback);
  }
  // ����PIDǰ��ֵ
  void setPIDControllerFeedforward(float feedforward)
  {
    pid_controller.setFeedforward(feedforward);
  }
  // ����PID����ֵ
  void setPIDControllerInput(float expect, float feedback, float feedforward)
  {
    pid_controller.setInput(expect, feedback, feedforward);
  }
  float getOutput()
  {
    return pid_controller.getOutput();
  }
  float getLimitedOutput()
  {
    return pid_controller.getLimitedOutput();
  }
  void limitOutput(){
    pid_controller.limitOutput();
  }
  void clearPID()
  {
    pid_controller.clear();
  }
  // ����PID
  void calculatePID(timeus_t _dT_us,           //���ڣ�us��
                    float _expect,			       //����ֵ
                    float _feedback,           //����ֵ
                    float _feedforward = 0.0f  //ǰ��ֵ
                   )
  {
    pid_controller.calculateWithCPU(_dT_us, _expect, _feedback, _feedforward);
  }
  // ����PID
  void calculatePID(timeus_t _dT_us)     //���ڣ�us��
  {
    pid_controller.calculateWithCPU(_dT_us);
//    pid_controller.calculateWithCPU(_dT_us,
//                             pid_controller.getPidData().expect,
//                             pid_controller.getPidData().feedback,
//                             pid_controller.getPidData().feedforward
//                            );
  }
	void printPID(void);

protected:
  PID_Controller pid_controller;
};


#endif