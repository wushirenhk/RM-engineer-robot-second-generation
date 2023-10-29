#ifndef __INERTIALSENSOR_BACKEND_H__
#define __INERTIALSENSOR_BACKEND_H__

#include "stdint.h"
#include "InertialSensor.h"
#include "Rotation.h"
#include "Filters.h"

// ���Դ�����backend�Ļ���
class InertialSensor_Backend
{
public:
  friend class InertialSensor;
  InertialSensor_Backend(InertialSensor &imu0);
  virtual bool init() = 0;
  virtual bool update() = 0;

protected:
  InertialSensor &imu;
  virtual void updateAccel(void) = 0; // ���¼��ٶ����ݣ���ѯ
  virtual void updateGyro(void) = 0;	// �������������ݣ���ѯ
  void correctAccel(void); // �������ٶ����ݣ���ѯ
  void correctGyro(void);	// �������������ݣ���ѯ
  void publishAccel(void);
  void publishGyro(void);
  virtual float getTemperature(void) = 0;
  virtual void calibGyro(uint32_t cnt) = 0;

  // ����imu��id
  uint16_t getId()
  {
    return id;
  }

  uint16_t id; // ��̨ID

  Vector3f gyro; // ���ٶ�����
  Vector3f accel; // ���ٶ�����
  
  LowPassFilter<Vector3f> accel_lpf;
  LowPassFilter<Vector3f> gyro_lpf;


  Vector3f gyro_offset; // ��������Ưֵ
  Vector3f gyro_calib; // �������������ֵ
};

#endif
