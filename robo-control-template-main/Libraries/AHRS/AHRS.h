#ifndef __AHRS_H__
#define __AHRS_H__

#include "stdint.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix3.h"

class AHRS // ������
{
  friend class MahonyAHRS;
public:
  virtual void init(void) = 0; // ���麯��������û�ж���ֻ������
  virtual void update(Vector3f accel, Vector3f gyro) = 0; // ���麯��������û�ж���ֻ������
  Quaternion getIMUQuat(void) const; 
  Vector3f getIMUEuler(void) const;
  Matrix3f getIMUCoordinateSystem(void) const;

protected:
  Quaternion imu_quat;
  Vector3f imu_euler;
  Matrix3f imu_coordinate_system;
};

#endif
