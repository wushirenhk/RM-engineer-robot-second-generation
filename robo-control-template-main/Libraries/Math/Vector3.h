#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#include "stdint.h"
#include "math.h"
#include "CommonMath.h"

template <typename T>
class Vector3
{
public:
  T x, y, z;

  // ��ʼ������Ϊ������
  Vector3<T>()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  // ͨ����������
  Vector3<T>(const Vector3<T> &v0)
  {
    x = v0.x;
    y = v0.y;
    z = v0.z;
  }

  // ͨ�����깹��
  Vector3<T>(const T x0, const T y0, const T z0)
  {
    x = x0;
    y = y0;
    z = z0;
  }
  
  // ͨ�������趨
  void set(const T x0, const T y0, const T z0)
  {
    x = x0;
    y = y0;
    z = z0;
  }
  
  // ͨ�������趨
  void set(const Vector3<T> &v)
  {
    x = v.x;
    y = v.y;
    z = v.z;
  }

  // �ж��Ƿ����
  bool isEqual(const Vector3<T> &v) const
  {
    return ((x == v.x) && (y == v.y) && (z == v.z));
  }

  // �ж��Ƿ񲻵�
  bool isNotEqual(const Vector3<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y) || !(z == v.z));
  }

  // ����
  Vector3<T> negation(void) const
  {
    return Vector3<T>(-x, -y, -z);
  }

  // ��
  Vector3<T> add(const Vector3<T> &v) const
  {
    return Vector3<T>(x + v.x, y + v.y, z + v.z);
  }

  // ��
  Vector3<T> subtract(const Vector3<T> &v) const
  {
    return Vector3<T>(x - v.x, y - v.y, z - v.z);
  }

  // �ȱ�����
  Vector3<T> uniformScaling(const T num) const
  {
    return Vector3<T>(x * num, y * num, z * num);
  }

  // ���ȱ�����
  Vector3<T> nonUniformScaling(const Vector3<T> &v)
  {
    return Vector3<T>(x * v.x, y * v.y, z * v.z);
  }

  // ���ȱ�����
  Vector3<T> nonUniformScaling(const T &x0, const T &y0, const T &z0)
  {
    return Vector3<T>(x * x0, y * y0, z * z0);
  }

  // ���
  T dotProduct(const Vector3<T> &v) const
  {
    return x * v.x + y * v.y + z * v.z;
  }

  // ���
  Vector3<T> crossProduct(const Vector3<T> &v) const
  {
    Vector3<T> temp(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    return temp;
  }

  // ��ϻ� det([*this, v, v1])
  T mixedProduct(const Vector3<T> &v, const Vector3<T> &v1) const
  {
    Vector3<T> cross = crossProduct(v);
    return cross.dotProduct(v1);
  }

  // �����������н�
  T angle(const Vector3<T> &v) const
  {
    const T len = this->length() * v.length();
    if (len <= 0)
    {
      return 0.0f;
    }
    const T cosv = (dotProduct(v)) / len;
    if (cosv > 1 || cosv < -1)
    {
      return 0.0f;
    }
    return acosf(cosv);
  }

  // ����Ƿ���һ������ΪNaN
  bool isNaN(void) const
  {
    return isnan(x) || isnan(y) || isnan(z);
  }

  // ����Ƿ���һ������ΪInf
  bool isInf(void) const
  {
    return isinf(x) || isinf(y) || isinf(z);
  }

  // ����Ƿ�Ϊ������
  bool isZero(void) const
  {
    return cmt::isZero(x) && cmt::isZero(y) && cmt::isZero(z);
  }

  // ��������Ϊ������
  void zero(void)
  {
    x = y = z = 0;
  }

  // ���������ģ��
  T length(void) const
  {
    return sqrtf(x * x + y * y + z * z);
  }
  
  // ���������ģ��ƽ��
  T length2(void) const
  {
    return x * x + y * y + z * z;
  }

  // ��׼��
  void normalize(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y + z * z);
    x *= recip_norm;
    y *= recip_norm;
    z *= recip_norm;
  }
  
  // ���ر�׼���������
  Vector3<T> normalized(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y + z * z);
    Vector3<T> v = *this;
    v.x *= recip_norm;
    v.y *= recip_norm;
    v.z *= recip_norm;
    return v;
  }

  // ��һ���������ƹ�һ��������ת
  Vector3<T> rotateAboutAxis(Vector3<T> axis, T angle) const
  {
    T cosine = arm_cos_f32(angle);
    T sine = arm_sin_f32(angle);

    T dot = dotProduct(axis);
    Vector3<T> cross = crossProduct(axis);

    Vector3<T> dst;
    dst.x = cosine * x + (1 - cosine) * dot * axis.x - sine * cross.x;
    dst.y = cosine * y + (1 - cosine) * dot * axis.y - sine * cross.y;
    dst.z = cosine * z + (1 - cosine) * dot * axis.z - sine * cross.z;

    return dst;
  }

  // ��������%
  Vector3<T> operator %(const Vector3<T> &v) const
  {
    Vector3<T> temp(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    return temp;
  }
  
  // ��������*
  T operator *(const Vector3<T> &v) const
  {
    return x * v.x + y * v.y + z * v.z;
  }

  // �ȱ����Ÿ�ֵ�����*=
  Vector3<T> &operator *=(const T num)
  {
    x *= num;
    y *= num;
    z *= num;
    return *this;
  }

  // �ȱ����Ÿ�ֵ�����/=
  Vector3<T> &operator /=(const T num)
  {
    x /= num;
    y /= num;
    z /= num;
    return *this;
  }

  // ������ֵ�����-=
  Vector3<T> &operator -=(const Vector3<T> &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  // �ӷ���ֵ�����+=
  Vector3<T> &operator +=(const Vector3<T> &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  // �ȱ����������/
  Vector3<T> operator /(const T num) const
  {
    return Vector3<T>(x / num, y / num, z / num);
  }

  // �ȱ����������*
  Vector3<T> operator *(const T num) const
  {
    return Vector3<T>(x * num, y * num, z * num);
  }

  // ���������-
  Vector3<T> operator -(const Vector3<T> &v) const
  {
    return Vector3<T>(x - v.x, y - v.y, z - v.z);
  }

  // �ӷ������+
  Vector3<T> operator +(const Vector3<T> &v) const
  {
    return Vector3<T>(x + v.x, y + v.y, z + v.z);
  }

  // ���������-
  Vector3<T> operator -(void) const
  {
    return Vector3<T>(-x, -y, -z);
  }

  // �ж��Ƿ���������==
  bool operator ==(const Vector3<T> &v) const
  {
    return ((x == v.x) && (y == v.y) && (z == v.z));
  }

  // �ж��Ƿ񲻵��������=
  bool operator !=(const Vector3<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y) || !(z == v.z));
  }

};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<double>                 Vector3d;
typedef Vector3<float>                  Vector3f;


#endif
