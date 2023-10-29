#ifndef __VECTOR2_H__
#define __VECTOR2_H__

#include "stdint.h"
#include "math.h"
#include "CommonMath.h"

template <typename T>
class Vector2
{
public:
  T x, y;

  // ��ʼ������Ϊ������
  Vector2<T>()
  {
    x = 0;
    y = 0;
  }

  // ͨ����������
  Vector2<T>(const Vector2<T> &v0)
  {
    x = v0.x;
    y = v0.y;
  }

  // ͨ�����깹��
  Vector2<T>(const T x0, const T y0)
  {
    x = x0;
    y = y0;
  }
  
  // ͨ�������趨
  void set(const T x0, const T y0)
  {
    x = x0;
    y = y0;
  }
  
  // ͨ�������趨
  void set(const Vector2<T> &v)
  {
    x = v.x;
    y = v.y;
  }

  // �ж��Ƿ����
  bool isEqual(const Vector2<T> &v) const
  {
    return ((x == v.x) && (y == v.y));
  }

  // �ж��Ƿ񲻵�
  bool isNotEqual(const Vector2<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y));
  }

  // ����
  Vector2<T> negation(void) const
  {
    return Vector2<T>(-x, -y);
  }

  // ��
  Vector2<T> add(const Vector2<T> &v) const
  {
    return Vector2<T>(x + v.x, y + v.y);
  }

  // ��
  Vector2<T> subtract(const Vector2<T> &v) const
  {
    return Vector2<T>(x - v.x, y - v.y);
  }

  // �ȱ�����
  Vector2<T> uniformScaling(const T num) const
  {
    return Vector2<T>(x * num, y * num);
  }

  // ���ȱ�����
  Vector2<T> nonUniformScaling(const Vector2<T> &v)
  {
    return Vector2<T>(x * v.x, y * v.y);
  }

  // ���ȱ�����
  Vector2<T> nonUniformScaling(const T &x0, const T &y0)
  {
    return Vector2<T>(x * x0, y * y0);
  }

  // ���
  T dotProduct(const Vector2<T> &v) const
  {
    return x * v.x + y * v.y;
  }
  
  // ���
  T crossProduct(const Vector2<T> &v) const
  {
    return x * v.y - y * v.x;
  }

  // ����������v�ļн�[0, PI]
  T angle(const Vector2<T> &v) const
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

  // ����ת������v�������Сת��[-PI, PI]
  T anglePolar(const Vector2<T> &v) const
  {
    return atan2f(crossProduct(v), dotProduct(v));
  }
  
  // ����Ƿ���һ������ΪNaN
  bool isNaN(void) const
  {
    return isnan(x) || isnan(y);
  }

  // ����Ƿ���һ������ΪInf
  bool isInf(void) const
  {
    return isinf(x) || isinf(y);
  }

  // ����Ƿ�Ϊ������
  bool isZero(void) const
  {
    return cmt::isZero(x) && cmt::isZero(y);
  }

  // ��������Ϊ������
  void zero(void)
  {
    x = y = 0;
  }

  // ���������ģ��
  T length(void) const
  {
    return sqrtf(x * x + y * y);
  }
  
  // ���������ģ��ƽ��
  T length2(void) const
  {
    return x * x + y * y;
  }

  // ��׼��
  void normalize(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y);
    x *= recip_norm;
    y *= recip_norm;
  }
  
  // ���ر�׼���������
  Vector2<T> normalized(void)
  {
    T recip_norm = (T) cmt::invSqrt(x * x + y * y);
    Vector2<T> v = *this;
    v.x *= recip_norm;
    v.y *= recip_norm;
    return v;
  }

  // ��������*
  T operator *(const Vector2<T> &v) const
  {
    return x * v.x + y * v.y;
  }

  // �ȱ����Ÿ�ֵ�����*=
  Vector2<T> &operator *=(const T num)
  {
    x *= num;
    y *= num;
    return *this;
  }

  // �ȱ����Ÿ�ֵ�����/=
  Vector2<T> &operator /=(const T num)
  {
    x /= num;
    y /= num;
    return *this;
  }

  // ������ֵ�����-=
  Vector2<T> &operator -=(const Vector2<T> &v)
  {
    x -= v.x;
    y -= v.y;
    return *this;
  }

  // �ӷ���ֵ�����+=
  Vector2<T> &operator +=(const Vector2<T> &v)
  {
    x += v.x;
    y += v.y;
    return *this;
  }

  // �ȱ����������/
  Vector2<T> operator /(const T num) const
  {
    return Vector2<T>(x / num, y / num);
  }

  // �ȱ����������*
  Vector2<T> operator *(const T num) const
  {
    return Vector2<T>(x * num, y * num);
  }

  // ���������-
  Vector2<T> operator -(const Vector2<T> &v) const
  {
    return Vector2<T>(x - v.x, y - v.y);
  }

  // �ӷ������+
  Vector2<T> operator +(const Vector2<T> &v) const
  {
    return Vector2<T>(x + v.x, y + v.y);
  }

  // ���������-
  Vector2<T> operator -(void) const
  {
    return Vector2<T>(-x, -y);
  }

  // �ж��Ƿ���������==
  bool operator ==(const Vector2<T> &v) const
  {
    return ((x == v.x) && (y == v.y));
  }

  // �ж��Ƿ񲻵��������=
  bool operator !=(const Vector2<T> &v) const
  {
    return (!(x == v.x) || !(y == v.y));
  }

};

typedef Vector2<int16_t>                Vector2i;
typedef Vector2<uint16_t>               Vector2ui;
typedef Vector2<int32_t>                Vector2l;
typedef Vector2<uint32_t>               Vector2ul;
typedef Vector2<double>                 Vector2d;
typedef Vector2<float>                  Vector2f;


#endif
