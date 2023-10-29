#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"
#include "arm_math.h"
#include "Task.h"
#include "CommonMath.h"

typedef enum
{
  PID_DELTA = 0,//����ʽ
  PID_ABSOLUTE = 1//λ��ʽ������ʽ��
} PID_Type;

//PID����ʱ��ϵ��
typedef struct _PID_Params_t
{

  PID_Type type_selection;            // �������

  float ki_able_error_range;          // ki��������Ϊ0�رմ˹���
  float kd_able_error_range;          // kd��������Ϊ0�رմ˹���

  float kp;                           // ����ϵ��
  float ki;                           // ����ϵ��
  float kd_fb;                        // ΢������ϵ��
  float kd_ex;                        // ΢��ϵ��
  float k_ff;                         // ǰ��ϵ��

  float max_out_value;                // ����޷�
  float min_out_value;                // ����޷�
  bool limit_output;                  // ��������޷�

  float max_integral;                 // �����޷�
  float min_integral;                 // �����޷�
  bool limit_integral;                // ���������޷�

  cmt::AdvancedAdd_Base<float> *add;       // PID�еļӼ������㷽ʽ����������Ӧ�Լ������ڡ�PIλ�õ�����
  
  float interval;

} PID_Params_t;

typedef struct _PID_Data_t
{

  float	expect;                       // ����
  float d_expect;                     // ��������
  float dd_expect;                    // ��������������
  float last_expect;                  // �ϴ�����
  float last_d_expect;                // �ϴΣ�����������

  float feedback;                     // ����
  float d_feedback;                   // ��������
  float dd_feedback;                  // ��������������
  float last_feedback;                // �ϴη���
  float last_d_feedback;              // �ϴΣ�����������

  float feedforward;                  // ǰ��

  float error;                        // ���
  float last_error;                   // �ϴ����

  float last_dT_s;                    // �ϴ�ʱ������
  float last_freq;                    // �ϴ�Ƶ��

  float integral;                     // ������

  float out;                          // ���ֵ

} PID_Data_t;


class PID_Controller//������Ҫ�ǽṹ��PID_Data_t��PID_Params_t
{
public:
  PID_Controller();//���캯��
  PID_Controller(PID_Type _type_selection,//�вι��캯��
                 cmt::AdvancedAdd_Base<float> *adder,
                 float _kp, float _ki,
                 float _kd_fb, float _kd_ex, float _k_ff,
                 float _max_out_value, float _min_out_value, bool _limit_output,
                 float _max_integral, float _min_integral, bool _limit_integral,
                 float _interval,
                 float _ki_able_error_range = 0,
                 float _kd_able_error_range = 0
                );
								 
								 
  PID_Controller(PID_Data_t &_pid_data, PID_Params_t &_pid_params);
	//�������캯��
  PID_Controller(const PID_Controller & _pid_controller);

	//������������pid_data
  PID_Data_t getPidData()
  {
    return pid_data;
  }
	//������������pid_params
  PID_Params_t getPidParams()
  {
    return pid_params;
  }

  void setParams(PID_Type _type_selection,
                 cmt::AdvancedAdd_Base<float> *adder,
                 float _kp, float _ki,
                 float _kd_fb, float _kd_ex, float _k_ff,
                 float _max_out_value, float _min_out_value, bool _limit_output,
                 float _max_integral, float _min_integral, bool _limit_integral,
                 float _interval,
                 float _ki_able_error_range = 0,
                 float _kd_able_error_range = 0
                );
  void setParams(const PID_Params_t &_pid_params);

  float calculateWithCPU(
    timeus_t dT_us,      //���ڣ�us��
    float _expect,				//����ֵ
    float _feedback,     //����ֵ
    float _feedforward = 0.0f	  //ǰ��ֵ
  );
		
	//����PID
  float calculateWithCPU(timeus_t dT_us)      //���ڣ�us��
  {
    return calculateWithCPU(dT_us,
                            pid_data.expect,
                            pid_data.feedback,
                            pid_data.feedforward
                           );
  }
	//���û����޷�ֵ
  void setIntegralLimit(float min, float max)
  {
    pid_params.min_integral = min;
    pid_params.max_integral = max;
  }
	//��������޷�ֵ
  void setOutputLimit(float min, float max)
  {
    pid_params.max_out_value = max;
    pid_params.min_out_value = min;
  }
	//��������
  void setExpect(float expect)
  {
    pid_data.expect = expect;
  }
	//���÷���ֵ
  void setFeedback(float feedback)
  {
    pid_data.feedback = feedback;
  }
	//����ǰ��ֵ
  void setFeedforward(float feedforward)
  {
    pid_data.feedforward = feedforward;
  }
	//�������루����+����+ǰ����
  void setInput(float expect, float feedback, float feedforward)
  {
    pid_data.expect = expect;
    pid_data.feedback = feedback;
    pid_data.feedforward = feedforward;
  }
	//����PID���ֵ
  float getOutput()
  {
    return pid_data.out;
  }
	//pid_data�ṹ��ĳ�Ա����
  void clear();
	
	//��������޷���Ľ��
  float getLimitedOutput()
  {
    return limit(pid_data.out, pid_params.min_out_value, pid_params.max_out_value);
  }
	//��������޷���Ľ��
  void limitOutput(void)
  {
    pid_data.out = limit(pid_data.out, pid_params.min_out_value, pid_params.max_out_value);
  }

	//����pid_data�ĵ�ַ
  PID_Data_t *getPIDDataPointer()
  {
    return &pid_data;
  }

	//����pid_params�ĵ�ַ
  PID_Params_t *getPIDParamsPointer()
  {
    return &pid_params;
  }

protected:
  PID_Data_t pid_data;
  PID_Params_t pid_params;

};

#endif
