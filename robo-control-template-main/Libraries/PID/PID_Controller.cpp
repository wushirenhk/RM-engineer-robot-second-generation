/***************************************************************************
**   					             ����������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   PID_Controller.cpp
** �ļ�˵����   PID ������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-18
**	 1.1							   ����ע��						     ����Դ     	     2022-12-10
***************************************************************************/

#include "PID_Controller.h"
#include "CommonMath.h"
#include "Task.h"
#include "Error.h"

/***********************************************************************
** �� �� ���� PID_Controller::PID_Controller()
** ����˵���� ����PID�ļӷ�����Ϊ��ͨ�ӷ�
**---------------------------------------------------------------------
** ��������� �����������ָ�롢�������ָ�롢CAN���ݰ�ָ�롢��ʱ��us��
** ���ز����� ��
***********************************************************************/
PID_Controller::PID_Controller()
{
  pid_params.add = &cmt::simple_adder_instance_f;
}

/***********************************************************************
** �� �� ���� PID_Controller::PID_Controller
** ����˵���� PID������ʼ���Ĺ��캯��
**---------------------------------------------------------------------
** ��������� _PID_Params_t �ṹ��Ĳ���
** ���ز����� ��
***********************************************************************/
PID_Controller::PID_Controller(PID_Type _type_selection,
                               cmt::AdvancedAdd_Base<float> *adder,
                               float _kp, float _ki,
                               float _kd_fb, float _kd_ex, float _k_ff,
                               float _max_out_value, float _min_out_value, bool _limit_output,
                               float _max_integral, float _min_integral, bool _limit_integral,
                               float _interval,
                               float _ki_able_error_range,
                               float _kd_able_error_range
                              )
{
  pid_params.type_selection = _type_selection;
  pid_params.kp = _kp;
  pid_params.ki = _ki;
  pid_params.kd_fb = _kd_fb;
  pid_params.kd_fb = _kd_ex;
  pid_params.k_ff = _k_ff;
  pid_params.min_integral = _min_integral;
  pid_params.max_integral = _max_integral;
  pid_params.ki_able_error_range = _ki_able_error_range;
  pid_params.kd_able_error_range = _kd_able_error_range;
  pid_params.max_out_value = _max_out_value;
  pid_params.min_out_value = _min_out_value;
  pid_params.add = adder;
  pid_params.interval = _interval;
}


/***********************************************************************
** �� �� ���� PID_Controller::PID_Controller
** ����˵���� ����PID_Data_t�ṹ�壬PID_Params_t�ṹ��Ĺ��캯��
**---------------------------------------------------------------------
** ��������� _PID_Params_t �ṹ��Ĳ���
** ���ز����� ��
***********************************************************************/
PID_Controller::PID_Controller(PID_Data_t &_pid_data, PID_Params_t &_pid_params)
{
  pid_data = _pid_data;
  pid_params = _pid_params;
}


/***********************************************************************
** �� �� ���� PID_Controller::PID_Controller
** ����˵���� �������캯��
**---------------------------------------------------------------------
** ��������� PID_Controller��Ķ���
** ���ز����� ��
***********************************************************************/
PID_Controller::PID_Controller(const PID_Controller & _pid_controller)
{
  this->pid_data = _pid_controller.pid_data;
  this->pid_params = _pid_controller.pid_params;
}

/***********************************************************************
** �� �� ���� PID_Controller::setParams
** ����˵���� ����PID_Params_t�ṹ�����
**---------------------------------------------------------------------
** ��������� PID_Controller��Ķ���
** ���ز����� ��
***********************************************************************/
void PID_Controller::setParams(PID_Type _type_selection,
                               cmt::AdvancedAdd_Base<float> *adder,
                               float _kp, float _ki,
                               float _kd_fb, float _kd_ex, float _k_ff,
                               float _max_out_value, float _min_out_value, bool _limit_output,
                               float _max_integral, float _min_integral, bool _limit_integral,
                               float _interval,
                               float _ki_able_error_range,
                               float _kd_able_error_range

                              )
{
  pid_params.type_selection = _type_selection;
  pid_params.kp = _kp;
  pid_params.ki = _ki;
  pid_params.kd_fb = _kd_fb;
  pid_params.kd_fb = _kd_ex;
  pid_params.k_ff = _k_ff;
  pid_params.min_integral = _min_integral;
  pid_params.max_integral = _max_integral;
  pid_params.ki_able_error_range = _ki_able_error_range;
  pid_params.kd_able_error_range = _kd_able_error_range;
  pid_params.max_out_value = _max_out_value;
  pid_params.min_out_value = _min_out_value;
  pid_params.add = adder;
  pid_params.interval = _interval;
}

/***********************************************************************
** �� �� ���� PID_Controller::setParams
** ����˵���� ��������
**---------------------------------------------------------------------
** ��������� _pid_params�ṹ��
** ���ز����� ��
***********************************************************************/
void PID_Controller::setParams(const PID_Params_t &_pid_params)
{
  pid_params = _pid_params;
}

/***********************************************************************
** �� �� ���� PID_Controller::clear
** ����˵���� pid_data�ṹ��ĳ�Ա����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void PID_Controller::clear()
{
  pid_data.expect            = 0;            // ����
  pid_data.d_expect          = 0;            // ��������
  pid_data.dd_expect         = 0;            // ��������������
  pid_data.last_expect       = 0;            // �ϴ�����
  pid_data.last_d_expect     = 0;            // �ϴΣ�����������

  pid_data.feedback          = 0;            // ����
  pid_data.d_feedback        = 0;            // ��������
  pid_data.dd_feedback       = 0;            // ��������������
  pid_data.last_feedback     = 0;            // �ϴη���
  pid_data.last_d_feedback   = 0;            // �ϴΣ�����������

  pid_data.feedforward       = 0;            // ǰ��

  pid_data.error             = 0;            // ���
  pid_data.last_error        = 0;            // �ϴ����

  pid_data.last_dT_s         = 0;            // �ϴ�ʱ������
  pid_data.last_freq         = 0;            // �ϴ�Ƶ�ʣ��ÿռ任ʱ�䣩

  pid_data.integral          = 0;            // ������

  pid_data.out               = 0;            // ���ֵ

}

/***********************************************************************
** �� �� ���� PID_Controller::calculateWithCPU
** ����˵���� PID���㺯��
**---------------------------------------------------------------------
** ��������� �������ڣ�����ֵ������ֵ��ǰ��ֵ
** ���ز����� ��
***********************************************************************/
float PID_Controller::calculateWithCPU(
  timeus_t _dT_us,     //���ڣ�us��
  float _expect,				//����ֵ
  float _feedback,     //����ֵ
  float _feedforward	//ǰ��ֵ
)
{
  pid_data.expect = _expect;
  pid_data.feedback = _feedback;
  pid_data.feedforward = _feedforward;
  float _dT_s = safeDiv(_dT_us, 1e6f, 0);
  float _freq = safeDiv(1, _dT_s, 0);
  float _kp = 0.0f;
  float _ki = 0.0f;
  float _kd_fb = 0.0f;
  float _kd_ex = 0.0f;
  float _k_ff = 0.0f;

#ifdef DEBUG_MODE
	//�ж�PID�ӷ�ָ���Ƿ��
  if(pid_params.add == NULL)
  {
    error.error_code.pid_controller_adder_not_initialized  = 1;
    error.Error_Handler();
  }
#endif
#define add (*pid_params.add)

  switch(pid_params.type_selection)
  {
  case 0://����ʽ�����㹫ʽOUT = kp*(e(k)-e(k-1))+ki*(e(k)*dt)+kd_ex*[(x(k)-x(k-1))-(x(k-1)-x(k-2))]/dt-kd_fb*[(y(k)-y(k-1))-(y(k-1)-y(k-2))]/dt+k_ff*feedforward
		
    // ΢���� _kd_ex 
    pid_data.d_expect = add(pid_data.expect, -pid_data.last_expect);
    _kd_ex = pid_data.d_expect * pid_params.kd_ex * _freq - pid_data.last_d_expect * pid_params.kd_ex * pid_data.last_freq;

    pid_data.d_feedback = add(pid_data.feedback, - pid_data.last_feedback);
    _kd_fb = pid_data.d_feedback * pid_params.kd_fb * _freq - pid_data.last_d_feedback * pid_params.kd_fb * pid_data.last_freq;

    _kd_ex -= _kd_fb;

    // ������ _kp
    pid_data.error = add(pid_data.expect, - pid_data.feedback);
    _kp = pid_params.kp * (pid_data.error - pid_data.last_error);

    // ������ _ki
    _ki = pid_data.error * _dT_s * pid_params.ki;

    // ǰ���� _k_ff
    _k_ff = pid_data.feedforward * pid_params.k_ff;

    // ���
    _kp += _kd_ex;
    _ki += _kp;

    // ����޷�
    if(pid_params.limit_output)
      pid_data.out = limit(pid_data.out + _ki + _k_ff, pid_params.min_out_value, pid_params.max_out_value);
    else
      pid_data.out += _ki + _k_ff;

    pid_data.last_expect = pid_data.expect;
    pid_data.last_feedback = pid_data.feedback;
    pid_data.last_d_feedback = pid_data.d_feedback;
    pid_data.last_d_expect = pid_data.d_expect;
    pid_data.last_dT_s = _dT_s;
    pid_data.last_freq = _freq;
    pid_data.last_error = pid_data.error;

    break;

  case 1: // λ��ʽ�����㹫ʽOUT = kp*e(k)+ki*(��(e(k)*dt))+kd_ex*(x(k)-x(k-1))/dt-kd_fb*(y(k)-y(k-1))/dt+k_ff*feedforward

    // ΢���� _kd_ex
    pid_data.d_expect = add(pid_data.expect, - pid_data.last_expect);
    _kd_ex = pid_data.d_expect * pid_params.kd_ex * _freq;

    pid_data.d_feedback = add(pid_data.feedback, - pid_data.last_feedback);
    _kd_fb = pid_data.d_feedback * pid_params.kd_fb * _freq;

    _kd_ex -= _kd_fb;

    // ������ _kp
    pid_data.error = add(pid_data.expect, - pid_data.feedback);
    _kp = pid_params.kp * pid_data.error;

    // ������ _ki
    _ki = pid_data.error * _dT_s;
//    pid_data.integral += _ki;

    // �����޷�
    if(pid_params.limit_integral)
      pid_data.integral = limit(pid_data.integral + _ki, pid_params.min_integral, pid_params.max_integral);
    else
      pid_data.integral += _ki;

    _ki = pid_data.integral * pid_params.ki;

    // ǰ���� _k_ff
    _k_ff = pid_data.feedforward * pid_params.k_ff;


    // ���
    _kp += _kd_ex;
    _ki += _kp;

    // ����޷�
    if(pid_params.limit_output)
      pid_data.out = limit(_ki + _k_ff, pid_params.min_out_value, pid_params.max_out_value);
    else
      pid_data.out = _ki + _k_ff;


    pid_data.last_expect = pid_data.expect;
    pid_data.last_feedback = pid_data.feedback;
    pid_data.last_d_feedback = pid_data.d_feedback;
    pid_data.last_d_expect = pid_data.d_expect;

    break;
  default:
    break;
  }
  return pid_data.out;
}
