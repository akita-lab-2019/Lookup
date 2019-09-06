#include "TailController.h"

/**
 * コンストラクタ
 * @param tail_motor 尻尾モータ
 */
TailController::TailController(ev3api::Motor &tail_motor, PID *pid)
    : m_tail_motor(tail_motor), m_pid(pid)
{
}

/**
 * 初期化する
 */
void TailController::init()
{
    m_tail_motor.reset();
    float tail_pid_gain[3] = {2.3, 0.0, 0.6};
    m_pid->init(tail_pid_gain);
}

/**
 * 尻尾モータの目標角度を設定する
 * @param target_angle 尻尾の目標角度
 */
void TailController::setAngle(int target_angle)
{
    m_target_angle = target_angle;
}

/**
 * 尻尾モータの最高速度を設定する
 * @param target_angle 尻尾の最高速度
 */
void TailController::setMaxSpeed(int max_speed)
{
    m_max_speed = max_speed;
}

/**
 * 尻尾モータの最高速度を設定する
 * @param target_angle 尻尾の最高速度
 */
void TailController::setTailSpeed(int tail_speed)
{
    m_tail_motor.setPWM(tail_speed);
}

/**
 * 尻尾モータを制御する
 */
void TailController::control()
{
    int pwm = m_pid->calculate(m_target_angle, m_tail_motor.getCount());

    pwm = (pwm > m_max_speed) ? m_max_speed : pwm;
    pwm = (pwm < -m_max_speed) ? -m_max_speed : pwm;

    if (pwm == 0)
    {
        m_tail_motor.stop();
    }
    else
    {
        m_tail_motor.setPWM(pwm);
    }
}
