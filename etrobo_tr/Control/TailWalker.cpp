
#include "TailWalker.h"

/**
 * コンストラクタ
 */
TailWalker::TailWalker()
{
}

/**
 * 尻尾走行に必要なものをリセットする
 */
void TailWalker::init()
{
}

/**
 * 尻尾走行する
 */
void TailWalker::update()
{
    m_left_pwm = m_forward + K * m_turn;
    m_right_pwm = m_forward - K * m_turn;

    if (m_left_pwm > 100)
    {
        m_left_pwm = 100;
    }
    else if (m_left_pwm < -100)
    {
        m_left_pwm = -100;
    }

    if (m_right_pwm > 100)
    {
        m_right_pwm = 100;
    }
    else if (m_right_pwm < -100)
    {
        m_right_pwm = -100;
    }
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void TailWalker::setCommand(int forward, int turn)
{
    m_forward = forward;
    if (m_forward > 100)
    {
        m_forward = 100;
    }
    else if (m_forward < -100)
    {
        m_forward = -100;
    }

    m_turn = turn;
    if (m_turn > 100)
    {
        m_turn = 100;
    }
    else if (m_turn < -100)
    {
        m_turn = -100;
    }
}

/**
 * 右車輪のPWM値を取得する
 * @return 右車輪のPWM値
 */
int8_t TailWalker::getPwmRight()
{
    return m_right_pwm;
}

/**
 * 左車輪のPWM値を取得する
 * @return 左車輪のPWM値
 */
int8_t TailWalker::getPwmLeft()
{
    return m_left_pwm;
}
