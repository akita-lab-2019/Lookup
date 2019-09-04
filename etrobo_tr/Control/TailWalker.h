/**
 *******************************************************************************
 ** 尻尾走行の計算を行う
 *******************************************************************************
 **/

#ifndef EV3_TAILWALKER_H_
#define EV3_TAILWALKER_H_

#include "ev3api.h"

class TailWalker
{
public:
    TailWalker();

    void init();
    void update();
    void setCommand(int forward, int turn);
    int8_t getPwmRight();
    int8_t getPwmLeft();

private:
    int m_forward;
    int m_turn;
    int8_t m_right_pwm;
    int8_t m_left_pwm;

    float K = 1;
};

#endif
