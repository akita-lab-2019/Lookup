/**
 *******************************************************************************
 ** 倒立走行の計算を行う
 ** 倒立振子制御ライブラリを内部に移植してある
 *******************************************************************************
 **/

#ifndef EV3_UNIT_BALANCERCPP_H_
#define EV3_UNIT_BALANCERCPP_H_

#include "ev3api.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((void *)0)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((void)0)
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm) ((void *)0)
#endif

class Balancer
{
public:
    Balancer();

    void init(float offset);
    void update(int angle, int rwEnc, int lwEnc, int battery);
    void setCommand(int forward, int turn);
    int8_t getPwmRight();
    int8_t getPwmLeft();

private:
    float CMD_MAX = 100.0F;
    float DEG2RAD = 0.01745329238F;
    float EXEC_PERIOD = 0.00400000000F;

    int cancelBacklash(int8_t pwm, int enc);

    void balance_init(void);
    void balance_control(float args_cmd_forward,
                         float args_cmd_turn,
                         float args_gyro,
                         float args_gyro_offset,
                         float args_theta_m_l,
                         float args_theta_m_r,
                         float args_battery,
                         signed char *ret_pwm_l,
                         signed char *ret_pwm_r);

    int mForward;
    int mTurn;
    int mOffset;
    int8_t mRightPwm;
    int8_t mLeftPwm;
};

#endif // EV3_UNIT_BALANCERCPP_H_
