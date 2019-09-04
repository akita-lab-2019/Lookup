
#ifndef EV3_APP_LINETRACER_H_
#define EV3_APP_LINETRACER_H_

#include <Clock.h>
#include "GuageManager.h"
#include "LineMonitor.h"
#include "GyroSensor.h"
#include "PID.h"
#include "Motor.h"
#include "Section.h"
#include "Odometer.h"
#include "TailController.h"
#include "TailWalker.h"
#include "stdlib.h"

class LineTracer
{
public:
    LineTracer(GuageManager *robot_info,
               Section *section,
               TailWalker *tail_walker,
               PID *pid,
               Balancer *balancer,
               ev3api::Motor &wheel_L,
               ev3api::Motor &wheel_R);

    void init();
    void update();
    void run();
    void setIsInverted(bool is_inverted);
    void setForward(float forward);
    void setCurvature(float curvature);
    void setPidParm(float pid_param[3]);
    void setColorTarget(float target);

    enum
    {
        L,
        R
    };

private:
    GuageManager *m_robot_info;
    Section *m_section;
    TailWalker *m_tail_walker;
    PID *m_pid;
    Balancer *m_balancer;
    ev3api::Motor &m_wheel_L;
    ev3api::Motor &m_wheel_R;

    void invertedRun(int forward_v, int turn_v);
    void tailRun(int forward_v, int turn_v);

    bool m_is_initialized;
    int m_forward;        // 前進指令速度[%]
    int m_turn;           // 旋回指令速度[%]
    float m_curvature;    // 曲率[m^-1]
    float m_pid_parm[3];  // PIDパラメータ
    float m_color_target; // カラーセンサの目標値
    bool m_is_inverted;   // 倒立走行の有無
    int m_pwm[2];         // ホイールの出力指令値
};

#endif // EV3_APP_LINETRACER_H_
