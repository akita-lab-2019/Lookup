/**
 *******************************************************************************
 ** PID計算を行う
 *******************************************************************************
 **/

#ifndef EV3_PID_H_
#define EV3_PID_H_

class PID
{
public:
    PID();

    void init(float K[3]);
    float calculate(float target_val, float now_val);
    float getTeamValue(int team);

    enum
    {
        P,
        I,
        D
    };

private:
    float m_K[3] = {0};        // 各項の係数
    float m_team_val[3] = {0}; // 各項の値
    float m_integral = 0;      // 積分値
    float m_diff[2] = {0};     // 0: 現在, 1: 過去
};

#endif
