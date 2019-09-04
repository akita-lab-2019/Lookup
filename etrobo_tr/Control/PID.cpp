#include "PID.h"

/**
 * コンストラクタ
 */
PID::PID()
{
}

/**
 * 初期化する
 * @param K[3] PID係数（0:P, 1:I, 2:D）
 */
void PID::init(float K[3])
{
    for (int i = 0; i < 3; i++)
    {
        m_K[i] = K[i];
    }

    m_integral = 0;
    m_diff[0] = 0;
    m_diff[1] = 0;
}

/**
 * PID計算を行う
 * 4ms周期で呼び出される前提
 * @param target_val 目標値
 * @param now_val    現在の値
 */
float PID::calculate(float target_val, float now_val)
{
    float dt = 0.004; // 制御周期は4ms

    m_diff[0] = target_val - now_val;
    m_integral += (m_diff[0] + m_diff[1]) * (dt / 2.0);

    m_team_val[P] = m_K[P] * m_diff[0];
    m_team_val[I] = m_K[I] * m_integral;
    m_team_val[D] = m_K[D] * ((m_diff[0] - m_diff[1]) / dt);

    // 次回演算用状態量保存処理
    m_diff[1] = m_diff[0];

    return m_team_val[P] + m_team_val[I] + m_team_val[D];
}

/**
 * 各項の値を取得する
 * 4ms周期で呼び出される前提
 * @param team 項の番号（0:P, 1:I, 2:D）
 * @return 各項の値
 */
float PID::getTeamValue(int team)
{
    return m_team_val[team];
}
