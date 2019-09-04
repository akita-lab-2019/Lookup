#include "Odometer.h"

/**
 * コンストラクタ
 * @param wheel_L 左モータ
 * @param wheel_L 右モータ
 */
Odometer::Odometer(ev3api::Motor &wheel_L, ev3api::Motor &wheel_R)
    : m_wheel_L(wheel_L), m_wheel_R(wheel_R)
{
}

/**
 * 初期化する
 */
void Odometer::init()
{
    m_robot_pose[X] = 0;
    m_robot_pose[Y] = 0;
    m_robot_pose[YAW] = 0;
    m_robot_dis = 0;
}

/**
 * 計測する
 */
void Odometer::measure()
{
    // ホイールの回転角度を算出する
    m_wheel_deg[L] = m_wheel_L.getCount();
    m_wheel_deg[R] = m_wheel_R.getCount();

    calculate();
}

/**
 * オドメトリを計算する
 */
void Odometer::calculate()
{
    // ホイールの累計走行距離を計算
    float wheel_dis[2];
    wheel_dis[L] = m_wheel_deg[L] * TO_RAD * WHEEL_RADIUS;
    wheel_dis[R] = m_wheel_deg[R] * TO_RAD * WHEEL_RADIUS;

    // ロボットの累計走行距離を計算
    m_robot_dis = (wheel_dis[R] + wheel_dis[L]) / 2;
    m_robot_pose[YAW] = (wheel_dis[R] - wheel_dis[L]) / WHEEL_DIST * TO_DEG;

    // ロボットの座標を計算
    float dl = m_robot_dis - m_pre_robot_dis;
    m_robot_pose[X] = m_pre_robot_pose[X] + dl * cos(m_robot_pose[YAW] * TO_RAD);
    m_robot_pose[Y] = m_pre_robot_pose[Y] + dl * sin(m_robot_pose[YAW] * TO_RAD);

    // 次のループにデータを渡す
    m_pre_robot_dis = m_robot_dis;
    for (int i = 0; i < 3; i++)
        m_pre_robot_pose[i] = m_robot_pose[i];
}

/**
 * ロボットの位置を取得する
 * @parm 軸番号（X:0, Y:1, YAW:2）
 * @return ロボットの位置（X:[m], Y:[m], YAW[deg]）
 */
float Odometer::getRobotPose(int axis)
{
    return m_robot_pose[axis];
}

/**
 * ロボットの走行距離を取得
 * @return ロボットの走行距離[m]
 */
float Odometer::getRobotDistance()
{
    return m_robot_dis;
}

/**
 * ホイール角度を取得
 * @parm ホイール番号（L:0, R:1）
 * @return ホイール角度[deg]
 */
float Odometer::getWheelPose(int wheel)
{
    return m_wheel_deg[wheel];
}