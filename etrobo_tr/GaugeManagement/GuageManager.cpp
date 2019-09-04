#include "GuageManager.h"

/**
 * コンストラクタ
 */
GuageManager::GuageManager(ev3api::Clock &clock,
                           ev3api::GyroSensor &gyro_sensor,
                           ev3api::SonarSensor &sonar_sensor,
                           LineMonitor *line_monitor,
                           Odometer *odometer)
    : m_clock(clock),
      m_gyro_sensor(gyro_sensor),
      m_sonar_sensor(sonar_sensor),
      m_line_monitor(line_monitor),
      m_odometer(odometer)
{
}

/**
 * 初期化する
 */
void GuageManager::init()
{
}

/**
 * 情報を更新する
 */
float pre_time;
void GuageManager::update()
{
    m_odometer->measure();
    m_runTime = (float)m_clock.now() / 1000.0;
    m_battery = ev3_battery_voltage_mV() / 1000.0;
    m_brightness = m_line_monitor->getBrightness();
    m_pitch_vel = m_gyro_sensor.getAnglerVelocity();
    m_wheel_pos[L] = m_odometer->getWheelPose(Odometer::L);
    m_wheel_pos[R] = m_odometer->getWheelPose(Odometer::R);
    m_robot_pos[X] = m_odometer->getRobotPose(Odometer::X);
    m_robot_pos[Y] = m_odometer->getRobotPose(Odometer::Y);
    m_robot_pos[YAW] = m_odometer->getRobotPose(Odometer::YAW);
    m_robot_dis = m_odometer->getRobotDistance();
    m_sonar_distance = (float)m_sonar_sensor.getDistance() / 100.0;
}

void GuageManager::setCourse(int course)
{
    m_course = course;
}

int GuageManager::getCourse()
{
    return m_course;
}

/**
 * 走行開始からの時間を取得する
 * @return 走行開始からの時間[sec]
 */
float GuageManager::getRunTime()
{
    return m_runTime;
}

/**
 * バッテリー電圧を取得する
 * @return バッテリー電圧[V]
 */
float GuageManager::getBatteryVoltage()
{
    return m_battery;
}

/**
 * カラーセンサからの輝度を取得する
 * @return カラーセンサから取得した輝度
 */
int GuageManager::getBrightness()
{
    return m_brightness;
}

/**
 * pitchの角速度を取得する
 * @return pitchの角速度[deg/rad]
 */
int GuageManager::getPitchVel()
{
    return m_pitch_vel;
}

/**
 * 車輪の角位置を取得する
 * @parm 車輪番号（左:0, 右:1）
 * @return 車輪の角位置（左[deg], 右[deg]）
 */
int GuageManager::getWheelPos(int wheel)
{
    return m_wheel_pos[wheel];
}

/**
 * 尻尾モータの角位置を取得する
 * @return 尻尾モータの角位置[deg]
 */
float GuageManager::getTailPos()
{
    return m_tail_pos;
}

/**
 * カラーセンサから取得した輝度と目標値との偏差を取得する
 * @return カラーセンサから取得した輝度と目標値との偏差
 */
int GuageManager::getBrightnessGap(int target)
{
    return (int)m_line_monitor->getGap(target);
    ;
}

/**
 * ロボットの座標を取得する
 * @parm 軸番号（X:0, Y:1, YAW:2）
 * @return ロボットの座標（x[m], y[m], yaw[deg]）
 */
float GuageManager::getRobotPos(int axis)
{
    return m_robot_pos[axis];
}

/**
 * 走行距離を取得する
 * @return 走行距離[m]
 */
float GuageManager::getRobotDis()
{
    return m_robot_dis;
}

/**
 * ソナーセンサからの距離を取得する
 * @return ソナーセンサからの距離[m]
 */
float GuageManager::getSonarDistance()
{
    return m_sonar_distance;
}

void GuageManager::setForward(int forward)
{
    m_forward = forward;
}

void GuageManager::setTurn(int turn)
{
    m_turn = turn;
}

void GuageManager::setPWM(int pwm_l, int pwm_r)
{
    m_pwm[L] = pwm_l;
    m_pwm[R] = pwm_r;
}

int GuageManager::getForward()
{
    return m_forward;
}

int GuageManager::getTurn()
{
    return m_turn;
}

int GuageManager::getPWM(int wheel)
{
    return m_pwm[wheel];
}

void GuageManager::setGyroOffset(float offset)
{
    m_gyro_offset = offset;
}

float GuageManager::getGyroOffset()
{
    return m_gyro_offset;
}
