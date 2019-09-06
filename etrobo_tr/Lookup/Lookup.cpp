#include "Lookup.h"

/**
 * コンストラクタ
 */
Lookup::Lookup(ev3api::Clock &clock,
               ev3api::GyroSensor &gyro,
               ev3api::Motor &wheel_L,
               ev3api::Motor &wheel_R,
               GuageManager *guage,
               LineTracer *line_tracer,
               TailController *tail)
    : m_clock(clock),
      m_gyro(gyro),
      m_wheel_L(wheel_L),
      m_wheel_R(wheel_R),
      m_guage(guage),
      m_line_tracer(line_tracer),
      m_tail(tail)
{
}

/**
 * 初期化する
 */
void Lookup::init()
{
}

/**
 * 情報を更新する
 */
void Lookup::update()
{
}

/**
 * 走行を行う
 */

void Lookup::run()
{
    int fwd;
    switch (m_sequence_num)
    {
    // ゴール通知
    case 0:
        ev3_speaker_play_tone(262, 500);
        m_sequence_num++;
        break;

    // ゲート検知まで減速
    case 1:
        // ゴールゲートから遠ざかるほど前進量を下げる
        fwd = 100 - 100 * (m_guage->getRobotDis() - 0.37);

        // 前進量の下限は70
        if (fwd < 70)
            fwd = 70;

        lineRun(1, fwd, 0, 35);

        // ゲートを検知
        if (m_guage->getSonarDistance() < 0.1)
        {
            m_sequence_num++;
        }
        break;

    // 着地
    case 2:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        m_wheel_L.setPWM(90);
        m_wheel_R.setPWM(90);
        m_clock.sleep(150);
        m_landing_dis = m_guage->getRobotDis();
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(10);
        m_sequence_num++;
        break;

    // 低い姿勢で前進
    case 3:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        lineRun(0, 8, 1, 5);

        if (m_guage->getRobotDis() > 0.25)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
            m_clock.sleep(250);
        }
        break;

    // 低い姿勢で後進
    case 4:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        lineRun(0, -9, 1, 5);

        if (m_guage->getRobotDis() < -0.10)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
            m_clock.sleep(250);
        }
        break;

    // 低い姿勢で前進
    case 5:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        lineRun(0, 8, 1, 5);

        if (m_guage->getRobotDis() > 0.25)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
            m_clock.sleep(250);
        }
        break;

    // 低い姿勢で後進
    case 6:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        lineRun(0, -9, 1, 5);

        if (m_guage->getRobotDis() < -0.10)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
            m_clock.sleep(250);
        }
        break;

    // 低い姿勢で前進
    case 7:
        m_tail->setAngle(67);
        m_tail->setMaxSpeed(40);
        lineRun(0, 8, 1, 5);

        if (m_guage->getRobotDis() > 0.25)
        {
            ev3_speaker_play_tone(262, 300);
            m_sequence_num++;
            m_clock.sleep(250);
        }
        break;

    // 姿勢を上げる
    case 8:
        m_wheel_L.setPWM(-50);
        m_wheel_R.setPWM(-50);
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(61);
        m_clock.sleep(155);
        m_tail->setAngle(85);
        m_tail->setMaxSpeed(90);
        m_sequence_num++;
        break;
    // 高い姿勢で前進
    case 9:

        lineRun(0, 8, 1, 15);

        if (m_guage->getRobotDis() > 0.90)
        {
            ev3_speaker_play_tone(262, 1000);
            m_sequence_num++;
        }
        break;

    // 停止
    case 10:
        m_wheel_L.reset();
        m_wheel_R.reset();
        break;

    default:
        break;
    }
}

void Lookup::upBody()
{
    m_tail->setAngle(90);
    m_tail->setMaxSpeed(100);
}

void Lookup::downBody()
{
    m_tail->setAngle(70);
    m_tail->setMaxSpeed(100);
}

/**
 * ライントレースする
 *
 */
void Lookup::lineRun(bool is_inverted, int forward, int pid_index, int target)
{
    m_line_tracer->setIsInverted(is_inverted);
    m_line_tracer->setForward(forward);
    m_line_tracer->setCurvature(0);
    // m_line_tracer->setPidParm(m_run_pid_param[pid_index]);
    m_line_tracer->setColorTarget(target);
    m_line_tracer->run();
}

void Lookup::landing()
{
}
