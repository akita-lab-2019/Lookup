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
    switch (m_sequence_num)
    {
    case 0:

        // 尻尾をつく
        ev3_speaker_play_tone(262, 100);
        landing();

        ev3_speaker_play_tone(262, 100);
        m_clock.sleep(1000);

        ev3_speaker_play_tone(262, 100);
        upBody();

        m_clock.sleep(1000);

        m_sequence_num++;
        break;

    case 1:
        // 高姿勢で前進
        lineRun(15, 0, 15);

        if (m_guage->getSonarDistance() < 0.05)
        {
            m_clock.sleep(10);
            if (m_guage->getSonarDistance() < 0.05)
            {
                m_sequence_num++;
            }
        }
        break;

    case 2:
        // 倒す
        ev3_speaker_play_tone(262, 100);
        downBody();

        m_wheel_R.reset();
        m_wheel_L.reset();
        m_clock.sleep(1000);
        ev3_speaker_play_tone(262, 100);
        m_sequence_num++;

    case 3:
        // 低い姿勢で前進
        lineRun(15, 1, 5);

        if (m_guage->getRobotDis() > 0.15)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    case 4:
        // 低い姿勢で前進
        lineRun(-15, 1, 5);

        if (m_guage->getRobotDis() < -0.2)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    case 5:
        // 低い姿勢で前進
        lineRun(15, 1, 5);

        if (m_guage->getRobotDis() > 0.15)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    case 6:
        // 低い姿勢で前進
        lineRun(-15, 1, 5);

        if (m_guage->getRobotDis() < -0.2)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    case 7:
        // 低い姿勢で前進
        lineRun(15, 1, 5);

        if (m_guage->getRobotDis() > 0.15)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    case 8:
        // 高姿勢で前進
        lineRun(15, 0, 15);

        if (m_guage->getRobotDis() > 0.45)
        {
            ev3_speaker_play_tone(262, 100);
            m_wheel_R.reset();
            m_wheel_L.reset();
            m_sequence_num++;
        }
        break;

    case 9:
        m_wheel_R.reset();
        m_wheel_L.reset();
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
void Lookup::lineRun(int forward, int pid_index, int target)
{
    m_line_tracer->setIsInverted(false);
    m_line_tracer->setForward(forward);
    m_line_tracer->setCurvature(0);
    m_line_tracer->setPidParm(m_run_pid_param[pid_index]);
    m_line_tracer->setColorTarget(target);
    m_line_tracer->run();
}

void Lookup::landing()
{
    m_tail->setAngle(75);
    m_tail->setMaxSpeed(40);
    m_wheel_L.setPWM(90);
    m_wheel_R.setPWM(90);
    m_clock.sleep(150);

    m_wheel_R.reset();
    m_wheel_L.reset();

    // g_wheel_L.setPWM(0);
    // g_wheel_R.setPWM(0);
    // g_wheel_L.setBrake(true);
    // g_wheel_R.setBrake(true);
}
