#include "SectionTracer.h"

/**
 * コンストラクタ
 * @param lineMonitor    ライン判定
 * @param invertedWalker 倒立走行
 */
SectionTracer::SectionTracer(GuageManager *robot_info,
                             Section *section,
                             LineTracer *line_tracer)
    : m_robot_info(robot_info),
      m_section(section),
      m_line_tracer(line_tracer),
      m_is_initialized(false)
{
}

/**
 * ライントレースする
 */
bool b_flag = 0;
bool SectionTracer::run()
{
    if (m_is_initialized == false)
    {
        m_section->update(m_section_num);
        m_line_tracer->update();
        m_line_tracer->setIsInverted(1);
        m_is_initialized = true;
    }

    if (m_robot_info->getRobotDis() > m_section->getDistance())
    {
        m_section_num++;
        if (m_section_num < 15)
        {
            m_section->update(m_section_num);
            m_line_tracer->update();
            m_line_tracer->setIsInverted(1);
            ev3_speaker_play_tone((int)(261.63 * pow(M_E, m_section_num * 0.1)), 100);
        }
        else
        {
            return true;
        }
    }

    if (m_section_num % 2)
    {
        ev3_led_set_color(LED_ORANGE);
    }
    else
    {
        ev3_led_set_color(LED_GREEN);
    }

    m_line_tracer->run();

    return false;
}
