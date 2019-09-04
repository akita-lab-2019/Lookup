#include "LineMonitor.h"

LineMonitor::LineMonitor(ev3api::ColorSensor &colorSensor) : m_color_sensor(colorSensor)
{
}

/**
 * 輝度偏差を取得する
 * @return 輝度偏差
 */
int LineMonitor::getBrightness()
{
    return m_color_sensor.getBrightness();
}

/**
 * 輝度偏差を取得する
 * @return 輝度偏差
 */
float LineMonitor::getGap(float target)
{
    float sensor_val = target - (0.9 * m_pre_sensor_val + 0.1 * m_color_sensor.getBrightness());
    m_pre_sensor_val = m_color_sensor.getBrightness();

    return sensor_val;
}
