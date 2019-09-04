#ifndef EV3_UNIT_LINEMONITOR_H_
#define EV3_UNIT_LINEMONITOR_H_

#include "ColorSensor.h"

// 定義
class LineMonitor
{
public:
    explicit LineMonitor(ev3api::ColorSensor &colorSensor);
    int getBrightness();
    float getGap(float target);

private:
    const ev3api::ColorSensor &m_color_sensor;

    // 輝度目標値
    float m_target = 30;

    // 前回の輝度値
    float m_pre_sensor_val = 0;
};

#endif // EV3_UNIT_LINEMONITOR_H_
