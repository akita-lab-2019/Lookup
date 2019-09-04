#ifndef EV3_START_MANAGER_H_
#define EV3_START_MANAGER_H_

#include "GuageManager.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "BalancerCpp.h"
#include "stdlib.h"
#include <TouchSensor.h>
#include "BluetoothManager.h"

class StartManager
{
public:
    StartManager(BluetoothManager *bt,
                 ev3api::TouchSensor &touch_sensor,
                 ev3api::Clock &clock);

    void init();
    void waitForStart();
    void start();

private:
    BluetoothManager *m_bt;
    ev3api::TouchSensor &m_touch_sensor;
    ev3api::Clock &m_clock;
};

#endif