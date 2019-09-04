#include "StartManager.h"

/**
 * コンストラクタ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 */
StartManager::StartManager(BluetoothManager *bt,
                           ev3api::TouchSensor &touch_sensor,
                           ev3api::Clock &clock)
    : m_bt(bt),
      m_touch_sensor(touch_sensor),
      m_clock(clock)
{
}

void StartManager::init()
{
}

void StartManager::waitForStart()
{
    while (1)
    {
        // BlueToothスタート
        if (m_bt->getStartSignal() == BluetoothManager::START_L)
            break;

        if (m_bt->getStartSignal() == BluetoothManager::START_R)
            break;

        // タッチセンサスタート
        if (m_touch_sensor.isPressed())
            break;

        m_clock.sleep(4);
        m_clock.reset();
    }
}

void StartManager::start()
{
}
