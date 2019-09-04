
#ifndef EV3_BLUETOOTH_MANAGER_H_
#define EV3_BLUETOOTH_MANAGER_H_

#include "GuageManager.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "BalancerCpp.h"
#include "stdlib.h"

class BluetoothManager
{
public:
    BluetoothManager();

    void init();
    void send(char *str);
    void receive();
    int getStartSignal();

    enum
    {
        NONE,
        START_L,
        START_R,
        STOP
    };

private:
    uint8_t m_rcv_char = 0;
};

#endif