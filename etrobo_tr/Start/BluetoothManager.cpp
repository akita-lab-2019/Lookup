
#include "BluetoothManager.h"

static FILE *bt = NULL;
/**
 * コンストラクタ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 */
BluetoothManager::BluetoothManager()
{
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
}

/**
 * バランス走行に必要なものをリセットする
 */
void BluetoothManager::init()
{
}

void BluetoothManager::send(char *str)
{
    fprintf(bt, "%s\r\n", str);
}

void BluetoothManager::receive()
{
    m_rcv_char = fgetc(bt);
}

int BluetoothManager::getStartSignal()
{
    switch (m_rcv_char)
    {
    case 'l':
        return START_L;
    case 'r':
        return START_R;
    case 's':
        return STOP;
    default:
        return NONE;
    }
}
