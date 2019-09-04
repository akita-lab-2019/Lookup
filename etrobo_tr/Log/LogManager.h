/**
 *******************************************************************************
 ** 走行のログを管理する
 ** ログを取得し，Bluetooth，SD，ディスプレイに各情報を渡す
 *******************************************************************************
 **/

#ifndef EV3_LOG_MANAGER_H_
#define EV3_LOG_MANAGER_H_

#include "Recorder.h"
#include "GuageManager.h"
#include "BluetoothManager.h"
#include "Section.h"

class LogManager
{
public:
    LogManager(Recorder *recorder, BluetoothManager *bt, GuageManager *robot_info, Section *section);

    void init();
    void update();

private:
    char m_data_str[32][32];
    // FILE *m_bt;
    Recorder *m_recorder;
    BluetoothManager *m_bt;
    GuageManager *m_robot_info;
    Section *m_section;

    void readData();
    void sendToBT();
    void sendToSD();
    void sendToDisplay();
};

#endif // EV3_LOG_MANAGER_H_
