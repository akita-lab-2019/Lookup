#include "Recorder.h"

/**
 * コンストラクタ
 */
Recorder::Recorder()
{
}

/**
 * 初期化する
 */
void Recorder::init()
{
    // ファイル名を生成し代入
    sprintf(m_logfile_name, "log/log_%04d.csv", getFileNum());

    // ヘッダを書く
    writeHeader();
}

/**
 * ファイル番号を取得する
 */
int Recorder::getFileNum()
{
    char str_data[5];
    int file_num = 0;
    FILE *fp = NULL;
    fp = fopen("log/log_num.txt", "r");

    // ファイルを開いて値を読み出す
    if (fp != NULL)
    {
        fgets(str_data, 5, fp);
        file_num = atoi(str_data);
    }
    fclose(fp);

    // 次回用にファイル数をインクリメントして書き込んでおく
    fp = fopen("log/log_num.txt", "w");
    fprintf(fp, "%d", file_num + 1);
    fclose(fp);

    return file_num;
}

/**
 * ヘッダをSDカードに書き込む
 */
void Recorder::writeHeader()
{
    // ログファイルをオープン
    FILE *log_file = NULL;
    log_file = fopen(m_logfile_name, "a");

    // パラメータファイル
    FILE *parm_file = NULL;
    char parm_str[256];
    parm_file = fopen("parm.txt", "r");

    // パラメータファイル中身をログファイルにコピー
    if (parm_file != NULL)
    {
        for (int i = 0; i < parm_mum; i++)
        {
            fgets(parm_str, 256, parm_file);   // 取得
            fprintf(log_file, "%s", parm_str); // 書き込み
        }
    }
    fclose(parm_file);

    // 凡例を記入
    fprintf(log_file, "\r\nCourse,Section,Time[s],Battery[V],ColorSensor,ColorSensorGap,Forward[%%],Turn[%%],PWM_L[%%],PWM_R[%%],PoseX[m],PoseY[m],Angle[deg],Distance[m],GyroOffset,PitchV[deg/s],SonarDist[m]\r\n");
    fclose(log_file);
}

/**
 * SDカードに記録する
 */
void Recorder::record(char *str)
{
    FILE *log_file = NULL;
    log_file = fopen(m_logfile_name, "a");
    fprintf(log_file, "%s\r\n", str);
    fclose(log_file);
}