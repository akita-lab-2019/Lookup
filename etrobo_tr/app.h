#ifdef __cplusplus
extern "C"
{
#endif

#include "ev3api.h"

/*
 *  各タスクの優先度の定義
 */
#define MAIN_PRIORITY TMIN_APP_TPRI + 1

/*
 *  ターゲットに依存する可能性のある定数の定義
 */
#ifndef STACK_SIZE
#define STACK_SIZE 4096 /* タスクのスタックサイズ */
#endif                  /* STACK_SIZE */

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

    extern void main_task(intptr_t exinf);
    extern void info_task(intptr_t exinf);
    extern void tracer_task(intptr_t exinf);
    extern void tail_task(intptr_t exinf);
    extern void log_task(intptr_t exinf);
    extern void bt_recieve_task(intptr_t exinf);
    void doLookup();
    void landing();

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
