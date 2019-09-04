#include "BalancerCpp.h"
#include "BalancerParam.h"

/**
 * コンストラクタ
 */
Balancer::Balancer()
    : mForward(0),
      mTurn(0),
      mOffset(0),
      mRightPwm(0),
      mLeftPwm(0)
{
}

/**
 * バランサを初期化する
 * @param offset ジャイロセンサオフセット値
 */
void Balancer::init(float offset)
{
    mOffset = offset;
    balance_init(); // 倒立振子制御初期化
}

/**
 * バランサの値を更新する
 * @param angle   角速度
 * @param rwEnc   右車輪エンコーダ値
 * @param lwEnc   左車輪エンコーダ値
 * @param battery バッテリ電圧値
 */
void Balancer::update(int angle, int rwEnc, int lwEnc, int battery)
{
    // 倒立振子制御APIを呼び出し、倒立走行するための
    // 左右モータ出力値を得る
    balance_control(
        static_cast<float>(mForward),
        static_cast<float>(mTurn),
        static_cast<float>(angle),
        static_cast<float>(mOffset),
        static_cast<float>(cancelBacklash(mLeftPwm, lwEnc)),
        static_cast<float>(cancelBacklash(mRightPwm, rwEnc)),
        static_cast<float>(battery),
        &mLeftPwm,
        &mRightPwm);
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Balancer::setCommand(int forward, int turn)
{
    mForward = forward;
    mTurn = turn;
}

/**
 * 右車輪のPWM値を取得する
 * @return 右車輪のPWM値
 */
int8_t Balancer::getPwmRight()
{
    return mRightPwm;
}

/**
 * 左車輪のPWM値を取得する
 * @return 左車輪のPWM値
 */
int8_t Balancer::getPwmLeft()
{
    return mLeftPwm;
}

/**
 * 直近のPWM値に応じてエンコーダー値にバックラッシュ分の値を追加する
 * @param pwm モーターPWM値 ※前回の出力値
 * @param enc モーターエンコーダー値
 * @return バックラッシュを追加したモーターエンコーダー値
 */
int Balancer::cancelBacklash(int8_t pwm, int enc)
{
    const int kBacklashHalf = 5; // バックラッシュの半分[deg]

    if (pwm == 0)
        return enc;
    return pwm > 0 ? enc - kBacklashHalf : enc + kBacklashHalf;
}

#define rt_SATURATE(sig, ll, ul) (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)))

static float ud_err_theta;
static float ud_psi;
static float ud_theta_lpf;
static float ud_theta_ref;
static float ud_thetadot_cmd_lpf;

void Balancer::balance_control(float args_cmd_forward, float args_cmd_turn, float args_gyro, float args_gyro_offset, float args_theta_m_l, float args_theta_m_r, float args_battery, signed char *ret_pwm_l, signed char *ret_pwm_r)
{
    {
        float tmp_theta;
        float tmp_theta_lpf;
        float tmp_pwm_r_limiter;
        float tmp_psidot;
        float tmp_pwm_turn;
        float tmp_pwm_l_limiter;
        float tmp_thetadot_cmd_lpf;
        float tmp[4];
        float tmp_theta_0[4];
        long tmp_0;

        tmp_thetadot_cmd_lpf = (((args_cmd_forward / CMD_MAX) * K_THETADOT) * (1.0F - A_R)) + (A_R * ud_thetadot_cmd_lpf);
        tmp_theta = (((DEG2RAD * args_theta_m_l) + ud_psi) + ((DEG2RAD * args_theta_m_r) + ud_psi)) * 0.5F;
        tmp_theta_lpf = ((1.0F - A_D) * tmp_theta) + (A_D * ud_theta_lpf);
        tmp_psidot = (args_gyro - args_gyro_offset) * DEG2RAD;
        tmp[0] = ud_theta_ref;
        tmp[1] = 0.0F;
        tmp[2] = tmp_thetadot_cmd_lpf;
        tmp[3] = 0.0F;
        tmp_theta_0[0] = tmp_theta;
        tmp_theta_0[1] = ud_psi;
        tmp_theta_0[2] = (tmp_theta_lpf - ud_theta_lpf) / EXEC_PERIOD;
        tmp_theta_0[3] = tmp_psidot;
        tmp_pwm_r_limiter = 0.0F;
        for (tmp_0 = 0; tmp_0 < 4; tmp_0++)
        {
            tmp_pwm_r_limiter += (tmp[tmp_0] - tmp_theta_0[tmp_0]) * K_F[(tmp_0)];
        }
        tmp_pwm_r_limiter = (((K_I * ud_err_theta) + tmp_pwm_r_limiter) /
                             ((BATTERY_GAIN * args_battery) - BATTERY_OFFSET)) *
                            100.0F;
        tmp_pwm_turn = (args_cmd_turn / CMD_MAX) * K_PHIDOT;
        tmp_pwm_l_limiter = tmp_pwm_r_limiter + tmp_pwm_turn;
        tmp_pwm_l_limiter = rt_SATURATE(tmp_pwm_l_limiter, -100.0F, 100.0F);
        (*ret_pwm_l) = (signed char)tmp_pwm_l_limiter;
        tmp_pwm_r_limiter -= tmp_pwm_turn;
        tmp_pwm_r_limiter = rt_SATURATE(tmp_pwm_r_limiter, -100.0F, 100.0F);
        (*ret_pwm_r) = (signed char)tmp_pwm_r_limiter;
        tmp_pwm_l_limiter = (EXEC_PERIOD * tmp_thetadot_cmd_lpf) + ud_theta_ref;
        tmp_pwm_turn = (EXEC_PERIOD * tmp_psidot) + ud_psi;
        tmp_pwm_r_limiter = ((ud_theta_ref - tmp_theta) * EXEC_PERIOD) +
                            ud_err_theta;
        ud_err_theta = tmp_pwm_r_limiter;
        ud_theta_ref = tmp_pwm_l_limiter;
        ud_thetadot_cmd_lpf = tmp_thetadot_cmd_lpf;
        ud_psi = tmp_pwm_turn;
        ud_theta_lpf = tmp_theta_lpf;
    }
}

void Balancer::balance_init(void)
{
    ud_err_theta = 0.0F;
    ud_theta_ref = 0.0F;
    ud_thetadot_cmd_lpf = 0.0F;
    ud_psi = 0.0F;
    ud_theta_lpf = 0.0F;
}
