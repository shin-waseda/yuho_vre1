#include "app/test_mode.h"

#include "interface/timer.h"
#include "interface/encoder.h"
#include "tim.h"
#include "interface/gyro.h"
#include "app/control_loop.h"
#include "app/failsafe.h"

// TESTは診断用なので、フェイルセーフが発動しても止めずに状態を表示し続ける
// (モーターはISR側で停止済み)。
void TestMode_Run(void) {
    while (1) {
        GyroData g = App_GetGyroRaw(); // ジャイロのSPIはISRが占有しているので直接読まない
        printf("X: %6d  Y: %6d  Z: %6d  (Z: %7.1f dps)\r\n", g.x, g.y, g.z, App_GetGyroZ_dps());

        printf("VBAT: %.2f V  FAILSAFE: %s\r\n",
              FailSafe_GetFilteredVoltage(), FailSafe_CauseName(FailSafe_GetCause()));

        uint16_t enc_l = Encoder_GetCountL();
        uint16_t enc_r = Encoder_GetCountR();
        printf("ENC_L: %5u  ENC_R: %5u\r\n", enc_l, enc_r);

        printf("R:%4d FR:%4d FL:%4d L:%4d\r\n",
              ad_r, ad_fr, ad_fl, ad_l);

        Pose pose = App_GetPose();
        WheelVelocity wv = App_GetActualVelocity();
        printf("POSE X:%7.1f Y:%7.1f TH:%6.3f  VL:%6.1f VR:%6.1f\r\n",
              pose.x_mm, pose.y_mm, pose.theta_rad, wv.left_mm_s, wv.right_mm_s);

        Timer_DelayMs(100);
    }
}
