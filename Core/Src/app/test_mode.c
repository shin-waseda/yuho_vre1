#include "app/test_mode.h"

#include "main.h"
#include "tim.h"
#include "interface/gyro.h"
#include "app/control_loop.h"

void TestMode_Run(void) {
    while (1) {
        GyroData g = ICM_ReadGyro();
        printf("X: %6d  Y: %6d  Z: %6d\r\n", g.x, g.y, g.z);

        uint16_t enc_l = __HAL_TIM_GET_COUNTER(&htim4);
        uint16_t enc_r = __HAL_TIM_GET_COUNTER(&htim8);
        printf("ENC_L: %5u  ENC_R: %5u\r\n", enc_l, enc_r);

        printf("R:%4d FR:%4d FL:%4d L:%4d\r\n",
              ad_r, ad_fr, ad_fl, ad_l);

        Pose pose = App_GetPose();
        WheelVelocity wv = App_GetActualVelocity();
        printf("POSE X:%7.1f Y:%7.1f TH:%6.3f  VL:%6.1f VR:%6.1f\r\n",
              pose.x_mm, pose.y_mm, pose.theta_rad, wv.left_mm_s, wv.right_mm_s);

        HAL_Delay(100);
    }
}
