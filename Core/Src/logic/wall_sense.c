#include "logic/wall_sense.h"

WallObservation WallSense_Judge(WallSensorValues v) {
    WallObservation obs;
    obs.left = v.l > WALL_TH_L;
    obs.right = v.r > WALL_TH_R;
    obs.front = (uint32_t)v.fl + v.fr > WALL_TH_FRONT_SUM;
    return obs;
}
