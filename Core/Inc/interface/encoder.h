#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "global.h"
#include "params.h"

void Encoder_Init(void);
int16_t Encoder_GetDeltaL(void);
int16_t Encoder_GetDeltaR(void);

#endif