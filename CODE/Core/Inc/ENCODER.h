#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"
#include "stdbool.h"
#include "stdint.h"

void Encoder_C11(int16_t *number, int16_t min, int16_t max);
void Encoder_C11_int8(int8_t *number, int8_t min, int8_t max);
void Encorder_C11_Init(uint8_t st);
bool Encoder_BT(void);

#endif
