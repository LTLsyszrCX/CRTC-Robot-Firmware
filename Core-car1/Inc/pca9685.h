#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include "main.h"

extern volatile uint8_t  pca9685_ok;
extern volatile uint8_t  pca9685_last_status;
extern volatile uint16_t pca9685_errors;
extern volatile uint16_t pca9685_recoveries;

void PCA9685_Init(void);
void PCA9685_SetPWM(uint8_t ch, uint16_t off);

void PCA9685_Rewake(void);

void PCA9685_CheckAndRepair(void);

#endif
