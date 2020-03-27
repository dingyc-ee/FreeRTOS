#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "gpio.h"

// °´¼ü×´Ì¬ºê
#define KEY_ON 1
#define KEY_OFF 0

uint8_t key_scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* __BSP_KEY_H */
