#ifndef __BSP_BEEP_H
#define __BSP_BEEP_H

#include "gpio.h"

// 直接操作寄存器的方法控制IO
#define digitalHi(port, pin)        do { port->BSRR |= pin; } while (0)     // 置位
#define digitalLo(port, pin)        do { port->BRR  |= pin; } while (0)     // 复位
#define digitalToggle(port, pin)    do { port->ODR  ^= pin; } while (0)     // 反转

#define BEEP_TOGGLE                 digitalToggle(BEEP_GPIO_Port, BEEP_Pin)
#define BEEP_ON                     digitalHi(BEEP_GPIO_Port, BEEP_Pin)
#define BEEP_OFF                    digitalLo(BEEP_GPIO_Port, BEEP_Pin)

#endif  /* __BSP_BEEP_H */
