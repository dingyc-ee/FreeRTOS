#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "gpio.h"

// ֱ�Ӳ����Ĵ����ķ�������IO
#define digitalHi(port, pin)        do { port->BSRR |= pin; } while (0)     // ��λ
#define digitalLo(port, pin)        do { port->BRR  |= pin; } while (0)     // ��λ
#define digitalToggle(port, pin)    do { port->ODR  ^= pin; } while (0)     // ��ת

// �������IO�ĺ�
#define LED_R_TOGGLE        digitalToggle(LED_R_GPIO_Port, LED_R_Pin)
#define LED_R_ON            digitalLo(LED_R_GPIO_Port, LED_R_Pin)
#define LED_R_OFF           digitalHi(LED_R_GPIO_Port, LED_R_Pin)

#define LED_G_TOGGLE        digitalToggle(LED_G_GPIO_Port, LED_G_Pin)
#define LED_G_ON            digitalLo(LED_G_GPIO_Port, LED_G_Pin)
#define LED_G_OFF           digitalHi(LED_G_GPIO_Port, LED_G_Pin)

#define LED_B_TOGGLE        digitalToggle(LED_B_GPIO_Port, LED_B_Pin)
#define LED_B_ON            digitalLo(LED_B_GPIO_Port, LED_B_Pin)
#define LED_B_OFF           digitalHi(LED_B_GPIO_Port, LED_B_Pin)

// ��ɫ����
#define LED_RED             do { LED_R_ON; LED_G_OFF; LED_B_OFF; } while (0)
#define LED_GREEN           do { LED_R_OFF; LED_G_ON; LED_B_OFF; } while (0)
#define LED_BLUE            do { LED_R_OFF; LED_G_OFF; LED_B_ON; } while (0)
#define LED_YELLOW          do { LED_R_ON; LED_G_ON; LED_B_OFF; } while (0)
#define LED_PURPLE          do { LED_R_ON; LED_G_OFF; LED_B_ON; } while (0)
#define LED_CYAN            do { LED_R_OFF; LED_G_ON; LED_B_ON; } while (0)
#define LED_ALL_ON          do { LED_R_ON; LED_G_ON; LED_B_ON; } while (0)
#define LED_ALL_OFF         do { LED_R_OFF; LED_G_OFF; LED_B_OFF; } while (0)

#endif /* __BSP_LED_H */
