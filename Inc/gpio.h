#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// ?? LED ?????
#define LED_PIN GPIO_PIN_0    // PE0
#define LED_PORT GPIOE

#define KEY0_PIN GPIO_PIN_2   // PE2
#define KEY1_PIN GPIO_PIN_3   // PE3
#define KEY2_PIN GPIO_PIN_4   // PE4
#define KEY_PORT GPIOE

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __GPIO_H */
