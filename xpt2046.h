#ifndef __XPT2046_H
#define __XPT2046_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ----  touch pin mapping ---- */
#define XPT2046_CS_PORT       GPIOB
#define XPT2046_CS_PIN        GPIO_PIN_12

#define XPT2046_IRQ_PORT      GPIOC
#define XPT2046_IRQ_PIN       GPIO_PIN_6

/* Touch SPI handle from CubeMX */
extern SPI_HandleTypeDef hspi2;
#define XPT2046_SPI_HANDLE    hspi2

void XPT2046_Init(void);
bool XPT2046_Touched(void);
bool XPT2046_GetRaw(uint16_t *x, uint16_t *y);
bool Touch_GetPoint(uint16_t *x, uint16_t *y);

#endif
