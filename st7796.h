#ifndef __ST7796_H
#define __ST7796_H

#include "main.h"
#include <stdint.h>

/* LCD control pins */
#define ST7796_CS_PORT      GPIOE
#define ST7796_CS_PIN       GPIO_PIN_12

#define ST7796_RST_PORT     GPIOC
#define ST7796_RST_PIN      GPIO_PIN_0

#define ST7796_DC_PORT      GPIOC
#define ST7796_DC_PIN       GPIO_PIN_3

/* Use SPI1 for LCD */
extern SPI_HandleTypeDef hspi1;
#define ST7796_SPI_HANDLE   hspi1

#define ST7796_BLACK        0x0000
#define ST7796_BLUE         0x001F
#define ST7796_RED          0xF800
#define ST7796_GREEN        0x07E0
#define ST7796_CYAN         0x07FF
#define ST7796_MAGENTA      0xF81F
#define ST7796_YELLOW       0xFFE0
#define ST7796_WHITE        0xFFFF
#define ST7796_ORANGE       0xFD20
#define ST7796_GRAY         0x8410

extern uint16_t ST7796_WIDTH;
extern uint16_t ST7796_HEIGHT;

void ST7796_Init(void);
void ST7796_SetRotation(uint8_t rotation);
void ST7796_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7796_FillScreen(uint16_t color);
void ST7796_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7796_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7796_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

#endif
