#include "st7796.h"

static uint8_t st7796_rotation = 0;
uint16_t ST7796_WIDTH = 320;
uint16_t ST7796_HEIGHT = 480;

#define COLOR_BURST_PIXELS  256

static void ST7796_Select(void)
{
    HAL_GPIO_WritePin(ST7796_CS_PORT, ST7796_CS_PIN, GPIO_PIN_RESET);
}

static void ST7796_Unselect(void)
{
    HAL_GPIO_WritePin(ST7796_CS_PORT, ST7796_CS_PIN, GPIO_PIN_SET);
}

static void ST7796_DC_Command(void)
{
    HAL_GPIO_WritePin(ST7796_DC_PORT, ST7796_DC_PIN, GPIO_PIN_RESET);
}

static void ST7796_DC_Data(void)
{
    HAL_GPIO_WritePin(ST7796_DC_PORT, ST7796_DC_PIN, GPIO_PIN_SET);
}

static void ST7796_Reset(void)
{
    HAL_GPIO_WritePin(ST7796_RST_PORT, ST7796_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(ST7796_RST_PORT, ST7796_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);

    HAL_GPIO_WritePin(ST7796_RST_PORT, ST7796_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(150);
}

static void ST7796_WriteCommand(uint8_t cmd)
{
    ST7796_Select();
    ST7796_DC_Command();
    HAL_SPI_Transmit(&ST7796_SPI_HANDLE, &cmd, 1, HAL_MAX_DELAY);
    ST7796_Unselect();
}

static void ST7796_WriteData(uint8_t *data, uint16_t size)
{
    ST7796_Select();
    ST7796_DC_Data();
    HAL_SPI_Transmit(&ST7796_SPI_HANDLE, data, size, HAL_MAX_DELAY);
    ST7796_Unselect();
}

static void ST7796_WriteDataByte(uint8_t data)
{
    ST7796_WriteData(&data, 1);
}

static void ST7796_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    ST7796_WriteCommand(0x2A); // CASET
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    ST7796_WriteData(data, 4);

    ST7796_WriteCommand(0x2B); // RASET
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    ST7796_WriteData(data, 4);

    ST7796_WriteCommand(0x2C); // RAMWR
}

static void ST7796_WriteColorBurst(uint16_t color, uint32_t count)
{
    static uint8_t buffer[COLOR_BURST_PIXELS * 2];
    uint16_t i;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    for (i = 0; i < COLOR_BURST_PIXELS; i++)
    {
        buffer[2 * i] = hi;
        buffer[2 * i + 1] = lo;
    }

    ST7796_Select();
    ST7796_DC_Data();

    while (count > 0)
    {
        uint16_t chunk = (count > COLOR_BURST_PIXELS) ? COLOR_BURST_PIXELS : (uint16_t)count;
        HAL_SPI_Transmit(&ST7796_SPI_HANDLE, buffer, chunk * 2, HAL_MAX_DELAY);
        count -= chunk;
    }

    ST7796_Unselect();
}

void ST7796_SetRotation(uint8_t rotation)
{
    uint8_t madctl = 0x48; // default portrait, BGR

    st7796_rotation = rotation % 4;

    switch (st7796_rotation)
    {
        case 0:
            madctl = 0x48;
            ST7796_WIDTH = 320;
            ST7796_HEIGHT = 480;
            break;

        case 1:
            madctl = 0x28;
            ST7796_WIDTH = 480;
            ST7796_HEIGHT = 320;
            break;

        case 2:
            madctl = 0x88;
            ST7796_WIDTH = 320;
            ST7796_HEIGHT = 480;
            break;

        case 3:
            madctl = 0xE8;
            ST7796_WIDTH = 480;
            ST7796_HEIGHT = 320;
            break;
    }

    ST7796_WriteCommand(0x36); // MADCTL
    ST7796_WriteDataByte(madctl);
}

void ST7796_Init(void)
{
    /* ST7796S init sequence used on many SPI modules */
    uint8_t data[16];

    HAL_GPIO_WritePin(ST7796_CS_PORT, ST7796_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7796_DC_PORT, ST7796_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7796_RST_PORT, ST7796_RST_PIN, GPIO_PIN_SET);

    ST7796_Reset();

    ST7796_WriteCommand(0x01); // SWRESET
    HAL_Delay(150);

    ST7796_WriteCommand(0xF0);
    data[0] = 0xC3;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xF0);
    data[0] = 0x96;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0x36); // MADCTL
    data[0] = 0x48;            // BGR, portrait default
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0x3A); // COLMOD
    data[0] = 0x55;            // RGB565
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xB4);
    data[0] = 0x01;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xB7);
    data[0] = 0xC6;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xE8);
    data[0] = 0x48;
    data[1] = 0x8A;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x29;
    data[5] = 0x19;
    data[6] = 0xA5;
    data[7] = 0x33;
    ST7796_WriteData(data, 8);

    ST7796_WriteCommand(0xC1);
    data[0] = 0x06;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xC2);
    data[0] = 0xA7;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xC5);
    data[0] = 0x18;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xE0); // Positive gamma
    data[0]  = 0xF0;
    data[1]  = 0x09;
    data[2]  = 0x0B;
    data[3]  = 0x06;
    data[4]  = 0x04;
    data[5]  = 0x15;
    data[6]  = 0x2F;
    data[7]  = 0x54;
    data[8]  = 0x42;
    data[9]  = 0x3C;
    data[10] = 0x17;
    data[11] = 0x14;
    data[12] = 0x18;
    data[13] = 0x1B;
    ST7796_WriteData(data, 14);

    ST7796_WriteCommand(0xE1); // Negative gamma
    data[0]  = 0xE0;
    data[1]  = 0x09;
    data[2]  = 0x0B;
    data[3]  = 0x06;
    data[4]  = 0x04;
    data[5]  = 0x03;
    data[6]  = 0x2B;
    data[7]  = 0x43;
    data[8]  = 0x42;
    data[9]  = 0x3B;
    data[10] = 0x16;
    data[11] = 0x14;
    data[12] = 0x17;
    data[13] = 0x1B;
    ST7796_WriteData(data, 14);

    ST7796_WriteCommand(0xF0);
    data[0] = 0x3C;
    ST7796_WriteData(data, 1);

    ST7796_WriteCommand(0xF0);
    data[0] = 0x69;
    ST7796_WriteData(data, 1);

    // ST7796_WriteCommand(0x21); // Display inversion ON
    ST7796_WriteCommand(0x11); // Sleep out
    HAL_Delay(120);

    ST7796_WriteCommand(0x29); // Display ON
    HAL_Delay(20);

    ST7796_SetRotation(1);     // landscape by default
    ST7796_FillScreen(ST7796_BLACK);
}

void ST7796_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t data[2];

    if (x >= ST7796_WIDTH || y >= ST7796_HEIGHT)
        return;

    ST7796_SetAddrWindow(x, y, x, y);

    data[0] = color >> 8;
    data[1] = color & 0xFF;
    ST7796_WriteData(data, 2);
}

void ST7796_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t pixels;

    if (x >= ST7796_WIDTH || y >= ST7796_HEIGHT)
        return;

    if ((x + w) > ST7796_WIDTH)
        w = ST7796_WIDTH - x;

    if ((y + h) > ST7796_HEIGHT)
        h = ST7796_HEIGHT - y;

    ST7796_SetAddrWindow(x, y, x + w - 1, y + h - 1);
    pixels = (uint32_t)w * h;
    ST7796_WriteColorBurst(color, pixels);
}

void ST7796_FillScreen(uint16_t color)
{
    ST7796_FillRect(0, 0, ST7796_WIDTH, ST7796_HEIGHT, color);
}

void ST7796_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    ST7796_FillRect(x, y, w, 1, color);
}

void ST7796_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    ST7796_FillRect(x, y, 1, h, color);
}
