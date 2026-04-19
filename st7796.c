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

/* Public CS helpers used by arducam.c during pixel streaming */
void ST7796_CS_Low(void)
{
    HAL_GPIO_WritePin(ST7796_CS_PORT, ST7796_CS_PIN, GPIO_PIN_RESET);
}

void ST7796_CS_High(void)
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

void ST7796_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
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
/* ─── 5x7 font (ASCII 32–126) ─────────────────────────────────────────────── */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' 32 */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x10,0x08,0x08,0x10,0x08}, /* '~' */
};

#define FONT_W  5
#define FONT_H  7

void ST7796_DrawChar(uint16_t x, uint16_t y, char ch,
                     uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *glyph = font5x7[ch - 32];

    for (uint8_t col = 0; col < FONT_W; col++)
    {
        uint8_t colData = glyph[col];
        for (uint8_t row = 0; row < FONT_H; row++)
        {
            uint16_t color = (colData & (1 << row)) ? fg : bg;
            if (scale == 1)
            {
                ST7796_DrawPixel(x + col, y + row, color);
            }
            else
            {
                ST7796_FillRect(x + col * scale, y + row * scale,
                                scale, scale, color);
            }
        }
    }
    /* spacing column */
    ST7796_FillRect(x + FONT_W * scale, y, scale, FONT_H * scale, bg);
}

void ST7796_DrawString(uint16_t x, uint16_t y, const char *str,
                       uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t cx = x;
    uint16_t cellW = (uint16_t)((FONT_W + 1) * scale);
    uint16_t cellH = (uint16_t)((FONT_H + 1) * scale);

    while (*str)
    {
        if (*str == '\n')
        {
            cx = x;
            y += cellH;
        }
        else
        {
            if (cx + cellW > ST7796_WIDTH)
            {
                cx = x;
                y += cellH;
            }
            ST7796_DrawChar(cx, y, *str, fg, bg, scale);
            cx += cellW;
        }
        str++;
    }
}
