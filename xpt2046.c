#include "xpt2046.h"
#include "st7796.h"

static void XPT2046_Select(void)
{
    HAL_GPIO_WritePin(XPT2046_CS_PORT, XPT2046_CS_PIN, GPIO_PIN_RESET);
}

static void XPT2046_Unselect(void)
{
    HAL_GPIO_WritePin(XPT2046_CS_PORT, XPT2046_CS_PIN, GPIO_PIN_SET);
}

static uint16_t XPT2046_Read12(uint8_t cmd)
{
    uint8_t tx[3];
    uint8_t rx[3];
    uint16_t value;

    tx[0] = cmd;
    tx[1] = 0x00;
    tx[2] = 0x00;

    XPT2046_Select();
    HAL_SPI_TransmitReceive(&XPT2046_SPI_HANDLE, tx, rx, 3, HAL_MAX_DELAY);
    XPT2046_Unselect();

    value = ((uint16_t)rx[1] << 8) | rx[2];
    value >>= 3;   // 12-bit result
    return value;
}

void XPT2046_Init(void)
{
    HAL_GPIO_WritePin(XPT2046_CS_PORT, XPT2046_CS_PIN, GPIO_PIN_SET);
}

bool XPT2046_Touched(void)
{
    /* TIRQ is usually active low */
    return (HAL_GPIO_ReadPin(XPT2046_IRQ_PORT, XPT2046_IRQ_PIN) == GPIO_PIN_RESET);
}

bool XPT2046_GetRaw(uint16_t *x, uint16_t *y)
{
    uint32_t x_sum = 0;
    uint32_t y_sum = 0;
    uint8_t i;

    if (!XPT2046_Touched())
        return false;

    for (i = 0; i < 8; i++)
    {
        x_sum += XPT2046_Read12(0xD0);
        y_sum += XPT2046_Read12(0x90);
    }

    *x = (uint16_t)(x_sum / 8);
    *y = (uint16_t)(y_sum / 8);

    return true;
}

static int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool Touch_GetPoint(uint16_t *x, uint16_t *y)
{
    uint16_t raw_x, raw_y;
    int32_t sx, sy;

    /* may need to adjust later */
    #define RAW_X_MIN  200
    #define RAW_X_MAX  3900
    #define RAW_Y_MIN  200
    #define RAW_Y_MAX  3900

    if (!XPT2046_GetRaw(&raw_x, &raw_y))
        return false;

    sx = map_value(raw_y, RAW_Y_MAX, RAW_Y_MIN, 0, ST7796_WIDTH - 1);
    sy = map_value(raw_x, RAW_X_MAX, RAW_X_MIN, 0, ST7796_HEIGHT - 1);

    if (sx < 0) sx = 0;
    if (sx >= ST7796_WIDTH) sx = ST7796_WIDTH - 1;
    if (sy < 0) sy = 0;
    if (sy >= ST7796_HEIGHT) sy = ST7796_HEIGHT - 1;

    *x = (uint16_t)sx;
    *y = (uint16_t)sy;
    return true;
}
