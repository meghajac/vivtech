/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "xpt2046.h"
#include "st7796.h"
#include <stdlib.h>
#include "pid.h"
#include "arducam.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BT_CMD_TEMP_UP      'A'
#define BT_CMD_TEMP_DOWN    'B'
#define BT_CMD_HUM_UP       'C'
#define BT_CMD_HUM_DOWN     'D'
#define BT_CMD_CAMERA       'E'
#define BT_CMD_LIGHT        'F'

/* ArduCAM SPI registers */
#define ARDUCHIP_TRIG       0x41
#define CAP_DONE_MASK       0x08

/* Image dimensions */
#define QVGA_W              320U
#define QVGA_H              240U
#define QVGA_BYTES          (QVGA_W * QVGA_H * 2U)   /* 153,600 */
#define CAM_CHUNK_BYTES     640U   /* one row at a time = 320px x 2 bytes */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t bt_rx_byte  = 0;   /* filled by ISR */
volatile uint8_t bt_rx_ready = 0;   /* flag: 1 = new command waiting */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// PID CODE
#define AHT20_ADDRESS 0x38<<1
float Temperature, Temperature_C, Humidity;
void AHT20_Init (void) {
	HAL_Delay(40);
		uint8_t status;
		HAL_I2C_Mem_Read(&hi2c3, AHT20_ADDRESS, 0x71, 1, &status, 1, 1000);
		if ((status>>3 & 0x01) == 0)  // if CAL = 0
		{
			uint8_t init_commands[3] = {0xBE, 0x08, 0x00};
			HAL_I2C_Master_Transmit(&hi2c3, AHT20_ADDRESS, init_commands, 3, 1000);
			HAL_Delay (10);
		}
}
void AHT20_Measure (void)
{
	uint8_t measure_command[3] = {0xAC, 0x33, 0x00};
	HAL_I2C_Master_Transmit(&hi2c3, AHT20_ADDRESS, measure_command, 3, 1000);
	HAL_Delay(80);

	uint8_t status;
	do {
		HAL_I2C_Mem_Read(&hi2c3, AHT20_ADDRESS, 0x71, 1, &status, 1, 1000);
		HAL_Delay(100);
	}
	while ((status>>7 & 0x01) == 1);

	uint8_t RxData[7];
	HAL_I2C_Master_Receive(&hi2c3, AHT20_ADDRESS, RxData, 7, 1000);

uint32_t HUM_DATA = (RxData[1]<<16)|(RxData[2]<<8)|RxData[3];  // accumulated 24bit data
HUM_DATA = HUM_DATA>>4;  // 20bit data
Humidity = (float) ((HUM_DATA/pow(2,20)) * 100);
uint32_t TEMP_DATA = (RxData[3]<<16)|(RxData[4]<<8)|RxData[5];  // accumulated 24bit data
TEMP_DATA = TEMP_DATA&0xFFFFF;  // first 20bit data
Temperature_C = (float) (((TEMP_DATA/pow(2,20)) * 200) - 50);
Temperature = (Temperature_C * 1.8f) + 32.0f;
}




#define LED_PORT GPIOG
#define LED_PIN  GPIO_PIN_0

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} Button_t;

static void UI_DrawLightIndicatorSmall(uint16_t panelX, uint16_t panelY, uint8_t on);
static void UI_DrawCameraButton(const Button_t *btn, uint8_t pressed);
static void UI_DrawBaskingStatus(uint8_t occupied);

static void LightLed_Set(uint8_t on)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---------- Layout ---------- */
#define TEMP_X      12
#define TEMP_Y      12
#define TEMP_W      220
#define TEMP_H      140

#define HUM_X       248
#define HUM_Y       12
#define HUM_W       220
#define HUM_H       140

#define LIGHT_X     12
#define LIGHT_Y     168
#define LIGHT_W     220
#define LIGHT_H     140

#define CAM_X       248
#define CAM_Y       168
#define CAM_W       220
#define CAM_H       140

#define UI_PANEL_BG     0x1082   /* dark gray-blue */
#define UI_SEG_OFF      0x2945   /* dim segment color */

static const Button_t BTN_TEMP_UP    = { TEMP_X + 170, TEMP_Y + 18,  34, 40 };
static const Button_t BTN_TEMP_DOWN  = { TEMP_X + 170, TEMP_Y + 82,  34, 40 };
static const Button_t BTN_TEMP_MODE  = { TEMP_X + 16,  TEMP_Y + 106, 140, 24 };

static const Button_t BTN_HUM_UP     = { HUM_X  + 170, HUM_Y  + 18,  34, 40 };
static const Button_t BTN_HUM_DOWN   = { HUM_X  + 170, HUM_Y  + 82,  34, 40 };
static const Button_t BTN_HUM_MODE   = { HUM_X  + 16,  HUM_Y  + 106, 140, 24 };

static const Button_t BTN_LIGHT      = { LIGHT_X + 52,  LIGHT_Y + 92, 116, 32 };
static const Button_t BTN_CAMERA     = { CAM_X   + 52,  CAM_Y   + 18, 116, 54 };
static const Button_t BTN_BASKING    = { CAM_X   + 26,  CAM_Y   + 92, 168, 28 };

/* ---------- Basic helpers ---------- */
static uint8_t Button_Contains(const Button_t *btn, uint16_t px, uint16_t py)
{
    return (px >= btn->x &&
            px < (btn->x + btn->w) &&
            py >= btn->y &&
            py < (btn->y + btn->h));
}

static void UI_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1)
    {
        if (x0 >= 0 && x0 < ST7796_WIDTH && y0 >= 0 && y0 < ST7796_HEIGHT)
        {
            ST7796_DrawPixel((uint16_t)x0, (uint16_t)y0, color);
        }

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }

        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static void UI_DrawRectOutline(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ST7796_DrawHLine(x, y, w, color);
    ST7796_DrawHLine(x, y + h - 1, w, color);
    ST7796_DrawVLine(x, y, h, color);
    ST7796_DrawVLine(x + w - 1, y, h, color);
}

static void UI_FillCircle(int cx, int cy, int r, uint16_t color)
{
    for (int y = -r; y <= r; y++)
    {
        for (int x = -r; x <= r; x++)
        {
            if ((x * x + y * y) <= (r * r))
            {
                int px = cx + x;
                int py = cy + y;

                if (px >= 0 && px < ST7796_WIDTH && py >= 0 && py < ST7796_HEIGHT)
                {
                    ST7796_DrawPixel((uint16_t)px, (uint16_t)py, color);
                }
            }
        }
    }
}

/* ---------- Icons ---------- */
static void UI_DrawArrowButton(const Button_t *btn, uint8_t up)
{
    uint16_t bg = 0x4208;
    int cx = btn->x + btn->w / 2;
    int cy = btn->y + btn->h / 2;
    int size = 10;

    ST7796_FillRect(btn->x, btn->y, btn->w, btn->h, bg);
    UI_DrawRectOutline(btn->x, btn->y, btn->w, btn->h, ST7796_WHITE);

    if (up)
    {
        for (int i = 0; i <= size; i++)
        {
            UI_DrawLine(cx - i, cy - i, cx + i, cy - i, ST7796_WHITE);
        }
        ST7796_FillRect(cx - 2, cy + 1, 4, size, ST7796_WHITE);
    }
    else
    {
        for (int i = 0; i <= size; i++)
        {
            UI_DrawLine(cx - i, cy + i, cx + i, cy + i, ST7796_WHITE);
        }
        ST7796_FillRect(cx - 2, cy - size, 4, size, ST7796_WHITE);
    }
}

static void UI_DrawDegreeIcon(int x, int y, uint16_t color)
{
    UI_FillCircle(x, y, 5, color);
    UI_FillCircle(x, y, 2, UI_PANEL_BG);
}

static void UI_DrawPercentIcon(int x, int y, uint16_t color)
{
    UI_FillCircle(x, y, 5, color);
    UI_FillCircle(x + 18, y + 24, 5, color);
    UI_DrawLine(x + 2, y + 22, x + 16, y + 2, color);

    UI_FillCircle(x, y, 2, UI_PANEL_BG);
    UI_FillCircle(x + 18, y + 24, 2, UI_PANEL_BG);
}

static void UI_DrawToggleButton(const Button_t *btn, uint8_t on)
{
    uint16_t trackColor = on ? ST7796_GREEN : ST7796_RED;
    int r = (btn->h / 2) - 4;
    int knobY = btn->y + btn->h / 2;
    int knobX = on ? (btn->x + btn->w - r - 4) : (btn->x + r + 4);

    ST7796_FillRect(btn->x, btn->y, btn->w, btn->h, trackColor);
    UI_DrawRectOutline(btn->x, btn->y, btn->w, btn->h, ST7796_WHITE);
    UI_FillCircle(knobX, knobY, r, ST7796_WHITE);
}

/* ---------- 7-segment digits ---------- */
static void UI_SegH(int x, int y, int len, int thick, uint16_t color)
{
    ST7796_FillRect((uint16_t)x, (uint16_t)y, (uint16_t)len, (uint16_t)thick, color);
}

static void UI_SegV(int x, int y, int thick, int len, uint16_t color)
{
    ST7796_FillRect((uint16_t)x, (uint16_t)y, (uint16_t)thick, (uint16_t)len, color);
}

static void UI_DrawDigit(int x, int y, int digit, uint16_t onColor, uint16_t offColor, uint8_t scale)
{
    static const uint8_t seg[10][7] =
    {
        {1,1,1,1,1,1,0}, /* 0 */
        {0,1,1,0,0,0,0}, /* 1 */
        {1,1,0,1,1,0,1}, /* 2 */
        {1,1,1,1,0,0,1}, /* 3 */
        {0,1,1,0,0,1,1}, /* 4 */
        {1,0,1,1,0,1,1}, /* 5 */
        {1,0,1,1,1,1,1}, /* 6 */
        {1,1,1,0,0,0,0}, /* 7 */
        {1,1,1,1,1,1,1}, /* 8 */
        {1,1,1,1,0,1,1}  /* 9 */
    };

    int t = 2 * scale;   /* thickness */
    int l = 6 * scale;   /* segment length */

    uint16_t a = seg[digit][0] ? onColor : offColor;
    uint16_t b = seg[digit][1] ? onColor : offColor;
    uint16_t c = seg[digit][2] ? onColor : offColor;
    uint16_t d = seg[digit][3] ? onColor : offColor;
    uint16_t e = seg[digit][4] ? onColor : offColor;
    uint16_t f = seg[digit][5] ? onColor : offColor;
    uint16_t g = seg[digit][6] ? onColor : offColor;

    UI_SegH(x + t,         y,               l, t, a);               /* A */
    UI_SegV(x + l + t,     y + t,           t, l, b);               /* B */
    UI_SegV(x + l + t,     y + l + 2 * t,   t, l, c);               /* C */
    UI_SegH(x + t,         y + 2 * l + 2*t, l, t, d);               /* D */
    UI_SegV(x,             y + l + 2 * t,   t, l, e);               /* E */
    UI_SegV(x,             y + t,           t, l, f);               /* F */
    UI_SegH(x + t,         y + l + t,       l, t, g);               /* G */
}

static void UI_Draw2DigitValue(int x, int y, int value, uint16_t onColor)
{
    int tens = (value / 10) % 10;
    int ones = value % 10;

    UI_DrawDigit(x,      y, tens, onColor, UI_SEG_OFF, 3);
    UI_DrawDigit(x + 38, y, ones, onColor, UI_SEG_OFF, 3);
}

static void UI_DrawModeButton(const Button_t *btn, uint8_t showCurrent)
{
    uint16_t bg  = showCurrent ? 0x03EF : 0x8C51; /* teal = current, amber = desired */
    const char *label = showCurrent ? "CURRENT" : "DESIRED";

    ST7796_FillRect(btn->x, btn->y, btn->w, btn->h, bg);
    UI_DrawRectOutline(btn->x, btn->y, btn->w, btn->h, ST7796_WHITE);

    uint8_t  len   = 0;
    const char *p  = label;
    while (*p++) len++;
    uint16_t tx = btn->x + (btn->w - len * 6) / 2;
    uint16_t ty = btn->y + (btn->h - 7) / 2;
    ST7796_DrawString(tx, ty, label, ST7796_WHITE, bg, 1);
}

/* ---------- Panels ---------- */
static void UI_DrawTempPanel(int displayValue, uint8_t showCurrent)
{
    ST7796_FillRect(TEMP_X, TEMP_Y, TEMP_W, TEMP_H, UI_PANEL_BG);
    UI_DrawRectOutline(TEMP_X, TEMP_Y, TEMP_W, TEMP_H, ST7796_ORANGE);

    UI_Draw2DigitValue(TEMP_X + 20, TEMP_Y + 30, displayValue, ST7796_WHITE);
    UI_DrawDegreeIcon(TEMP_X + 116, TEMP_Y + 40, ST7796_ORANGE);

    UI_DrawArrowButton(&BTN_TEMP_UP,   1);
    UI_DrawArrowButton(&BTN_TEMP_DOWN, 0);
    UI_DrawModeButton(&BTN_TEMP_MODE, showCurrent);
}

static void UI_DrawHumPanel(int displayValue, uint8_t showCurrent)
{
    ST7796_FillRect(HUM_X, HUM_Y, HUM_W, HUM_H, UI_PANEL_BG);
    UI_DrawRectOutline(HUM_X, HUM_Y, HUM_W, HUM_H, ST7796_CYAN);

    UI_Draw2DigitValue(HUM_X + 20, HUM_Y + 30, displayValue, ST7796_WHITE);
    UI_DrawPercentIcon(HUM_X + 116, HUM_Y + 38, ST7796_CYAN);

    UI_DrawArrowButton(&BTN_HUM_UP,   1);
    UI_DrawArrowButton(&BTN_HUM_DOWN, 0);
    UI_DrawModeButton(&BTN_HUM_MODE, showCurrent);
}

static void UI_DrawLightPanel(uint8_t lightOn)
{
    ST7796_FillRect(LIGHT_X, LIGHT_Y, LIGHT_W, LIGHT_H, UI_PANEL_BG);
    UI_DrawRectOutline(LIGHT_X, LIGHT_Y, LIGHT_W, LIGHT_H, ST7796_YELLOW);

    UI_DrawLightIndicatorSmall(LIGHT_X, LIGHT_Y, lightOn);
    UI_DrawToggleButton(&BTN_LIGHT, lightOn);
}

static void UI_DrawCameraPanel(uint8_t baskingOccupied, uint8_t cameraPressed)
{
    ST7796_FillRect(CAM_X, CAM_Y, CAM_W, CAM_H, UI_PANEL_BG);
    UI_DrawRectOutline(CAM_X, CAM_Y, CAM_W, CAM_H, ST7796_MAGENTA);

    UI_DrawCameraButton(&BTN_CAMERA, cameraPressed);
    UI_DrawBaskingStatus(baskingOccupied);
}

static void UI_DrawAll(int screenTemp, int humidity, uint8_t lightOn, uint8_t baskingOccupied)
{
    ST7796_FillScreen(ST7796_BLACK);
    UI_DrawTempPanel(screenTemp, 0);
    UI_DrawHumPanel(humidity, 0);
    UI_DrawLightPanel(lightOn);
    UI_DrawCameraPanel(baskingOccupied, 0);
}

static void UI_DrawLightIndicatorSmall(uint16_t panelX, uint16_t panelY, uint8_t on)
{
    uint16_t lampColor = on ? ST7796_YELLOW : ST7796_GRAY;

    int boxW = 80;
    int boxH = 60;
    int boxX = panelX + (LIGHT_W - boxW) / 2;
    int boxY = panelY + 18;

    int cx = boxX + boxW / 2;
    int cy = boxY + boxH / 2;

    UI_FillCircle(cx, cy, 18, lampColor);
    UI_DrawRectOutline(boxX, boxY, boxW, boxH, ST7796_WHITE);

    if (on)
    {
        UI_DrawLine(cx - 24, cy,      cx - 34, cy,      ST7796_YELLOW);
        UI_DrawLine(cx + 24, cy,      cx + 34, cy,      ST7796_YELLOW);
        UI_DrawLine(cx,      cy - 24, cx,      cy - 34, ST7796_YELLOW);
        UI_DrawLine(cx - 17, cy - 17, cx - 25, cy - 25, ST7796_YELLOW);
        UI_DrawLine(cx + 17, cy - 17, cx + 25, cy - 25, ST7796_YELLOW);
    }
}

static void UI_DrawCameraButton(const Button_t *btn, uint8_t pressed)
{
    uint16_t bg = pressed ? ST7796_CYAN : 0x4208;
    int bx = btn->x;
    int by = btn->y;

    ST7796_FillRect(btn->x, btn->y, btn->w, btn->h, bg);
    UI_DrawRectOutline(btn->x, btn->y, btn->w, btn->h, ST7796_WHITE);

    /* camera body */
    ST7796_FillRect(bx + 28, by + 18, 58, 22, ST7796_BLACK);
    UI_DrawRectOutline(bx + 28, by + 18, 58, 22, ST7796_WHITE);

    /* top bump */
    ST7796_FillRect(bx + 42, by + 12, 16, 6, ST7796_WHITE);

    /* lens */
    UI_FillCircle(bx + 57, by + 29, 8, ST7796_WHITE);
    UI_FillCircle(bx + 57, by + 29, 4, ST7796_BLACK);
}

static void UI_DrawBaskingStatus(uint8_t occupied)
{
    uint16_t fill = occupied ? ST7796_GREEN : ST7796_RED;

    ST7796_FillRect(BTN_BASKING.x, BTN_BASKING.y, BTN_BASKING.w, BTN_BASKING.h, fill);
    UI_DrawRectOutline(BTN_BASKING.x, BTN_BASKING.y, BTN_BASKING.w, BTN_BASKING.h, ST7796_WHITE);

    /* indicator dot on left */
    UI_FillCircle(BTN_BASKING.x + 16, BTN_BASKING.y + BTN_BASKING.h / 2, 6, ST7796_WHITE);

    /* simple basking platform icon on right */
    ST7796_FillRect(BTN_BASKING.x + 122, BTN_BASKING.y + 16, 28, 4, 0xA145);  /* brown-ish */
    ST7796_DrawVLine(BTN_BASKING.x + 136, BTN_BASKING.y + 6, 10, 0xA145);

    if (occupied)
    {
        UI_FillCircle(BTN_BASKING.x + 136, BTN_BASKING.y + 10, 5, ST7796_YELLOW);
    }
    else
    {
        UI_FillCircle(BTN_BASKING.x + 136, BTN_BASKING.y + 10, 5, ST7796_GRAY);
    }
}


/* ─── Camera capture + display ─────────────────────────────────────────── */
static void UI_CaptureAndShow(void)
{
    /* ── 1. Clear screen and draw header ── */
    ST7796_FillScreen(ST7796_BLACK);
    ST7796_DrawString(10, 8, "TAP SCREEN OR PRESS BUTTON TO EXIT",
                      ST7796_WHITE, ST7796_BLACK, 1);
    ST7796_DrawHLine(0, 22, ST7796_WIDTH, ST7796_DARKGREY);

    /* Show "capturing" message while we wait */
    ST7796_DrawString(160, 160, "CAPTURING...", ST7796_YELLOW, ST7796_BLACK, 2);

    /* ── 2. Set display window for the image ── */
    /*
     * Display is landscape 480x320.
     * Image is QVGA 320x240.
     * Centre it: x offset = (480-320)/2 = 80, y offset = 28 (below header)
     */
    uint16_t img_x = (ST7796_WIDTH  - 320U) / 2U;  /* 80 */
    uint16_t img_y = 28U;

    /* Open the address window on the ST7796 — sends CASET, RASET, RAMWR */

    /* ── 3. Capture and stream directly to display ── */
    uint8_t result = Arducam_CaptureDecodeDisplay();

    if (result == ARDUCAM_ERR_TIMEOUT)
    {
        ST7796_FillScreen(ST7796_BLACK);
        ST7796_DrawString(60, 150, "CAPTURE TIMEOUT", ST7796_RED,   ST7796_BLACK, 2);
        ST7796_DrawString(60, 175, "TAP TO RETURN",   ST7796_WHITE, ST7796_BLACK, 1);
    }
    else if (result == ARDUCAM_ERR_DECODE || result == ARDUCAM_ERR_SIZE)
    {
        ST7796_FillScreen(ST7796_BLACK);
        ST7796_DrawString(80, 150, "DECODE ERROR",  ST7796_RED,   ST7796_BLACK, 2);
        ST7796_DrawString(60, 175, "TAP TO RETURN", ST7796_WHITE, ST7796_BLACK, 1);
    }
}


/* ─── Wait for any input, then redraw the dashboard ────────────────────── */
static void UI_WaitForExitThenDashboard(
        int     screenTemp,
        int     humidity,
        uint8_t lightOn,
        uint8_t baskingOccupied,
        uint8_t tempShowCurrent,
        uint8_t humShowCurrent)
{
    uint8_t exit_view = 0;

    /* Drain any stale touch event from the capture tap */
    HAL_Delay(300);

    while (!exit_view)
    {
        /* Touch anywhere exits */
        uint16_t tx, ty;
        if (Touch_GetPoint(&tx, &ty))
        {
            exit_view = 1;
        }

        /* Any BT remote button exits (consume without acting on it) */
        if (bt_rx_ready)
        {
            bt_rx_ready = 0;
            exit_view   = 1;
        }

        HAL_Delay(15);
    }

    /* Repaint the full dashboard */
    UI_DrawAll(screenTemp, humidity, lightOn, baskingOccupied);
}

typedef enum
{
    HUMIDIFIER_MODE_OFF = 0,
    HUMIDIFIER_MODE_ALWAYS_ON,
    HUMIDIFIER_MODE_CONT_PULSE
} HumidifierMode_t;

static HumidifierMode_t humidifier_mode = HUMIDIFIER_MODE_OFF;
static uint32_t last_humidifier_burst_tick = 0;

#define HUMIDIFIER_BUTTON_MS    50U
#define HUMIDIFIER_MODE_GAP_MS  50U
#define HUMIDIFIER_SPRAY_MS     25000U /* 25 second spray */
#define HUMIDIFIER_SETTLE_MS    180000U   /* 3 minutes */
#define HUMIDITY_HYSTERESIS_PCT 2.0f

static void humidifier_button_press(void)
{
    /* Simulate one mode-button press */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(HUMIDIFIER_BUTTON_MS);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(HUMIDIFIER_MODE_GAP_MS);
}

static void humidifier_advance_mode(void)
{
    humidifier_button_press();

    if (humidifier_mode == HUMIDIFIER_MODE_OFF)
    {
        humidifier_mode = HUMIDIFIER_MODE_ALWAYS_ON;
    }
    else if (humidifier_mode == HUMIDIFIER_MODE_ALWAYS_ON)
    {
        humidifier_mode = HUMIDIFIER_MODE_CONT_PULSE;
    }
    else
    {
        humidifier_mode = HUMIDIFIER_MODE_OFF;
    }
}

static void humidifier_set_mode(HumidifierMode_t target)
{
    while (humidifier_mode != target)
    {
        humidifier_advance_mode();
    }
}

static void humidifier_spray_burst(void)
{
    /* OFF -> ALWAYS ON */
    humidifier_set_mode(HUMIDIFIER_MODE_ALWAYS_ON);

    /* stay in ALWAYS ON briefly to produce one mist burst */
    HAL_Delay(HUMIDIFIER_SPRAY_MS);

    /* ALWAYS ON -> CONT_PULSE -> OFF */
    humidifier_set_mode(HUMIDIFIER_MODE_OFF);

    last_humidifier_burst_tick = HAL_GetTick();
}

static void humidifier_update(uint32_t now_ms, float current_humidity, int desired_humidity)
{
    /* Do nothing if we are already at/above target (with a little hysteresis) */
    if (current_humidity >= ((float)desired_humidity - HUMIDITY_HYSTERESIS_PCT))
    {
        return;
    }

    /* Wait after each burst so humidity has time to spread through the tank */
    if ((now_ms - last_humidifier_burst_tick) < HUMIDIFIER_SETTLE_MS)
    {
        return;
    }

    humidifier_spray_burst();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
   HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
   int screenTemp = 85;
   int humidity = 30;
   uint8_t lightOn = 0;
   uint8_t baskingOccupied = 0;
   uint8_t touchLatched = 0;
   uint8_t tempShowCurrent = 0;   /* 0=desired, 1=current */
   uint8_t humShowCurrent = 0;   /* 0=desired, 1=current */

   ST7796_Init();
   XPT2046_Init();
   AHT20_Init();

   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
   HAL_Delay(10);

   uint8_t cam_err = OV2640_JPEG_Init();
   if (cam_err != ARDUCAM_OK)
   {
       /* SPI to ArduCAM chip not working — show error and halt */
       ST7796_FillScreen(ST7796_BLACK);
       ST7796_DrawString(60, 140, "CAM SPI FAILED", ST7796_RED, ST7796_BLACK, 2);
       ST7796_DrawString(40, 165, "CHECK WIRING+NSSPMode", ST7796_WHITE, ST7796_BLACK, 1);
       while (1) {}
   }

   UI_DrawAll(screenTemp, humidity, lightOn, baskingOccupied);
   LightLed_Set(lightOn);
   uint32_t last_sensor_tick = 0U;
   const uint32_t SENSOR_INTERVAL_MS = 2000U;  /* read sensor every 2 seconds */

   // PID tuners
   float kp = 1.5, ki = 0.02, kd = 0.5;

   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   /* idle high */
   humidifier_mode = HUMIDIFIER_MODE_OFF;

   last_humidifier_burst_tick = HAL_GetTick() - HUMIDIFIER_SETTLE_MS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint16_t x, y;
	  if (Touch_GetPoint(&x, &y))
	  {
	      if (!touchLatched)
	      {
	          touchLatched = 1;

	          /* ── Temperature panel ── */
	          if (Button_Contains(&BTN_TEMP_UP, x, y))
	          {
	              if (screenTemp <= 94) screenTemp += 5;
	              UI_DrawTempPanel(tempShowCurrent ? (int)Temperature : screenTemp,
	                               tempShowCurrent);
	          }
	          else if (Button_Contains(&BTN_TEMP_DOWN, x, y))
	          {
	              if (screenTemp >= 5) screenTemp -= 5;
	              UI_DrawTempPanel(tempShowCurrent ? (int)Temperature : screenTemp,
	                               tempShowCurrent);
	          }
	          else if (Button_Contains(&BTN_TEMP_MODE, x, y))
	          {
	              tempShowCurrent = !tempShowCurrent;
	              UI_DrawTempPanel(tempShowCurrent ? (int)Temperature : screenTemp,
	                               tempShowCurrent);
	          }

	          /* ── Humidity panel ── */
	          else if (Button_Contains(&BTN_HUM_UP, x, y))
	          {
	              if (humidity <= 94) humidity += 5;
	              UI_DrawHumPanel(humShowCurrent ? (int)Humidity : humidity,
	                              humShowCurrent);
	          }
	          else if (Button_Contains(&BTN_HUM_DOWN, x, y))
	          {
	              if (humidity >= 5) humidity -= 5;
	              UI_DrawHumPanel(humShowCurrent ? (int)Humidity : humidity,
	                              humShowCurrent);
	          }
	          else if (Button_Contains(&BTN_HUM_MODE, x, y))
	          {
	              humShowCurrent = !humShowCurrent;
	              UI_DrawHumPanel(humShowCurrent ? (int)Humidity : humidity,
	                              humShowCurrent);
	          }

	          /* ── Light panel ── */
	          else if (Button_Contains(&BTN_LIGHT, x, y))
	          {
	              lightOn = !lightOn;
	              LightLed_Set(lightOn);
	              UI_DrawLightPanel(lightOn);
	          }

	          /* ── Camera panel ── */
	          else if (Button_Contains(&BTN_CAMERA, x, y))
	          {
	              UI_DrawCameraPanel(baskingOccupied, 1);
	              HAL_Delay(120);
	              UI_CaptureAndShow();
	              UI_WaitForExitThenDashboard(screenTemp, humidity, lightOn,
	                                          baskingOccupied,
	                                          tempShowCurrent, humShowCurrent);
	          }
	          else if (Button_Contains(&BTN_BASKING, x, y))
	          {
	              baskingOccupied = !baskingOccupied;
	              UI_DrawCameraPanel(baskingOccupied, 0);
	          }
	      }
	  }
	  else
	  {
	      touchLatched = 0;
	  }

	  if (bt_rx_ready)
	  {
	      uint8_t cmd  = bt_rx_byte;   /* snapshot before ISR overwrites */
	      bt_rx_ready  = 0;            /* clear */

	      switch (cmd)
	      {
	          case BT_CMD_TEMP_UP:
	              if (screenTemp <= 94) screenTemp += 5;
	              UI_DrawTempPanel(tempShowCurrent ? (int)Temperature : screenTemp,
	                               tempShowCurrent);
	              break;

	          case BT_CMD_TEMP_DOWN:
	              if (screenTemp >= 5) screenTemp -= 5;
	              UI_DrawTempPanel(tempShowCurrent ? (int)Temperature : screenTemp,
	                               tempShowCurrent);
	              break;

	          case BT_CMD_HUM_UP:
	              if (humidity <= 94) humidity += 5;
	              UI_DrawHumPanel(humShowCurrent ? (int)Humidity : humidity,
	                              humShowCurrent);
	              break;

	          case BT_CMD_HUM_DOWN:
	              if (humidity >= 5) humidity -= 5;
	              UI_DrawHumPanel(humShowCurrent ? (int)Humidity : humidity,
	                              humShowCurrent);
	              break;

	          case BT_CMD_LIGHT:
	              lightOn = !lightOn;
	              LightLed_Set(lightOn);
	              UI_DrawLightPanel(lightOn);
	              break;

	          case BT_CMD_CAMERA:
	              UI_DrawCameraPanel(baskingOccupied, 1);
	              HAL_Delay(120);
	              UI_CaptureAndShow();
	              UI_WaitForExitThenDashboard(screenTemp, humidity, lightOn,
	                                          baskingOccupied,
	                                          tempShowCurrent, humShowCurrent);
	              break;

	          default:
	              break;
	      }
	  }

	  uint32_t now = HAL_GetTick();
	  if (now - last_sensor_tick >= SENSOR_INTERVAL_MS)
	  {
	      last_sensor_tick = now;

	      AHT20_Measure();

	      humidifier_update(now, Humidity, humidity);

	      PIDloop(Temperature, screenTemp, kp, ki, kd, false);

	      if (tempShowCurrent)
	      {
	          UI_Draw2DigitValue(TEMP_X + 20, TEMP_Y + 30, (int)Temperature, ST7796_WHITE);
	      }
	      if (humShowCurrent)
	      {
	          UI_Draw2DigitValue(HUM_X + 20, HUM_Y + 30, (int)Humidity, ST7796_WHITE);
	      }
	  }

	  HAL_Delay(15);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10A077A8;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10A077A8;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|LD3_Pin|LD2_Pin
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|USB_PowerSwitchOn_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 LD3_Pin LD2_Pin
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|LD3_Pin|LD2_Pin
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 USB_PowerSwitchOn_Pin PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|USB_PowerSwitchOn_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_TX_Pin STLINK_RX_Pin */
  GPIO_InitStruct.Pin = STLINK_TX_Pin|STLINK_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Set the ready flag */
        bt_rx_ready = 1;

        /* Re-arm immediately so we don't miss next byte */
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
