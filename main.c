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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_8

static void LightLed_Set(uint8_t on)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} Button_t;

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
#define LIGHT_W     456
#define LIGHT_H     140

#define UI_PANEL_BG     0x1082   /* dark gray-blue */
#define UI_SEG_OFF      0x2945   /* dim segment color */

static const Button_t BTN_TEMP_UP   = { TEMP_X + 170, TEMP_Y + 18, 34, 40 };
static const Button_t BTN_TEMP_DOWN = { TEMP_X + 170, TEMP_Y + 82, 34, 40 };

static const Button_t BTN_HUM_UP    = { HUM_X + 170, HUM_Y + 18, 34, 40 };
static const Button_t BTN_HUM_DOWN  = { HUM_X + 170, HUM_Y + 82, 34, 40 };

static const Button_t BTN_LIGHT     = { LIGHT_X + 300, LIGHT_Y + 38, 120, 60 };

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
    int knobX = on ? (btn->x + btn->w - 28) : (btn->x + 28);
    int knobY = btn->y + btn->h / 2;

    ST7796_FillRect(btn->x, btn->y, btn->w, btn->h, trackColor);
    UI_DrawRectOutline(btn->x, btn->y, btn->w, btn->h, ST7796_WHITE);
    UI_FillCircle(knobX, knobY, 18, ST7796_WHITE);
}

static void UI_DrawLightIndicator(uint8_t on)
{
    uint16_t lampColor = on ? ST7796_YELLOW : ST7796_GRAY;
    int cx = LIGHT_X + 85;
    int cy = LIGHT_Y + 70;

    UI_FillCircle(cx, cy, 26, lampColor);
    UI_DrawRectOutline(LIGHT_X + 35, LIGHT_Y + 25, 100, 90, ST7796_WHITE);

    if (on)
    {
        UI_DrawLine(cx - 34, cy, cx - 48, cy, ST7796_YELLOW);
        UI_DrawLine(cx + 34, cy, cx + 48, cy, ST7796_YELLOW);
        UI_DrawLine(cx, cy - 34, cx, cy - 48, ST7796_YELLOW);
        UI_DrawLine(cx - 24, cy - 24, cx - 36, cy - 36, ST7796_YELLOW);
        UI_DrawLine(cx + 24, cy - 24, cx + 36, cy - 36, ST7796_YELLOW);
    }
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

/* ---------- Panels ---------- */
static void UI_DrawTempPanel(int temperature)
{
    ST7796_FillRect(TEMP_X, TEMP_Y, TEMP_W, TEMP_H, UI_PANEL_BG);
    UI_DrawRectOutline(TEMP_X, TEMP_Y, TEMP_W, TEMP_H, ST7796_ORANGE);

    UI_Draw2DigitValue(TEMP_X + 20, TEMP_Y + 38, temperature, ST7796_WHITE);
    UI_DrawDegreeIcon(TEMP_X + 116, TEMP_Y + 48, ST7796_ORANGE);

    UI_DrawArrowButton(&BTN_TEMP_UP, 1);
    UI_DrawArrowButton(&BTN_TEMP_DOWN, 0);
}

static void UI_DrawHumPanel(int humidity)
{
    ST7796_FillRect(HUM_X, HUM_Y, HUM_W, HUM_H, UI_PANEL_BG);
    UI_DrawRectOutline(HUM_X, HUM_Y, HUM_W, HUM_H, ST7796_CYAN);

    UI_Draw2DigitValue(HUM_X + 20, HUM_Y + 38, humidity, ST7796_WHITE);
    UI_DrawPercentIcon(HUM_X + 116, HUM_Y + 46, ST7796_CYAN);

    UI_DrawArrowButton(&BTN_HUM_UP, 1);
    UI_DrawArrowButton(&BTN_HUM_DOWN, 0);
}

static void UI_DrawLightPanel(uint8_t lightOn)
{
    ST7796_FillRect(LIGHT_X, LIGHT_Y, LIGHT_W, LIGHT_H, UI_PANEL_BG);
    UI_DrawRectOutline(LIGHT_X, LIGHT_Y, LIGHT_W, LIGHT_H, ST7796_YELLOW);

    UI_DrawLightIndicator(lightOn);
    UI_DrawToggleButton(&BTN_LIGHT, lightOn);
}

static void UI_DrawAll(int temperature, int humidity, uint8_t lightOn)
{
    ST7796_FillScreen(ST7796_BLACK);
    UI_DrawTempPanel(temperature);
    UI_DrawHumPanel(humidity);
    UI_DrawLightPanel(lightOn);
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
  /* USER CODE BEGIN 2 */
  int temperature = 72;
  int humidity = 45;
  uint8_t lightOn = 0;
  uint8_t touchLatched = 0;

  ST7796_Init();
  XPT2046_Init();

  UI_DrawAll(temperature, humidity, lightOn);
  LightLed_Set(lightOn);


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

	            if (Button_Contains(&BTN_TEMP_UP, x, y))
	            {
	                if (temperature < 99) temperature++;
	                UI_DrawTempPanel(temperature);
	            }
	            else if (Button_Contains(&BTN_TEMP_DOWN, x, y))
	            {
	                if (temperature > 0) temperature--;
	                UI_DrawTempPanel(temperature);
	            }
	            else if (Button_Contains(&BTN_HUM_UP, x, y))
	            {
	                if (humidity < 99) humidity++;
	                UI_DrawHumPanel(humidity);
	            }
	            else if (Button_Contains(&BTN_HUM_DOWN, x, y))
	            {
	                if (humidity > 0) humidity--;
	                UI_DrawHumPanel(humidity);
	            }
	            else if (Button_Contains(&BTN_LIGHT, x, y))
	            {
	                lightOn = !lightOn;
	                LightLed_Set(lightOn);     // real-world LED control
	                UI_DrawLightPanel(lightOn);
	            }
	        }
	    }
	    else
	    {
	        touchLatched = 0;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
