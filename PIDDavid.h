#ifndef PID_H
#define PID_H


#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
unsigned long currTime;
float sweep;
float prop;
float integral;
float deriv;
float error;
float prevError;
extern UART_HandleTypeDef hlpuart1;
static uint8_t heaterState = 0;

// void debug_print(const char *fmt, ...) {
//    char buffer[128];
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(buffer, sizeof(buffer), fmt, args);
//    va_end(args);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
// }

void PIDsetup(){
  currTime = HAL_GetTick();
  sweep = 0.1;
  prop = 0;
  integral = 0;
  deriv = 0;
  error = 0;
  prevError = 0;
}

float PIDloop (float currTemp, int tTemp, float Kp, float Ki, float Kd, bool reset){
	float dt = (HAL_GetTick() - currTime) / 1000.0;
	    if (dt <= 0) dt = 0.001f;
	    currTime = HAL_GetTick();

	    error = tTemp - currTemp;

	    // 1. Reset or Calculate Integral with a TIGHTER Clamp
	    if (reset) {
	        integral = 0;
	    } else {
	        integral += error * dt * Ki;
	    }

	    // Clamp Integral to 50 instead of 100 to prevent massive overshoot
	    if (integral > 50) integral = 50;
	    if (integral < -50) integral = -50;

	    prop = Kp * error;
	    deriv = Kd * (error - prevError) / dt;
	    prevError = error;

	    float output = prop + integral + deriv;

	    // 2. Control Heater based on PID Output
	    // This allows the PID to decide when to shut off BEFORE hitting the target
	    if (output > 0) {
	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
	        heaterState = 1;
	    } else {
	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
	        heaterState = 0;
	    }

    // ---- DEBUG PRINT ----
    char msg[120];
    snprintf(msg, sizeof(msg),
        "[Temp=%.2f][Target=%d][P=%.2f][I=%.2f][D=%.2f][OUT=%.2f][H=%d]\r\n",
        currTemp, tTemp, prop, integral, deriv, output, heaterState);

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    return output;

}

#endif
