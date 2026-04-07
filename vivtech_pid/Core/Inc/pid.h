#ifndef PID_H
#define PID_H

#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

unsigned long currTime;
float sweep;
float prop;
float integral;
float deriv;
float error;
float prevError;

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
    currTime=HAL_GetTick();
    error = tTemp - currTemp;
    
    prop = Kp * error;

    integral += error * dt * Ki;
    if (reset) {integral = 0;}
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;

    deriv = Kd * (error - prevError) / dt;
    prevError = error;

    float output = deriv + integral + prop; 

    // Control
    float buffer = 1.0; // buffer to prevent overshoot adjust as needed

    if (currTemp < tTemp - buffer) {
        HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_SET); // HEATER ON Send 5
        heaterState = 1;
    }
    else if (currTemp > tTemp - buffer) {
        HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET); // HEATER OFF Send 4
        heaterState = 0;
    }

    // ---- DEBUG PRINT ----
    char msg[120];
    sprintf(msg,
        "[Temp=%.2f][Target=%d][P=%.2f][I=%.2f][D=%.2f][OUT=%.2f][H=%d]\r\n",
        currTemp, tTemp, prop, integral, deriv, output, heaterState);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    return output;

}

#endif
