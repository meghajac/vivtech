#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include "main.h"

void Watchdog_Init(void);
void Watchdog_Update(float currentTemp, int targetTemp);

uint8_t Watchdog_IsSafe(void); // 1 if safe, 0 otherwise

#endif