#include "watchdog.h"
#include "main.h"

// limits
#define TEMP_MARGIN     5.0
#define TEMP_MAX        100.0 // lizard has max of around 104
#define TEMP_MIN        60.0 // below this frank gets sick

#define FAULT_TRIGGER_TIME_MS   2000 // time for trigger
#define RECOVER_TIME_MS         2000 // time for recovery

static uint8_t fault = 0;

// TIMERS
static uint32_t fault_starttime = 0;
static uint32_t safe_starttime  = 0;

// Initalize checker
void Watchdog_Init(void) {
    fault = 0;
    fault_starttime = 0;
    safe_starttime = 0;
}

void Watchdog_Update(float currentTemp, int targetTemp)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t unsafe = 0;

    // check the unsfae conditions
    if (currentTemp > (targetTemp + TEMP_MARGIN))
        unsafe = 1;

    if (currentTemp > TEMP_MAX || currentTemp < TEMP_MIN)
        unsafe = 1;

    // Check if unsafe
    if (unsafe)
    {
        if (fault_starttime == 0)
            fault_starttime = now_ms;

        if ((now_ms - fault_starttime) >= FAULT_TRIGGER_TIME_MS) // fault must persist for at least 2s
            fault = 1;

        safe_starttime = 0; // reset recovery timer
    }
    else
    {
        fault_starttime = 0;

        if (fault)
        {
            if (safe_starttime == 0)
                safe_starttime = now_ms;

            if ((now_ms - safe_starttime) >= RECOVER_TIME_MS) // safe must persist for at least 2 seconds before turning fault off
                fault = 0;
        }
        else
        {
            safe_starttime = 0;
        }
    }

    // Shut down cuz we know fault persisted
    if (fault)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

uint8_t Watchdog_IsSafe(void){ // yay it's safe so no fault. Frank lives
    return (fault == 0);
}

