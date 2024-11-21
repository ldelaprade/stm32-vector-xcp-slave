
#include <stdint.h>
#include "patch.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_def.h"
#include "xcp_measures.h"
#include "cmsis_os.h"


//---------------------------------------------------------------
//  __  ___      ___  ___          __   __   ___  __   __  
// /__`  |   /\   |  |__      /\  /  ` /  ` |__  /__` /__` 
// .__/  |  /~~\  |  |___    /~~\ \__, \__, |___ .__/ .__/ 
//
// (c) https://manytools.org/hacker-tools/ascii-banner/
//---------------------------------------------------------------

extern osMutexId xcpStateAccessMutexHandle; // generated in main.c by CubeMX

//#define MTA_TEMPERATURE     0x1000
static volatile float temperature;
float GetTemperature(void)    { return temperature; }

//#define MTA_BUTTON_STATUS   0x2000
static volatile uint8_t buttonState;
uint8_t GetButtonState(void) { return buttonState; }

uint8_t XcpReadMta( uint32_t MtaAddr, uint8_t size, uint8_t *data )
{
    uint8_t *mtaData = NULL; 
    switch(MtaAddr)
    {
        case MTA_TEMPERATURE: mtaData = (uint8_t *)&temperature; break;
        case MTA_BUTTON_STATUS: mtaData = (uint8_t *)&buttonState; break;
    }

    if(mtaData && (xSemaphoreTake(xcpStateAccessMutexHandle, portMAX_DELAY) == pdTRUE) )
    {
      
        memcpy(data, mtaData, size);
        xSemaphoreGive(xcpStateAccessMutexHandle);
        return size;
    }

    return 0; // Access violation ?
}

//--------------------------------------------------------------------------------------
//  __  ___      ___  ___     __             __                 ___  __   __   __  
// /__`  |   /\   |  |__     /  `  /\  |    /  ` |  | |     /\   |  /  \ |__) /__` 
// .__/  |  /~~\  |  |___    \__, /~~\ |___ \__, \__/ |___ /~~\  |  \__/ |  \ .__/ 
//
//--------------------------------------------------------------------------------------

// ╔╦╗┌─┐┌┬┐┌─┐┌─┐┬─┐┌─┐┌┬┐┬ ┬┬─┐┌─┐
//  ║ ├┤ │││├─┘├┤ ├┬┘├─┤ │ │ │├┬┘├┤ 
//  ╩ └─┘┴ ┴┴  └─┘┴└─┴ ┴ ┴ └─┘┴└─└─┘

ADC_HandleTypeDef hadc1;

// Internal temperature sensor calibration values
#define TS_CAL1 ((uint16_t*)((uint32_t)0x1FF0F44C))
#define TS_CAL2 ((uint16_t*)((uint32_t)0x1FF0F44E))
#define VREFINT_CAL ((uint16_t*)((uint32_t)0x1FF0F44A))
#define TS_CAL1_TEMP 30.0f  // Calibration temperature for TS_CAL1 (30°C)
#define TS_CAL2_TEMP 110.0f // Calibration temperature for TS_CAL2 (110°C)

// Function to get ADC value
uint32_t Get_Temperature_ADC_Value(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

// Convert ADC value to temperature (Celsius)
void CalcTemperature(void)
{
    uint16_t ts_cal1 = *TS_CAL1;
    uint16_t ts_cal2 = *TS_CAL2;
    uint32_t adc = Get_Temperature_ADC_Value();
    temperature = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / ((float)ts_cal2 - ts_cal1)) * (adc - ts_cal1) + TS_CAL1_TEMP;
}

// ╔╗ ┬ ┬┌┬┐┌┬┐┌─┐┌┐┌  ╔═╗┌┬┐┌─┐┌┬┐┌─┐
// ╠╩╗│ │ │  │ │ ││││  ╚═╗ │ ├─┤ │ ├┤ 
// ╚═╝└─┘ ┴  ┴ └─┘┘└┘  ╚═╝ ┴ ┴ ┴ ┴ └─┘

void CalcButtonState(void) 
{
    // Implement the actual button status reading.
    buttonState =  (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ? 1 : 0);
}




void RefreshState(void)
{
    // Refresh states for XCP Measures
    if (xSemaphoreTake(xcpStateAccessMutexHandle, portMAX_DELAY) == pdTRUE)
    {                
        // Critical section
        CalcTemperature();
        CalcButtonState();
        // Give the mutex back
        xSemaphoreGive(xcpStateAccessMutexHandle);
    }

}
