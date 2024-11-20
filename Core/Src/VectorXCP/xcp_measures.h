#ifndef XCP_MEASURES_H
#define XCP_MEASURES_H


#include <stdint.h>


// MTA addresses 
#define MTA_TEMPERATURE     0x1000
#define MTA_BUTTON_STATUS   0x2000

void CalcTemperature(void);
float GetTemperature(void);

void CalcButtonState(void);
uint8_t GetButtonState(void);

uint8_t XcpReadMta( uint32_t MtaAddr, uint8_t size, uint8_t *data );

void RefreshState(void);



#endif
