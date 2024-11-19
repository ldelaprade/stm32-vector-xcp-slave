#ifndef XCP_ETH_SLAVE_H
#define XCP_ETH_SLAVE_H

#include "stm32f7xx_hal.h"

HAL_StatusTypeDef XCP_Eth_Slave_Init(void);
HAL_StatusTypeDef XCP_Udp_Send_Response(uint8_t* data, uint16_t length);


#endif
