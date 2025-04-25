/*
 * MAX6675.h
 *
 *  Created on: Mar 17, 2025
 *      Author: Lucas
 */

#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_

#include "stm32f1xx_hal.h"

void MAX6675_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
float MAX6675_ReadTemperature(void);

#endif /* INC_MAX6675_H_ */
