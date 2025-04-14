/*
 * MAX6675.c
 *
 *  Created on: Mar 17, 2025
 *      Author: Lucas
 */


#include "MAX6675.h"

GPIO_TypeDef *MAX6675_CS_PORT;
uint16_t MAX6675_CS_PIN;
static SPI_HandleTypeDef *max6675_hspi;

void MAX6675_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN) {
    max6675_hspi = hspi;
    MAX6675_CS_PORT = CS_PORT;
    MAX6675_CS_PIN = CS_PIN;
    HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET);
}

float MAX6675_ReadTemperature(void) {
    uint8_t data[2] = {0};
    uint16_t raw_value;
    float temperature;

    HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_Receive(max6675_hspi, data, 2, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET);

    raw_value = (data[0] << 8) | data[1];

    if (raw_value & 0x4) {
        return -1.0;  // Sensor no conectado
    }

    raw_value >>= 3; // Descartar los 3 bits menos significativos
    temperature = raw_value * 0.25;

    return temperature;
}

