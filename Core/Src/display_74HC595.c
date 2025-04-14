/*
 * display_74HC595.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Lucas
 */


#include "display_74hc595.h"

extern TIM_HandleTypeDef htim1;

// Mapa de segmentos para dígitos 0-9
const uint8_t digit_map[10] = {
    0x3F, // 0 -> 0b00111111
    0x06, // 1 -> 0b00000110
    0x5B, // 2 -> 0b01011011
    0x4F, // 3 -> 0b01001111
    0x66, // 4 -> 0b01100110
    0x6D, // 5 -> 0b01101101
    0x7D, // 6 -> 0b01111101
    0x07, // 7 -> 0b00000111
    0x7F, // 8 -> 0b01111111
    0x6F, // 9 -> 0b01101111
};

const uint8_t special_digit_map[] = {
	0x01, // segmento a -> 0b00000001
	0x02, // segmento b -> 0b00000010
	0x04, // segmento c -> 0b00000100
	0x08, // segmento d -> 0b00001000
	0x10, // segmento e -> 0b00010000
	0x20, // segmento f -> 0b00100000
	0x40  // segmento g -> 0b01000000
};

static uint16_t current_number = 0;
static uint8_t current_digit = 0;

uint8_t cant_digitos_encendidos = TRES_DIGITOS;

GPIO_TypeDef *enable_port;
uint16_t enable_pin;
GPIO_TypeDef *laclk_port;
uint16_t laclk_pin;
GPIO_TypeDef *shclk_port;
uint16_t shclk_pin;
GPIO_TypeDef *reset_port;
uint16_t reset_pin;
GPIO_TypeDef *data_port;
uint16_t data_pin;
GPIO_TypeDef *unidad_port;
uint16_t unidad_pin;
GPIO_TypeDef *decena_port;
uint16_t decena_pin;
GPIO_TypeDef *centena_port;
uint16_t centena_pin;

void Display_PIN_EN_Assigment(GPIO_TypeDef *GPIO_enable, uint16_t PIN_enable) {
	enable_port = GPIO_enable;
	enable_pin = PIN_enable;
}

void Display_PIN_LACLK_Assigment(GPIO_TypeDef *GPIO_laclk, uint16_t PIN_laclk) {
	laclk_port = GPIO_laclk;
	laclk_pin = PIN_laclk;
}

void Display_PIN_SHCLK_Assigment(GPIO_TypeDef *GPIO_shclk, uint16_t PIN_shclk) {
	shclk_port = GPIO_shclk;
	shclk_pin = PIN_shclk;
}

void Display_PIN_RESET_Assigment(GPIO_TypeDef *GPIO_reset, uint16_t PIN_reset) {
	reset_port = GPIO_reset;
	reset_pin = PIN_reset;
}

void Display_PIN_DATA_Assigment(GPIO_TypeDef *GPIO_data, uint16_t PIN_data) {
	data_port = GPIO_data;
	data_pin = PIN_data;
}

void Display_PIN_UNIDAD_Assigment(GPIO_TypeDef *GPIO_unidad, uint16_t PIN_unidad) {
	unidad_port = GPIO_unidad;
	unidad_pin = PIN_unidad;
}

void Display_PIN_DECENA_Assigment(GPIO_TypeDef *GPIO_decena, uint16_t PIN_decena) {
	decena_port = GPIO_decena;
	decena_pin = PIN_decena;
}

void Display_PIN_CENTENA_Assigment(GPIO_TypeDef *GPIO_centena, uint16_t PIN_centena) {
	centena_port = GPIO_centena;
	centena_pin = PIN_centena;
}

void Display_Init(void) {
    HAL_GPIO_WritePin(RESET_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RESET_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_RESET);
}

void Display_SendByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(DATA_PIN, (data & 0x80) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(SHIFT_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SHIFT_PIN, GPIO_PIN_RESET);
        data <<= 1;
    }
    HAL_GPIO_WritePin(LATCH_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LATCH_PIN, GPIO_PIN_RESET);
}

void Display_Update(uint16_t number) {
    current_number = number;
}

void Display_UpdateDigit(void) {
	HAL_GPIO_WritePin(HUNDRED_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UNIT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TENS_PIN, GPIO_PIN_SET);

	uint8_t units = current_number % 10;
	uint8_t tens = (current_number / 10) % 10;
	uint8_t hundreds = (current_number / 100) % 10;
	if(hundreds == 0 && tens == 0)
		cant_digitos_encendidos = UN_DIGITO;
	else if(hundreds == 0)
		cant_digitos_encendidos = DOS_DIGITOS;
	else
		cant_digitos_encendidos = TRES_DIGITOS;

    switch (current_digit) {
        case UNIDAD:
        	Display_SendByte(digit_map[units]);
            HAL_GPIO_WritePin(UNIT_PIN, GPIO_PIN_RESET);
        break;

        case DECENA:
        	Display_SendByte(digit_map[tens]);
            HAL_GPIO_WritePin(TENS_PIN, GPIO_PIN_RESET);
        break;

        case CENTENA:
        	Display_SendByte(digit_map[hundreds]);
            HAL_GPIO_WritePin(HUNDRED_PIN, GPIO_PIN_RESET);
        break;
    }

    current_digit = (current_digit + 1) % cant_digitos_encendidos;	// Rango ciclico 0-1-2
}

