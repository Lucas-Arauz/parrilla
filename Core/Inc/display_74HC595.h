// display_74hc595.h
#ifndef DISPLAY_74HC595_H
#define DISPLAY_74HC595_H

#include "stm32f1xx_hal.h"

// Definición de pines
#define ENABLE_PIN     enable_port, enable_pin
#define LATCH_PIN      laclk_port, laclk_pin
#define SHIFT_PIN      shclk_port, shclk_pin
#define RESET_PIN      reset_port, reset_pin
#define DATA_PIN       data_port, data_pin

#define UNIT_PIN       unidad_port, unidad_pin
#define TENS_PIN       decena_port, decena_pin
#define HUNDRED_PIN    centena_port, centena_pin

#define UNIDAD	0
#define DECENA	1
#define CENTENA	2

#define UN_DIGITO		1
#define DOS_DIGITOS		2
#define TRES_DIGITOS	3

#define SEGMENTO_a 0x01
#define SEGMENTO_b 0x02
#define SEGMENTO_c 0x04
#define SEGMENTO_d 0x08
#define SEGMENTO_e 0x10
#define SEGMENTO_f 0x20
#define SEGMENTO_g 0x40

// Prototipos de funciones
void Display_PIN_EN_Assigment(GPIO_TypeDef *GPIO_enable, uint16_t PIN_enable);
void Display_PIN_LACLK_Assigment(GPIO_TypeDef *GPIO_laclk, uint16_t PIN_laclk);
void Display_PIN_SHCLK_Assigment(GPIO_TypeDef *GPIO_shclk, uint16_t PIN_shclk);
void Display_PIN_RESET_Assigment(GPIO_TypeDef *GPIO_reset, uint16_t PIN_reset);
void Display_PIN_DATA_Assigment(GPIO_TypeDef *GPIO_data, uint16_t PIN_data);
void Display_PIN_UNIDAD_Assigment(GPIO_TypeDef *GPIO_unidad, uint16_t PIN_unidad);
void Display_PIN_DECENA_Assigment(GPIO_TypeDef *GPIO_decena, uint16_t PIN_decena);
void Display_PIN_CENTENA_Assigment(GPIO_TypeDef *GPIO_centena, uint16_t PIN_centena);
void Display_Init(void);
void Display_SendByte(uint8_t data);
void Display_Update(uint16_t number);	// Actualiza el numero que se va a mostrar
void Display_UpdateDigit(void);			// Proceso de encendido de digito

#endif
