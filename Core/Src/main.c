/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <display_74HC595.h>
#include <MAX6675.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STAND_BY,
	RAMPA_SUBIDA,
	CONTROL,
	TERMINANDO
} Estados_t;

typedef enum {
	CLEAR,
	TEMP_MAS,
	TEMP_MENOS,
	TIEMPO_MAS,
	TIEMPO_MENOS,
	ST
} Botones_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_grados_on() 	HAL_GPIO_WritePin(LED_INDIC_GPIO_Port, LED_INDIC_Pin, GPIO_PIN_SET)
#define LED_minutos_on()	HAL_GPIO_WritePin(LED_INDIC_GPIO_Port, LED_INDIC_Pin, GPIO_PIN_RESET)
#define LED_proceso_on()	HAL_GPIO_WritePin(LED_PROCESO_GPIO_Port, LED_PROCESO_Pin, GPIO_PIN_SET)
#define LED_proceso_off()	HAL_GPIO_WritePin(LED_PROCESO_GPIO_Port, LED_PROCESO_Pin, GPIO_PIN_RESET)
#define LED_CALIENTE_on()	HAL_GPIO_WritePin(LED_CALIENTE_GPIO_Port, LED_CALIENTE_Pin, GPIO_PIN_SET)
#define LED_CALIENTE_off()	HAL_GPIO_WritePin(LED_CALIENTE_GPIO_Port, LED_CALIENTE_Pin, GPIO_PIN_RESET)
#define CALEFACTOR_on()		HAL_GPIO_WritePin(CALEFACTOR_GPIO_Port, CALEFACTOR_Pin, GPIO_PIN_SET)
#define CALEFACTOR_off()	HAL_GPIO_WritePin(CALEFACTOR_GPIO_Port, CALEFACTOR_Pin, GPIO_PIN_RESET)
#define DISPLAY_on()		HAL_GPIO_WritePin(HC595_OE_GPIO_Port, HC595_OE_Pin, GPIO_PIN_RESET)
#define DISPLAY_off()		HAL_GPIO_WritePin(HC595_OE_GPIO_Port, HC595_OE_Pin, GPIO_PIN_SET)
#define DISPLAY_toggle()	HAL_GPIO_TogglePin(HC595_OE_GPIO_Port, HC595_OE_Pin)

#define MAX_TEMP 	400	// en grados celsius
#define MIN_TEMP 	80
#define MAX_TIEMPO 	900	// en minutos
#define MIN_TIEMPO	0

#define NUM_MEDICIONES 15

//#define KP 4.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Variables de control de temperatura
float temperatura_leida = 0;
float temperatura_leida_antes = 0;
float temperatura_filtrada = 0;
int setpoint = 80;
float alpha;
float temperature_buffer[NUM_MEDICIONES] = {0};
uint8_t buffer_index = 0;
float sum = 0;
int cont_duty_cycle = 0;

bool leer_temp = false;
bool controlar = false;

int tiempo_coccion = 20;
int tiempo_coccion_segs;
int tiempo_on_SSR = 0;

float Kp = 2.0;
float Ki = 0.5;
float Kd = 1.0;

float integral = 0;
float previousError = 0;
uint32_t relayOnTimeMs = 0; // Duración que el relé debe estar encendido

uint8_t relayState = 0; // 1 = encendido, 0 = apagado
uint32_t relayTimerCounter = 0; // En ms

// Varibles de estados de control
Estados_t estado_anterior = STAND_BY;
Estados_t estado = STAND_BY;
Estados_t estado_siguiente = STAND_BY;

// Variables de control de botones
Botones_t boton = CLEAR;
uint32_t tiempo_ultima_interrupcion = 0;
int cont_toques_temp = 0;
int cont_toques_tiempo = 0;

// Variables de control de display
int tiempo_mostrar_set = 0;
int cont_parpadeo_display = 0;
int cont_mostrar_temp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void config_HC595();
void Send_Debug_Info(float averageTemp, float error, float output, uint32_t relayOnTimeMs, uint8_t relayState);
void UART_Receive_And_Parse_Command(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  MAX6675_Init(&hspi1, MAX6675_CS_GPIO_Port, MAX6675_CS_Pin);	// Iniciliza el MAX6675 (termocupla)
  temperatura_leida = MAX6675_ReadTemperature();				// Lee la temperatura por primera vez

  config_HC595();	// Configura los pines del 74HC595 para poder utilizar la libreria
  Display_Init();
  HAL_TIM_Base_Start_IT(&htim1);

  LED_grados_on();

  tiempo_coccion_segs = tiempo_coccion * 120;
  temperatura_leida_antes = temperatura_leida;


  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(leer_temp)
	  {
		  leer_temp = false;
		  temperatura_leida = MAX6675_ReadTemperature();

		  if(temperatura_leida != -1) {
			  // Promedio de las ultimas 10 mediciones (buffer anillo)
			  sum -= temperature_buffer[buffer_index];				// Restar el valor mas antiguo de la suma
			  temperature_buffer[buffer_index] = temperatura_leida;	// Almacenar el nuevo valor en el buffer
			  sum += temperatura_leida;								// Sumar el nuevo valor a la suma
			  buffer_index = (buffer_index + 1) % NUM_MEDICIONES;	// Avanzar el índice en anillo
			  temperatura_filtrada = sum / NUM_MEDICIONES;			// Guarda el promedio
		  }

		  if(tiempo_mostrar_set > 0)
		  {
			  tiempo_mostrar_set--;
			  DISPLAY_toggle();
		  }
		  else if(cont_mostrar_temp > 30)
		  {
			  DISPLAY_on();
			  Display_Update(tiempo_coccion);
			  LED_minutos_on();
	  	  }
		  else
		  {
			  DISPLAY_on();
			  Display_Update(temperatura_filtrada);
			  LED_grados_on();
			  cont_toques_temp = 0;
			  cont_toques_tiempo = 0;
		  }
	  }

	  if(boton != CLEAR)
	  {
		  switch(boton)
		  {
			  case CLEAR:
			  break;

			  case TEMP_MAS:
				  if(HAL_GPIO_ReadPin(TOUCH_TEMPMAS_GPIO_Port, TOUCH_TEMPMAS_Pin)) {
					  cont_toques_temp++;
					  tiempo_mostrar_set = 10;
					  LED_grados_on();

					  if(cont_toques_temp > 1) {
						  if(setpoint >= MAX_TEMP)
							  setpoint = MAX_TEMP;
						  else
							  setpoint += 10;
					  }
					  Display_Update(setpoint);
					  boton = CLEAR;
				  }

			  break;

			  case TEMP_MENOS:
				  if(HAL_GPIO_ReadPin(TOUCH_TEMPMENOS_GPIO_Port, TOUCH_TEMPMENOS_Pin)) {
					  cont_toques_temp++;
					  tiempo_mostrar_set = 10;

					  LED_grados_on();
					  if(cont_toques_temp > 1) {
						  if(setpoint <= MIN_TEMP)
							  setpoint = MIN_TEMP;
						  else
							  setpoint -= 10;
					  }
					  Display_Update(setpoint);
					  boton = CLEAR;
				  }
			  break;

			  case TIEMPO_MAS:
				  if(HAL_GPIO_ReadPin(TOUCH_TIEMPOMAS_GPIO_Port, TOUCH_TIEMPOMAS_Pin)) {
					  cont_toques_tiempo++;
					  tiempo_mostrar_set = 10;
					  LED_minutos_on();

					  if(cont_toques_tiempo > 1) {
						  if(tiempo_coccion >= MAX_TIEMPO)
							  tiempo_coccion = MAX_TIEMPO;
						  else
							  tiempo_coccion += 5;

						  tiempo_coccion_segs = tiempo_coccion * 120;
					  }
					  Display_Update(tiempo_coccion);
					  boton = CLEAR;
				  }
			  break;

			  case TIEMPO_MENOS:
				  if(HAL_GPIO_ReadPin(TOUCH_TIEMPOMENOS_GPIO_Port, TOUCH_TIEMPOMENOS_Pin)) {
					  cont_toques_tiempo++;
					  tiempo_mostrar_set = 10;
					  LED_minutos_on();

					  if(cont_toques_tiempo > 1) {
						  if(tiempo_coccion <= MIN_TIEMPO)
							  tiempo_coccion = MIN_TIEMPO;
						  else
							  tiempo_coccion -= 5;

						  tiempo_coccion_segs = tiempo_coccion * 120;
					  }
					  Display_Update(tiempo_coccion);
					  boton = CLEAR;
				  }
			  break;

			  case ST:
				  if(HAL_GPIO_ReadPin(TOUCH_ST_GPIO_Port, TOUCH_ST_Pin)) {
					  tiempo_mostrar_set = 0;
					  if(estado == STAND_BY || estado == TERMINANDO) {
						  estado = RAMPA_SUBIDA;
						  LED_proceso_on();
						  CALEFACTOR_on();
					  }
					  else {
						  estado = TERMINANDO;
						  LED_proceso_off();
						  CALEFACTOR_off();
						  LED_proceso_off();
					  }
					  Display_Update(temperatura_filtrada);
					  LED_grados_on();
					  boton = CLEAR;
				  }
			  break;
		  }
	  }


	  switch(estado)
	  {
	  	  case STAND_BY:
	  		if(temperatura_filtrada > 50)
			  LED_CALIENTE_on();
	  		else if(temperatura_filtrada < 40)
			  LED_CALIENTE_off();
	  	  break;

	  	  case RAMPA_SUBIDA:

	  		  if(setpoint - temperatura_filtrada < 25) {
	  			  CALEFACTOR_off();
	  			  estado = CONTROL;
	  		  }
	  		  if(temperatura_filtrada > 50)
	  			  LED_CALIENTE_on();
	  	  break;

	  	  case CONTROL:
	  		if(temperatura_filtrada > 50)
	  			LED_CALIENTE_on();

	  		if(controlar) {
	  			/*
	  			int error = setpoint - temperatura_filtrada;
	  			int duty_cycle = error * KP;
	  			*/

	  			tiempo_coccion_segs--;
	  			tiempo_coccion = tiempo_coccion_segs / 120;
	  			if(tiempo_coccion == 0)
	  			{
	  				estado = TERMINANDO;
	  				LED_proceso_off();
	  			}

	  			// PID
				float error = setpoint - temperatura_filtrada;
				integral += error * 0.5; // 0.5s intervalo
				float derivative = (error - previousError) / 0.5;
				float output = Kp * error + Ki * integral + Kd * derivative;
				previousError = error;

				// Convertir salida PID a tiempo ON del relé
				// Saturación y mapeo (ej: max 2000ms)
				if(output > 2000)
					output = 2000;
				if(output < 0)
					output = 0;
				relayOnTimeMs = (uint32_t)output;

				// Manejar estado del relé (salida ON/OFF controlada por tiempo)
				if(relayOnTimeMs > 0)
				{
					CALEFACTOR_on();
					relayState = 1;
					relayTimerCounter = 0; // reiniciar temporizador
				} else
				{
					CALEFACTOR_off();
					relayState = 0;
				}
				// Si ya está encendido, contar el tiempo
				if(relayState)
				{
					relayTimerCounter += 500;
					if(relayTimerCounter >= relayOnTimeMs)
					{
						CALEFACTOR_off();
						relayState = 0;
					}
				}

				controlar = false;	// Bajar bandera de control
				Send_Debug_Info(temperatura_filtrada, error, output, relayOnTimeMs, relayState);

	  			/*
	  			else if(cont_duty_cycle == 0) {
					if(duty_cycle < 0)
						tiempo_on_SSR = 0;
					else if(duty_cycle < 4)		//  1°C * 4.0f = 4
						tiempo_on_SSR = 1;
					else if(duty_cycle < 8)		//  2°C * 4.0f = 8
						tiempo_on_SSR = 3;
					else if(duty_cycle < 20)	//  5°C * 4.0f = 20
						tiempo_on_SSR = 3;
					else if(duty_cycle < 40)	// 10°C * 4.0f = 40
						tiempo_on_SSR = 5;
					else if(duty_cycle < 60)	// 15°C * 4.0f = 60
						tiempo_on_SSR = 7;
					else if(duty_cycle < 80)	// 20°C * 4.0f = 80
						tiempo_on_SSR = 9;
					else if (duty_cycle < 100)	// 25°C * 4.0f = 100
						tiempo_on_SSR = 11;
					else
						tiempo_on_SSR = 13;

					CALEFACTOR_on();
	  			}

	  			if(tiempo_on_SSR == cont_duty_cycle)
	  				CALEFACTOR_off();

	  			cont_duty_cycle = (cont_duty_cycle + 1) % NUM_MEDICIONES;
	  			*/

	  		}
	  	  break;

	  	  case TERMINANDO:
	  		  if(temperatura_filtrada < 40) {
	  			  LED_CALIENTE_off();
	  			  estado = STAND_BY;
	  		  }
	  	  break;

	  }
  }
  UART_Receive_And_Parse_Command();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC595_OE_Pin|HC595_LACLK_Pin|HC595_SHCLK_Pin|HC595_DATA_Pin
                          |DISP_UNIDAD_Pin|DISP_DECENA_Pin|DISP_CENTENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC595_RESET_GPIO_Port, HC595_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CALEFACTOR_Pin|LED_INDIC_Pin|LED_PROCESO_Pin|LED_CALIENTE_Pin
                          |MAX6675_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HC595_OE_Pin HC595_LACLK_Pin HC595_SHCLK_Pin HC595_RESET_Pin
                           HC595_DATA_Pin DISP_UNIDAD_Pin DISP_DECENA_Pin */
  GPIO_InitStruct.Pin = HC595_OE_Pin|HC595_LACLK_Pin|HC595_SHCLK_Pin|HC595_RESET_Pin
                          |HC595_DATA_Pin|DISP_UNIDAD_Pin|DISP_DECENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DISP_CENTENA_Pin */
  GPIO_InitStruct.Pin = DISP_CENTENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DISP_CENTENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CALEFACTOR_Pin LED_INDIC_Pin LED_PROCESO_Pin LED_CALIENTE_Pin
                           MAX6675_CS_Pin */
  GPIO_InitStruct.Pin = CALEFACTOR_Pin|LED_INDIC_Pin|LED_PROCESO_Pin|LED_CALIENTE_Pin
                          |MAX6675_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_ST_Pin TOUCH_TIEMPOMENOS_Pin TOUCH_TIEMPOMAS_Pin TOUCH_TEMPMENOS_Pin */
  GPIO_InitStruct.Pin = TOUCH_ST_Pin|TOUCH_TIEMPOMENOS_Pin|TOUCH_TIEMPOMAS_Pin|TOUCH_TEMPMENOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_TEMPMAS_Pin */
  GPIO_InitStruct.Pin = TOUCH_TEMPMAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_TEMPMAS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*
 * Interrupciones por desbordamiento de timers 1 y 2
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {	// Interrupcion cada 5,5ms aprox
        Display_UpdateDigit();
    }
    else if(htim->Instance == TIM2) {	// Interrupcion cada 0,5s aprox
    	leer_temp = true;
    	controlar = true;
    	cont_mostrar_temp = (cont_mostrar_temp + 1) % 40;
    }
}

/*
 * Interrupciones externas
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    switch(GPIO_Pin) {
        case TOUCH_TEMPMAS_Pin: 	// TOUCH_TEMPMAS
        	boton = TEMP_MAS;
        break;

        case TOUCH_TEMPMENOS_Pin: 	// TOUCH_TEMPMENOS
        	boton= TEMP_MENOS;
        break;

        case TOUCH_TIEMPOMAS_Pin: 	// TOUCH_TIEMPOMAS
        	boton = TIEMPO_MAS;
        break;

        case TOUCH_TIEMPOMENOS_Pin: // TOUCH_TIEMPOMENOS
        	boton = TIEMPO_MENOS;
        break;

        case TOUCH_ST_Pin: 			// TOUCH_ST
        	boton = ST;
        break;

    }
}

void config_HC595() {
	Display_PIN_EN_Assigment(HC595_OE_GPIO_Port, HC595_OE_Pin);				// Asignacion de pines para libreria HC595
  	Display_PIN_LACLK_Assigment(HC595_LACLK_GPIO_Port, HC595_LACLK_Pin);
  	Display_PIN_SHCLK_Assigment(HC595_SHCLK_GPIO_Port, HC595_SHCLK_Pin);
  	Display_PIN_RESET_Assigment(HC595_RESET_GPIO_Port, HC595_RESET_Pin);
  	Display_PIN_DATA_Assigment(HC595_DATA_GPIO_Port, HC595_DATA_Pin);
  	Display_PIN_UNIDAD_Assigment(DISP_UNIDAD_GPIO_Port, DISP_UNIDAD_Pin);
  	Display_PIN_DECENA_Assigment(DISP_DECENA_GPIO_Port, DISP_DECENA_Pin);
  	Display_PIN_CENTENA_Assigment(DISP_CENTENA_GPIO_Port, DISP_CENTENA_Pin);
}

void Send_Debug_Info(float averageTemp, float error, float output, uint32_t relayOnTimeMs, uint8_t relayState) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
        "T=%.1f°C | Error=%.1f | PID=%.1f | RelayON=%ums | Estado=%s\r\n",
        averageTemp, error, output, (unsigned int)relayOnTimeMs, relayState ? "ON" : "OFF"
    );
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART_Receive_And_Parse_Command(void) {
    char rxBuffer[64];
    uint8_t rxChar;
    uint8_t idx = 0;

    // Leer hasta Enter o hasta llenar el buffer
    while (HAL_UART_Receive(&huart1, &rxChar, 1, HAL_MAX_DELAY) == HAL_OK && idx < sizeof(rxBuffer) - 1) {
        if (rxChar == '\r' || rxChar == '\n') break; // Fin de línea
        rxBuffer[idx++] = rxChar;
    }
    rxBuffer[idx] = '\0';

    // Parseo básico
    if (strncmp(rxBuffer, "KP=", 3) == 0) {
        Kp = atof(&rxBuffer[3]);
    } else if (strncmp(rxBuffer, "KI=", 3) == 0) {
        Ki = atof(&rxBuffer[3]);
    } else if (strncmp(rxBuffer, "KD=", 3) == 0) {
        Kd = atof(&rxBuffer[3]);
    } else if (strncmp(rxBuffer, "SP=", 3) == 0) {
        setpoint = atof(&rxBuffer[3]);
    }

    // Confirmación
    char msg[64];
    snprintf(msg, sizeof(msg), "Parametros PID: KP=%.2f KI=%.2f KD=%.2f SP=%d\r\n", Kp, Ki, Kd, setpoint);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
