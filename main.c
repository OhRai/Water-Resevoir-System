/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t b_counter = 0;

uint8_t byte[2];
uint8_t rpm_tick_count = 0;
uint8_t msg_buffer[64] = {0};
uint8_t txd_msg_buffer[128] = {0};

uint8_t us100_buffer[2] = {0};
volatile uint16_t distance = 0;
volatile double dist_percent = 0;
volatile uint8_t us100_Rx_flag = 0;
uint8_t cmd_dist = 0x55;
volatile int clock_secs = 0;
int pwm = 0;
int zero = 0;
int full = 0;

uint8_t ADC_CH9 = 0;

#define MIN_DIST 40
#define MAX_DIST 180
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rcv_intpt_flag = 0;

void ADC_Select_CH(int CH)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	switch (CH)
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	}
}

int process_input()
{
	rcv_intpt_flag = 00;
	HAL_UART_Receive_IT(&huart6, byte, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);
	while (rcv_intpt_flag == (00))
	{
	}
	if (byte[1] == 13)
	{
		return byte[0] - '0';
	}
	else
	{
		return (byte[0] - '0') * 10 + byte[1] - '0';
	}
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
	uint8_t DIGITA_VAL = 0x0F & DIGIT_A; // mask off higher4 bits
	int Abit0 = (DIGITA_VAL) & 1;		 // extract Abit0 of the 4-bit value
	int Abit1 = (DIGITA_VAL >> 1) & 1;	 // extract Abit1 of the 4-bit value
	int Abit2 = (DIGITA_VAL >> 2) & 1;	 // extract Abit2 of the 4-bit value
	int Abit3 = (DIGITA_VAL >> 3) & 1;	 // extract Abit3 of the 4-bit value

	uint8_t DIGITB_VAL = 0x0F & DIGIT_B; // mask off higher4 bits
	int Bbit0 = (DIGITB_VAL) & 1;		 // extract Bbit0 of the 4-bit value
	int Bbit1 = (DIGITB_VAL >> 1) & 1;	 // extract Bbit1 of the 4-bit value
	int Bbit2 = (DIGITB_VAL >> 2) & 1;	 // extract Bbit2 of the 4-bit value
	int Bbit3 = (DIGITB_VAL >> 3) & 1;	 // extract Bbit3 of the 4-bit value

	if (Abit0 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
	}
	if (Abit1 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
	}
	if (Abit2 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
	}
	if (Abit3 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);
	}

	if (Bbit0 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);
	}
	if (Bbit1 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);
	}
	if (Bbit2 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);
	}
	if (Bbit3 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);
	}
}

void get_dist()
{
	HAL_UART_Receive_IT(&huart1, us100_buffer, 2);
	HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
	distance = us100_buffer[0];
	distance = (distance << 8);
	distance += us100_buffer[1];

	dist_percent = distance - MIN_DIST;
	if (dist_percent < 0)
	{
		dist_percent = 0;
	}
	if (dist_percent >= MAX_DIST - MIN_DIST)
	{
		dist_percent = MAX_DIST - MIN_DIST;
	}
	dist_percent = (dist_percent / (MAX_DIST - MIN_DIST)) * 100;
	dist_percent = 99 - dist_percent;
	b_counter = dist_percent;
	//	  sprintf((char*)txd_msg_buffer, "\r\n balls: %d", b_counter);
	//	  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	if (b_counter > 99)
	{
		b_counter = 99;
	}
	if (b_counter == 0)
	{
		DIGITS_Display(0, 0);
		zero++;
	}
	else
	{
		uint8_t tens = b_counter / 10;
		uint8_t ones = b_counter % 10;
		DIGITS_Display(tens, ones);
	}
	HAL_Delay(100);
	while (us100_Rx_flag == (00))
	{
	}
}

int get_channel(int a, int b, int c, int d, int e, int f, int g, int h)
{
	a *= 6;
	b *= 6;
	c *= 6;
	d *= 6;
	e *= 6;
	f *= 6;
	g *= 6;
	h *= 6;
	if (clock_secs >= a && clock_secs <= b)
	{
		return 0;
	}
	else if (clock_secs >= c && clock_secs <= d)
	{
		return 1;
	}
	else if (clock_secs >= e && clock_secs <= f)
	{
		return 2;
	}
	else if (clock_secs >= g && clock_secs <= h)
	{
		return 3;
	}
	else
	{
		return -1;
	}
}

void set_servo(int x)
{
	if (x == 0)
	{
		TIM2->CCR1 = 0;
	}
	if (x == 1)
	{
		TIM2->CCR1 = 1000;
	}
	if (x == 2)
	{
		TIM2->CCR1 = 1250;
	}
	if (x == 3)
	{
		TIM2->CCR1 = 1500;
	}
	HAL_Delay(500);
}

void set_motor(int x, int direction)
{
	int multiplier = 0;
	if (x == -1)
	{
		multiplier = 0;
	}
	else if (x == 0)
	{
		multiplier = ADC_CH9 * 100 / 255;
	}
	else if (x == 1)
	{
		multiplier = 60;
	}
	else if (x == 2)
	{
		multiplier = 80;
	}
	else
	{
		multiplier = 99;
	}
	pwm = multiplier;
	int max = 2000;
	if (direction)
	{
		TIM3->CCR1 = (multiplier * max) / 100;
		TIM3->CCR3 = 0;
	}
	else
	{
		TIM3->CCR3 = (multiplier * max) / 100;
		TIM3->CCR1 = 0;
	}
}

char *curr_zone(int x)
{
	if (x == 0)
	{
		return "Inlet";
	}
	else if (x == 1)
	{
		return "Zone 1";
	}
	else if (x == 2)
	{
		return "Zone 2";
	}
	else if (x == 3)
	{
		return "Zone 3";
	}
	else
	{
		return "none";
	}
}

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
	int setup_mode = 1;
	int inlet_speed = 0;
	int z1_speed = 0;
	int z2_speed = 0;
	int z3_speed = 0;
	int curr_wall_clk_time = 0;
	int inlet_wall_clk_start = 0;
	int inlet_wall_clk_stop = 0;
	int z1_wall_clk_start = 0;
	int z1_wall_clk_stop = 0;
	int z2_wall_clk_start = 0;
	int z2_wall_clk_stop = 0;
	int z3_wall_clk_start = 0;
	int z3_wall_clk_stop = 0;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim5);

	//	int TIM2_CH1_DCVAL = 500;
	//	int TIM2_CH1_STEP = 100;

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->PSC = 16 - 1;
	TIM2->ARR = 20000 - 1;
	//	TIM2->CCR1 = 0;

	//  int TIM3_Ch1_DCVAL = 1200;
	//  int TIM3_Ch3_DCVAL = 1200;
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->PSC = 16 - 1;
	TIM3->ARR = 2000 - 1;
	TIM3->CCR1 = 0;
	TIM3->CCR3 = 0;

	//  sprintf((char*)txd_msg_buffer, "\r\n INPUT A CHARACTER:");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//  set_servo(1);
	//  TIM2->CCR1 = 1500;
	sprintf((char *)txd_msg_buffer, "\r\n ----- SETUP MODE -----\n\n");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n INLET MOTOR SPEED PWM (option 0-3): ");
	inlet_speed = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n INLET MOTOR SPEED: %d", inlet_speed);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 1 MOTOR SPEED PWM (option 0-3): ");
	z1_speed = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 1 MOTOR SPEED: %d", z1_speed);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 2 MOTOR SPEED PWM (option 0-3): ");
	z2_speed = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n INLET MOTOR SPEED: %d", z2_speed);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 3 MOTOR SPEED PWM (option 0-3): ");
	z3_speed = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 3 MOTOR SPEED: %d", z3_speed);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n CURRENT WALL CLOCK TIME (0-23): ");
	curr_wall_clk_time = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n CURRENT WALL CLOCK TIME: %d", curr_wall_clk_time);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n INLET WALL CLOCK START TIME (0-23): ");
	inlet_wall_clk_start = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n INLET WALL CLOCK START TIME: %d", inlet_wall_clk_start);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n INLET WALL CLOCK STOP TIME (0-23): ");
	inlet_wall_clk_stop = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n INLET WALL CLOCK STOP TIME: %d", inlet_wall_clk_stop);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 1 WALL CLOCK START TIME (0-23): ");
	z1_wall_clk_start = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 1 WALL CLOCK START TIME: %d", z1_wall_clk_start);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 1 WALL CLOCK STOP TIME (0-23): ");
	z1_wall_clk_stop = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 1 WALL CLOCK STOP TIME: %d", z1_wall_clk_stop);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 2 WALL CLOCK START TIME (0-23): ");
	z2_wall_clk_start = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 2 WALL CLOCK START TIME: %d", z2_wall_clk_start);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 2 WALL CLOCK STOP TIME (0-23): ");
	z2_wall_clk_stop = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 2 WALL CLOCK STOP TIME: %d", z2_wall_clk_stop);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 3 WALL CLOCK START TIME (0-23): ");
	z3_wall_clk_start = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 3 WALL CLOCK START TIME: %d", z3_wall_clk_start);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char *)txd_msg_buffer, "\r\n ZONE 3 WALL CLOCK STOP TIME (0-23): ");
	z3_wall_clk_stop = process_input();
	//	sprintf((char*)txd_msg_buffer, "\r\n ZONE 3 WALL CLOCK STOP TIME: %d", z3_wall_clk_stop);
	//	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	if (inlet_speed < 0 || z1_speed < 0 || z2_speed < 0 || z3_speed < 0 ||
		inlet_speed > 3 || z1_speed > 3 || z2_speed > 3 || z3_speed > 3 ||
		inlet_wall_clk_start > inlet_wall_clk_stop || z1_wall_clk_start > z1_wall_clk_stop ||
		z2_wall_clk_start > z2_wall_clk_stop || z3_wall_clk_start > z3_wall_clk_stop)
	{
		sprintf((char *)txd_msg_buffer, "\r\n INVALID INPUT, PLEASE HIT RESET AND TRY AGAIN: ");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);
		return;
	}

	while (setup_mode)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);
		if (!HAL_GPIO_ReadPin(GPIOC, BLUE_PB_Pin))
		{
			setup_mode = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
	}
			sprintf((char *)txd_msg_buffer, "\r\n\n Wall-Clock Time| Zone/Inlet | Motor Speed %%PWM | Motor RPM | Water Reservoir Depth");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);
	clock_secs = curr_wall_clk_time * 6;

	int current_time = -1;

	while (1)
	{
		if (clock_secs >= 24 * 6)
		{
			set_motor(-1, 0);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin | Green_Pin | Red_Pin | GPIO_PIN_5, GPIO_PIN_RESET);
			break;
		}
		int channel = get_channel(inlet_wall_clk_start, inlet_wall_clk_stop, z1_wall_clk_start,
								  z1_wall_clk_stop, z2_wall_clk_start, z2_wall_clk_stop, z3_wall_clk_start, z3_wall_clk_stop);
		if (zero >= 7 && channel != 0 && (clock_secs > inlet_wall_clk_stop * 6))
		{
			char *curr = curr_zone(channel);
			sprintf((char *)txd_msg_buffer, "\r\n\n %d | %s | %d%% | %d | %d", (clock_secs / 6), curr, pwm, rpm_tick_count, b_counter);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);
			set_motor(-1, 0);
			set_servo(0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			while (1)
			{
				HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
			}
		}
		if (channel == 0)
		{
			zero = 0;
		}
		ADC_Select_CH(9);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_CH9 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		get_dist();

		if (((clock_secs - curr_wall_clk_time) % 6 == 0) && (current_time != clock_secs / 6))
		{
			current_time = clock_secs / 6;
			char *curr = curr_zone(channel);
			sprintf((char *)txd_msg_buffer, "\r\n\n %d | %s | %d%% | %d | %d", (clock_secs / 6), curr, pwm, rpm_tick_count, b_counter);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char *)txd_msg_buffer), 1000);
		}
		
		switch (channel)
		{
		case 0:
			HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
			set_servo(0);
			if (b_counter == 99)
			{
				full++;
				zero = 0;
			}
			if (full >= 5)
			{
				set_motor(-1, 1);
			}
			else
			{
				switch (inlet_speed)
				{
				case 0:
					set_motor(0, 1);
					break;
				case 1:
					set_motor(1, 1);
					break;
				case 2:
					set_motor(2, 1);
					break;

				case 3:
					set_motor(3, 1);
					break;
					//	  		  	  default:
					//	  		  		  set_motor(0, 1);
					//	  		  		  break;
				}
			}
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
			set_servo(1);
			switch (z1_speed)
			{
			case 0:
				set_motor(0, 0);
				break;
			case 1:
				set_motor(1, 0);
				break;

			case 2:
				set_motor(2, 0);
				break;

			case 3:
				set_motor(3, 0);
				break;
				//				  default:
				//				  	  set_motor(0, 0);
				//				  	  break;
			}
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
			set_servo(2);
			switch (z2_speed)
			{
			case 0:
				set_motor(0, 0);
				break;
			case 1:
				set_motor(1, 0);
				break;

			case 2:
				set_motor(2, 0);
				break;

			case 3:
				set_motor(3, 0);
				break;
				//				  default:
				//					  set_motor(0, 0);
				//					  break;
			}
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_SET);
			set_servo(3);
			switch (z3_speed)
			{
			case 0:
				set_motor(0, 0);
				break;
			case 1:
				set_motor(1, 0);
				break;

			case 2:
				set_motor(2, 0);
				break;

			case 3:
				set_motor(3, 0);
				break;
				//				  default:
				//					  set_motor(0, 0);
				//					  break;
			}
			break;
		default:
			set_motor(-1, 0);
			HAL_GPIO_WritePin(GPIOA, Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Blue_Pin, GPIO_PIN_RESET);
			break;
		}

		//	  sprintf((char*)txd_msg_buffer, "\r\n Seconds: %d", clock_secs);
		//	  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		//	  HAL_Delay(500);
		//	  int stateOfPushButton = HAL_GPIO_ReadPin(GPIOC, BLUE_PB_Pin);
		//	  if ( stateOfPushButton == 1 ) {
		//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		//	  } else {
		//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//	  }
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1200 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 16000 - 1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1000 - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
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
	huart1.Init.BaudRate = 9600;
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin | DIGIT_B0_Pin | DIGIT_B1_Pin | DIGIT_B2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | Green_Pin | Blue_Pin | Red_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin | DIGIT_A1_Pin | DIGIT_A2_Pin | DIGIT_A3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BLUE_PB_Pin */
	GPIO_InitStruct.Pin = BLUE_PB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLUE_PB_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIGIT_B3_Pin DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin */
	GPIO_InitStruct.Pin = DIGIT_B3_Pin | DIGIT_B0_Pin | DIGIT_B1_Pin | DIGIT_B2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin Green_Pin Blue_Pin Red_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | Green_Pin | Blue_Pin | Red_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : RPM_TICK_Pin */
	GPIO_InitStruct.Pin = RPM_TICK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIGIT_A0_Pin DIGIT_A1_Pin DIGIT_A2_Pin DIGIT_A3_Pin */
	GPIO_InitStruct.Pin = DIGIT_A0_Pin | DIGIT_A1_Pin | DIGIT_A2_Pin | DIGIT_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6)
	{
		HAL_UART_Transmit(&huart6, byte, strlen((char *)byte), 100);
		rcv_intpt_flag = 1;
	}
	if (huart->Instance == USART1)
	{
		us100_Rx_flag = 01;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == RPM_TICK_Pin)
	{
		rpm_tick_count += 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		clock_secs += 1; // this could be a variable for seconds etc.
	}
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART1)
//	{
//		us100_Rx_flag = 01;
//	}
// }
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

#ifdef USE_FULL_ASSERT
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
