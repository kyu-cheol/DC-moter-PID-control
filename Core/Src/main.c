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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P_GAIN_SPEED 5
#define I_GAIN_SPEED 0.1
#define D_GAIN_SPEED 0.2

#define P_GAIN_POS 1.9
#define I_GAIN_POS 0.02
#define D_GAIN_POS 0.1

#define dt 0.05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t enc_pulse_edge_cnt;
volatile int32_t enc_pos_cnt;

volatile float current_state;
volatile uint32_t target_state;

volatile float realError;
volatile float accError;
volatile float errorGap;

volatile float pControl;
volatile float iControl;
volatile float dControl;

volatile float ccr_f;

uint8_t mode;
uint8_t speed_control_mode;
uint8_t pos_control_mode;

uint8_t button_flag;

char input_buf[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Moter_Init(void);
void Moter_Start(void);
void Moter_Reverse_Start(void);
void Moter_Stop(void);

void calculateErrors(void);
void pidControl_speed(void);
void pidControl_pos(void);
uint32_t getUserInput(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar (int ch)
{
	if (ch == '\n') {
		HAL_UART_Transmit(&huart1, (uint8_t *)&"\r", 1, HAL_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int16_t tmp;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_TIM_PWM_DeInit(&htim1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // ========================================== System Initialization ===============================================
  printf("\nWaiting for system ready.....\n\n");
  Moter_Init();
  Moter_Stop();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// 20KHZ PWM freq. resolution 1000. valid range (550 ~ 999)
  TIM1->CCR1 = 0;								// MIN speed 8rpm, MAX speed 90rpm (rpm = (encoder freq/8/120) * 60)

  HAL_Delay(2000);
  enc_pulse_edge_cnt = 0;
  enc_pos_cnt = 0;
  //ccr_f = 0.;
  // ================================================================================================================

  printf("\n================================================\n");
  printf("\nSelect Mode\n");
  printf("1: speed control mode\t2: position control mode\n");
  printf("\n================================================\n");

  while (HAL_UART_Receive(&huart1, &mode, 1, 10) != HAL_OK);

  if (mode == '1') {
	  HAL_TIM_Base_Start_IT(&htim2);		// RPM PID control freq timer start.

	  HAL_UART_Transmit(&huart1, "enter RPM : ", 12, 10);
	  target_state = getUserInput();
	  if (target_state > 120 || target_state < 0) {
		  printf("\nRPM range 0 ~ 120\n");
		  printf("Press reset again.\n");
	  } else {
		  Moter_Start();
		  speed_control_mode = 1;
	  }
  } else if (mode == '2'){
	  HAL_TIM_Base_Start_IT(&htim3);		// Angle PID control freq timer start.

	  HAL_UART_Transmit(&huart1, "enter ANGLE : ", 14, 10);
	  target_state = getUserInput();
	  if (target_state > 360 || target_state < 0) {
		  printf("\nPOS range 0 ~ 360\n");
		  printf("Press reset again.\n");
	  } else {
		  Moter_Start();
		  pos_control_mode = 1;
	  }
  } else {
	  printf("Invalid argument.\n");
	  printf("Press reset again.\n");
  }
  HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (button_flag) {
		  button_flag = 0;

		  if (speed_control_mode) {
			  HAL_UART_Transmit(&huart1, "enter RPM : ", 12, 10);
			  tmp = getUserInput();
			  if (tmp > 120 || tmp < 0) {
				  printf("\nWrong argument.\n");
			  } else {
				  target_state = tmp;
			  }
		  }
		  if (pos_control_mode) {
			  HAL_UART_Transmit(&huart1, "enter ANGLE : ", 14, 10);
			  tmp = getUserInput();
			  if (tmp > 360 || tmp < 0) {
				  printf("\nWrong argument.\n");
			  } else {
				  target_state = tmp;
			  }
		  }
		  HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	  }

	  if (speed_control_mode) {
		  printf("target: %d\tcurrent: %d\n", target_state, current_state);
		  HAL_Delay(1000);
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ENCODER_A_INPUT_Pin) {
		enc_pulse_edge_cnt++;

		if (HAL_GPIO_ReadPin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin)) {				// A channel rising edge.
			if (HAL_GPIO_ReadPin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin) == 1) {
				enc_pos_cnt++;
			} else {
				enc_pos_cnt--;
			}
		} else {											// A channel falling edge.
			if (HAL_GPIO_ReadPin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin) == 0) {
				enc_pos_cnt++;
			} else {
				enc_pos_cnt--;
			}
		}
	}

	if (GPIO_Pin == ENCODER_B_INPUT_Pin) {
		enc_pulse_edge_cnt++;

		if (HAL_GPIO_ReadPin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin)) {				// B channel rising edge.
			if (HAL_GPIO_ReadPin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin) == 0) {
				enc_pos_cnt++;
			} else {
				enc_pos_cnt--;
			}
		} else {											// B channel falling edge.
			if (HAL_GPIO_ReadPin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin) == 1) {
				enc_pos_cnt++;
			} else {
				enc_pos_cnt--;
			}
		}
	}

	if (GPIO_Pin == GPIO_PIN_0) {
		button_flag = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {		// 100ms
		current_state = 10. * (float)enc_pulse_edge_cnt / 64.;		// (60 * enc_pulse_edge_cnt) / (0.1 * 960 * 4)

		calculateErrors();
		pidControl_speed();

		//printf("RPM : %d\n", current_RPM);
		enc_pulse_edge_cnt = 0;
	}

	if (htim->Instance == TIM3) {		// 50ms
		current_state = 360. * (float)enc_pos_cnt / 3840.;			// (360 * enc_pos_cnt) / (8 * 120 * 4)

		calculateErrors();
		pidControl_pos();
	}
}

void calculateErrors(void)
{
	errorGap = (float)target_state - current_state - realError;	// D control
	realError = (float)target_state - current_state;		// P control
	accError += realError * dt;					// I control
}

void pidControl_speed(void)
{
	float amount_of_control;

	pControl = P_GAIN_SPEED * realError;
	iControl = I_GAIN_SPEED * accError;
	dControl = D_GAIN_SPEED * (errorGap / dt);

	amount_of_control = pControl + iControl + dControl;
	if (amount_of_control < 0) amount_of_control = 0;
	if (amount_of_control > 999) amount_of_control = 999;

	TIM1->CCR1 = (uint16_t)amount_of_control;
}

void pidControl_pos(void)
{
	float amount_of_control;

	pControl = P_GAIN_POS * realError;
	iControl = I_GAIN_POS * accError;
	dControl = D_GAIN_POS * (errorGap / dt);

	amount_of_control = pControl + iControl + dControl;
	if (amount_of_control < 0) {
		Moter_Reverse_Start();
		amount_of_control *= -1;
	} else {
		Moter_Start();
	}

	if (amount_of_control > 999) {
		amount_of_control = 999;
	}

	//ccr_f = 0.4 * (float)amount_of_control + 600.;
	ccr_f = 0.45 * (float)amount_of_control + 550.;		// Duty cycle 550 ~ 999.
	TIM1->CCR1 = (int16_t)ccr_f;
}

void Moter_Init(void)
{
	HAL_GPIO_WritePin(IN1_OUTPUT_GPIO_Port, IN1_OUTPUT_Pin, 0);
	HAL_GPIO_WritePin(IN2_OUTPUT_GPIO_Port, IN2_OUTPUT_Pin, 0);
}

void Moter_Start(void)
{
	HAL_GPIO_WritePin(IN1_OUTPUT_GPIO_Port, IN1_OUTPUT_Pin, 1);
	HAL_GPIO_WritePin(IN2_OUTPUT_GPIO_Port, IN2_OUTPUT_Pin, 0);
}

void Moter_Reverse_Start(void)
{
	HAL_GPIO_WritePin(IN1_OUTPUT_GPIO_Port, IN1_OUTPUT_Pin, 0);
	HAL_GPIO_WritePin(IN2_OUTPUT_GPIO_Port, IN2_OUTPUT_Pin, 1);
}

void Moter_Stop(void)
{
	HAL_GPIO_WritePin(IN1_OUTPUT_GPIO_Port, IN1_OUTPUT_Pin, 1);
	HAL_GPIO_WritePin(IN2_OUTPUT_GPIO_Port, IN2_OUTPUT_Pin, 1);
}

uint32_t getUserInput(void)
{
	uint8_t i = 0;
	uint8_t ch;
	uint32_t ret;

	while(1) {		// blocking for user input.
		while (HAL_UART_Receive(&huart1, &ch, 1, 10) != HAL_OK);

		if (ch == 13) break;
		input_buf[i++] = ch;
		HAL_UART_Transmit(&huart1, &ch, 1, 10);
	}

	ret = atoi(input_buf);
	for (i = 0; i < 9; i++) {
		input_buf[i] = 0;
	}

	return ret;
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
