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
#include "spi.h"
#include "modbus.h"
#include "freq.h"
#include "led.h"
//#include "buttons.h"
#include "stdbool.h" //allows support of boolean variable types
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEFAULTSPEED 50 //150 RPM default speed
#define ON 1 //motor on definition
#define OFF 0 //motor off definition
#define SETUP 2 //setup mode definition
#define RUN 3 //run mode definition

struct MotorStruct
{
	int status; //motor status on or off
	uint16_t speed; //motor speed in terms of VFD frequency
	uint16_t maxSpeed; //motor max speed setting
};

struct ConfigurationStruct
{
	int mode;
};


//define a pointer to MotorStruct
struct MotorStruct* motorPtr = NULL;

volatile int motorStartFlag = 0;	//flag to indicate start button was pushed
volatile int motorStopFlag = 0;		//flag to indicate stop button was pushed
volatile int motorRPMUpFlag = 0;	//flag to indicate RPM up button was pushed
volatile int motorRPMDownFlag = 0;	//flag to indicate RPM down button was pushed

//startup function
void startup()
{
	LEDPowerOn();
	HAL_Delay(200);
	LEDUSBOn();
	HAL_Delay(200);
	LEDTestActiveOn();
	HAL_Delay(200);
	LEDTestInactiveOn();
	HAL_Delay(200);
	LEDUSBOff();
	LEDTestActiveOff();
	LEDTestInactiveOn();
	motorStartupTest(); //run the motor test on startup
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //set CS (PB12) high for default state
	resetDisplay(); //send a display reset command before turning on display
	initializeDisplay(); //send display initialization commands
	displayRPM(0); //display the set RPM on the LCD
}

//update LEDs based on status
void updateLEDs(struct MotorStruct* motor)
{
	if(motor->status == 0)
	{
		LEDTestInactiveOn();
		LEDTestActiveOff();
	}
	if(motor->status == 1)
	{
		LEDTestInactiveOff();
		LEDTestActiveOn();
	}
}

//***************************INTERRUPT SERVICE ROUTINES***************************
//GPIO EXTI Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if the start button was pushed (PB1)
	if (GPIO_Pin == GPIO_PIN_1)
	{
		if(motorStartFlag != 1)
		{
			motorStartFlag = 1;
		}
	}
	//if stop button was pushed (PB0)
	if (GPIO_Pin == GPIO_PIN_0)
	{
		if(motorStopFlag != 1)
		{
			motorStopFlag = 1;
		}
	}
	//if RPM up is pressed
	if(GPIO_Pin == GPIO_PIN_7)
	{
		if(motorRPMUpFlag != 1)
		{
			motorRPMUpFlag = 1;
		}
	}
	//if RPM down is pressed
	if(GPIO_Pin == GPIO_PIN_6)
	{
		if(motorRPMDownFlag != 1)
		{
			motorRPMDownFlag = 1;
		}
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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  struct MotorStruct motor;
  motor.speed = 0;
  motor.status = OFF;

  struct ConfigurationStruct config;
  config.mode = SETUP; //start in setup mode

  startup(); //call the startup function
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //if startup was triggered by the start button
	  if(motorStartFlag == 1)
	  {
		  motorStartFlag = 0; //reset the start flag
		  if (motor.status == OFF)
		  {
			  motor.status = ON;
			  motor.speed = DEFAULTSPEED;
			  startMotor();
			  setMotorSpeed(DEFAULTSPEED);
			  displayRPM(DEFAULTSPEED);
			  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1); //start frequency measurement
			  updateLEDs(&motor);
		  }
	  }

	  if(motorStopFlag == 1)
	  {
		  motorStopFlag = 0;
		  motor.status = OFF; //motor on == false
		  motor.speed = 10;
		  stopMotor();
		  displayRPM(0); //display the set RPM on the LCD
		  HAL_TIM_IC_Stop_IT(&htim14, TIM_CHANNEL_1); //stop frequency measurement
		  updateLEDs(&motor);
	  }

	  //if motor.speed <= 350
	  if(motor.speed <= 700 && motor.status == ON)
	  {
		  HAL_Delay(1500); //wait 1.5 seconds for stabilization
		  readyforTQCapture = 1; //set capture flag
		  Is_First_Captured_TIM14 = 0; //reset the timer interrupt flag in case
		  outputTorque(); //retrieve torque and send on USB
		  motor.speed += 5; //increment motor speed variable by 5 RPM
		  setMotorSpeed(motor.speed); //update motor speed
		  displayRPM(motor.speed); //update LCD RPM
		  updateLEDs(&motor); //update status LEDs
	  }

	  //else if motor.speed > 350
	  else if(motor.speed > 700)
	  {
		  //stop the motor when test completed
		  motorStopFlag = 1;
		  updateLEDs(&motor);
	  }

	  /*if(motorRPMUpFlag == 1)
	  {
		  motorRPMUpFlag = 0;
		  if(motor.status == ON)
		  {
			  if (motor.speed < 1330) //increase speed if max speed not reached
				  motor.speed += 100;
			  else
				  motor.speed += 0; //do nothing to the speed
			  setMotorSpeed(motor.speed);
			  displayRPM(motor.speed); //display the set RPM on the LCD
		  }
	  }

	  if(motorRPMDownFlag == 1)
	  {
		  motorRPMDownFlag = 0;
		  if(motor.status == ON)
		  {
			  if (motor.speed > 0) //decrease speed if speed != 0
				  motor.speed -= 100;
			  else
				  motor.speed -= 0; //do nothing to the speed
			  setMotorSpeed(motor.speed);
			  displayRPM(motor.speed); //display the set RPM on the LCD
		  }
	  }*/

	  //if PuTTY sends a command
	  /*if(HAL_UART_Receive(&huart1, &rxdata, 1, 1000)==HAL_OK) //receive from UART and store in rx buffer
	  {
		  LEDUSBOn();
		  switch(rxdata)
		  {
		  	  //motor speed (+) from keyboard input
		  	  case '+':
				  if(motor.status == ON)
				  {
					  if (motor.speed < 1300) //increase speed if max speed not reached
						  motor.speed += 5;
					  else
						  motor.speed += 0; //do nothing to the speed
					  setMotorSpeed(motor.speed);
					  displayRPM(motor.speed); //display the set RPM on the LCD
				  }
				  break;
			  //motor speed (-) from keyboard input
		  	  case '-':
				  if(motor.status == ON)
				  {
					  if (motor.speed > 0) //decrease speed if speed != 0
						  motor.speed -= 5;
					  else
						  motor.speed -= 0; //do nothing to the speed
					  setMotorSpeed(motor.speed);
					  displayRPM(motor.speed); //display the set RPM on the LCD
				  }
				  break;
			  //stop motor
		  	  case 's':
		  		  motor.status = OFF; //motor on == false
		  		  motor.speed = 10;
		  		  stopMotor();
		  		  displayRPM(0); //display the set RPM on the LCD
		  		  HAL_UART_Transmit(&huart1, stopmsg, sizeof(stopmsg), 1000);
		  		  HAL_TIM_IC_Stop_IT(&htim14, TIM_CHANNEL_1); //stop frequency measurement
		  		  break;
			  //start motor
		  	  case 'g':
		  		  //only start if motor is off
				  if (motor.status == OFF)
				  {
					  motor.status = ON; //motor status == true
					  motor.speed = DEFAULTSPEED;	//set default speed on startup
					  startMotor();
					  setMotorSpeed(DEFAULTSPEED);
					  displayRPM(DEFAULTSPEED);
					  HAL_UART_Transmit(&huart1, startmsg, sizeof(startmsg), 1000);
					  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1); //start frequency measurement
				  }
				  break;
			  default:
				  break;
		  }
		  LEDUSBOff();
	  }
	  updateLEDs(&motor);*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin|LCD_CS_Pin|USB_LED_Pin|PWR_LED_Pin
                          |TEST_ACTIVE_LED_Pin|TEST_INACTIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RPM_DOWN_BUTTON_Pin RPM_UP_BUTTON_Pin */
  GPIO_InitStruct.Pin = RPM_DOWN_BUTTON_Pin|RPM_UP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_BUTTON_Pin START_BUTTON_Pin */
  GPIO_InitStruct.Pin = STOP_BUTTON_Pin|START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CS_Pin USB_LED_Pin PWR_LED_Pin
                           TEST_ACTIVE_LED_Pin TEST_INACTIVE_LED_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CS_Pin|USB_LED_Pin|PWR_LED_Pin
                          |TEST_ACTIVE_LED_Pin|TEST_INACTIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
