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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Redefine RS485 DE/RE* pins PA1/PA0 for ease of use
#define DE_PIN GPIO_PIN_1 //Drive enable == PA1
#define RE_PIN GPIO_PIN_0 //Receive enable == PA0
#define RS485_GPIO_PORT GPIOA //DE/RE on GPIO Port A

//sets RS485 in transmit mode
void MODBUS_SetTXMode()
{
	HAL_GPIO_WritePin(RS485_GPIO_PORT, DE_PIN, GPIO_PIN_SET);  // DE High
	HAL_GPIO_WritePin(RS485_GPIO_PORT, RE_PIN, GPIO_PIN_SET);  // RE* High (Drive TX)
}

//sets RS485 in receive mode
void MODBUS_SetRXMode()
{
	HAL_GPIO_WritePin(RS485_GPIO_PORT, DE_PIN, GPIO_PIN_RESET); // DE Low
	HAL_GPIO_WritePin(RS485_GPIO_PORT, RE_PIN, GPIO_PIN_RESET); // RE* Low (Listen RX)
}

//MODBUS cyclic redundancy check generator function
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

//Send MODBUS frame to starting motor
void startMotor()
{
	uint8_t frame[8]; //define an unsigned 8-bit (1 byte) transmission frame
	uint16_t crc; //define an unsigned 16-bit (2 bytes) cyclic redundancy check
	//Generate each byte of the MODBUS frame
	frame[0] = 0x01;              // Slave Address (adjust if needed)
	frame[1] = 0x06;              // Function Code (Write Single Holding Register)
	frame[2] = 0x00;              // Register Address High Byte (Drive Control Word)
	frame[3] = 0x00;              // Register Address Low Byte
	frame[4] = 0x00;              // Value High Byte (0x0001 to Start Motor)
	frame[5] = 0x01;              // Value Low Byte

	crc = Modbus_CRC16(frame, 6); //generate the crc token
	frame[6] = crc & 0xFF;        // CRC low Byte
	frame[7] = (crc >> 8) & 0xFF; // CRC high Byte

	MODBUS_SetTXMode(); //set RS485 to transmit mode
	//%%%%%%%%
	HAL_Delay(10); //wait 10ms in case
	HAL_UART_Transmit(&huart2, frame, sizeof(frame),2000); //transmit the MODBSU frame
	HAL_Delay(5);
	MODBUS_SetRXMode(); //set RS485 to receive mode to receive a response from VFD
}

void stopMotor()
{
	uint8_t frame[8]; //define an unsigned 8-bit (1 byte) transmission frame
	uint16_t crc; //define an unsigned 16-bit (2 bytes) cyclic redundancy check
	//Generate each byte of the MODBUS frame
	frame[0] = 0x01;              // Slave Address (adjust if needed)
	frame[1] = 0x06;              // Function Code (Write Single Holding Register)
	frame[2] = 0x00;              // Register Address High Byte (Drive Control Word)
	frame[3] = 0x00;              // Register Address Low Byte
	frame[4] = 0x00;              // Value High Byte (0x0000 to Stop Motor)
	frame[5] = 0x00;              // Value Low Byte

	crc = Modbus_CRC16(frame, 6); //generate the crc token
	frame[6] = crc & 0xFF;        // CRC low Byte
	frame[7] = (crc >> 8) & 0xFF; // CRC high Byte

	MODBUS_SetTXMode(); //set RS485 to transmit mode
	//%%%%%%%%
	HAL_Delay(10); //wait 10ms in case
	HAL_UART_Transmit(&huart2, frame, sizeof(frame),2000); //transmit the MODBSU frame
	HAL_Delay(5);
	MODBUS_SetRXMode(); //set RS485 to receive mode to receive a response from VFD
}

//Set the motor frequency to half speed
void setMotor30()
{
	uint8_t frame[8]; //define an unsigned 8-bit (1 byte) transmission frame
	uint16_t crc; //define an unsigned 16-bit (2 bytes) cyclic redundancy check
	//Generate each byte of the MODBUS frame
	frame[0] = 0x01;              // Slave Address (adjust if needed)
	frame[1] = 0x06;              // Function Code (Write Single Holding Register)
	frame[2] = 0x00;              // Register Address High Byte (Drive Control Word)
	frame[3] = 0x01;              // Register Address Low Byte
	frame[4] = 0x01;              // Value High Byte
	frame[5] = 0x2C;              // Value Low Byte

	crc = Modbus_CRC16(frame, 6); //generate the crc token
	frame[6] = crc & 0xFF;        // CRC low Byte
	frame[7] = (crc >> 8) & 0xFF; // CRC high Byte

	MODBUS_SetTXMode(); //set RS485 to transmit mode
	//%%%%%%%%
	HAL_Delay(10); //wait 10ms in case
	HAL_UART_Transmit(&huart2, frame, sizeof(frame),2000); //transmit the MODBSU frame
	HAL_Delay(5);
	MODBUS_SetRXMode(); //set RS485 to receive mode to receive a response from VFD
}

//Set the motor frequency to full speed
void setMotor60()
{
	uint8_t frame[8]; //define an unsigned 8-bit (1 byte) transmission frame
	uint16_t crc; //define an unsigned 16-bit (2 bytes) cyclic redundancy check
	//Generate each byte of the MODBUS frame
	frame[0] = 0x01;              // Slave Address (adjust if needed)
	frame[1] = 0x06;              // Function Code (Write Single Holding Register)
	frame[2] = 0x00;              // Register Address High Byte (Drive Control Word)
	frame[3] = 0x01;              // Register Address Low Byte
	frame[4] = 0x02;              // Value High Byte
	frame[5] = 0x58;              // Value Low Byte

	crc = Modbus_CRC16(frame, 6); //generate the crc token
	frame[6] = crc & 0xFF;        // CRC low Byte
	frame[7] = (crc >> 8) & 0xFF; // CRC high Byte

	MODBUS_SetTXMode(); //set RS485 to transmit mode
	//%%%%%%%%
	HAL_Delay(10); //wait 10ms in case
	HAL_UART_Transmit(&huart2, frame, sizeof(frame),2000); //transmit the MODBSU frame
	HAL_Delay(5);
	MODBUS_SetRXMode(); //set RS485 to receive mode to receive a response from VFD
}

//Set the motor frequency to a speed defined by input parameter
void setMotorSpeed(uint16_t speed)
{
	uint8_t frame[8]; //define an unsigned 8-bit (1 byte) transmission frame
	uint16_t crc; //define an unsigned 16-bit (2 bytes) cyclic redundancy check

	//Split the 16-bit speed input into two 8-bit values for frames 4/5
	uint8_t speedUpper = (speed>>8) & 0x00FF;  //Frame 4 (high byte)
	uint8_t speedLower = speed & 0x00FF;  //Frame 5 (low byte)
	//Generate each byte of the MODBUS frame
	frame[0] = 0x01;              // Slave Address (adjust if needed)
	frame[1] = 0x06;              // Function Code (Write Single Holding Register)
	frame[2] = 0x00;              // Register Address High Byte (Drive Control Word)
	frame[3] = 0x01;              // Register Address Low Byte
	frame[4] = speedUpper;              // Value High Byte
	frame[5] = speedLower;              // Value Low Byte

	crc = Modbus_CRC16(frame, 6); //generate the crc token
	frame[6] = crc & 0xFF;        // CRC low Byte
	frame[7] = (crc >> 8) & 0xFF; // CRC high Byte

	MODBUS_SetTXMode(); //set RS485 to transmit mode
	//%%%%%%%%
	HAL_Delay(10); //wait 10ms in case
	HAL_UART_Transmit(&huart2, frame, sizeof(frame),2000); //transmit the MODBSU frame
	HAL_Delay(5);
	MODBUS_SetRXMode(); //set RS485 to receive mode to receive a response from VFD
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
  /* USER CODE BEGIN 2 */
  uint8_t tx [7] = "Hello\n\r";
  uint8_t m1 [21] = "Motor speed 150 RPM\n\r";
  uint8_t m2 [21] = "Motor speed 300 RPM\n\r";
  uint8_t m3 [21] = "Motor speed 600 RPM\n\r";
  uint8_t m4 [21] = "Motor speed 900 RPM\n\r";
  uint8_t m5 [22] = "Motor speed 1200 RPM\n\r";
  uint8_t m6 [22] = "Motor speed 1500 RPM\n\r";
  uint8_t m7 [22] = "Motor speed 1800 RPM\n\r";
  uint8_t m8 [22] = "Motor speed 2100 RPM\n\r";
  uint8_t m9 [22] = "Motor speed 2400 RPM\n\r";
  uint8_t m10 [22] = "Motor speed 2700 RPM\n\r";
  uint8_t m11 [22] = "Motor speed 3000 RPM\n\r";
  uint8_t m12 [22] = "Motor speed 3300 RPM\n\r";
  uint8_t m13 [22] = "Motor speed 3600 RPM\n\r";
  uint8_t stopmsg [19] = "Motor stopping...\n\r";
  uint8_t startmsg [19] = "Motor starting...\n\r";
  uint16_t speed = 0;
  uint8_t rxdata;

  startMotor();//call the function to start the motor
  HAL_Delay(250); //wait .25 seconds
  setMotor30();

  HAL_UART_Transmit(&huart1, tx, sizeof(tx), 1000); //transmit a "Hello" string on UART1_TX

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_UART_Receive(&huart1, &rxdata, 1, 1000) == HAL_OK) //receive from UART and store in rx buffer
	  {
		  //ascii 1
		  if(rxdata == 49){
			  setMotorSpeed(100);
			  HAL_UART_Transmit(&huart1, m2, sizeof(m2), 1000);
		  }
		  //ascii 2
		  else if(rxdata == 50){
			  setMotorSpeed(200);
			  HAL_UART_Transmit(&huart1, m3, sizeof(m3), 1000);
		  }
		  //ascii 3
		  else if (rxdata == 51){
			  setMotorSpeed(300);
			  HAL_UART_Transmit(&huart1, m4, sizeof(m4), 1000);
		  }
		  //ascii 4
		  else if (rxdata == 52){
			  setMotorSpeed(400);
			  HAL_UART_Transmit(&huart1, m5, sizeof(m5), 1000);
		  }
		  //ascii 5
		  else if (rxdata == 53){
			  setMotorSpeed(500);
			  HAL_UART_Transmit(&huart1, m6, sizeof(m6), 1000);
		  }
		  //ascii 6
		  else if (rxdata == 54){
			  setMotorSpeed(600);
			  HAL_UART_Transmit(&huart1, m7, sizeof(m7), 1000);
		  }
		  //ascii 7
		  else if (rxdata == 55){
			  setMotorSpeed(700);
			  HAL_UART_Transmit(&huart1, m8, sizeof(m8), 1000);
		  }
		  //ascii 8
		  else if (rxdata == 56){
			  setMotorSpeed(800);
			  HAL_UART_Transmit(&huart1, m9, sizeof(m9), 1000);
		  }
		  //ascii 9
		  else if (rxdata == 57){
			  setMotorSpeed(900);
			  HAL_UART_Transmit(&huart1, m10, sizeof(m10), 1000);
		  }
		  //ascii /
		  else if (rxdata == 47){
			  setMotorSpeed(1000);
			  HAL_UART_Transmit(&huart1, m11, sizeof(m11), 1000);
		  }
		  //ascii *
		  else if (rxdata == 42){
			  setMotorSpeed(1100);
			  HAL_UART_Transmit(&huart1, m12, sizeof(m12), 1000);
		  }
		  //ascii -
		  else if (rxdata == 45){
			  setMotorSpeed(1200);
			  HAL_UART_Transmit(&huart1, m13, sizeof(m13), 1000);
		  }
		  //ascii g (start/go)
		  else if (rxdata == 103){
			  startMotor();
			  HAL_UART_Transmit(&huart1, startmsg, sizeof(startmsg), 1000);
		  }
		  //ascii s (stop)
		  else if (rxdata == 115){
			  stopMotor();
			  HAL_UART_Transmit(&huart1, stopmsg, sizeof(stopmsg), 1000);
		  }
		  else{
			  setMotorSpeed(50);
			  HAL_UART_Transmit(&huart1, m1, sizeof(m1), 1000);
		  }
	  }
	  //HAL_Delay(5000);
	  //setMotor30();

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
