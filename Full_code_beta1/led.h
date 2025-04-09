/*
LED.H
Defines functions used to turn on/off status LEDs
*/

//LED ON FUNCTIONS
void LEDPowerOn()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //turn on the power LED
}

void LEDUSBOn()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); //turn on USB connect LED
}

void LEDTestInactiveOn()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //turn on test active LED
}

void LEDTestActiveOn()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //turn on test inactive LED
}

//LED OFF FUNCTIONS
void LEDPowerOff()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //turn on the power LED
}

void LEDUSBOff()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); //turn on USB connect LED
}

void LEDTestInactiveOff()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //turn on test active LED
}

void LEDTestActiveOff()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //turn on test inactive LED
}
