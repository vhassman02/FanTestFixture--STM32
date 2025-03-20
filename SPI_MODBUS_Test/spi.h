/*
SPI.H
Defines the functions used to communicate with the LCD display over SPI 4-wire protocol
*/

SPI_HandleTypeDef hspi2;

//reset display (set low to reset)
void resetDisplay()
{
	//RST is on PB11
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(300); //hold low for 0.3 seconds
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10); //delay for 10ms before continuing to other commands
}

//sends data to display over SPI using software-controlled chip select
void sendSPI(uint8_t data[])
{
	//CS is on PB12
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //set CS low for transmission
	HAL_SPI_Transmit(&hspi2, data, 3, 1000); //send data to display
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //set CS high to end transmission
}

//clears the display of all characters
void clearDisplay()
{
	uint8_t clear[3] = {0xF8, 0x80, 0x00};
	sendSPI(clear); //send the clear command
}

//turns off the display cursor and cursor blink
void cursorOff()
{
	uint8_t cursorBlinkOff[3] = {0xF8, 0x30, 0x00};
	sendSPI(cursorBlinkOff);
}

//initialize the display
void initializeDisplay()
{
	uint8_t funcSet1[3] = {0xF8, 0x50, 0xC0};
	uint8_t exFuncSet[3]= {0xF8, 0x90, 0x00};
	uint8_t entryModeSet[3] = {0xF8, 0x60, 0x00};
	uint8_t biasSetting[3] = {0xF8, 0x70, 0x80};
	uint8_t funcSet2[3] = {0xF8, 0x90, 0xC0};
	uint8_t internalOSC[3] = {0xF8, 0xD0, 0x80};
	uint8_t followerCtrl[3] = {0xF8, 0x70, 0x60};
	uint8_t pwrCtrl[3] = {0xF8, 0x60, 0xA0};
	uint8_t contrastSet[3] = {0xF8, 0x50, 0xE0};
	uint8_t funcSet3[3] = {0xF8, 0x10, 0xC0};
	uint8_t on[3] = {0xF8, 0xF0, 0x00};

	//send a dummy byte to ensure SPI_CLK is high before starting transmission
	//CS is not low during this transmission so display does not receive dummy data
	uint8_t dummyData[1] = {0xFE};
	HAL_SPI_Transmit(&hspi2, dummyData, 1, 1000); //send data to display

	//INITIALIZATION COMMANDS:
	sendSPI(funcSet1);
	sendSPI(exFuncSet);
	sendSPI(entryModeSet);
	sendSPI(biasSetting);
	sendSPI(funcSet2);
	sendSPI(internalOSC);
	sendSPI(followerCtrl);
	sendSPI(pwrCtrl);
	sendSPI(contrastSet);
	sendSPI(funcSet3);
	sendSPI(on);

	//EXTRA COMMANDS:
	clearDisplay(); //clear the display before any data will be sent
	cursorOff(); //turns off the cursor for better viewing
}

//sends text data to display's DDRAM
void sendSPIData(uint8_t data[])
{
	uint8_t lower = data[0] & 0x0F; 	//bitwise AND to extract lower nibble
	lower = lower << 4; 				//bit shift to move lower nibble to upper nibble
	uint8_t upper = data[0] & 0xF0; 	//bitwise AND to extract upper nibble (no bit shift)
	uint8_t transmission[3] = {0xFA, lower, upper}; //send the data, first bit always 0xFA
	sendSPI(transmission);
}
