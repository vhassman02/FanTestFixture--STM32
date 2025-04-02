/*
MODBUS.H 
Defines the functions used to communicate with the EC Titanium VFD over RS485 using MODBUS protocol
*/

//Redefine RS485 DE/RE* pins PA1/PA0 for ease of use
#define DE_PIN GPIO_PIN_1 //Drive enable == PA1
#define RE_PIN GPIO_PIN_0 //Receive enable == PA0
#define RS485_GPIO_PORT GPIOA //DE/RE on GPIO Port A

UART_HandleTypeDef huart2; 

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
	frame[4] = speedUpper;        // Value High Byte
	frame[5] = speedLower;        // Value Low Byte

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
