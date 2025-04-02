/*
FREQ.H
Defines the functions used for transducer frequency measurement
*/
#include "stdio.h"

TIM_HandleTypeDef htim14; //Timer 14 (PA4)
UART_HandleTypeDef huart1; //USB UART

#define TIMCLOCK   8000000
#define PRESCALER  1

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured_TIM14 = 0;

uint8_t frequencyCapture[64];

/* Measure Frequency */
float frequencySpeed = 0;
float frequencyTorque = 0;

int captureCounter = 0; //counts the number of times a capture has occured
int skipCounter = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(skipCounter > 0)
	{
		skipCounter--;
		return;
	}

	if (Is_First_Captured_TIM14==0) // if the first rising edge is not captured
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
		Is_First_Captured_TIM14 = 1;  // set the first captured as true
	}

	else   // If the first rising edge is captured, now we will capture the second edge
	{
		IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

		if (IC_Val2 > IC_Val1)
		{
			Difference = IC_Val2 - IC_Val1;
		}

		else if (IC_Val1 > IC_Val2)
		{
			Difference = (0xffff - IC_Val1) + IC_Val2;
		}

		float refClock = TIMCLOCK/(PRESCALER);

		frequencyTorque = refClock/Difference;

		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
		Is_First_Captured_TIM14 = 0; // set first edge captured status back to false

		uint8_t nl[2] = "\n\r";
		sprintf((char*)frequencyCapture, "Frequency: %f\n\r", frequencyTorque); //converts frequencyTorque into a character string
		HAL_UART_Transmit(&huart1, frequencyCapture, sizeof(frequencyCapture), 1000);
		HAL_UART_Transmit(&huart1, nl, sizeof(nl), 1000);

		//HAL_TIM_IC_Stop_IT(&htim14, TIM_CHANNEL_1); //stop the timer interrupt after capture

		skipCounter = 9998;
	}
}
