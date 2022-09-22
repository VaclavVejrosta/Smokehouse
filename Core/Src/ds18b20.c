/*
 * ds18b20.c
 *
 *  Created on: 22. 9. 2022
 *      Author: Vejrosta
 */

#include "main.h"
#include "ds18b20.h"

uint8_t DS18B20_Start (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Response = 0;
	Set_Pin_Output(GPIOx, GPIO_Pin);   // set the pin as output
	HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 0);  // pull the pin low
	delay_us (480);   // delay according to datasheet

	Set_Pin_Input(GPIOx, GPIO_Pin);    // set the pin as input
	delay_us (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)))
	{
		Response = 1;    // if the pin is low i.e the presence pulse is detected
	}
	else
	{
		Response = -1;
	}

	delay_us (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t data)
{
	Set_Pin_Output(GPIOx, GPIO_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(GPIOx, GPIO_Pin);  // set as output
			HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 0);  // pull the pin LOW
			delay_us (1);  // wait for 1 us

			Set_Pin_Input(GPIOx, GPIO_Pin);  // set as input
			delay_us (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(GPIOx, GPIO_Pin);
			HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 0);  // pull the pin LOW
			delay_us (50);  // wait for 60 us

			Set_Pin_Input(GPIOx, GPIO_Pin);
		}
	}
}

uint8_t DS18B20_Read (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t value=0;

	Set_Pin_Input(GPIOx, GPIO_Pin);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(GPIOx, GPIO_Pin);   // set as output

		HAL_GPIO_WritePin (GPIOx, GPIO_Pin, 0);  // pull the data pin LOW
		delay_us (1);  // wait for > 1us

		Set_Pin_Input(GPIOx, GPIO_Pin);  // set as input
		if (HAL_GPIO_ReadPin (GPIOx, GPIO_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us (50);  // wait for 60 us
	}
	return value;
}
