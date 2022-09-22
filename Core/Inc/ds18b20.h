/*
 * ds18b20.h
 *
 *  Created on: 22. 9. 2022
 *      Author: Vejrosta
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

uint8_t DS18B20_Start (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DS18B20_Write (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t data);
uint8_t DS18B20_Read (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* INC_DS18B20_H_ */
