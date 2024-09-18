/*
 * DHT11.h
 *
 *  Created on: Dec 10, 2023
 *      Author: DELL
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f1xx_hal.h"

//typedef struct {
//    uint8_t Hum;   // Humidity
//    uint8_t Temp;  // Temperature
//} DHT11_Data_TypeDef;
//
//#define DHT11_OK      0
//#define DHT11_ERROR   1
//
//uint8_t DHT11_Read_Data(DHT11_Data_TypeDef *DHT11_Data);



#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_5

void Display_Temp (float Temp);

void Display_Rh (float Humid);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void DHT11_Start (void);

uint8_t DHT11_Check_Response (void);

uint8_t DHT11_Read (void);

#endif /* INC_DHT11_H_ */
