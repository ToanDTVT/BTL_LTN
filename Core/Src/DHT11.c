#include "DHT11.h"
#include "LCD_i2c.h"
#include <stdio.h>
#include "main.h"
//extern TIM_HandleTypeDef htim1;
//
//static void DHT11_Delay(uint16_t us) {
//    __HAL_TIM_SET_COUNTER(&htim1, 0);
//    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
//}
//
//static uint8_t DHT11_Read_Bit(void) {
//    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
//    DHT11_Delay(40);
//    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) {
//        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET);
//        return 1;
//    } else {
//        return 0;
//    }
//}
//
//static uint8_t DHT11_Read_Byte(void) {
//    uint8_t i, data = 0;
//    for (i = 0; i < 8; i++) {
//        data <<= 1;
//        data |= DHT11_Read_Bit();
//    }
//    return data;
//}
//
//uint8_t DHT11_Read_Data(DHT11_Data_TypeDef *DHT11_Data) {
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//    HAL_Delay(18);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//    DHT11_Delay(30);
//    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
//        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
//        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET);
//        DHT11_Data->Hum = DHT11_Read_Byte();
//        DHT11_Read_Byte(); // Skip decimal point
//        DHT11_Data->Temp = DHT11_Read_Byte();
//        return DHT11_OK;
//    } else {
//        return DHT11_ERROR;
//    }
//}


void Display_Temp (float Temp)
{
	char str[20] = {0};
	lcd_goto_XY(0,0);
	sprintf (str, "Temp: %.2f", Temp);
	lcd_send_string(str);
	lcd_send_data(0b11011111);
	lcd_send_data('C');
}

//function to display RH
void Display_Rh (float Humid)
{
	char str[20] = {0};
	lcd_goto_XY(1, 0);
	sprintf (str, "Humid: %.2f", Humid);
	lcd_send_string(str);
	lcd_send_data('%');
}

// variables declarations
//    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
//    uint16_t SUM, RH, TEMP;
//
//    float Temperature = 0;
//    float Humidity = 0;
//    uint8_t Presence = 0;
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//function to set pin as input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //can be changed to PULLUP if no data is received from the pin
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
//***************************DHT11 FUNCTIONS BEGIN HERE**********************************************/

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_5

//Function to send the start signal
void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN); //set the dht pin as output
	/***********************************************/
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1); //initialize with data pin high
	HAL_Delay(1000); //wait for 1000 milliseconds
	/***********************************************/

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0); //pull the pin low
	delay(18000); //wait 18 milliseconds
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1); //pull the pin high
	delay(20); //wait for 20 microseconds
	Set_Pin_Input(DHT11_PORT, DHT11_PIN); //set the pin as input
}

//dh11 function to check response
uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay(40); //first wait for 40 microseconds
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) //check for pin to be low
	{
		delay(80); //wait for 80 microseconds
		if((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1; //check if pin is high and return 1 to show sensor is present
		else Response = -1; //255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to go low again

	return Response;
}

//function to read data from dht11 signal pin
uint8_t DHT11_Read (void)
{
	uint8_t i, j;
	for (j=0;j<8;j++)
	{
		while(!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to change to high
		delay(40); //wait for 40 microseconds
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) //if the pin is low
		{
			i&= ~(1<<(7-j)); //write 0
		}
		else i|= (1<<(7-j)); //if the pin is high write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to go low
	}

	return i;
}

