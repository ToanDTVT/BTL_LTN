/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "KeyPad.h"
#include "KeyPadConfig.h"
#include "LCD_i2c.h"
#include "DHT11.h"
#include "ds1307_for_stm32_hal.h"
#include "control.h"
#include <stdio.h>
//#define DHT11_PORT GPIOB
//#define DHT11_PIN GPIO_PIN_5

#define DHT11_OK      0
#define DHT11_ERROR   1
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
unsigned char password[4] = "3112";
unsigned char key_password[4];
uint32_t var;

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
   uint16_t SUM, RH, TEMP;

   float Temperature = 0;
   float Humidity = 0;
   uint8_t Presence = 0;

//typedef struct {
//    uint8_t Hum;   // Humidity
//    uint8_t Temp;  // Temperature
//} DHT11_Data_TypeDef ;
//
//DHT11_Data_TypeDef DHT11_Data;

//DHT11_Data_TypeDef DHT11_Data;
//
bool check_pass() {
    for (int i = 0; i < 4; ++i) {
        if (password[i] != key_password[i]) {
            return false;
        }
    }
    return true;
}


void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}

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


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum StateMachine {
	STATE_IDLE,
	STATE_ACTIVE,
	STATE_EMERGENCY,
	STATE_SAFE,
	STATE_OPEN_DOOR,
	STATE_SCREEN_PASSWORD,
	STATE_PASSWORD,
	STATE_WRONG_PASSWORD,
	STATE_TRUE_PASSWORD,
	STATE_MENU_1,
	STATE_MENU_2,
	STATE_GAS,
	STATE_TEMP,
	STATE_TIME
};
volatile enum StateMachine currentstate = STATE_IDLE;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
      HAL_ADC_Start_DMA(&hadc1, &var, 1);

      lcd_init();
        lcd_clear_display();
        lcd_goto_XY(1, 2);
        lcd_send_string("SMART HOME");

        const char *DAYS_OF_WEEK[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
        	/* Start DS1307 timing. Pass user I2C handle pointer to function. */
        	DS1307_Init(&hi2c2);
        	/* To test leap year correction. */
        	DS1307_SetTimeZone(+8, 00);
        	DS1307_SetDate(12);
        	DS1307_SetMonth(12);
        	DS1307_SetYear(2023);
        	DS1307_SetDayOfWeek(2);
        	DS1307_SetHour(9);
        	DS1307_SetMinute(17);
        	DS1307_SetSecond(30);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
          unsigned char press_keypad = KEYPAD_GetChar();
            uint8_t date = DS1307_GetDate();
			uint8_t month = DS1307_GetMonth();
			uint16_t year = DS1307_GetYear();
			uint8_t dow = DS1307_GetDayOfWeek();
			uint8_t hour = DS1307_GetHour();
			uint8_t minute = DS1307_GetMinute();
			uint8_t second = DS1307_GetSecond();
			int8_t zone_hr = DS1307_GetTimeZoneHour();
			uint8_t zone_min = DS1307_GetTimeZoneMin();
//          DHT11_Start();
          /* USER CODE END WHILE */

             switch(currentstate){
             case STATE_IDLE:
               lcd_goto_XY(1, 0);
               lcd_send_string("  SMART HOME");
               if(press_keypad != 0){
                 currentstate = STATE_SCREEN_PASSWORD;
               }
               break;

             case STATE_ACTIVE:
               lcd_clear_display();
               break;

             case STATE_EMERGENCY:
            	 lcd_goto_XY(1, 0);
            	 lcd_send_string("GAS IS LEAKING !");
            	 lcd_goto_XY(2, 0);
            	 lcd_send_string(" EMERGENCY !");
            	 HAL_Delay(2000);
            	 lcd_clear_display();
            	 char gas_var2 = gas(var);
            	 if(gas_var2 <= 30){HAL_Delay(200); lcd_clear_display(); currentstate = STATE_SAFE; break;}
            	 else{
            		 lcd_goto_XY(1, 0);
            		 lcd_send_string("OPEN THE DOOR !");
            		 if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE; break;}
            		 if(press_keypad == '*'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE; break;}
            	 }
            	 if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}

               break;

             case STATE_SAFE:
            	 lcd_goto_XY(1, 0);
            	 lcd_send_string("IT'S SAFE NOW");
            	 HAL_Delay(2000);
            	 lcd_clear_display();
            	 currentstate = STATE_GAS;
               break;

             case STATE_OPEN_DOOR:
               break;

             case STATE_SCREEN_PASSWORD:
               currentstate = STATE_PASSWORD;
               lcd_clear_display();
               lcd_goto_XY(1, 0);
               lcd_send_string("  PASSWORD");
               lcd_goto_XY(2, 2);
               int k = 0;
               break;

             case STATE_PASSWORD:
               HAL_Delay(100);
               if(press_keypad != 0){
                 HAL_Delay(100);
                 lcd_send_data(press_keypad);
                 HAL_Delay(100);
                 key_password[k] = press_keypad;
                 k++;
                 if(k == 4){
                   if(check_pass() == true){
                     currentstate = STATE_TRUE_PASSWORD;
                     HAL_Delay(100);
                     lcd_clear_display();
                     lcd_goto_XY(1, 0);
                     lcd_send_string("  SUCCESS !!");
                     lcd_goto_XY(2, 1);
                     lcd_send_string("OPEN THE DOOR");
                     HAL_Delay(1000);
                     lcd_clear_display();
                   }
                   else{
                     HAL_Delay(100);
                     lcd_clear_display();
                     lcd_goto_XY(1, 0);
                     lcd_send_string("  FAILURE !!");
                     lcd_goto_XY(2, 1);
                     lcd_send_string("TRY AGAIN");
                     HAL_Delay(1000);
                     currentstate = STATE_WRONG_PASSWORD;
                   }
                   HAL_Delay(100);
                 }
               }
               break;

             case STATE_TRUE_PASSWORD:
               HAL_Delay(100);
               lcd_goto_XY(1, 0);
               lcd_send_string("  MENU");
               lcd_goto_XY(2, 1);
               lcd_send_string("1. GAS");
               if(press_keypad == 'R'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_1;}
               else if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate =STATE_MENU_2;}
               else if(press_keypad == 'E'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_GAS;}
               if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
               break;

             case STATE_WRONG_PASSWORD:
               currentstate = STATE_SCREEN_PASSWORD;
               break;

             case STATE_MENU_1:
               HAL_Delay(100);
             lcd_goto_XY(1, 0);
             lcd_send_string("  MENU");
             lcd_goto_XY(2, 1);
             lcd_send_string("2. TEMP");
             if(press_keypad == 'R'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_2;}
             else if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_TRUE_PASSWORD;}
             else if(press_keypad == 'E'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_TEMP;}
             if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
             break;

             case STATE_MENU_2:
               HAL_Delay(100);
             lcd_goto_XY(1, 0);
             lcd_send_string("  MENU");
             lcd_goto_XY(2, 1);
             lcd_send_string("3. TIME");
             if(press_keypad == 'R'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_TRUE_PASSWORD;}
             else if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_1;}
             else if(press_keypad == 'E'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_TIME;}
             if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
               break;

             case STATE_GAS:
               HAL_Delay(100);
               		 char gas_var = gas(var);
                     lcd_goto_XY(1, 0);
                     lcd_send_string("GAS: ");
                     lcd_send_data(gas_var/100 + '0');
                     lcd_send_data((gas_var%100)/10 + '0');
                     lcd_send_data(gas_var%10 + '0');
                     if(gas_var >= 90){HAL_Delay(200); lcd_clear_display(); currentstate = STATE_EMERGENCY; break;}
                    if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
                    if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_TRUE_PASSWORD;}
               break;

             case STATE_TEMP:
               HAL_Delay(100);
               DHT11_Start();
              	  	     Presence = DHT11_Check_Response(); // record the response from the sensor

              	  	     // Five bytes of data
              	  	     Rh_byte1 = DHT11_Read();
              	  	     Rh_byte2 = DHT11_Read();
              	  	     Temp_byte1 = DHT11_Read();
              	  	     Temp_byte2 = DHT11_Read();
              	  	     SUM = DHT11_Read();

              	  	     TEMP = Temp_byte1;
              	  	     RH = Rh_byte1;

              	  	     Temperature = (float)TEMP;
              	  	     Humidity = (float)RH;

              	  	     Display_Temp(Temperature);
              	  	     Display_Rh(Humidity);

              	  	     HAL_Delay(1200);
            	   if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE; break;}
				   if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_1; break;}


    //              if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
    //              if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_1;}
               break;

             case STATE_TIME:
               HAL_Delay(100);

               		char buffer[100] = { 0 };
               		sprintf(buffer, "%s: %02d-%02d-%04d \n", DAYS_OF_WEEK[dow], date, month, year);
               		lcd_goto_XY(1, 0);
               		lcd_send_string(buffer);

               		sprintf(buffer, "%02d:%02d:%02d  \n",hour, minute, second);
					lcd_goto_XY(2, 0);
					lcd_send_string(buffer);
               		/* May show warning below. Ignore and proceed. */
//               		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), 1000);
               		HAL_Delay(250);
                  if(press_keypad == 'C'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_IDLE;}
                  if(press_keypad == 'L'){HAL_Delay(200);  lcd_clear_display(); currentstate = STATE_MENU_2;}
                       break;
               break;
             }
        /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
