/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int HalfComplete =0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
HalfComplete = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
FullComplete = 1;
}*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define DAC_NUM_Samples_Ch1 9125											// Number of samples in file 1
#define DAC_NUM_Samples_Ch2 9125											// Number of samples in file 2 
#define POWER_WIN_LEN 1000

int32_t offset=0;


char formattedSignal_Ch1[6*DAC_NUM_Samples_Ch1]; 			// Formatted signal ( ASCII) read from the SD Card 
//char formattedSignal_Ch2[6*DAC_NUM_Samples_Ch2]; 

uint32_t Tx_Signal_Ch1[DAC_NUM_Samples_Ch1];					// Transmitted Signal after conversion to HEX 
uint32_t Tx_Signal_Ch2[DAC_NUM_Samples_Ch2];	


int32_t ADC_Buffer[POWER_WIN_LEN];


// These variables to work with FAT file system

FATFS fatfs1;
FIL file1,file2;
UINT testByte1,testByte2;

// These variables for FM signal detection

int Fm_on = 0;
uint32_t Fm_sum = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DAC_Init();
  MX_SDIO_SD_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
if(f_mount(&fatfs1,SD_Path,1)==FR_OK){
	
			f_open(&file1,"Tx1.txt",FA_READ); 																							
			f_read(&file1,formattedSignal_Ch1,6*DAC_NUM_Samples_Ch1,&testByte1);
			f_close(&file1);

	// Convert ASCII formatted signal to HEX
	
	if((6*DAC_NUM_Samples_Ch1)==testByte1){
	for(int i=0;i<=DAC_NUM_Samples_Ch1;i++){
	Tx_Signal_Ch1[i]=1000*(formattedSignal_Ch1[6*i]-48)+
	100*(formattedSignal_Ch1[6*i+1]-48)+10*(formattedSignal_Ch1[6*i+2]-48)+
	1*(formattedSignal_Ch1[6*i+3]-48);
}
}
}

// Read file 2 from the SD Card

	
			f_open(&file2,"Tx2.txt",FA_READ);
			f_read(&file2,formattedSignal_Ch1,6*DAC_NUM_Samples_Ch2,&testByte2);
			f_close(&file2);

// Convert ASCII formatted signal to HEX
	if((6*DAC_NUM_Samples_Ch2)==testByte2){
	for(int i=0;i<=DAC_NUM_Samples_Ch2;i++){
	Tx_Signal_Ch2[i]=1000*(formattedSignal_Ch1[6*i]-48)+100*(formattedSignal_Ch1[6*i+1]-48)+10*(formattedSignal_Ch1[6*i+2]-48)+1*(formattedSignal_Ch1[6*i+3]-48);
}
}
	


HAL_Delay(25);

//BB treshold detector !!!!!
for(uint16_t i=0;i<100;i++)
{
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	offset = offset + HAL_ADC_GetValue(&hadc1);
	HAL_Delay(5);
}
offset = offset/100;


int32_t value=0,treshold=15000;
uint16_t f_element=0,l_element=1;

for(uint32_t i=0;i<POWER_WIN_LEN;i++)ADC_Buffer[i]=0;


	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	HAL_ADC_GetValue(&hadc1);

HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);

while(value < treshold)
{
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);
	
	int32_t ADC_val;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	ADC_val=(int32_t)HAL_ADC_GetValue(&hadc1) - offset;

	
	ADC_Buffer[f_element]=ADC_val;
	value = abs((int)(value + ADC_Buffer[f_element]-ADC_Buffer[l_element]));
	
	f_element++;
	f_element = f_element%POWER_WIN_LEN;
	
	l_element++;
	l_element = l_element%POWER_WIN_LEN;
}

HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
	/*HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_Delay(50);	*/
	

NVIC_EnableIRQ(DMA1_Stream5_IRQn);
NVIC_EnableIRQ(DMA1_Stream6_IRQn);

HAL_TIM_Base_Start(&htim4);																																			// Start timer 4 

//HAL_Delay(2);
HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,Tx_Signal_Ch1,DAC_NUM_Samples_Ch1,DAC_ALIGN_12B_R);				// Transmit Signal 1
HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,Tx_Signal_Ch2,DAC_NUM_Samples_Ch2,DAC_ALIGN_12B_R);				// Transmit Signal 2

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
		HAL_Delay(100);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
