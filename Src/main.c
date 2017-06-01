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
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "stm32f1xx_hal.h"
#include "dac.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <assert.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const int SCI_MODE 		= 0x00;
const int SCI_CLOCKF 	= 0x03;
const int SCI_VOL 		= 0x0b;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void setReg(uint8_t reg, uint16_t data) {
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_RESET);
	uint8_t cmd[4] = {0x02, reg, data>>8, data};
	HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_SET);
}

uint16_t getReg(uint8_t reg) {
	uint8_t cmd[2] = {0x03, reg};
	uint8_t ans[2];
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, cmd, 2, 100);
	HAL_SPI_Receive(&hspi2, ans, 2, 100);
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_SET);
	uint16_t ret = ans[0];
	ret <<= 8;
	ret += ans[1];
	return ret;
}

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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DAC_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
	FATFS fs;
	if (f_mount(&fs, SD_Path, 1)!=FR_OK)
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);
	
	int cntMusicFiles = 0;
	char* pszMusicFiles[256];
	
	DIR dp;
	f_opendir(&dp, "0:/");
	FILINFO fno;	
	
	f_readdir(&dp, &fno);
	while (fno.fname[0]) {
		if (strstr(fno.fname,".MID")==NULL)
			if (strstr(fno.fname,".WAV")==NULL)
				if (strstr(fno.fname,".MP3")==NULL) {
					f_readdir(&dp, &fno);
					continue;
		}
		pszMusicFiles[cntMusicFiles] = (char*) malloc(13);
		strcpy(pszMusicFiles[cntMusicFiles++], fno.fname);
		f_readdir(&dp, &fno);
	}
	f_closedir(&dp);
	
	for (int i=0;i<cntMusicFiles;++i) 
		printf("%s\n", pszMusicFiles[i]);
	
	assert(cntMusicFiles);
	
	/*
	FIL fp;
	if (f_open(&fp, pszMusicFiles[0], FA_READ)!=FR_OK)
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);
	
	setReg(SCI_MODE, 0x4804);
	HAL_Delay(10);
	setReg(SCI_CLOCKF, 0x8BE8);
	HAL_Delay(10);
	//setReg(SCI_VOL, 0x3030);
	
	printf("%x", getReg(SCI_MODE));
	
	while (!f_eof(&fp)) {
		while (HAL_GPIO_ReadPin(DREQ_GPIO_Port, DREQ_Pin)==GPIO_PIN_RESET) ;
		//HAL_Delay(500);
		uint8_t data[32];
		UINT br;
		f_read(&fp,data,32,&br);
		HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, data, br, 100);
		HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_SET);
	}
	
	uint8_t setexpara[4] = {0x02, 0x07, 0x1e, 0x06};
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, setexpara, 4, 100);
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_SET);
	
	uint8_t ans[2];
	setexpara[0]=0x03;
	setexpara[1]=0x06;
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, setexpara, 2, 100);
	HAL_SPI_Receive(&hspi2, ans, 2, 100);
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_RESET);
	for (int i=0;i<2052;++i) {
		HAL_SPI_Transmit(&hspi2, &(ans[1]), 1, 100);
	}
	HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_SET);
	
	uint8_t cmd_can[4] = {0x02, 0x00, 0x48, 0x08};
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, cmd_can, 4, 100);
	HAL_GPIO_WritePin(XCS_GPIO_Port, XCS_Pin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_RESET);
	for (int i=0;i<32;++i) {
		HAL_SPI_Transmit(&hspi2, &(ans[1]), 1, 100);
	}
	HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_SET);
	
	while (!f_eof(&fp)) {
		while (HAL_GPIO_ReadPin(DREQ_GPIO_Port, DREQ_Pin)==GPIO_PIN_RESET) ;
		//HAL_Delay(500);
		uint8_t data[32];
		UINT br;
		f_read(&fp,data,32,&br);
		HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, data, br, 100);
		HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_SET);
	}*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		static int idInPlay = 0;
		if (idInPlay == cntMusicFiles)
			idInPlay = 0;
		
		static FIL fp;
		f_open(&fp, pszMusicFiles[idInPlay], FA_READ);
	
		setReg(SCI_MODE, 0x4804);
		HAL_Delay(10);
		setReg(SCI_CLOCKF, 0x8BE8);
		HAL_Delay(10);
		
		int run = 1;
		while (!f_eof(&fp)&&run) {
			while (HAL_GPIO_ReadPin(DREQ_GPIO_Port, DREQ_Pin)==GPIO_PIN_RESET) 
				if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==GPIO_PIN_RESET)
					run = 0;
			uint8_t data[32];
			UINT br;
			f_read(&fp,data,32,&br);
			HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, data, br, 100);
			HAL_GPIO_WritePin(XDCS_GPIO_Port, XDCS_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
		f_close(&fp);
		++idInPlay;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

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
