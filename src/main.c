/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "eth.h"
#include "i2c.h"
#include "iwdg.h"
#include "quadspi.h"
#include "rtc.h"
#include "sai.h"
#include "sdmmc.h"
#include "spdifrx.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "wwdg.h"
#include "gpio.h"
#include "fmc.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WINDOWS_WITH			280
#define WINDOWS_HEIGHT		280

#define	cTIME_MEASURE_1		0
#define	cTIME_MEASURE_2		1
#define	cTIME_MEASURE_3		2
#define	cTIME_MEASURE_4		3
#define	cTIME_MEASURE_5		4
/* Use this macro to keep track of the time */
#define TIM_MEASURE_START {time_start = __HAL_TIM_GET_COUNTER(&TimHandle);}
/* Use this macro with index to save a partial time. */
#define TIM_MEASURE_END(x) {time_end = __HAL_TIM_GET_COUNTER(&TimHandle);     \
														time_diff = time_end - time_start;								\
														tdiff[x]=(float)time_diff;}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  lcd_status = LCD_OK;
uint32_t ts_status = TS_OK;
TS_StateTypeDef  TS_State = {0};
uint16_t x1, y1;
float PxlNet[28*28];
float PxlNet2D[28][28];
uint32_t idx=0;
uint32_t ysrc, xsrc, xstp, ystp, pxl;
uint32_t xold, yold, firstTime;
uint32_t tmp;
uint32_t LCDAddress;
uint32_t *ptr;
uint32_t *mem;
extern uint32_t classification_result;
char riga[250];
volatile uint32_t time_start, time_end, time_diff;
float tdiff[5];
uint32_t xpos, ypos;

TIM_HandleTypeDef     TimHandle;
void TimeMeasureInit( void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CPU_CACHE_Enable(void);
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point );
static void MPU_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CPU_CACHE_Enable();
	MPU_Config();
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	TimeMeasureInit();
  /* Initialize the LCD */
  lcd_status = BSP_LCD_Init();
  while(lcd_status != LCD_OK);
  
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);
  /* Select the LCD Background Layer */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);
	BSP_LCD_SetLayerVisible( LTDC_ACTIVE_LAYER_BACKGROUND, ENABLE);
	
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_ADC1_Init();
//  MX_ADC3_Init();
//  MX_ETH_Init();
//  MX_FMC_Init();
//  MX_I2C1_Init();
//  MX_I2C4_Init();
//  MX_IWDG_Init();
//  MX_QUADSPI_Init();
//  MX_RTC_Init();
//  MX_SAI1_Init();
//  MX_SAI2_Init();
//  MX_SDMMC2_MMC_Init();
//  MX_SPDIFRX_Init();
//  MX_SPI2_Init();
//  MX_TIM3_Init();
//  MX_TIM10_Init();
//  MX_TIM11_Init();
//  MX_TIM12_Init();
//  MX_UART5_Init();
//  MX_USB_OTG_HS_PCD_Init();
//  MX_WWDG_Init();
  MX_CRC_Init();
  MX_X_CUBE_AI_Init();
	
  /* USER CODE BEGIN 2 */
	
	/* Disegno una finestra in cui scrivere. */
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawRect((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2)-1, (BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)-1, WINDOWS_WITH+2, WINDOWS_HEIGHT+2);
	/* OK */
	BSP_LCD_FillCircle( BSP_LCD_GetXSize()-50,BSP_LCD_GetYSize()-50, 48);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	sprintf(riga, "OK");
	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()-50-13,BSP_LCD_GetYSize()-50-13,(uint8_t*)riga,LEFT_MODE);
	/* msg */
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	sprintf(riga, "Write letter/number inside the box!");
	BSP_LCD_DisplayStringAt(0,420,(uint8_t*)riga,CENTER_MODE);
	
  /* Reset touch data information */
  BSP_TS_ResetTouchData(&TS_State);
  /* Touchscreen initialization */
  ts_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
	/* Loop per la scrittura della cifra */
  while (1)
  {
		/* Loop per la gestione del touch e sscrittura su LCD. */
		firstTime=0;
		xold=yold=0;
		
		while(1)
		{
			ts_status = BSP_TS_GetState(&TS_State);
			if(TS_State.touchDetected)
			{
				x1 = TS_State.touchX[0];
				y1 = TS_State.touchY[0];
				/* Verifico se il tap è sull'exit. */
				if ( CheckRectangle(BSP_LCD_GetXSize()-100,BSP_LCD_GetYSize()-100,100,100,x1,y1) && firstTime)
				{
					/* Esco dal loop per interpretare l'immagine. */
					break;
				} 
				/* Verifico se sta scrivendo all'interno della finestra. */
				else if ( CheckRectangle( (BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2), (BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2),WINDOWS_WITH,WINDOWS_HEIGHT,x1,y1)) 
				{
					if ( firstTime==0)
					{
						xold=x1; yold=y1;
						firstTime=1;
						BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
						BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
						sprintf(riga, "                                   ");
						BSP_LCD_DisplayStringAt(0,420,(uint8_t*)riga,CENTER_MODE);						
					}
					/* Disegno un cerchio alle coordinate del tap. */
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_FillCircle(x1, y1, 7);
					//BSP_LCD_DrawLine(xold, yold, x1,y1);
					xold=x1; yold=y1;
				}
			}
		} /* end while(1) */
		/* Riduco la finestra di scrittura a 28x28 pixel. */
		idx=0;
		pxl=0;
		/* */
		for ( ysrc=(BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2),ypos=0; ysrc<((BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2))+WINDOWS_HEIGHT; ysrc+=10,ypos++)
		{
			for ( xsrc=(BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2),xpos=0; xsrc<((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2))+WINDOWS_WITH; xsrc+=10,xpos++)
			{
				for ( ystp=0; ystp<10; ystp++)
				{
					for ( xstp=0; xstp<10; xstp++)
					{
						tmp = BSP_LCD_ReadPixel( xsrc+xstp,ysrc+ystp);
						pxl += (tmp&0xFF);
					}
				}
				pxl /= 100;
				if (pxl>0) pxl=255;
				PxlNet2D[ypos][xpos] = (float)pxl;
				BSP_LCD_DrawPixel(xsrc/10,ysrc/10,(0xFF<<24)|(pxl<<16)|(pxl<<8)|pxl);
				pxl=0;
			}
		}		
    /* USER CODE END WHILE */
		//TIM_MEASURE_START(cTIME_MEASURE_1);
		MX_X_CUBE_AI_Process();
		//TIM_MEASURE_END(cTIME_MEASURE_1);
    /* USER CODE BEGIN 3 */
		if ( classification_result <= 9)
		{
			sprintf(riga,"The Number is: %d",classification_result);
		} else {
			if ( classification_result <= 36)
			{
				sprintf(riga,"The Letter is: %c",(classification_result-10)+65);
			} else {
				sprintf(riga,"The Letter is: %c",(classification_result-36)+97);
			}
		}
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0,20,(uint8_t*)riga,CENTER_MODE);
		sprintf(riga, "AI process:%2.3f", tdiff[cTIME_MEASURE_1]*0.001);
		BSP_LCD_DisplayStringAt(0,420,(uint8_t*)riga,CENTER_MODE);
		/* */
		HAL_Delay(3000);
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawRect((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2)-1, (BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)-1, WINDOWS_WITH+2, WINDOWS_HEIGHT+2);		
		/* */
		BSP_LCD_FillCircle( BSP_LCD_GetXSize()-50,BSP_LCD_GetYSize()-50, 48);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(riga, "OK");
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()-50-13,BSP_LCD_GetYSize()-50-13,(uint8_t*)riga,LEFT_MODE);
		/* */
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		sprintf(riga, "Write letter/number inside the box!");
		BSP_LCD_DisplayStringAt(0,420,(uint8_t*)riga,CENTER_MODE);		
  }
  /* USER CODE END 3 */
}

#if 1
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
#else
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_SDMMC2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 3;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

/* USER CODE BEGIN 4 */

/**
 * @brief
 * @param[in] none
 * @retval 		none
*/
void TimeMeasureInit(void)
{
	 /* Set Timers instance */
	TimHandle.Instance = TIM2;

	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_ACTIVATED); // run the timer on HCLK freq

	/*====================== Master configuration : TIM2 =======================*/
	/* Initialize TIM2 peripheral in counter mode*/
	TimHandle.Init.Period            = 0xFFFFFFFF;
	TimHandle.Init.Prescaler         = 99999;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}  

	HAL_TIM_Base_Start(&TimHandle);
}

/**
  * @brief  Configure the MPU attributes .
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();
  
  /* Configure the MPU attributes for extSRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/*************************************************************************
 * Function Name: CheckRectangle
 * Parameters: Int32U X_up, Int32U Y_up - rectangle coordinate
 *             Int32U X_size, Int32U Y_size - rectangle size
 *             Int32U X_poin, Int32U Y_point - point coordinate
 *
 * Return: Boolean
 *    TRUE  - the point is inside  from the rectangle
 *    FALSE - the point is outside from the rectangle
 *
 * Description: Check whether the coordinate of point is inside from a rectangle
 *
 *************************************************************************/
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point )
{
  if((X_up <= X_point) && (X_point <= X_up+X_size) &&
     (Y_up <= Y_point) && (Y_point <= Y_up+Y_size))
  {
    return (1);
  }
  return (0);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
