/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "string.h"
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "bitmap.h"
#include "horse_anim.h"
#include "my_flash.h"
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

/* USER CODE BEGIN PV */
uint16_t SPIdata[] = {0x0001};

// Encoder value changed flag
uint8_t encoderCountChanged = 0;

// Flash Memory Test Vars
uint8_t myTestWrite[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
uint8_t myTestRead[5];

// Buffer for USB serial read
uint8_t readBuffer[64];
uint8_t bufferUpdated = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
  uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
  encoderCountChanged++;
}

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  MY_FLASH_SetSectorAddrs(11, 0x080E0000); // set user flash to last sector
  uint8_t clearShiftRef = 0x00;
  HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &clearShiftRef, 1, 1); 
  HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_SET);

  //MY_FLASH_WriteN(0, myTestWrite, 5, DATA_TYPE_8);

  HAL_Delay(1000);

  SSD1306_Init();
  printf("OLED Init\n\r");

  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("HELLO", &Font_16x26, 1);
  SSD1306_GotoXY(10, 30);
  SSD1306_Puts("WORLD", &Font_16x26, 1);
  SSD1306_UpdateScreen();
  printf("Print Hello World\n\r");

  SPIdata[0] = 0xFF00; // = 1 1 1 1  1 1 1 1  1 1 1 1  1 1 1 1

  //HAL_Delay(2500);

  SSD1306_Scrolldiagright(0x00, 0x0f);
  printf("Scroll Right\n\r");
  HAL_Delay(2000);
  
  SSD1306_Clear();

  SSD1306_DrawBitmap(0, 0, moog_logo, 128, 64, 1);
  SSD1306_UpdateScreen();
  SSD1306_ScrollRight(0x00, 0x0f);
  printf("Scroll Right\n\r");
  HAL_Delay(4000);
  SSD1306_Stopscroll();
  // HAL_Delay(2000);
  // SSD1306_Scrolldiagleft(0x00, 0x0f);
  // printf("Scroll Diagonal Left\n\r");
  // HAL_Delay(2000);
  // SSD1306_Scrolldiagright(0x00, 0x0f);
  // printf("Scroll Diagonal Right\n\r");
  // HAL_Delay(2000);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int inverted = 0;
  int delayTime = 0;
  uint8_t outputBuffer = 0x01;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Toggle USER_LED and invert diplay @ 1Hz
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // SSD1306_InvertDisplay(inverted);
    // if(inverted == 0) inverted = 1;
    // else inverted = 0;
    // HAL_Delay(1000);  
    // printSr("OH NO FUCK\n\r");

    // Moog Logo Scroll "Screensaver"
    // SSD1306_Scrolldiagleft(0x00, 0x0f);
    // printf("Scroll Diagonal Left\n\r");
    // HAL_Delay(2000);
    // SSD1306_Scrolldiagright(0x00, 0x0f);
    // printf("Scroll Diagonal Right\n\r");
    // HAL_Delay(2000);


    // Read and diplay current encoder count 

    /*
    if(encoderCountChanged > 1)
    {
      uint32_t tim2_cnt = __HAL_TIM_GET_COUNTER(&htim2) / 2;
      char data [16];

      // Write Time value to OLED
      SSD1306_Clear();
      sprintf(data, "%d", tim2_cnt);
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts(data, &Font_16x26, 1);
      SSD1306_UpdateScreen();

      // Shift Counter value to register diplaying binary on LEDs
      HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &tim2_cnt, 1, 1); 
      HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_SET);

      // Write counter value to USB serial
      printf("%d\n\r", tim2_cnt);

      //Flash On board LED with value chage
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      encoderCountChanged = 0;
      //HAL_Delay(5000);
      //MY_FLASH_ReadN(0, myTestRead, 5, DATA_TYPE_8);
      
      // SSD1306_Clear();
      // SSD1306_DrawBitmap(0, 0, moog_logo, 128, 64, 1);
      // SSD1306_UpdateScreen();
    }


    if(bufferUpdated)
    {
      printf("> %c\n\r", readBuffer[0]);
      bufferUpdated = 0;
    }
    */
    
    
/*
    // Test SPI Shift Out
    HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &outputBuffer, 1, 1); 
    HAL_GPIO_WritePin(SHFT_LATCH_GPIO_Port, SHFT_LATCH_Pin, GPIO_PIN_SET);
    //outputBuffer = (outputBuffer << 1);
    outputBuffer <<= 1;
    if (outputBuffer == 0) outputBuffer = 1;
    HAL_Delay(100);
*/
    

    // Horse Animation
    // {
    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse1, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse2, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse3, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse4, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse5, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse6, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse7, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse8, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse9, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);

    // SSD1306_Clear();
    // SSD1306_DrawBitmap(0, 0, horse10, 128, 64, 1);
    // SSD1306_UpdateScreen();
    // //HAL_Delay(delayTime);
    // }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Overridden to enable printf over USB Virtual Com Port
// osDelay is there in case it doesnt work with FreeRTOS
int _write(int file, char *ptr, int len) { 
    CDC_Transmit_FS((uint8_t*) ptr, len); 
    //osDelay(1);
    return len; 
}

// void TIM2_IRQHandler(void)
// {
//     printf("%d\n\r", (int)(__HAL_TIM_GET_COUNTER(&htim2)/2));
// }

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
