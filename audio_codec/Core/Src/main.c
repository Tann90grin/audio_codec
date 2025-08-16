/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 4096
#define DEVICE_ADDR 0x34  // Example: MPU6050
#define REG_ADDR    0x06
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t txBuffer[BUFSIZE] = {0};
volatile int16_t rxBuffer[BUFSIZE] = {0};

volatile uint8_t cplt = 0;
volatile uint8_t hplt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C3_Init();
  MX_I2S2_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  void wm8731_send(uint16_t addr, uint16_t data){
  	uint16_t cmd = (addr << 9) | (data & 0x1FF);
  	uint8_t cmd_buf[2] = {cmd >> 8, cmd & 0xFF};
  	HAL_I2C_Master_Transmit(&hi2c3, 0x1A<<1, cmd_buf, 2, HAL_MAX_DELAY);
  }

  void wm8731_reset(void){
  	uint16_t cmd = (0x0F << 9) | (0x00 & 0x1FF);
  	uint8_t cmd_buf[2] = {cmd >> 8, cmd & 0xFF};
  	HAL_I2C_Master_Transmit(&hi2c3, 0x1A<<1, cmd_buf, 2, HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  char msg[128];


  wm8731_reset();
  wm8731_send(0x06,0x00);
  wm8731_send(0x08,0x00);
  wm8731_send(0x00,0b000011011);
  wm8731_send(0x07,0b00000010);
  wm8731_send(0x05,0xb00110);
  wm8731_send(0x04,0b00010010);
  wm8731_send(0x02,0b001111111);
  wm8731_send(0x09,0x01);

  HAL_I2SEx_TransmitReceive_DMA(&hi2s2,(uint16_t *)txBuffer,(uint16_t *)rxBuffer,BUFSIZE);

  while (1)
  {

	  if(cplt == 1){
		  cplt = 0;
		  //HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_9);

	  }
	  if(hplt == 1){
		  hplt = 0;
		  //HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_9);

	  }

      //snprintf(msg, sizeof(msg), "%d\r", (int16_t)received_sample[0]);
      //HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float soft_clp(float Q, float d, int16_t x_raw){
	float x = x_raw/32767.0;
	return ((x-Q)/(1-exp(0-d*(x-Q))))+(Q/(1-exp(d*Q)));
}



HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	memcpy((void*)&txBuffer[0], (void*)&rxBuffer[0], BUFSIZE * sizeof(int16_t)/2);
	for(int i=0;i<BUFSIZE/2;i++){
//		if(txBuffer[i]>=100){
//			txBuffer[i]=100;
//		}
		txBuffer[i]=soft_clp(-0.2,10.0,txBuffer[i]);
	}
	hplt = 1;
}
HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
//	HAL_I2SEx_TransmitReceive_DMA(&hi2s2,(uint16_t *)txBuffer,(uint16_t *)rxBuffer,BUFSIZE);
	memcpy((void*)&txBuffer[BUFSIZE/2], (void*)&rxBuffer[BUFSIZE/2], BUFSIZE * sizeof(int16_t)/2);
	for(int i=BUFSIZE/2;i<BUFSIZE;i++){
//		if(txBuffer[i]>=100){
//			txBuffer[i]=100;
//		}
		txBuffer[i]=soft_clp(-0.2,10.0,txBuffer[i]);
	}
	cplt = 1;
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
