/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usart_fifo.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pro_v2.h"
#include "pro.h"
#include "control.h"
#include <stdbool.h>
#include <string.h>
#include "tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 128
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t RxBuffer1[RX_BUFFER_SIZE];
uint8_t RxBuffer2[RX_BUFFER_SIZE];
uint8_t ParseBuffer[RX_BUFFER_SIZE];
uint8_t TxBuffer[1024];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
extern uint8_t UartDMAReveiveBuffer[UART_DMA_RECEIVE_LENGTH];
extern uint8_t UartSendBuffer[UART_SEND_LENGTH];
extern uint8_t uartRecvOneFrame;
extern uint8_t uartSendOneFrame;
extern USART_FIFO usartRecvFIFO;
extern USART_FIFO usartSendFIFO;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct
{
    uint32_t last_recive_pos;
    uint32_t rec_counter;
    bool is_recieve;
    PackageDataStruct pack;
    uint32_t data_len;
    uint32_t send_counter;
} CommConfig;

void Comm_SendData(uint8_t data_id, uint8_t *in_buf, uint32_t in_len)
{
    PackageDataStruct send_pack;
    uint32_t out = 0;

    send_pack.data_id = data_id;
    send_pack.data_in_buff = in_buf;
    send_pack.data_in_len = in_len;
    send_pack.data_out_buff = TxBuffer;
    send_pack.data_out_len = &out;
    send_pack.offset = 0;

    Package(send_pack);

    HAL_UART_Transmit(&huart1, send_pack.data_out_buff, out, 100);			
}

void Comm_RecProcess(void)
{
    uint16_t index;
    uint32_t len;
    index = RX_BUFFER_SIZE - hdma_usart1_rx.Instance->CNDTR;

    if (index < CommConfig.last_recive_pos)
    {
        int len1 = RX_BUFFER_SIZE - CommConfig.last_recive_pos;
        int len2 = index;

        len = len1 + len2;
        if (len > RX_BUFFER_SIZE)
        {
            CommConfig.last_recive_pos = index;
            return;
        }
        memcpy(RxBuffer2, RxBuffer1 + CommConfig.last_recive_pos, len1);
        memcpy(RxBuffer2 + len1, RxBuffer1, len2);
    }
    else
    {
        len = index - CommConfig.last_recive_pos;
        if (len > RX_BUFFER_SIZE)
        {
            CommConfig.last_recive_pos = index;
            return;
        }
        memcpy(RxBuffer2, RxBuffer1 + CommConfig.last_recive_pos, index - CommConfig.last_recive_pos);
    }

    CommConfig.pack.data_in_buff = RxBuffer2;
    CommConfig.pack.data_out_buff = ParseBuffer;
    CommConfig.pack.data_out_len = &CommConfig.data_len;
    CommConfig.pack.data_in_len = len;
    if (Unpacking(&CommConfig.pack))
    {
        CommConfig.last_recive_pos = index;
    }
}

int32_t test_pos;
uint32_t test_speed;
uint8_t test_sw;

  /* USER CODE END 0 */
  /**
  * @brief  The application entry point.
  * @retval int
  */
  int main(void)
  {
  /* USER CODE BEGIN 1 */
	uint16_t uartSendlen = 0;
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
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
 

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  FIFO_init(&usartRecvFIFO);
  FIFO_init(&usartSendFIFO);
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, RxBuffer1, RX_BUFFER_SIZE);
  memset(&CommConfig, 0, sizeof(CommConfig));
  //HAL_GPIO_WritePin(EN_GPIO_Port , EN_Pin , GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 static uint8_t temp[] = "hello";
  while (1)
  {
      if(CommConfig.pack.data_id != PACK_NULL)
      {
          switch (CommConfig.pack.data_id)
          {
              case PACK_MOTOR_RUN: //相对位移
              {
                  static int8_t last_run_dir;
                  int8_t direction;
                  int32_t pos = *(int32_t *)CommConfig.pack.data_out_buff;
                  uint32_t speed = *(uint32_t *)(CommConfig.pack.data_out_buff + 4);
                  
                  Motor_stop(); //赋值控制参数，先关闭定时器
                  
                  direction = (pos >= 0) ? 0 : 1;
                  if(direction != last_run_dir)
                  {
                      HAL_Delay(100);
                  }
                  
                  Motor_SetStep(pos, speed);
                  Motor_start();
                  Motor_Output();
                  last_run_dir = (pos >= 0) ? 0 : 1;
                  break;
              }

              case PACK_SET_LED:
                if(*(uint32_t *)CommConfig.pack.data_out_buff != 0)
                {
                    HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET);
                }
                else
                {
                    HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);
                }
                break;

              case PACK_MOTOR_STOP:
                Motor_stop();
                break;

              case PACK_POS: //与复位点的绝对距离
              {
                  static int8_t last_pos_dir;
                  int8_t direction;
                  int32_t pos = *(int32_t *)CommConfig.pack.data_out_buff;
                  uint32_t speed = *(uint32_t *)(CommConfig.pack.data_out_buff + 4);

                  pos -=  Motor_GetCurrPos();
                  Motor_stop(); //赋值控制参数，先关闭定时器
                  
                  direction = (pos >= 0) ? 0 : 1;
                  if(direction != last_pos_dir)
                  {
                      HAL_Delay(100);
                  }
                  
                  Motor_SetStep(pos, speed);
                  Motor_start();
                  Motor_Output();
                  last_pos_dir = (pos >= 0) ? 0 : 1;
                  break; 
              }

              default:
                break;
          }

          CommConfig.pack.data_id = PACK_NULL;
      }
	  if (uartRecvOneFrame)
	  {
        uartRecvOneFrame = 0;
		uartSendlen = FIFO_getDataLength(&usartRecvFIFO);
		FIFO_popData(&usartRecvFIFO, UartSendBuffer, uartSendlen);
		CDC_Transmit_FS(UartSendBuffer, uartSendlen);
	  }
//	  if (uartSendOneFrame)
//	  {
//		uartSendOneFrame = 0;
//		uartSendlen = FIFO_getDataLength(&usartSendFIFO);
//		FIFO_popData(&usartSendFIFO, UartSendBuffer, uartSendlen);
//		HAL_UART_Transmit_DMA(&huart3,UartSendBuffer,uartSendlen);
//	  }
	  HAL_UART_Transmit_DMA(&huart3,temp,sizeof(temp));
	  HAL_Delay(100);
		Comm_RecProcess();
		Motor_GetPos(); 
		Motor_LimitSwitch(); 
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */
  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn,3,3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);//¿ÕÏÐÖÐ¶Ï
	HAL_UART_Receive_DMA(&huart3,UartDMAReveiveBuffer,UART_DMA_RECEIVE_LENGTH);
  /* USER CODE END USART3_Init 2 */

}

void MX_USART_Init(UART_HandleTypeDef *usart)
{
	usart->Instance = USART3;
	usart->Init.Mode = UART_MODE_TX_RX;
	usart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	usart->Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(usart) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn,3,3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);//¿ÕÏÐÖÐ¶Ï
	HAL_UART_Receive_DMA(&huart3,UartDMAReveiveBuffer,UART_DMA_RECEIVE_LENGTH);
}
/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
/* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LIMIT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIMIT_SW_Port, &GPIO_InitStruct);

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
