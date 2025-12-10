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
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "board_lib.h"
//#include "startup_task.h"
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
int16_t motor_rpm1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setup_can(){
 CAN_FilterTypeDef can_filter_st = {0};
 can_filter_st.FilterActivation = ENABLE;
 can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
 can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
 can_filter_st.FilterIdHigh = 0;
 can_filter_st.FilterIdLow = 0;
 can_filter_st.FilterMaskIdHigh = 0;
 can_filter_st.FilterMaskIdLow = 0;
 can_filter_st.FilterBank=0;
 can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
 HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
 HAL_CAN_Start(&hcan1);
 HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
CAN_RxHeaderTypeDef rx_header;
uint8_t rx_buffer[8];
HAL_CAN_DeactivateNotification(hcan,
  CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer);

if (rx_header.StdId == 0x203)
{
	motor_rpm1 = (rx_buffer[2] << 8) + rx_buffer[3];
}

HAL_CAN_ActivateNotification(hcan,
  CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
}

void ctrl_motor(int16_t torque1){
 uint8_t tx_msg[8];
 CAN_TxHeaderTypeDef CAN_tx_message;
 uint32_t send_mail_box;
 CAN_tx_message.IDE = CAN_ID_STD;
 CAN_tx_message.RTR = CAN_RTR_DATA;
 CAN_tx_message.DLC = 0x08;
 CAN_tx_message.StdId = 0x200;
 tx_msg[4] = torque1>>8;
 tx_msg[5] = torque1;
 HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, tx_msg, &send_mail_box);
}


#define KP 10
#define KI 0.1
#define KD 0
#define INT_MAX 5000
int16_t pid_lol(int16_t setpt, int16_t curr_pt){
	static float integral;
	static float prev_error;
	float error = setpt - curr_pt;
	integral += error * KI;
	integral = (integral > INT_MAX) ? INT_MAX : (integral < -INT_MAX) ? -INT_MAX : integral;
	float diff = (prev_error - error) * KD;
	prev_error = error;
	return (error * KP) + integral + diff;

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM8_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  setup_can();
//  startup_task();
////  MX_USB_DEVICE_Init();
//  HAL_TIM_Base_Start(&htim3); //start micros clock
//  NVIC_SetPriority(SysTick_IRQn, 15);
//  HAL_RNG_GenerateRandomNumber(hrng, random32bit)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t motor = 100;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 ctrl_motor(pid_lol(motor, motor_rpm1));
	 HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
