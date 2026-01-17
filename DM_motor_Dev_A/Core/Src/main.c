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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "hx711.h"
#include "./LCD_tft/z_displ_ILI9XXX.h"
#include "./LCD_tft/z_touch_XPT2046.h"
#include "bsp_damiao.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int16_t motor_rpm1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId loadcellTaskHandle;
/* USER CODE BEGIN PV */
RC_ctrl_t rc_ctrl;
HX711 hx;
int32_t weight;
int32_t value;
extern int16_t _width;       								///< (oriented) display width
extern int16_t _height;
extern dm_motor_t dm_pitch_motor;
static float target_rpm = 0.0f;          /* Desired speed setpoint in RPM; update this variable to change target */
static float commanded_rpm = 0.0f;       /* Internally ramped RPM command */
static float measured_rpm = 0.0f;
static float pid_int = 0.0f;
static float pid_prev_err = 0.0f;


static float target_position = 0.0f;
static float current_position = 0.0f;
static int round_counter = 0;
static float tolerance = 0.06f;
static float prev_pos = 0.0f;

//faster pid values
//static float pos_kp = 7.0f;
//static float pos_kd = 0.4f;

//not as fast pid values
static float pos_kp = 0.5f;
static float pos_kd = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

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
/* DM4310 feedback frame: StdId = master ID (0x00). Byte0 low nibble holds motor ID. */
if (rx_header.StdId == 0x01) {
	uint8_t motor_id = rx_buffer[0] & 0x0F;
	if (motor_id == (dm_pitch_motor.id & 0x0F)) {
		dm4310_fbdata(&dm_pitch_motor, rx_buffer);
		measured_rpm = dm_pitch_motor.para.vel * (60.0f / (2.0f * (float)M_PI));
		if ((dm_pitch_motor.para.pos < - 12.5 + tolerance) && (prev_pos > 12.5 - tolerance) ){
			round_counter += 1;
		}
		if ((dm_pitch_motor.para.pos > 12.5 - tolerance) && (prev_pos < - 12.5 + tolerance)) {
			round_counter -= 1;
		}
		current_position = round_counter*8*PI + dm_pitch_motor.para.pos;
		prev_pos = dm_pitch_motor.para.pos;
	}
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


static float pid_speed_step(float setpoint_rpm, float measured_rpm_val, float dt_s)
{
	const float kp = 0.8f;
	const float ki = 0.6f;
	const float kd = 0.0f;
	const float i_limit = 500.0f;    /* Integral windup guard */
	const float out_limit = 2000.0f; /* Command clamp in RPM */

	float err = setpoint_rpm - measured_rpm_val;
	pid_int += err * ki * dt_s;
	if (pid_int > i_limit) pid_int = i_limit;
	else if (pid_int < -i_limit) pid_int = -i_limit;

	float der = (err - pid_prev_err) / dt_s;
	pid_prev_err = err;

	float out = kp * err + pid_int + kd * der;
	if (out > out_limit) out = out_limit;
	else if (out < -out_limit) out = -out_limit;
	return out;
}


#define SPEED rc_ctrl.rc.ch[0]

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		hx.ready = true;
	}
	if (GPIO_Pin==TOUCH_INT_Pin){
			Touch_HandlePenDownInterrupt();
		}
}

void tft_set_addr_window(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
    uint8_t caset[4] = { x0>>8, x0&0xFF, x1>>8, x1&0xFF };
    uint8_t raset[4] = { y0>>8, y0&0xFF, y1>>8, y1&0xFF };

    Displ_WriteCommand(0x2A);
    Displ_WriteData(caset,4,0);

    Displ_WriteCommand(0x2B);
    Displ_WriteData(raset,4,0);

    Displ_WriteCommand(0x2C); // RAMWR
}

void tft_put_pixel(uint16_t x,uint16_t y,uint16_t color)
{
    tft_set_addr_window(x,y,x,y);
    uint8_t px[2] = {color>>8, color&0xFF};
    Displ_WriteData(px, 2, 1);
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
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  setup_can();

//  remote_control_init();
//  hx711_init(&hx, GPIOF, GPIO_PIN_0, GPIOF, GPIO_PIN_1,HX711_GAIN_128);
//  Displ_Init(Displ_Orientat_0);
//  InitMenu();

  // Example: red, green, blue pixels in top row
//  tft_put_pixel(0,0,0xF800);
//  tft_put_pixel(1,0,0x07E0);
//  tft_put_pixel(2,0,0x001F);
//  Displ_WriteCommand(0x2A);
//  uint8_t colData[4] = {0x00,0x00, 0x01,0x3F};
//  Displ_WriteData(colData,4,0);
//
//  // Page range 0–479
//  Displ_WriteCommand(0x2B);
//  uint8_t pageData[4] = {0x00,0x00, 0x01,0xDF};
//  Displ_WriteData(pageData,4,0);
//
//  // Memory write
//  Displ_WriteCommand(0x2C);


//  Displ_FillArea(0,0, _width, _height, RED);
//  Displ_WriteCommand(0x2A);   // Column Address Set
//  uint8_t colData[4] = {0x00, 0x00, 0x01, 0x3F}; // 0..319
//  Displ_WriteData(colData, 4, 0);
//
//
//  Displ_WriteCommand(0x2B);   // Page Address Set
//  uint8_t pageData[4] = {0x00, 0x00, 0x01, 0xDF}; // 0..479
//  Displ_WriteData(pageData, 4, 0);
//
//  Displ_WriteCommand(0x2C);
//
//  HAL_GPIO_WritePin(DISPL_DC_GPIO_Port, DISPL_DC_Pin, GPIO_PIN_SET);
//  for (int i = 0; i < 320 * 480; i++) {
//      uint8_t black[2] = {0xFF, 0xFF};
//      Displ_WriteData(black, 2, 0);
//  }


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of loadcellTask */
//  osThreadDef(loadcellTask, StartTask02, osPriorityIdle, 0, 128);
//  loadcellTaskHandle = osThreadCreate(osThread(loadcellTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
//	int16_t state = 0;
//	int16_t unlock = 0;
  vTaskDelay(1000);
  dm4310_motor_init();
  /* Configure for MIT mode speed control with a gentle slew to the target RPM */
//  dm_pitch_motor.ctrl.mode   = 0;     /* MIT mode */
//  dm_pitch_motor.ctrl.pos_set = 0.0f;
//  dm_pitch_motor.ctrl.kp_set  = 0.0f;
//  dm_pitch_motor.ctrl.kd_set  = 0.0f;
//  dm_pitch_motor.ctrl.tor_set = 0.0f;
  target_rpm = 100.0f;                /* Default target RPM; adjust as needed at runtime */
  float positive_rpm = 100.0f;
  const TickType_t loop_period_ms = 10;    /* Control loop period */
  /* Infinite loop */
  for(;;)
  {
	  volatile float final_pos;
	  volatile int count_spin = (int) (target_position + 4 * PI) / (8 * PI);
	  if (target_position < 0){
		  count_spin -= 1;
		  final_pos = fmodf(target_position - 4 * PI, 8*PI) + 4*PI;
	  } else {
		  final_pos = fmodf(target_position + 4 * PI, 8*PI) - 4*PI;
	  }
	  /* PID speed control using CAN feedback to avoid aggressive spin-up */
//	  const float dt = loop_period_ms / 1000.0f;
//	  commanded_rpm = pid_speed_step(target_rpm, measured_rpm, dt);

	  /* Convert RPM to rad/s for MIT velocity field */
	  if (count_spin == round_counter){
		  dm_pitch_motor.ctrl.vel_set = 0;
		  dm_pitch_motor.ctrl.pos_set = final_pos;
		  dm_pitch_motor.ctrl.kp_set = pos_kp;
		  dm_pitch_motor.ctrl.kd_set = pos_kd;
	  } else {
		  target_rpm = positive_rpm;
		  if (count_spin < round_counter){
			  target_rpm = - target_rpm;
		  }
		  dm_pitch_motor.ctrl.pos_set = 0;
		  const float cmd_rad_per_s = target_rpm * (2.0f * (float)M_PI / 60.0f);
		  dm_pitch_motor.ctrl.vel_set = cmd_rad_per_s;
		  dm_pitch_motor.ctrl.kp_set = 0;
		  dm_pitch_motor.ctrl.kd_set = 1.5;
	  }
//	  dm_pitch_motor.ctrl.pos_set = 0;
//	  const float cmd_rad_per_s = target_rpm * (2.0f * (float)M_PI / 60.0f);
//	  dm_pitch_motor.ctrl.vel_set = cmd_rad_per_s;
//	  dm_pitch_motor.ctrl.kp_set = 0;
//	  dm_pitch_motor.ctrl.kd_set = 1.5;
	  dm4310_ctrl_send(&hcan1, &dm_pitch_motor);
	  vTaskDelay(loop_period_ms);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the loadcellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
//	if (hx.ready){
//		hx.ready = false;
//		weight = hx711_read_raw(&hx);
//	}
//	  value = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
//	  if (value == 1){
//		  continue;
//	  }
//	  value = 0;
//	  for (uint8_t i = 0; i < 24; ++i) {
//		HAL_GPIO_WritePin(hx.sck_port, hx.sck_pin, GPIO_PIN_SET);
//		hx711_delay_us(hx.sck_delay_us);
//		value = (value << 1) | (HAL_GPIO_ReadPin(hx.dout_port, hx.dout_pin) == GPIO_PIN_SET ? 1 : 0);
//		HAL_GPIO_WritePin(hx.sck_port, hx.sck_pin, GPIO_PIN_RESET);
//		if (value == 1) {
//			__NOP();
//			hx711_delay_us(hx.sck_delay_us);
//		} else {
//			hx711_delay_us(hx.sck_delay_us);
//		}
//		__NOP();
//	  }
//	  if (value & 0x800000) { value |= 0xFF000000; }
	  weight = hx711_read_raw(&hx);
	  osDelay(1000);
  }
  /* USER CODE END StartTask02 */
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
