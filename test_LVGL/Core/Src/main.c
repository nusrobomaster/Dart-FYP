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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "hx711.h"
#include "./LCD_tft/z_displ_ILI9XXX.h"
#include "./LCD_tft/z_touch_XPT2046.h"
#include "lvgl.h"
//#include "lvgl_private.h"
#include <stdio.h>
//#include "ui.h"
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int16_t motor_rpm1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MY_DISP_VER_RES 320
#define MY_DISP_HOR_RES 480
/* Declare buffer for 1/10 screen size; BYTES_PER_PIXEL will be 2 for RGB565. */
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB888))
#define buffer_divide 10
static uint8_t buf1[DISPL_WIDTH * DISPL_HEIGHT / buffer_divide * BYTES_PER_PIXEL];
static uint8_t buf2[DISPL_WIDTH * DISPL_HEIGHT / buffer_divide * BYTES_PER_PIXEL];
volatile lv_mutex_t lv_mutex;
#define USE_UI_INIT2 0   /* Set to 0 to use ui_init1() instead */

#define USB_LEN 128
uint8_t usbTxtBuf[USB_LEN];
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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

static uint32_t ui_stack[4096];        // 4096 words = 16 KB
static StaticTask_t ui_tcb;

static const osThreadAttr_t ui_attr = {
  .name       = "ui",
  .stack_mem  = ui_stack,
  .stack_size = sizeof(ui_stack),      // BYTES
  .cb_mem     = &ui_tcb,
  .cb_size    = sizeof(ui_tcb),
  .priority   = osPriorityNormal,
};
RC_ctrl_t rc_ctrl;
HX711 hx;
int32_t weight;
int32_t value;
extern int16_t _width;       								///< (oriented) display width
extern int16_t _height;
int32_t count = 450;
char buf_txt[32];
lv_display_t * display1;
static lv_indev_t * touch_indev;
static uint32_t press_count;
//uint8_t Displ_SpiAvailable;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
void StartDefaultTask(void *argument);

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



#define SPEED rc_ctrl.rc.ch[0]

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


//	if (GPIO_Pin == GPIO_PIN_0) {
//		hx.ready = true;
//	}
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


void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    /* Flush only the area LVGL asked for; sending a bigger block corrupts the frame */
    const lv_coord_t width  = lv_area_get_width(area);
    const lv_coord_t height = lv_area_get_height(area);
    const size_t area_size_bytes = (size_t)width * height *
                                   lv_color_format_get_size(lv_display_get_color_format(display));

	Displ_SetAddressWindow(area->x1,area->y1, area->x2, area->y2);
	Displ_WriteData(px_map, area_size_bytes ,0);
    lv_display_flush_ready(display);
}

static void lvgl_touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    static lv_coord_t last_x;
    static lv_coord_t last_y;
    uint16_t touch_x = 0;
    uint16_t touch_y = 0;
    uint8_t is_touch = 0;

    Touch_GetXYtouch(&touch_x, &touch_y, &is_touch);
    if (is_touch) {
        last_x = (lv_coord_t)touch_x;
        last_y = (lv_coord_t)touch_y;
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = last_x;
        data->point.y = last_y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_x;
        data->point.y = last_y;
    }
}

static lv_obj_t *counter_label;
static lv_obj_t *timer_label;
static lv_obj_t *keypad_display;
static char keypad_value[32];

static void update_counter_label(void)
{
    static char text[24];
    lv_snprintf(text, sizeof(text), "Count: %lu", (unsigned long)press_count);
    lv_label_set_text(counter_label, text);
    lv_obj_center(counter_label);
}

static void update_timer_label(uint32_t value)
{
    static char text[24];
    lv_snprintf(text, sizeof(text), "Value: %lu", (unsigned long)value);
    lv_label_set_text(timer_label, text);
}

static void button_event_cb(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        press_count++;
        update_counter_label();
    }
}

static void keypad_update_display(void)
{
    /* Show a space when empty so the label keeps its height */
    const char * text = (keypad_value[0] != '\0') ? keypad_value : " ";
    lv_label_set_text(keypad_display, text);
}

static void keypad_btn_event_cb(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t * btnm = lv_event_get_target(e);
    const char * txt = lv_btnmatrix_get_btn_text(btnm, lv_btnmatrix_get_selected_btn(btnm));
    if (txt == NULL) {
        return;
    }

    size_t len = strlen(keypad_value);
    if (strcmp(txt, "Clr") == 0) {
        keypad_value[0] = '\0';
    } else if (strcmp(txt, "Del") == 0) {
        if (len > 0) {
            keypad_value[len - 1] = '\0';
        }
    } else if (len < sizeof(keypad_value) - 1) {
        keypad_value[len] = txt[0];
        keypad_value[len + 1] = '\0';
    }
    keypad_update_display();
}




//int32_t get_var_counter(){
//	return count;
//}

//void set_var_counter(int32_t value){
//	count = value;
//}



void ui_init1(void) {
	lv_obj_t * spinner = lv_spinner_create(lv_screen_active());
	lv_obj_set_size(spinner, 60, 60);
	lv_obj_align(spinner, LV_ALIGN_TOP_LEFT, 10, 10);
	lv_spinner_set_anim_params(spinner, 1000, 300);

	counter_label = lv_label_create(lv_scr_act());
	update_counter_label();
	lv_obj_center(counter_label);

	lv_obj_t * btn = lv_btn_create(lv_scr_act());
	lv_obj_set_size(btn, 140, 50);
	lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);
	lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t * btn_label = lv_label_create(btn);
	lv_label_set_text(btn_label, "Press me");
	lv_obj_center(btn_label);

	timer_label = lv_label_create(lv_scr_act());
	update_timer_label(0);
	lv_obj_align(timer_label, LV_ALIGN_BOTTOM_MID, 0, -60);
}

void ui_init2(void) {
    lv_obj_t * screen = lv_scr_act();

    lv_obj_t * display_panel = lv_obj_create(screen);
    lv_obj_set_size(display_panel, 280, 80);
    lv_obj_align(display_panel, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_pad_all(display_panel, 12, LV_PART_MAIN);
    lv_obj_set_style_radius(display_panel, 10, LV_PART_MAIN);

    keypad_display = lv_label_create(display_panel);
    lv_obj_set_width(keypad_display, lv_pct(100));
    lv_label_set_long_mode(keypad_display, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(keypad_display, LV_TEXT_ALIGN_CENTER, 0);
    keypad_value[0] = '\0';
    keypad_update_display();

    static const char * btn_map[] = {
        "1", "2", "3", "\n",
        "4", "5", "6", "\n",
        "7", "8", "9", "\n",
        "Clr", "0", "Del", ""
    };

    lv_obj_t * btnm = lv_btnmatrix_create(screen);
    lv_btnmatrix_set_map(btnm, btn_map);
    lv_obj_set_size(btnm, 320, 220);
    lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btnm, keypad_btn_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_style_pad_all(btnm, 6, LV_PART_MAIN);
}

uint8_t usbTxBuf[USB_LEN];
uint8_t usbRxBuf[USB_LEN];

uint16_t usbTxBufLen = 0;
uint16_t usbRxBufLen = 0;

uint8_t  usbRxFlag 	 = 0;


//void my_log_cb(lv_log_level_t level, const char *msg)
//{
//    static uint8_t line[USB_LEN];
//    size_t n = strlen(msg);
//    if (n > USB_LEN - 3) n = USB_LEN - 3;
//    memcpy(line, msg, n);
//    line[n++] = '\r';
//    line[n++] = '\n';
//    line[n] = '\0';
//    CDC_Transmit_FS(line, (uint16_t)n);
//}

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
  MX_USB_DEVICE_Init();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  setup_can();
  remote_control_init();
  hx711_init(&hx, GPIOF, GPIO_PIN_0, GPIOF, GPIO_PIN_1,HX711_GAIN_128);
  Displ_Init(Displ_Orientat_0);

  lv_init();
//  lv_log_register_print_cb(my_log_cb);
  lv_tick_set_cb(HAL_GetTick);
  display1 = lv_display_create(DISPL_WIDTH, DISPL_HEIGHT);
  /* Set display buffer for display `display1`. */
  lv_display_set_buffers(display1, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display1, my_flush_cb);
  touch_indev = lv_indev_create();
  lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_display(touch_indev, display1);
  lv_indev_set_read_cb(touch_indev, lvgl_touch_read_cb);
#if USE_UI_INIT2
  ui_init2();
#else
  ui_init1();
#endif
  uint32_t counting = 0;
  uint32_t counter = 0;
  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
//  lv_mutex_init(&lv_mutex);
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
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &ui_attr);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		lv_timer_handler_run_in_period(100);
		HAL_Delay(100);
		counting++;
		if (counting >= 50){
		  update_timer_label(counter);
		  counting = 0;
		  counter++;
		}
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
//void StartDefaultTask(void *argument)
//{
//  /* init code for USB_DEVICE */
////  MX_USB_DEVICE_Init();
//  /* USER CODE BEGIN 5 */
//
//  uint32_t counting = 0;
//
//  int counter = 42;
//  /* Infinite loop */
//  for(;;)
//  {
//
//	  lv_timer_periodic_handler();
////	  uint32_t time  =lv_timer_handler();
////	  if(time == LV_NO_TIMER_READY) time = LV_DEF_REFR_PERIOD;
//	  counting++;
//	  if (counting >= 5000){
//		  static char stable_buf[18];
//		  char *current_text;
//		  current_text = lv_label_get_text(label);
//		  lv_snprintf(stable_buf, sizeof(stable_buf), "Value");
//		  lv_mutex_lock(&lv_mutex);
//		  lv_label_set_text(label, stable_buf);
//		  lv_obj_invalidate(label);
//		  lv_mutex_unlock(&lv_mutex);
//		  usbTxBufLen = snprintf((char *) usbTxBuf, USB_LEN, "Value: %d\r\n", counter);
//		  CDC_Transmit_FS(usbTxBuf, USB_LEN);
//		  counting = 0;
//	  }
//	  osDelay(1);
//  }
  /* USER CODE END 5 */
//}

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
