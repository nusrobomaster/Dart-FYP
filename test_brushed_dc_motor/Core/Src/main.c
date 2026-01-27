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

//========================================= Power ================================================//
// LCD Module STM32 MCU
// VCC -> DC5V / 3.3V // Power
// GND -> GND // Ground
//======================================= LCD Interface ===========================================//
// The module defaults to SPI mode
// LCD Module STM32 MCU
// SDI (MOSI) -> PE6 // SPI data write
// SDO (MISO) -> PE5 // SPI data read (optional)
//======================================= LCD Interface ===========================================//
// LCD Module STM32 MCU
// LED -> PC1 // Backlight control (or tie to 5V / 3.3V)
// SCK -> PE12 // SPI clock
// LCD_RS -> PC5 // Data / Command select
// LCD_RST -> PA5 // LCD reset
// LCD_CS -> PA4 // Chip select
//===================================== Touch (Optional) ==========================================//
// If touch is not used, these pins can be left unconnected
// Touch Module STM32 MCU
// CTP_INT -> PF10 // Touch interrupt
// CTP_SDA -> PF0 // I2C data
// CTP_RST -> PI9 // Touch reset
// CTP_SCL -> PF1 // I2C clock
//======================================= HX711 Load Cell =========================================//
// HX711 STM32 MCU
// SCLK -> PC0 // HX711 clock
// DOUT -> PC4 // HX711 data output
//==================================== Electronic Lock ============================================//
// Lock Signal STM32 MCU
// Lock_Input -> PA0 // Lock status / sense input
// Lock_Output -> PA1 // Lock / unlock control output
//================================================================================================//
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "hx711.h"
#include "./LCD_CAP/LCD/lcd.h"
#include "./LCD_CAP/TOUCH/touch.h"
#include "./LCD_CAP/GUI/GUI.h"
#include "lvgl.h"
#include "bsp_damiao.h"
#include "motor_dm.h"
#include "motor_servo.h"
#include "motor_brushed.h"
#include "motor_safety.h"
//#include "lvgl_private.h"
#include <stdio.h>
#include <math.h>
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
#define MY_DISP_VER_RES LCD_H
#define MY_DISP_HOR_RES LCD_W
/* Declare buffer for 1/10 screen size; BYTES_PER_PIXEL will be 2 for RGB565. */
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define buffer_divide 10
static uint8_t buf1[LCD_W * LCD_H / buffer_divide * BYTES_PER_PIXEL];
//static uint8_t buf2[LCD_W * LCD_H / buffer_divide * BYTES_PER_PIXEL];
volatile lv_mutex_t lv_mutex;

#define USB_LEN 128
uint8_t usbTxtBuf[USB_LEN];

#define RAW_ZERO     -246337
#define RAW_KNOWN    -141436
#define WEIGHT_KNOWN 1000.0f
#define SCALE ((RAW_KNOWN - RAW_ZERO) / WEIGHT_KNOWN)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1028 * 4,
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
float weight;
int32_t value;
extern int16_t _width;       								///< (oriented) display width
extern int16_t _height;
extern dm_motor_t dm_pitch_motor;
static float dm_measured_rpm;
int32_t count = 450;
char buf_txt[32];
volatile lv_display_t * display1;
volatile uint8_t touch_flag;
//uint8_t Displ_SpiAvailable;
static lv_indev_t * touch_indev;
static lv_obj_t * label_weight;
static lv_obj_t * label_lock;
static lv_obj_t * label_rpm;
volatile float g_weight;
volatile GPIO_PinState g_unlock;
volatile int16_t g_motor_rpm;
GPIO_PinState unlock;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_DMA2D_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void lvgl_port_init(void);
static void touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data);
static void ui_update_labels(void);

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
	g_motor_rpm = motor_rpm1;
}
/* DM4310 feedback frame: StdId = master ID (0x00). Byte0 low nibble holds motor ID. */
dm_motor_manager_on_can_rx(rx_header.StdId, rx_buffer);
if (rx_header.StdId == 0x00) {
	uint8_t motor_id = rx_buffer[0] & 0x0F;
	if (motor_id == (dm_pitch_motor.id & 0x0F)) {
		dm_measured_rpm = dm_pitch_motor.para.vel * (60.0f / (2.0f * (float)M_PI));
		g_motor_rpm = (int16_t)dm_measured_rpm;
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



#define SPEED rc_ctrl.rc.ch[0]

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) {
        touch_flag = 1;
    }
}

void tft_set_addr_window(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
    uint8_t caset[4] = { x0>>8, x0&0xFF, x1>>8, x1&0xFF };
    uint8_t raset[4] = { y0>>8, y0&0xFF, y1>>8, y1&0xFF };

    LCD_WR_REG(0x2A);
    LCD_WR_DATA(caset[0]);
    LCD_WR_DATA(caset[1]);
    LCD_WR_DATA(caset[2]);
    LCD_WR_DATA(caset[3]);

    LCD_WR_REG(0x2B);
    LCD_WR_DATA(raset[0]);
    LCD_WR_DATA(raset[1]);
    LCD_WR_DATA(raset[2]);
    LCD_WR_DATA(raset[3]);

    LCD_WR_REG(0x2C); // RAMWR
}

void tft_put_pixel(uint16_t x,uint16_t y,uint16_t color)
{
    tft_set_addr_window(x,y,x,y);
    Lcd_WriteData_16Bit(color);
}


void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
    uint32_t size = w * h * BYTES_PER_PIXEL;

    LCD_SetWindows(area->x1, area->y1, area->x2, area->y2);
    LCD_CS_CLR;
    LCD_RS_SET;
    while (size > 0U) {
      uint16_t chunk = (size > 0xFFFFU) ? 0xFFFFU : (uint16_t)size;
      HAL_SPI_Transmit(&hspi4, px_map, chunk, HAL_MAX_DELAY);
      px_map += chunk;
      size -= chunk;
    }
    LCD_CS_SET;
    lv_display_flush_ready(display);
}




//int32_t get_var_counter(){
//	return count;
//}

//void set_var_counter(int32_t value){
//	count = value;
//}


static void lvgl_port_init(void)
{
  lv_init();
  lv_tick_set_cb(HAL_GetTick);

  display1 = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
  lv_display_set_color_format((lv_display_t *)display1, LV_COLOR_FORMAT_RGB565_SWAPPED);
  lv_display_set_buffers(display1, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display1, my_flush_cb);

  touch_indev = lv_indev_create();
  lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch_indev, touch_read_cb);
  lv_indev_set_display(touch_indev, (lv_display_t *)display1);
}

static void touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
  (void)indev;
  if (tp_dev.scan) {
    tp_dev.scan();
  }

  if (tp_dev.sta & TP_PRES_DOWN) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = tp_dev.x[0];
    data->point.y = tp_dev.y[0];
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void ui_init1(void) {
  lv_obj_t * screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
  lv_obj_set_style_text_color(screen, lv_color_hex(0xE0E6ED), 0);

  lv_obj_t * root = lv_obj_create(screen);
  lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_pad_all(root, 16, 0);
  lv_obj_set_style_pad_gap(root, 12, 0);
  lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  lv_obj_t * title = lv_label_create(root);
  lv_label_set_text(title, "Prototype3 Status");

  label_weight = lv_label_create(root);
  lv_label_set_text(label_weight, "Weight: --.- g");

  label_lock = lv_label_create(root);
  lv_label_set_text(label_lock, "Lock status: --");

  label_rpm = lv_label_create(root);
  lv_label_set_text(label_rpm, "Motor RPM: --");
}

static void ui_update_labels(void)
{
  static int32_t last_weight_centi = 0x7fffffff;
  static int16_t last_rpm = 0x7fff;
  static GPIO_PinState last_lock = GPIO_PIN_SET;
  char line[32];

  int32_t weight_centi = (int32_t)(g_weight * 100.0f);
  if (label_weight && weight_centi != last_weight_centi) {
    snprintf(line, sizeof(line), "Weight: %.2f g", g_weight);
    lv_label_set_text(label_weight, line);
    last_weight_centi = weight_centi;
  }

  if (label_lock && g_unlock != last_lock) {
    snprintf(line, sizeof(line), "Lock status: %s", (g_unlock ? "unlock" : "lock"));
    lv_label_set_text(label_lock, line);
    last_lock = g_unlock;
  }

  if (label_rpm && g_motor_rpm != last_rpm) {
    snprintf(line, sizeof(line), "Motor RPM: %d", g_motor_rpm);
    lv_label_set_text(label_rpm, line);
    last_rpm = g_motor_rpm;
  }
}

uint8_t usbTxBuf[USB_LEN];
uint8_t usbRxBuf[USB_LEN];

uint16_t usbTxBufLen = 0;
uint16_t usbRxBufLen = 0;

uint8_t  usbRxFlag 	 = 0;

uint32_t starting;

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
  MX_DMA2D_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();

//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  setup_can();
  remote_control_init();
  hx711_init(&hx, GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_0, HX711_GAIN_128);
  hx711_set_scale(&hx, SCALE);
  hx711_set_offset(&hx, RAW_ZERO);
  LCD_Init();
  TP_Init();

  lvgl_port_init();
  ui_init1();
//  uint32_t counting = 0;
//  uint32_t counter = 0;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3359;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI7 PI9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5;
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

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  int start_position = 0;
  int current_position = 3000;
  int direction = 1;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, start_position);
  HAL_TIM_Encoder_Start(&htim2,  TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  /* Infinite loop */
  for(;;)
  {
	  switch (direction) {
	  	  case 0:
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  		  break;
	  	  case 1:
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  		break;
	  	  case 2:
		  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		  	break;
	  	  default:
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  		break;
	  }
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, current_position);
	starting = __HAL_TIM_GET_COUNTER(&htim2);
	int dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

    osDelay(100);
  }
  /* USER CODE END 5 */
}

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
