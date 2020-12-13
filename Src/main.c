/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid_control_test.c"
#include "pid_control_test.h"
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
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

//spi
#define SpiEn HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //SPI1 NSS
#define SpiDis HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

//// Debug ////
int timer_test = 0;
int cnt_1ms = 0;
uint8_t st = 0;
int db = 0;
int degree_0 = 0;
int degree_30 = 1000;
int degree_60 = 2000;
int degree_90 = 3800;

int l_Target_ang = 0;
int r_Target_ang = 0;

int test_buff[] = {5,2,6,3,4,7,8,1,9};
//// FLAG ////
uint8_t a = 0;
uint8_t start_SW = 0;
uint8_t start_init = 0;
uint8_t LED_R = 0;
uint8_t LED_G = 0;
uint8_t LED_B = 0;
int time_10ms = 0;
int test = 0;
int test_angle = 0;
int start_control = 0;

uint8_t is_auto = 0;
uint8_t stop = 0;
uint8_t button = 0;
uint8_t pre_button_data = 0;
//// FLAG ////

//spi
int spi_opperate = 0;
int b = 0;
int error = 0;

uint8_t master_buffer_tx[4] = {0, };
uint8_t master_buffer_rx[4] = {0, };
//spi

uint8_t rx_buff[1] = {0, };
uint8_t tx_buff[27] = {255,170,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t check_sum = 0;
int tx_count = 0;
uint32_t adc_buff[4] = {0, };
int buff_size = 0;
int check_buff[28] = {0, };
int real_buff[28] = {100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100, 100, 100, 0 , 0, 0, 0};
const int Default[28] = {100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100, 100, 100, 0, 0, 0, 0}; //통신 끊겼을 시 기본값
int rx_state;
int count = 0;
enum {start1 = 0, start2, get_data};
int Encoder_Cnt[4] = {0,  };
int PWM[4] = {5000, 5000, 5000, 5000};
int booster = 0;
double j_booster = 0;

//// PID ////
double Target = 0;
int l_now_RPM =0;
int r_now_RPM =0;
int RPM = 0;

unsigned int l_EncoderData = 0;
unsigned int r_EncoderData = 0;

int l_EncoderCnt = 0;
int r_EncoderCnt = 0;

int l_buff = 0;
int r_buff = 0;

double l_EncoderSum = 0.0;
double r_EncoderSum = 0.0;

int l_Angle = 0;
int r_Angle = 0;

double l_Input[3] = {0.0, };
double r_Input[3] = {0.0, };

double l_Target = 0.0;
double r_Target = 0.0;

double l_TargetAngle = 0.0; //2000 ~ 5000 ~ 8000
double r_TargetAngle = 0.0;

LPID l_BLDC[3];         //left
LPID r_BLDC[3];         //right
//// PID ////

uint32_t analog_current = 0;
double current = 0;

/////////////////////////////absolute Encoder
int NowAngle[2]    = {0, };
int diff[2] = {0,};
int calcValue[2] = {0,};
int encStep = 0;
int encCnt = 0;
int encDelayCnt = 0;
int encResetCnt[2] = {0, };
uint8_t encReadData[2] = {0, };
int encDataTemp[2] = {0, };
int encNowData[2]  = {0, };
int raw_flipper_deg[2] = {0, };
int flipper_deg[2] = {0, };     // 신뢰 각도
enum{left = 0, right};
const int ENC_HEX[12] =
{
  0x0800, 0x0400, 0x0200, 0x0100, 
  0x0080, 0x0040, 0x0020, 0x0010, 
  0x0008, 0x0004, 0x0002, 0x0001 
};

int INC_enc_data[4] = {0,};
int b_flipper_deg[2][9] = {0, };
int ing_flipper_deg[2][9] = {0, };
uint8_t showtime = 0;
/////////////////////////////absolute Encoder

/////////////////robot control
int control_flipper_deg[2] = {0, };
/////////////////robot control

//통신 끊겼을 시 정지
int escapeCnt = 0;
int enc_cnt = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// void StartDefaultTaskNew(void const * argument);   

void AnalogRead();
void rpm2Degree(double rpm);
void Enc_Clk_ClrVal();
void Enc_Clk_SetVal();
void medianFilterEnc();
void Buzz_Lightyear();

//spi
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void potentiometer_processing();
//spi

void escape();
void init_position();
void control_bangkook();

void test_Target_ang();

void driving_mode();

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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim10);
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);
  
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
  
  
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_TC);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart3,(uint8_t *)(rx_buff), 1);
  HAL_UART_Transmit_IT(&huart3,(uint8_t *)(tx_buff), 26);
  rx_state = start1;
  
  //PID
  PID_Control_Long_Initialize(&l_BLDC[pos], pos);
  PID_Control_Long_Initialize(&l_BLDC[vel], vel);
  PID_Control_Long_Initialize(&r_BLDC[pos], pos);
  PID_Control_Long_Initialize(&r_BLDC[vel], vel);
  
  //spi
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_SPI_Init(&hspi2);
  
  
  
  /*    send the data to SPI   */
  if(HAL_SPI_TransmitReceive_DMA(&hspi2, master_buffer_tx, master_buffer_rx, 4)!=HAL_OK){
    Error_Handler();
  }
  
  /*    chenck the SPI state    */
  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY); //spi 상태 반환
  //spi
  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 179;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  
  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  
  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  
  /* USER CODE END TIM4_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */
  
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
  
  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */
  
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 89;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
  
  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
  
  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */
  
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 17;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
  
  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
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
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  
  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_B_Pin|LED_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENC1_CLK_Pin|ENC1_ZERO_Pin|ENC2_CLK_Pin|ENC2_ZERO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Stop_SW_Pin */
  GPIO_InitStruct.Pin = Stop_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Stop_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_DATA_Pin ENC2_DATA_Pin */
  GPIO_InitStruct.Pin = ENC1_DATA_Pin|ENC2_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_CLK_Pin ENC1_ZERO_Pin ENC2_CLK_Pin ENC2_ZERO_Pin */
  GPIO_InitStruct.Pin = ENC1_CLK_Pin|ENC1_ZERO_Pin|ENC2_CLK_Pin|ENC2_ZERO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart3,(uint8_t *)(rx_buff), 1);
  
  switch(rx_state)
  {
  case start1:
    if(rx_buff[0] == 255){
      rx_state = start2;
      check_sum += rx_buff[0];
    } 
    break;
  case start2:
    if(rx_buff[0] == 170){
      rx_state = get_data;
      check_sum += rx_buff[0];
    }
    break;
  case get_data:
    if(buff_size > 27){  
      if(check_sum == rx_buff[0]){
        for(int i = 0 ; i < buff_size ; i++)
        {
          real_buff[i] = check_buff[i];
          check_buff[i] = 0;
        }
      }
      buff_size = 0;
      check_sum = 0;
      rx_state = start1;
    }
    else
    {
      check_buff[buff_size] = rx_buff[0];
      check_sum += rx_buff[0];
      buff_size++;
    }
    break;
  }
  escapeCnt = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  tx_buff[0] = 255;
  tx_buff[1] = 170;
  tx_buff[2] = adc_buff[0] / 1000;
  tx_buff[3] = (adc_buff[0] % 1000) / 100;
  tx_buff[4] = (adc_buff[0] % 100) / 10;
  tx_buff[5] = (adc_buff[0] % 10);
  HAL_UART_Transmit_IT(&huart3,(uint8_t *)(tx_buff), 26);
}

void escape()
{
  escapeCnt++;
  if(escapeCnt > 10000)        //통신 끊겼을 시 Default
  {
    LED_G = 1;
    rx_buff[0] = 0x00;
    if(rx_buff[0] == 0x00)
    {
      for(int i = 0; i < 28; i++)
      {
        real_buff[i] = Default[i];
      }
    }
  }
  else LED_G = 0;
}

void init_position()            //fillper init
{
//  start_SW = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);
  if(real_buff[25] == 1) //스위치 눌렀을 때 || ui로 init버튼 눌렀을 때 
  {
    start_init = 1;
  }
  
  if(((flipper_deg[left] >= 0 && flipper_deg[left] < 5) || (flipper_deg[left] > 355 && flipper_deg[left] < 360))
     &&((flipper_deg[right] >= 0 && flipper_deg[right] < 5) || (flipper_deg[right] > 355 && flipper_deg[right] < 360)))
  {
    l_EncoderSum = 0;
    r_EncoderSum = 0;
    start_control = 1;
  }
  
  if(start_init == 1 && st == 0) // start button
  {    
    //left init
    if(flipper_deg[left] >= 180) // -방향
    {
      l_TargetAngle = -(360 - flipper_deg[left]) * 50;
    }
    else if(flipper_deg[left] < 180)  // +방향
    {
      l_TargetAngle = flipper_deg[left]*50;
    }
    
    //right init
    if(flipper_deg[right] >= 180) // -방향
    {
      r_TargetAngle = (360 - flipper_deg[right]) * 50;
    }
    else if(flipper_deg[right] < 180)  // +방향
    {
      r_TargetAngle = -flipper_deg[right]*50;
    }

    st = 1;
  }
}

void driving_mode()
{
  
  //// stop button
  button = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
  if(pre_button_data == 0 && button == 1)
  {
    if(stop == 0)
      stop = 1;
    else
      stop = 0;
  }
  pre_button_data = button;
  //// stop button
  
  
  
  if(real_buff[15] == 0) // 구동부일 때( 팔 조이스틱 아닌경우)
  {
    LED_R = 0;
    if(real_buff[24] == 1)        //자동
    {
      if(stop == 1)
      {
        PWM[0] = 5000;
        PWM[1] = 5000;
        PWM[2] = 5000;
        PWM[3] = 5000;
        LED_B = 0;
      }
      else
      {
        //        PWM[0] = real_buff[22]*30+2000;
        //        PWM[1] = real_buff[23]*30+2000;
        PWM[0] = real_buff[22]*15+3500;
        PWM[1] = real_buff[23]*15+3500;
        LED_B = 1;
      }
    }
    else  // 수동
    {
      LED_B = 0;
      booster = real_buff[27] * 750;   // 속도 조절 0,1,2단계
      j_booster = -(15 + real_buff[27] * 7.5); 
        
      if(real_buff[26] == 0)
      {
        if(real_buff[18] == 200 || real_buff[19] == 200 || real_buff[20] == 200 || real_buff[21] == 200) // Push SW mode
        {
          if(real_buff[18] == 200) // 전진
          {
            PWM[0] = 6500 + booster;      
            PWM[1] = 6500 + booster;
          }
          else if(real_buff[19] == 200) // 후진
          {
            PWM[0] = 3500 - booster;
            PWM[1] = 3500 - booster;
          }
          else if(real_buff[20] == 200) //좌회전
          {
            PWM[0] = 3500 - booster;
            PWM[1] = 6500 + booster;
          }
          else if(real_buff[21] == 200) // 우회전
          {
            PWM[0] = 6500 + booster;
            PWM[1] = 3500 - booster;
          }
          else
          {
            PWM[0] = 5000;
            PWM[1] = 5000;
          }
        }
        else        //Joy stick mode
        {
          //joystick 200:후진, 100:정지 0 전진
          PWM[0] =  (int)(real_buff[1] * j_booster) + 6500 + booster;       // 왼쪽은 VESC 빨간색 파란색 반대로 연결하고 진행할 것
          PWM[1] =  (int)(real_buff[3] * j_booster) + 6500 + booster;
        }
      }
      else // 조종 반전
      {
        if(real_buff[18] == 200 || real_buff[19] == 200 || real_buff[20] == 200 || real_buff[21] == 200)
        {
          if(real_buff[18] == 200) // 전진
          {
            PWM[0] = 3500 - booster;
            PWM[1] = 3500 - booster;
          }
          else if(real_buff[19] == 200) // 후진
          {
            PWM[0] = 6500 + booster;
            PWM[1] = 6500 + booster;
          }
          else if(real_buff[20] == 200) //좌회전
          {
            PWM[0] = 3500 - booster;
            PWM[1] = 6500 + booster;
          }
          else if(real_buff[21] == 200) // 우회전
          {
            PWM[0] = 6500 + booster;
            PWM[1] = 3500 - booster;
          }
          else
          {
            PWM[0] = 5000;
            PWM[1] = 5000;
          }
        }
        else
        {
          //joystick 200:후진, 100:정지 0 전진
          PWM[0] =  (int)(real_buff[1] * -j_booster) + 3500 - booster;       // 왼쪽은 VESC 빨간색 파란색 반대로 연결하고 진행할 것
          PWM[1] =  (int)(real_buff[3] * -j_booster) + 3500 - booster;
          
        }
      }
    }
  }
  else
  {
    LED_R = 1;
    PWM[0] = 5000;
    PWM[1] = 5000;
  }
}

void control_bangkook()
{
  
  driving_mode(); //주석 풀 것
  //구동부
  
  //플리퍼
  l_TargetAngle = real_buff[4] * 50;
  r_TargetAngle = -real_buff[5] * 50;  //하모닉 감속비 곱하는 거임
  //플리퍼
  
  //  control_flipper_deg[left] = real_buff[9];        //left past
  //  control_flipper_deg[right] = real_buff[8];        //right past
  
}

void Buzz_Lightyear()
{
  
  if(LED_B == 0)
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
  
  if(LED_G == 0)
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
  
  if(LED_R == 0)
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
  else
  {
    a++;
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
  }
  
}

void AnalogRead()
{
  current = (int)((analog_current * (5.0 / 4096.0)  - 2.4654) * 0.022); 
}

void rpm2Degree(double rpm)
{
  Target = (rpm / 60.0 * 360.0); // Target -> degree / s
}

void test_Target_ang(int l_Target_ang, int r_Target_ang)
{
  l_TargetAngle = l_Target_ang;
  r_TargetAngle = r_Target_ang;
}

//absolute encoder
void Enc_Clk_ClrVal()
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
}
void Enc_Clk_SetVal()
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
}
void medianFilterEnc()
{
  b_flipper_deg[left][enc_cnt] = raw_flipper_deg[left];
  b_flipper_deg[right][enc_cnt++] = raw_flipper_deg[right];
  
  if(enc_cnt > 8)
    enc_cnt = 0;
  
  for(int i = 0; i < 9; i++)
  {
    ing_flipper_deg[left][i] = b_flipper_deg[left][i];
    ing_flipper_deg[right][i] = b_flipper_deg[right][i];
  }
  for(int j = 0; j < 9; j++)
  {
    for(int k = j+1; k < 9; k++)
    {
      if(ing_flipper_deg[left][j] < ing_flipper_deg[left][k])
      {
        int temp = ing_flipper_deg[left][j];
        ing_flipper_deg[left][j] = ing_flipper_deg[left][k];
        ing_flipper_deg[left][k] = temp;
      }
    }
  }
  for(int j = 0; j < 9; j++)
  {
    for(int k = j+1; k < 9; k++)
    {
      if(ing_flipper_deg[right][j] < ing_flipper_deg[right][k])
      {
        int temp = ing_flipper_deg[right][j];
        ing_flipper_deg[right][j] = ing_flipper_deg[right][k];
        ing_flipper_deg[right][k] = temp;
      }
    }
  }
  
  //result
  if(showtime > 4)
  {
    flipper_deg[left]= ing_flipper_deg[left][4];
    flipper_deg[right]= ing_flipper_deg[right][4];
  }
  showtime++;
}


// test spi -> 가변저항 센서값
void potentiometer_processing()
{
  master_buffer_tx[3] = (uint8_t)(adc_buff[0] / 1000);
  master_buffer_tx[2] = (uint8_t)((adc_buff[0] % 1000) / 100);
  master_buffer_tx[1] = (uint8_t)((adc_buff[0] % 100) / 10);
  master_buffer_tx[0] = (uint8_t)(adc_buff[0] % 10);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)            //0.1ms
  { 

    switch(encStep)
    {
    case 0:
      encCnt = 0;
      encDataTemp[0] = 0;
      encDataTemp[1] = 0;
      Enc_Clk_ClrVal();
      encStep = 1;
      break;
      
    case 1:
      Enc_Clk_SetVal();
      encStep = 2;
      break;
      
    case 2:
      encReadData[0] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
      encReadData[1] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
      Enc_Clk_ClrVal();
      encStep = 3;
      break;
      
    case 3:
      if(encReadData[0] != 0)
        encDataTemp[0] += ENC_HEX[encCnt];
      
      if(encReadData[1] != 0)
        encDataTemp[1] += ENC_HEX[encCnt];       
      Enc_Clk_SetVal();
      encCnt ++;
      
      if(encCnt < 12)
        encStep = 2;
      else
        encStep = 4;
      break;
      
    case 4:
      encNowData[0] = encDataTemp[0];
      encNowData[1] = encDataTemp[1];    
      raw_flipper_deg[left] = (int)(encNowData[left] / 11.37777);
      raw_flipper_deg[right] = -(int)(encNowData[right] / 11.37777 - 360);
      if(encDelayCnt++ > 120)
      {
        encDelayCnt = 0;
        encStep = 0;
      }
      break;
      
      // encoder 1 reset
    case 100:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
      if(encResetCnt[0]++ > 10000)
      {
        encResetCnt[0] = 0;
        encStep = 101;
      }
      break;
      
    case 101:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
      if(encResetCnt[0]++ > 15000)
      {
        encResetCnt[0] = 0;
        encStep = 102;
      }
      break;
      
    case 102:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
      if(encResetCnt[0]++ > 100)
      {
        encResetCnt[0] = 0;
        encStep = 0;
      }
      break;
      
      // encoder 2 reset
    case 200:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
      if(encResetCnt[1]++ > 10000)
      {
        encResetCnt[1] = 0;
        encStep = 201;
      }
      break;
      
    case 201:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
      if(encResetCnt[1]++ > 15000)
      {
        encResetCnt[1] = 0;
        encStep = 202;
      }
      break;
      
    case 202:
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
      if(encResetCnt[1]++ > 100)
      {
        encResetCnt[1] = 0;
        encStep = 0;
      }
      break;
    }
  }
  if(htim->Instance == TIM10) // 1ms
  {
    cnt_1ms++;
    escape();                   // 통신 끊겼을 시
    medianFilterEnc();
    Buzz_Lightyear();
    
    if(start_control != 1 && cnt_1ms > 10)              // 1 부터 시작  &&  바로 시작하면 start_control이 1이 되기때문에 시간 좀 줬다가
    {
      init_position();
    }
  }
  
  if(htim->Instance == TIM11) 
  {
    potentiometer_processing();
    SpiEn;
    HAL_SPI_TransmitReceive_DMA(&hspi2, master_buffer_tx, master_buffer_rx, 4);
    b++;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi -> Instance == hspi2.Instance)
  {
    spi_opperate++;
    SpiDis;
  }
}

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
  /* Infinite loop */
  for(;;)
  {
    timer_test++;
    //    Encoder_Cnt[0] = TIM5 ->CNT;
    //    Encoder_Cnt[1] = TIM4 ->CNT;
    //    Encoder_Cnt[2] = TIM3 ->CNT;
    //    Encoder_Cnt[3] = TIM2 ->CNT;

    
    driving_mode(); // 구동부 테스트 용 (주석할 것) 
    
    if(start_control != 0)
    {
      control_bangkook();
    }
    
    TIM1 -> CCR1 = PWM[0];                      //Left wheel
    TIM1 -> CCR2 = PWM[1];                      //right wheel
    TIM1 -> CCR3 = PWM[2];
    TIM1 -> CCR4 = PWM[3];
    
    ////                        filpper                          ////
    l_Angle = (int)l_Input[pos];
    r_Angle = (int)r_Input[pos];
    
    rpm2Degree(RPM);
    
    Encoder_Cnt[2] = TIM3 -> CNT;
    TIM3->CNT = 0;
    
    Encoder_Cnt[3] = TIM2 ->CNT;
    TIM2->CNT = 0;//right
    
    
    //left
    if(Encoder_Cnt[2] > 60000)
    {
      l_buff = (int)(Encoder_Cnt[2] - 65536);
      l_Input[vel] = (l_buff * 4.39453125);         // EnocoderData / 8192 * 360 *100  (degree/s)
      l_EncoderSum += l_buff;       //  엔코더 값 누적 
      l_Input[pos] = l_EncoderSum * 0.0439453125;       // 엔코더 각도로 변환(sum / 8192 * 360)
    }
    else
    {
      l_Input[vel] = (Encoder_Cnt[2] * 4.39453125);           // EnocoderData / 8192 * 360 *100  (degree/s)
      l_EncoderSum += Encoder_Cnt[2];        //  엔코더 값 누적 
      l_Input[pos] = l_EncoderSum * 0.0439453125;       // 엔코더 각도로 변환(sum / 8192 * 360)
    }
    
    //right
    if(Encoder_Cnt[3] > 60000)
    {
      r_buff = (int)(Encoder_Cnt[3] - 65536);
      r_Input[vel] = (r_buff * 4.39453125);         // EnocoderData / 8192 * 360 *100  (degree/s)
      r_EncoderSum += r_buff;       //  엔코더 값 누적 
      r_Input[pos] = r_EncoderSum * 0.0439453125;       // 엔코더 각도로 변환(sum / 8192 * 360)
    }
    else
    {
      r_Input[vel] = (Encoder_Cnt[3] * 4.39453125);           // EnocoderData / 8192 * 360 *100  (degree/s)
      r_EncoderSum += Encoder_Cnt[3];        //  엔코더 값 누적 
      r_Input[pos] = r_EncoderSum * 0.0439453125;       // 엔코더 각도로 변환(sum / 8192 * 360)
    }
    
    PID_Control(&l_BLDC[pos], l_TargetAngle, l_Input[pos]);
    PID_Control(&l_BLDC[vel], l_BLDC[pos].nowOutput, l_Input[vel]);
    PID_Control(&r_BLDC[pos], r_TargetAngle, r_Input[pos]);
    PID_Control(&r_BLDC[vel], r_BLDC[pos].nowOutput, r_Input[vel]);
    
    
    //    PID_Control(&BLDC[vel], Target, Input[vel]); //속도 제어만 할 경우 주석 풀고 위에 두 줄 주석ㄱㄱ
    
    TIM1->CCR3 = (int)(l_BLDC[vel].nowOutput * -100 + 5000);
    TIM1->CCR4 = (int)(r_BLDC[vel].nowOutput * -100 + 5000);
    
    
    //check value
    PWM[2] = (int)(l_BLDC[vel].nowOutput * -100 + 5000);
    PWM[3] = (int)(r_BLDC[vel].nowOutput * -100 + 5000);
    
    l_now_RPM = (int)(l_Input[vel] / 360.0 * 60.0); // rpm
    r_now_RPM = (int)(r_Input[vel] / 360.0 * 60.0); // rpm
    
    l_Angle = (int)l_Input[pos];
    r_Angle = (int)r_Input[pos];
    
    osDelay(10);
  }
  /* USER CODE END 5 */ 
}

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
