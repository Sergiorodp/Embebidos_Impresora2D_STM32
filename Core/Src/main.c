/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ch_protocol.h"
#include "IRresiver_ch.h"
#include "PID_ch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define code_revl 0x40
#define code_time 0x41
#define code_menu 0x42
#define code_tecleando 0x43
#define code_distancia 0x44

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

volatile uint8_t dataResiveSerial[125];

void rotate_data();
void select_data(); // selecionar el dato a cambiar

//contadores
volatile uint32_t lenBuffer, counterTime = 0;
volatile uint8_t read = 1;

uint8_t len             = 1,
		command         = 0x30,
		numData         = 0;

// DATOS

uint8_t menu = 0;
uint8_t revoluciones;
volatile uint32_t tiempo_motor;
uint8_t data_for_send[50];
volatile uint32_t counter_motor_time = 0;


// state sensores herradura

State_sensor actual, anterior;
Direction move;

uint8_t num_sensor, direccion_num,
max_menu = 2;

int distancia;
uint32_t pulsos;
uint32_t temp_dis;
uint16_t r1_enter;

// detectar velocidad

uint16_t delta_time = 1;
uint32_t velocidad = 0;

volatile uint16_t counterPID = 0;

GPIO_PinState start = GPIO_PIN_SET,
			  res_V = GPIO_PIN_RESET;
			  res_a = GPIO_PIN_RESET;

// adc_value

uint16_t adc_value;
uint8_t ampere;

float r1 = 0,r2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */

void state_motor( Motor * ref);
void detect_vel();
void Read_ADC();
void initMotor( Motor * ref, GPIO_TypeDef * ch,
		uint16_t pin_1 , uint16_t pin_2, uint16_t tim_ch);

void ref_update();

//uint32_t prueba();

// serial communication

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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  InitResiver(GPIOA,  GPIO_PIN_10);

  protocolInit( GPIO_PIN_SET );

  initMotor(&M1, GPIOA, GPIO_PIN_2, GPIO_PIN_3, TIM_CHANNEL_1);
  initMotor(&M2, GPIOB, GPIO_PIN_0, GPIO_PIN_1, TIM_CHANNEL_2);

  revoluciones = 50;
  tiempo_motor = 0;
  numData = 0;
  pulsos = 0;

	for( uint8_t i = 0; i < 10; i++){
		dataResiveSerial[i] = 0;
	}

  // timer init CCR1

  TIM1 -> CCR1 = 100 - 1;
  TIM1 -> CCR2 = 100 - 1;
  TIM1 -> ARR = 200 - 1;
  TIM1 -> PSC = 18;

	anterior = num_sensor;
	actual = num_sensor;

	//iniciar PID para cada motor

	initPID_ch_m(&M1);
	initPID_ch_m(&M2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)

  {

	  Read_ADC();

	  if(read){
		  reciveData();
		  read = 0;
	  }

	  if(resive){
		  leer_num();
		  select_data();
		  resive = GPIO_PIN_RESET;
	  }

	  rotate_data();

	  // controlador

	  if(counterPID > 40 && start ){

		  //PID();

		  PID_m(&M1);
		  PID_m(&M2);
		  M1.revoluciones = (uint8_t) abs( M1.pid.u - 50 );
		  M2.revoluciones = (uint8_t) abs( M2.pid.u - 50 );

		  //revoluciones = abs(u - 50);
		  counterPID = 0;

	  }

	  // actualizar estado motores
	  state_motor(&M1);
	  state_motor(&M2);

	  communication(
			  &dataResiveSerial, // donde se va a guardar los datos
			  &resive_protocol, // si ya resivio datos
			  &data_for_send, // arreglo que va a enviar
			  &command, // comando
			  &len, // longitud
			  &numData); // actualiza el dato a enviar

// dataResiveSerial[1] == 0x60 &&
	  if(  check  ){
		  ref_update();
		  M1.in_pos = 0;
		  M2.in_pos = 0;
		  check = GPIO_PIN_RESET;
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void ref_update(){

	M2.pid.ref = (((float) (( (uint16_t) dataResiveSerial[4] << 8)
			| dataResiveSerial[3]))/0.72);

	M1.pid.ref = ((
			(float) (( (uint16_t) dataResiveSerial[6] << 8)
			| dataResiveSerial[5])) /0.72);

	if(r1 != M1.pid.ref){

		M1.pid.k = M1.pid.ref;
		Params_choose_m(&M1);
		r1 = M1.pid.ref;

	}
	if(r2 != M2.pid.ref){

		M2.pid.k = M2.pid.ref;
		Params_choose_m(&M2);
		r2 = M2.pid.ref;

	}


}

void rotate_data(){

	if(numData > 4){ // longitud de los datos a enviar
		numData = 0;
	}

	switch(numData){
	case 0:

		data_for_send[0] = (uint8_t) M1.revoluciones;
		data_for_send[1] = (uint8_t) M1.move;
		data_for_send[2] = (uint8_t) M1.pid.ref;
		data_for_send[3] = (uint8_t) ( (uint16_t) M1.pid.ref >> 8);
		data_for_send[4] = (uint8_t) ( (uint32_t) M1.pid.ref>> 16);
		data_for_send[5] = (uint8_t) adc_value;
		data_for_send[6] = (uint8_t) (adc_value >> 8);
		command = code_revl;
		len = 7;

		break;
	case 1:

		data_for_send[0] = (uint8_t) M2.pid.ref;
		data_for_send[1] = (uint8_t) ( (uint32_t) M2.pid.ref >> 8);
		data_for_send[2] = (uint8_t) ( (uint32_t) M2.pid.ref >> 16);
		data_for_send[3] = (uint8_t) r1_enter;
		data_for_send[4] = (uint8_t) (r1_enter >> 8);
		data_for_send[5] = (uint8_t) ( (M1.in_pos << 1) | (M2.in_pos));

		command = code_time;

		len = 6;

		break;

	case 2:

		data_for_send[0] = (uint8_t) menu;
		data_for_send[1] = (uint8_t) start;

		data_for_send[2] = (uint8_t) res_V;
		data_for_send[3] = (uint8_t) res_a;

		len = 4;
		command = code_menu;

		break;

	case 3:
		/*
		data_for_send[0] = (uint8_t) numero_seleccionado;
		data_for_send[1] = (uint8_t) (numero_seleccionado >> 8);
		data_for_send[2] = (uint8_t) (numero_seleccionado >> 16);
		*/

		data_for_send[0] = (uint8_t) M2.distancia;
		data_for_send[1] = (uint8_t) (M2.distancia >> 8);
		data_for_send[2] = (uint8_t) (M2.distancia >> 16);

		command = code_tecleando;
		len = 3;
		break;

	case 4:

		if(M1.distancia < 0)
		data_for_send[3] = (uint8_t) 1;
		else
		data_for_send[3] = (uint8_t) 0;

		temp_dis = (uint32_t) abs( M1.distancia);

		data_for_send[0] = (uint8_t) temp_dis;
		data_for_send[1] = (uint8_t) (temp_dis >> 8);
		data_for_send[2] = (uint8_t) (temp_dis >> 16);

		command = code_distancia;
		len = 4;

	}
}

void select_data(){

	max_menu = 5;

	if(Compress[1] == 0xEB){
		switch(Compress[0]){

		case 0xaf: // arriba
			if((menu + 1) > max_menu){
				menu = 0; // max option of the menu
			}else{
				menu += 1;
			}
			break;
		case 0xcf:
			if((menu - 1) < 0){
				menu = max_menu; // max option of the menu
			}else{
				menu -= 1;
			}
			break;
		}

		Compress[0] = 0;
	}

	if(Compress[0] == 0x68 && Compress[1] == 0xcb){

		switch(menu){
		case 0:
			if( numero_temp < 100 )
			M1.revoluciones = (uint8_t) (numero_temp);

			break;
		case 1:
			pulsos = numero_temp;
			break;
		case 2:
			if( numero_temp < 1000){

				r1_enter = numero_temp;
				M1.pid.ref = ( (float)numero_temp/0.72 );
				M1.pid.k = M1.pid.ref;
				M2.pid.ref = M1.pid.ref;
				M2.pid.k = M2.pid.ref;

				Params_choose_m(&M1);
				Params_choose_m(&M2);
			}
			break;
		case 3:
			start = !start;
			break;
		case 4:
			res_V = !res_V;
			break;
		case 5:
			res_a = !res_a;
			break;

		}
	}
}

void initMotor( Motor * ref, GPIO_TypeDef * ch, uint16_t pin_1 ,
		uint16_t pin_2, uint16_t tim_ch){

	ref -> GPIO_Pin_1 = pin_1;
	ref -> GPIO_Pin_2 = pin_2;
	ref -> GPIOx = ch;
	ref -> revoluciones = 50;
	ref -> distancia = 555;
	ref -> move = stop;
	ref -> tim_chanel = tim_ch;
	ref -> num_sensor = 0;
	ref -> num_sensor = (HAL_GPIO_ReadPin(ch, pin_2) << 1) |
			HAL_GPIO_ReadPin(ch , pin_1);
	ref -> in_pos = 0;

}

void state_motor( Motor * ref){

	ref -> num_sensor = 0;
	ref -> num_sensor =
			(HAL_GPIO_ReadPin(ref->GPIOx, ref->GPIO_Pin_2) << 1) |
			HAL_GPIO_ReadPin(ref->GPIOx, ref->GPIO_Pin_1);

	switch(ref->num_sensor){
	case 0:
		ref->actual = off_off; // 2
		break;
	case 1:
		ref->actual = off_on; // 3
		break;
	case 2:
		ref->actual = on_off; // 1
		break;
	case 3:
		ref->actual = on_on; // 4
		break;
	}

	if(ref->anterior != ref->actual){

		/*
		if(pulsos > 0){
			pulsos--;
		}
		*/
/*
		if( actual == 1 ){

			delta_time = __HAL_TIM_GET_COUNTER(&htim3);
			detect_vel();
			__HAL_TIM_SET_COUNTER(&htim3,0);

		}
*/
		if((ref->actual - ref->anterior) == 1 ||
				(ref->actual - ref->anterior) == -3 ){

			ref->move = derecha;
			ref->distancia += 1;

		}else if( (ref->actual - ref->anterior) == -1 ||
				(ref->actual - ref->anterior) == 3 ){

			ref->move = izquierda;
			ref->distancia -= 1;

		}

		ref->pid.feedback = (float) ref->distancia;

		ref->anterior = ref->actual;

	}

	/*
	else if( __HAL_TIM_GET_COUNTER(&htim3) > 45000 ){

		move = stop;
		velocidad = 0;
		v1 = 0;
		__HAL_TIM_SET_COUNTER(&htim3,0);

	}
	*/

	if( !start )
	{
		ref->porcent = (2 * 50 );
		__HAL_TIM_SET_COMPARE(&htim1, ref->tim_chanel, ref->porcent);
	}
	else{
	ref->porcent = ( (2 * ref->revoluciones) );

	__HAL_TIM_SET_COMPARE(&htim1, ref->tim_chanel, ref->porcent);

	}

}

void detect_vel(){

	uint32_t delta = (100000/delta_time);

	if( abs(delta - velocidad) < 32 ){

	  velocidad = (uint32_t) ((velocidad + delta)/2);

	}

}

// agragar función a USB_DEVICE_cdc_if CON __WEAK
void resiveRx(uint8_t* Buf, uint32_t *Len ){

	memcpy( &dataResiveSerial, Buf, Buf[2] + 5);
	lenBuffer = dataResiveSerial[2];
	resive_protocol = GPIO_PIN_SET;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		read = 1;

		counterTime += 1;
		counterPID +=1;

		if(counterTime > 199){
			waitTime += 1;
			counterTime = 0;
		}

		if( tiempo_motor > 0 && revoluciones != 50){
			counter_motor_time += 1;
			if(counter_motor_time > 19){
				tiempo_motor --;
				counter_motor_time = 0;
			}
		}else{
			counter_motor_time = 0;
		}
	}
}

void Read_ADC(){

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t adc_temp = (uint16_t) HAL_ADC_GetValue(&hadc1);

	adc_value = ((adc_value + adc_temp)/2);

	//ampere = (uint16_t)( ((float)adc_value ) * 0.80); // 3300/4095 = 0.80

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
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
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
