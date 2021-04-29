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
typedef enum state{
		  on_off ,
		  off_off,
		  off_on ,
		  on_on  }
State_sensor;

typedef enum direccion_num{
			stop,
			derecha,
			izquierda
}Direction;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
uint16_t velocidad = 0;

GPIO_PinState start = GPIO_PIN_RESET,
			  res_V = GPIO_PIN_RESET;
			  res_a = GPIO_PIN_RESET;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void state_motor();
void detect_vel();
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
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  InitResiver(GPIOA,  GPIO_PIN_10);
  protocolInit( GPIO_PIN_SET );

  revoluciones = 50;
  tiempo_motor = 0;
  numData = 0;
  pulsos = 0;

  // timer init CCR1
  TIM1->CCR1 = (200 * revoluciones/100) ;
  TIM1->ARR = 200 - 1;
  TIM1->PSC = 18;

	num_sensor = 0;
	num_sensor =
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 1) |
		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	anterior = num_sensor;
	actual = num_sensor;

	initPID_ch();
	r1 = 0;
	v1 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

/*
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0){
		  	 prueba();
	  }

*/
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
	  state_motor();

	  communication(
			  &dataResiveSerial, // donde se va a guardar los datos
			  &resive_protocol, // si ya resivio datos
			  &data_for_send, // arreglo que va a enviar
			  &command, // comando
			  &len, // longitud
			  &numData); // actualiza el dato a enviar

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Period = 20000-1;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void rotate_data(){

	if(numData > 4){ // longitud de los datos a enviar
		numData = 0;
	}

	switch(numData){
	case 0:

		data_for_send[0] = (uint8_t) revoluciones;
		data_for_send[1] = (uint8_t) move;
		data_for_send[2] = (uint8_t) velocidad;
		data_for_send[3] = (uint8_t) (velocidad >> 8);
		command = code_revl;
		len = 4;

		break;
	case 1:

		data_for_send[0] = (uint8_t) pulsos;
		data_for_send[1] = (uint8_t) (pulsos >> 8);
		data_for_send[2] = (uint8_t) (pulsos >> 16);
		data_for_send[3] = (uint8_t) r1_enter;
		data_for_send[4] = (uint8_t) (r1_enter >> 8);

		command = code_time;

		len = 5;

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
		data_for_send[0] = (uint8_t) numero_seleccionado;
		data_for_send[1] = (uint8_t) (numero_seleccionado >> 8);
		data_for_send[2] = (uint8_t) (numero_seleccionado >> 16);
		command = code_tecleando;
		len = 3;
		break;

	case 4:

		if(distancia < 0)
		data_for_send[3] = (uint8_t) 1;
		else
		data_for_send[3] = (uint8_t) 0;

		temp_dis = (uint32_t) abs(distancia);

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
			if( numero_temp < 101 )
			revoluciones = (uint8_t) numero_temp;
			break;
		case 1:
			pulsos = numero_temp;
			break;
		case 2:
			if( numero_temp < 2300){
				r1_enter = numero_temp;
				r1 = (numero_temp/16);
				k=r1;
				actualizar_par();
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

void state_motor(){

	num_sensor = 0;
	num_sensor =
			(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 1) |
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	switch(num_sensor){
	case 0:
		actual = off_off; // 2
		break;
	case 1:
		actual = off_on; // 3
		break;
	case 2:
		actual = on_off; // 1
		break;
	case 3:
		actual = on_on; // 4
		break;
	}

	if(anterior != actual){

		if(pulsos > 0){
			pulsos--;
		}

		delta_time = __HAL_TIM_GET_COUNTER(&htim3);

		if(actual == 4 || actual == 1 ){
			detect_vel();
			__HAL_TIM_SET_COUNTER(&htim3,0);
			if(r1 > 0){
				PID(); // controlador
				revoluciones = (1/u) * 100;
			}
		}

		if((actual - anterior) == 1 || (actual - anterior) == -3 ){

			move = derecha;
			distancia += 1;

		}else if( (actual - anterior) == -1 || (actual - anterior) == 3 ){

			move = izquierda;
			distancia -= 1;

		}

		anterior = actual;

	}else if( __HAL_TIM_GET_COUNTER(&htim3) > 19000 ){

		move = stop;
		velocidad = 0;
		__HAL_TIM_SET_COUNTER(&htim3,0);

		if(r1 > 0){
			PID(); // controlador
			revoluciones = (1/u) * 100;
		}

	}

	if(pulsos == 0 || !start ){
		uint16_t porcent = (200 * 50/100 );
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, porcent);
	}else{
		uint16_t porcent = (200 * revoluciones/100 );
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, porcent);
	}
}

void detect_vel(){

	if(   ((20000/delta_time) - velocidad ) < 15 &&
		  ((20000/delta_time) - velocidad ) > (-15) ){

	  velocidad = (uint16_t) ((velocidad + (20000/delta_time))/2);
	  v1 = ((v1 + (20000/delta_time))/2);

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

		if(counterTime > 19999){
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
