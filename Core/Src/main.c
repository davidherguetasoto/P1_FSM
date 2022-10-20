/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"

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

//Estados FSM ON-OFF
enum start_state {
	OFF,
	ON,
};

//Estados FSM Lectura-Espera
enum lecture_state{
	ESPERA,
	LECTURA,
};

//Estados FSM LED azul
enum led_state{
	LED_OFF,
	LED_ON
};

//entradas
static uint8_t boton = 0;
static uint8_t sensorx = 0, sensory = 0, sensorz = 0;

//variables compartidas
static uint8_t activado = 0, timer_boton = 0, timer_lectura = 0, timer_led = 0;

//salidas
static uint8_t faultx, faulty, faultz;
static uint8_t led_activacion;

//funciones de transicion
static int boton_presionado (fsm_t* this) { /*if (timer_boton)*/ return boton; /*else return 0;*/ }
static int boton_no_presionado (fsm_t* this) {return !boton; }

static int sensorx_on (fsm_t* this) { return sensorx; }
static int sensory_on (fsm_t* this) { return sensory; }
static int sensorz_on (fsm_t* this) { return sensorz; }
static int sensorx_off (fsm_t* this) { return !sensorx; }
static int sensory_off (fsm_t* this) { return !sensory; }
static int sensorz_off (fsm_t* this) { return !sensorz; }

static int activado_on (fsm_t* this) { if (timer_lectura) return activado; else return 0; }
static int activado_off (fsm_t* this) { if (!timer_lectura || !activado) return 1; else return 0; }
static int activado_on_led (fsm_t* this) { if (timer_led) return activado; else return 0; }
static int activado_off_led (fsm_t* this) { if (timer_led) return !activado; else return 0; }

static int defecto (fsm_t* this)  {return 1;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void activacion (fsm_t* this)
{
  activado = 1;
}

static void desactivacion (fsm_t* this)
{
  activado = 0;
}

static void lectura_x (fsm_t* this)
{
  faultx = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
}

static void lectura_y (fsm_t* this)
{
  faulty = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
}

static void lectura_z (fsm_t* this)
{
  faultz = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
}

static void lectura_x_fin (fsm_t* this)
{
  faultx = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
}

static void lectura_y_fin (fsm_t* this)
{
  faulty = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
}

static void lectura_z_fin (fsm_t* this)
{
  faultz = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
}

static void lectura_fin (fsm_t* this)
{
  faultx = 0;
  faulty = 0;
  faultz = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
}

static void led_activado (fsm_t* this)
{
  led_activacion = 1;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
}

static void led_no_activado (fsm_t* this)
{
  led_activacion = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
}

static fsm_trans_t inicio[] = {
  { OFF, boton_presionado, ON, activacion},
  { ON, boton_no_presionado, OFF,  desactivacion },
  { ON, defecto, ON,  activacion },
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t lectura[] = {
  { ESPERA, activado_on, LECTURA, 0},
  { LECTURA, sensorx_on, LECTURA, lectura_x},
  { LECTURA, sensorx_off, LECTURA, lectura_x_fin},
  { LECTURA, sensory_on, LECTURA, lectura_y},
  { LECTURA, sensory_off, LECTURA, lectura_y_fin},
  { LECTURA, sensorz_on, LECTURA, lectura_z},
  { LECTURA, sensorz_off, LECTURA, lectura_z_fin},
  { LECTURA, activado_off, ESPERA, lectura_fin},
  {-1, NULL, -1, NULL },
  };

static fsm_trans_t led_activo[] = {
  { LED_OFF, activado_on_led, LED_ON, led_activado},
  { LED_ON, defecto, LED_ON, led_activado},
  { LED_ON, activado_off_led, LED_OFF, led_no_activado},
  {-1, NULL, -1, NULL },
  };

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	/*if (htim->Instance == TIM1){ //500ms
		if (timer_boton)
			timer_boton = 1;
		else
			timer_boton = 0;
	}*/
	if(htim->Instance == TIM2){ //1ms
		if (timer_lectura)
			timer_lectura = 1;
		else
			timer_lectura = 0;
	}
	if (htim->Instance == TIM3){ //1s
		if (timer_led)
			timer_led = 1;
		else
			timer_led = 0;
	}
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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

  //Cración de las FSM
  fsm_t* fsm_inicio = fsm_new (inicio);
  fsm_t* fsm_lectura = fsm_new (lectura);
  fsm_t* fsm_led_activo = fsm_new (led_activo);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    fsm_fire (fsm_inicio);
    fsm_fire (fsm_lectura);
    fsm_fire (fsm_led_activo);

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
    	sensorx = 1;
    else
    	sensorx = 0;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
       	sensory = 1;
    else
		sensory = 0;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
       	sensorz = 1;
    else
		sensorz = 0;
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_0){
		boton=~boton;
	}
}
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
