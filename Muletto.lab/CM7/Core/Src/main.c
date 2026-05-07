/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdlib.h>
#include<stdbool.h>
#include "DC_motor.h"
#include"bno055_stm32.h"
#include"Configuration.h"
#include"PID.h"
#include"servo_motor.h"
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SerialData {
	int enable;
	int valid;
	float offset;

	float curvature_radius_ref_m; // [m]
	float linear_speed_ref_m_s; //[m/s]
} serialData;

typedef struct VehicleData {
	//Trazione
	int counts;
	int ref_count;
	int delta_count;
	float delta_angle_deg; // [°]
	float motor_speed_deg_sec; // [°/s]
	float motor_speed_RPM; // RPM
	float linear_speed_m_s; //[m/s]
	float motor_speed_ref_RPM; // RPM
	uint8_t motor_direction_ref;

	//Sterzo
	double yaw_rate_rad_sec; // [rad/s]
	double yaw_rate_deg_sec; // [°/s]
	double yaw_rate_ref_rad_sec; //[rad/s]
} vehicleData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PD */

/* Private
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint32_t IC_Val1_CH1 = 0;
uint32_t IC_Val2_CH1 = 0;
uint32_t Differenza_CH1 = 0;
uint8_t Is_First_Captured_CH1 = 0; // 0 = aspetta salita, 1 = aspetta discesa
uint32_t steering_us = 1500;       // Il tempo del gas in microsecondi (1500 = fermo)
uint32_t throttle_us = 1500;       // Il tempo del gas in microsecondi (1500 = fermo)
serialData data;
vehicleData vehicleState;
volatile uint8_t new_rc_steering = 0;
volatile uint8_t new_rc_throttle = 0;
//PID
PID pid_traction, pid_steering;
double u_trazione = 0;
double u_sterzo = 0;

//Event control
int Flag_10ms = 0;
int HardwareEnable = 1;
int time_counter = 0;

//Serial input var
uint8_t msg[45] = { "\0" };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // 1. ACCENDIAMO L'INTERRUTTORE GENERALE DELLA PORTA D (FONDAMENTALE!)
   __HAL_RCC_GPIOD_CLK_ENABLE();


   // 2. CONFIGURIAMO IL PIN PD5 A MANO (Bypassiamo CubeMX)
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull (Muscoli attivi)
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   // (Assicurati di avere anche l'HAL_TIM_PWM_Start qui per il rosso)


  //PWM Servo
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      TIM1->BDTR |= TIM_BDTR_MOE;
      TIM1->CCR2 = 1500;
        HAL_Delay(2000); // Aspetta 2 secondi dritto

  	//PWM DC motor
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  	//ENCODER TIMER
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  	//10ms TIMER
  	HAL_TIM_Base_Start_IT(&htim6);

  	//HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);

  	//PID traction
  	init_PID(&pid_traction, TRACTION_SAMPLING_TIME, MAX_U_TRACTION,
  	MIN_U_TRACTION);
  	tune_PID(&pid_traction, KP_TRACTION, KI_TRACTION, 0);

  	//PID steering
  	init_PID(&pid_steering, STEERING_SAMPLING_TIME, MAX_U_STEERING,
  	MIN_U_STEERING);
  	tune_PID(&pid_steering, KP_STEERING, KI_STEERING, 0);

  	// IMU BNO055 config
  	//HAL_I2C_IsDeviceReady(&hi2c1, BNO055_I2C_ADDR << 1, 5, 1000);
  	//bno055_assignI2C(&hi2c1);
  	//bno055_setup();
  	//bno055_setOperationModeNDOF();

  	servo_motor(0);

  	printf("Initialization Completed!\r\n");



  	//Test without Raspberry
  	data.enable = 0;
  	data.valid = 1;
  	data.offset = 0.0;
  	data.curvature_radius_ref_m = MAX_CURVATURE_RADIUS_FOR_STRAIGHT; //with r=1000000 the car goes straight
  	data.linear_speed_ref_m_s = 0.05;

  	float RPM_2_m_s = (2 * M_PI / 60) * WHEEL_RADIUS
  			/ MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION;
  	RPM_2_m_s = RPM_2_m_s * 0.787; // correzione aggiunta xk non andava a 1m/s ma a 0.82m/s

  	data.enable = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  	uint32_t MAX_SPEED = 30;

  	float steering_angle = 0.0f;
  	float steering_filtered = 1500.0f;
  	float throttle_filtered = 1500.0f;


  	while (1)
  	{
  		if(new_rc_throttle)
  		{
  		    new_rc_throttle = 0;

  		    throttle_filtered =
  		        throttle_filtered * 0.8f +
  		        ((float)throttle_us * 0.2f);

  		    uint32_t duty_RC = 0;
  		    uint8_t dir_RC = 0;

  		    if(throttle_filtered > 1560)
  		    {
  		        dir_RC = 0;
  		        duty_RC = ((throttle_filtered - 1560) * MAX_SPEED) / (2000 - 1560);
  		    }
  		    else if(throttle_filtered < 1440)
  		    {
  		        dir_RC = 1;
  		        duty_RC = ((1440 - throttle_filtered) * MAX_SPEED) / (1440 - 1000);
  		    }
  		    else
  		    {
  		        duty_RC = 0;
  		        dir_RC = 0;
  		    }

  		    if(duty_RC > 0 && duty_RC < 20)
  		        duty_RC = 20;

  		    if(duty_RC > MAX_SPEED)
  		        duty_RC = MAX_SPEED;

  		    set_PWM_and_dir(duty_RC, dir_RC);
  		}

  		if(new_rc_steering)
  		{
  		    new_rc_steering = 0;

  		    steering_filtered =
  		        steering_filtered * 0.8f +
  		        ((float)steering_us * 0.2f);

  		    if(steering_filtered < 1000)
  		        steering_filtered = 1000;

  		    if(steering_filtered > 2000)
  		        steering_filtered = 2000;

  		    steering_angle =
  		        ((steering_filtered - 1500.0f) * 5.0f) / 500.0f;

  		    servo_motor(steering_angle);
  		}
  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	      //Controllo generale: eseguiamo solo se la macchina è "accesa"
	      /*if (data.enable == 1) {
	          if (Flag_10ms == 1) { // Il timer scandisce il tempo (esegue ogni 10ms)
	              Flag_10ms = 0;

	              // =============================================================
	              // 1. TRAZIONE (IL TEST ATTUALE)
	              // =============================================================

	              // Lettura velocità tramite Encoder
	              vehicleState.ref_count = TIM4->ARR / 2;
	              vehicleState.delta_count = vehicleState.counts - vehicleState.ref_count;

	              vehicleState.delta_angle_deg = (vehicleState.delta_count * 360)
	                      / ((double) (ENCODER_PPR * ENCODER_COUNTING_MODE * GEARBOX_RATIO));
	              vehicleState.motor_speed_deg_sec = vehicleState.delta_angle_deg / ENCODER_SAMPLING_TIME;
	              vehicleState.motor_speed_RPM = DegreeSec2RPM(vehicleState.motor_speed_deg_sec);

	              // Calcolo velocità di riferimento
	              vehicleState.motor_speed_ref_RPM = data.linear_speed_ref_m_s / RPM_2_m_s;
	              vehicleState.motor_direction_ref = Ref2Direction(vehicleState.motor_speed_ref_RPM);

	              // IL CERVELLO: Il PID calcola lo sforzo necessario
	              u_trazione = PID_controller(&pid_traction,
	                      fabs(vehicleState.motor_speed_RPM),
	                      fabs(vehicleState.motor_speed_ref_RPM));

	              // Convertiamo lo sforzo in Duty Cycle (0-100)
	              uint32_t duty_finale = (uint32_t) Voltage2Duty(u_trazione);


	              // LA CIMICE SUPREMA: Leggiamo anche il registro hardware!
	                    //char debug_msg[120];
	                   // int len = sprintf(debug_msg, "U_traz: %.2f | Duty: %lu | REGISTRO FISICO: %lu\r\n",
	                     //                 (float)u_trazione,
	                      //                duty_finale,
	                        //              TIM3->CCR1); // <-- Questo è il numero che va davvero al filo!
	                    //HAL_UART_Transmit(&huart3, (uint8_t*)debug_msg, len, 10);
	              // L'ORDINE AL MOTORE: Inviamo la decisione ai pin (Rosso e Verde)
	              set_PWM_and_dir(duty_finale, vehicleState.motor_direction_ref);


	              // =============================================================
	              // 2. STERZO E IMU (DA SCOMMENTARE E USARE IN FUTURO)
	              // =============================================================

	              // Get yawrate from IMU
	              bno055_vector_t v = bno055_getVectorGyroscope();
	              vehicleState.yaw_rate_deg_sec = v.z;
	              vehicleState.yaw_rate_rad_sec = (vehicleState.yaw_rate_deg_sec * M_PI) / 180;

	              if (data.curvature_radius_ref_m >= MAX_CURVATURE_RADIUS_FOR_STRAIGHT) {
	                  vehicleState.yaw_rate_ref_rad_sec = 0;
	                  u_sterzo = PID_controller(&pid_steering, vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec);
	                  servo_motor(-u_sterzo); // minus because yawrate and steering are opposite
	              } else {
	                  vehicleState.linear_speed_m_s = vehicleState.motor_speed_RPM * RPM_2_m_s;
	                  vehicleState.yaw_rate_ref_rad_sec = vehicleState.linear_speed_m_s / data.curvature_radius_ref_m;

	                  float yaw_rate_ref_rad_sec_abs = vehicleState.yaw_rate_ref_rad_sec;
	                  float yaw_rate_rad_sec_abs = vehicleState.yaw_rate_rad_sec;
	                  if (vehicleState.yaw_rate_ref_rad_sec < 0)
	                      yaw_rate_ref_rad_sec_abs = -vehicleState.yaw_rate_ref_rad_sec;
	                  if (vehicleState.yaw_rate_rad_sec < 0)
	                      yaw_rate_rad_sec_abs = -vehicleState.yaw_rate_rad_sec;

	                  u_sterzo = PID_controller(&pid_steering, yaw_rate_rad_sec_abs, yaw_rate_ref_rad_sec_abs);

	                  // minus because yawrate and steering are opposite
	                  if (data.curvature_radius_ref_m >= 0 && u_sterzo > 0)
	                      u_sterzo *= -1.0;
	                  if (data.curvature_radius_ref_m < 0 && u_sterzo < 0)
	                      u_sterzo *= -1.0;

	                  servo_motor((int) u_sterzo);
	              }

	              // =============================================================

	          }
	      } else {
	          // Se l'utente spegne la macchina (data.enable = 0), blocca tutto per sicurezza.
	          set_PWM_and_dir(0, vehicleState.motor_direction_ref);
	          servo_motor(0);
	      }*/



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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1332;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 65;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 63;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  htim6.Init.Prescaler = 64-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		Flag_10ms = 1;

		//Encoder
		vehicleState.counts = TIM4->CNT;
		TIM4->CNT = TIM4->ARR / 2;

		//References
		time_counter++;

		if (time_counter == 200) {
			data.curvature_radius_ref_m = 1.3; //2.2;//2.2; //1.3;
			pid_steering.Iterm = 0;
			pid_steering.e_old = 0;
		}

		if (time_counter == 580) {
			data.enable = 0;
		}
		/*
		 if (time_counter == 3000) {
		 data.enable = 0;
		 }
		 */

	}
}

//USART2 -> ST_Link UART for DEBUG with USB (e.g. PUTTY)
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		return len;
}

//BLUE user button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		if (HardwareEnable == 0) {
			HardwareEnable = 1;
		} else {
			HardwareEnable = 0;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{  // =========================
    // STERZO -> PA3 (TIM5 CH4)
    // =========================
    if (htim->Instance == TIM5 &&
        htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        static uint32_t ic_val1 = 0;
        uint32_t ic_val2 = 0;

        if (Is_First_Captured_CH1 == 0)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            Is_First_Captured_CH1 = 1;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                          TIM_CHANNEL_4,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

            if (ic_val2 > ic_val1)
                Differenza_CH1 = ic_val2 - ic_val1;
            else
                Differenza_CH1 = (0xFFFFFFFF - ic_val1) + ic_val2;

            steering_us = Differenza_CH1;
            new_rc_steering = 1;

            Is_First_Captured_CH1 = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                          TIM_CHANNEL_4,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    // =========================
    // TRAZIONE -> PA0 (TIM5 CH1)
    // =========================
    if (htim->Instance == TIM5 &&
        htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        static uint32_t ic_val1_t = 0;
        uint32_t ic_val2_t = 0;

        if (Is_First_Captured_CH1 == 0)
        {
            ic_val1_t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured_CH1 = 1;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                          TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2_t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (ic_val2_t > ic_val1_t)
                Differenza_CH1 = ic_val2_t - ic_val1_t;
            else
                Differenza_CH1 = (0xFFFFFFFF - ic_val1_t) + ic_val2_t;

            throttle_us = Differenza_CH1;
            new_rc_throttle = 1;

            Is_First_Captured_CH1 = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                          TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
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
#ifdef USE_FULL_ASSERT
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
