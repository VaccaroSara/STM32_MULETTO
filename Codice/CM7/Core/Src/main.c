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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define MAX_SAMPLES 300          // 300 campioni a 10ms = 3 secondi di registrazione
float log_ref[MAX_SAMPLES];      // Memoria per il radiocomando
float log_real[MAX_SAMPLES];     // Memoria per l'IMU
int log_index = 0;
int recording_state = 0;

//FIR_
#define FIR_ORDER 3 // Finestra di 3 campioni

float fir_buffer[FIR_ORDER] = {0}; // L'array per il buffer del FIR
int fir_index = 0;

//Memoria per la trazione
float fir_buffer_throttle[FIR_ORDER] = {0};
int fir_index_throttle = 0;

// Memoria per il FIR dell'IMU
float fir_buffer_imu[FIR_ORDER] = {0};
int fir_index_imu = 0;


float steering_filtered = 1500.0f;
float throttle_filtered = 1500.0f;

uint32_t IC_Val1_CH1 = 0;
uint32_t IC_Val2_CH1 = 0;
uint32_t Differenza_CH1 = 0;
uint8_t Is_First_Captured_Steering = 0; // 0 = aspetta salita, 1 = aspetta discesa
uint8_t Is_First_Captured_Throttle = 0; // 0 = aspetta salita, 1 = aspetta discesa
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

//Controllo velocità con encoder
#define SPEED_SAMPLES 300          // 3 secondi a 10ms
float log_speed_ref[SPEED_SAMPLES];   // Velocità di riferimento [m/s]
float log_speed_real[SPEED_SAMPLES];  // Velocità reale da encoder [m/s]
int speed_log_index = 0;
int speed_recording_state = 0;        // 0=attesa, 1=registrazione, 2=stampa, 10=finito

//RPM_2_m_s globale per usarla nel logger
float RPM_2_m_s_global = 0.0f;

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
static void MX_I2C2_Init(void);
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
  MX_I2C2_Init();
  HAL_Delay(700);

  /* USER CODE BEGIN 2 */
  //Accensione interruttore generale porta D
   __HAL_RCC_GPIOD_CLK_ENABLE();

   //Configurazione PIN PD5 Bypassando CubeMX
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

  	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);

  	//PID traction
  	init_PID(&pid_traction, TRACTION_SAMPLING_TIME, MAX_U_TRACTION,
  	MIN_U_TRACTION);
  	tune_PID(&pid_traction, KP_TRACTION, KI_TRACTION, 0);

  	//PID steering
  	init_PID(&pid_steering, STEERING_SAMPLING_TIME, MAX_U_STEERING,
  	MIN_U_STEERING);
  	tune_PID(&pid_steering, KP_STEERING, KI_STEERING, 0);

  	    // I2C SCANNER (Trova l'indirizzo reale dell'IMU)
  	    printf("\r\n--- AVVIO SCANSIONE I2C2 ---\r\n");
  	    uint8_t dispositivi_trovati = 0;
  	    for(uint8_t i = 1; i < 128; i++) {
  	        if(HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 100) == HAL_OK) {
  	            printf("Sensore trovato all'indirizzo: 0x%02X <<<\r\n", i);
  	            dispositivi_trovati++;
  	        }
  	    }
  	    if(dispositivi_trovati == 0) {
  	        printf("Nessun sensore trovato.\r\n");
  	    }
  	    printf("--- FINE SCANSIONE ---\r\n");
  	    HAL_Delay(2000); // Pausa di 2 secondi per darti il tempo di leggere sul Mac

  	// IMU BNO055 config
  	HAL_I2C_IsDeviceReady(&hi2c2, BNO055_I2C_ADDR << 1, 5, 1000);
  	bno055_assignI2C(&hi2c2);
  	bno055_setup();
  	bno055_setOperationModeNDOF();

  	servo_motor(0);

  	printf("Initialization Completed!\r\n");

  	//Test without Raspberry
  	data.enable = 0;
  	data.valid = 1;
  	data.offset = 0.0;
  	data.curvature_radius_ref_m = MAX_CURVATURE_RADIUS_FOR_STRAIGHT; // r=1000000 la macchina va dritta
  	data.linear_speed_ref_m_s = 0.05;

  	float RPM_2_m_s = (2 * M_PI / 60) * WHEEL_RADIUS
  			/ MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION;
  	RPM_2_m_s = RPM_2_m_s * 0.787; // correzione aggiunta perchè non andava a 1m/s ma a 0.82m/s

  	RPM_2_m_s_global = RPM_2_m_s;  // Salva per uso globale nel logger

  	data.enable = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  	uint32_t MAX_SPEED = 20;

  	float steering_angle = 0.0f;

  	//fir
  	for(int i = 0; i < FIR_ORDER; i++) {
  	  	    fir_buffer[i] = 1500.0f;
  	  	    fir_buffer_throttle[i] = 1500.0f;
  	  	}


  	while (1)
  	{
  		/*IIR FILTRO
  	    //Ricezione radio trazione
  	    if(new_rc_throttle)
  	    {
  	        new_rc_throttle = 0;
  	        throttle_filtered = throttle_filtered * 0.8f + ((float)throttle_us * 0.2f);


  	    }

  	    //Ricezione radio sterzo (calcolo del filtro)
  	    if(new_rc_steering)
  	    {
  	        new_rc_steering = 0;
  	      steering_filtered = steering_filtered * 0.3f + ((float)steering_us * 0.7f);  	        if(steering_filtered < 1000) steering_filtered = 1000;
  	        if(steering_filtered > 2000) steering_filtered = 2000;
  	    }


*/
  		  		//Filtro FIR trazione (Media Mobile)
  		  		if(new_rc_throttle)
  		  		{
  		  		    new_rc_throttle = 0;

  		  		    // Lettura segnale grezzo del gas
  		  		    float throttle_raw = (float)throttle_us;

  		  		    // Salvataggio nell'array dedicato alla trazione
  		  		    fir_buffer_throttle[fir_index_throttle] = throttle_raw;

  		  		    // Fa avanzare l'indice nell'array (ripartendo da 0 quando è pieno)
  		  		    fir_index_throttle = (fir_index_throttle + 1) % FIR_ORDER;

  		  		    //Calcolo media
  		  		    float sum_throttle = 0.0f;
  		  		    for(int i = 0; i < FIR_ORDER; i++) {
  		  		        sum_throttle += fir_buffer_throttle[i];
  		  		    }

  		  		    //Aggiornamento variabili globali
  		  		    throttle_filtered = sum_throttle / (float)FIR_ORDER;

  		  		    //Limiti di sicurezza
  		  		    if(throttle_filtered < 1000.0f) throttle_filtered = 1000.0f;
  		  		    if(throttle_filtered > 2000.0f) throttle_filtered = 2000.0f;
  		  		}

  		  	if(new_rc_steering)
  		  	    {
  		  	        new_rc_steering = 0;

  		  	    // Mappatura diretta del setpoint radiocomando (unfiltered)
  		  	        steering_filtered = (float)steering_us;

  		  	        //Limiti di sicurezza
  		  	        if(steering_filtered < 1000.0f) steering_filtered = 1000.0f;
  		  	        if(steering_filtered > 2000.0f) steering_filtered = 2000.0f;
  		  	    }

  	  //PID (ogni 10ms esatti grazie al Flag)
  	  if (Flag_10ms == 1)
  	  {
  	      Flag_10ms = 0;

  	      //Lettura encoder (feedback trazione)
  	      vehicleState.counts = TIM4->CNT;
  	      TIM4->CNT = TIM4->ARR / 2;
  	      vehicleState.ref_count = TIM4->ARR / 2;
  	      vehicleState.delta_count = vehicleState.counts - vehicleState.ref_count;

  	      vehicleState.delta_angle_deg = (vehicleState.delta_count * 360.0) /
  	                                     ((double) (ENCODER_PPR * ENCODER_COUNTING_MODE * GEARBOX_RATIO));
  	      vehicleState.motor_speed_deg_sec = vehicleState.delta_angle_deg / ENCODER_SAMPLING_TIME;
  	      vehicleState.motor_speed_RPM = DegreeSec2RPM(vehicleState.motor_speed_deg_sec);

  	    // STREAMING REAL-TIME (ogni 10ms)
  	    vehicleState.linear_speed_m_s = vehicleState.motor_speed_RPM * RPM_2_m_s_global;
  	    float speed_ref_test = vehicleState.motor_speed_ref_RPM * RPM_2_m_s_global;
  	    	printf("R,%.4f,%.4f\r\n", speed_ref_test, vehicleState.linear_speed_m_s);
  	      //Mappatura trazione PID
  	      float target_rpm = 0.0f;
  	      uint8_t target_dir = 0;

  	      float MAX_SPEED_M_S = 1.0f;
  	      float MAX_TARGET_RPM = MAX_SPEED_M_S / RPM_2_m_s;

  	      if(throttle_filtered > 1560.0f)
  	      {
  	          target_dir = 1;
  	          target_rpm = ((throttle_filtered - 1560.0f) * MAX_TARGET_RPM) / 433.0f;
  	      }
  	      else if(throttle_filtered < 1440.0f)
  	      {
  	          target_dir = 0;
  	          target_rpm = ((1440.0f - throttle_filtered) * MAX_TARGET_RPM) / 449.0f;
  	      }
  	      else
  	      {
  	          target_rpm = 0.0f;
  	          target_dir = 0;
  	          pid_traction.Iterm = 0;
  	          pid_traction.e_old = 0;
  	      }

  	      vehicleState.motor_speed_ref_RPM = target_rpm;
  	      vehicleState.motor_direction_ref = target_dir;

  	      u_trazione = PID_controller(&pid_traction,
  	                                  fabs(vehicleState.motor_speed_RPM),
  	                                  fabs(vehicleState.motor_speed_ref_RPM));

  	      uint32_t duty_finale = (uint32_t) Voltage2Duty(u_trazione);

  	      if(duty_finale > MAX_SPEED) duty_finale = MAX_SPEED;
  	      if(target_rpm == 0.0f) duty_finale = 0;

  	      set_PWM_and_dir(duty_finale, target_dir);

  	      // Mappatura sterzo PID con IMU (BNO055)

  	      //Lettura velocità di rotazione della macchina dal giroscopio
  	    static int imu_counter = 0;
  	    if (++imu_counter >= 5) {
  	        imu_counter = 0;
  	        bno055_vector_t v = bno055_getVectorGyroscope();

  	        //Lettura grezza
  	        float yaw_rate_grezzo = (v.z * M_PI) / 180.0f;

  	        //Filtro FIR sull'IMU
  	        fir_buffer_imu[fir_index_imu] = yaw_rate_grezzo;
  	        fir_index_imu = (fir_index_imu + 1) % FIR_ORDER;

  	        float sum_imu = 0.0f;
  	        for(int i = 0; i < FIR_ORDER; i++) {
  	            sum_imu += fir_buffer_imu[i];
  	        }

  	        //Dato salvato nella struct è quello pulito dal FIR
  	        vehicleState.yaw_rate_rad_sec = sum_imu / (float)FIR_ORDER;
  	    }

  	      //Calcolo azione utente con radiocomando
  	      float yaw_rate_ref = -((steering_filtered - 1500.0f) * 1.0f) / 500.0f;

  	      float ref_abs = fabs(yaw_rate_ref);
  	      float feedback_abs = fabs(vehicleState.yaw_rate_rad_sec);

  	      //Il PID calcola lo sforzo confrontando realtà (IMU) e obiettivo (Radiocomando)
  	      u_sterzo = PID_controller(&pid_steering, feedback_abs, ref_abs);

  	      if (yaw_rate_ref < 0) {
  	          u_sterzo = -u_sterzo;
  	      }

  	      //Reset zona morta per quando si lascia la levetta
  	      if (steering_filtered > 1460.0f && steering_filtered < 1560.0f) {
  	          pid_steering.Iterm = 0;
  	          pid_steering.e_old = 0;
  	          u_sterzo = 0.0f;
  	      }

  	      //Invio gradi calcolati al servomotore
  	        servo_motor((float)u_sterzo);

  	        //Registrazione successiva alla sterzata
  	        if (recording_state == 0 && fabs(yaw_rate_ref) > 0.3f) {
  	            recording_state = 1;
  	        }

  	        //Registrazione in memoria
  	        if (recording_state == 1) {
  	            if (log_index < MAX_SAMPLES) {
  	                log_ref[log_index] = yaw_rate_ref;
  	                log_real[log_index] = vehicleState.yaw_rate_rad_sec;
  	                log_index++;
  	            } else {
  	                recording_state = 2; //Memoria piena, passa alla stampa
  	            }
  	        }
  	  }
  	//Stampa dati velocità su UART
  	if (speed_recording_state == 2) {
  	    set_PWM_and_dir(0, 0);
  	    servo_motor(0);

  	    printf("\r\n--- INIZIO DATI VELOCITA' [m/s] ---\r\n");
  	    printf("ref_m_s,real_m_s\r\n");
  	    for (int i = 0; i < SPEED_SAMPLES; i++) {
  	        printf("%.4f,%.4f\r\n", log_speed_ref[i], log_speed_real[i]);
  	        HAL_Delay(5);
  	    }
  	    printf("--- FINE DATI VELOCITA' ---\r\n");

  	    speed_recording_state = 10;  // Fine. Reset
  	}
  	//Registrazione terminata, ferma l'auto e stampa
  	    if (recording_state == 2) {
  	        set_PWM_and_dir(0, 0);
  	        servo_motor(0);

  	        printf("\r\n--- INIZIO DATI TELEMETRIA ---\r\n");
  	        for (int i = 0; i < MAX_SAMPLES; i++) {
  	            printf("%.3f,%.3f\r\n", log_ref[i], log_real[i]);
  	            HAL_Delay(5); //Evita di sovraccaricare il terminale seriale in ricezione
  	        }
  	        printf("--- FINE DATI ---\r\n");

  	        recording_state = 10; // Finito. Per rifare il test, premi il tasto Reset nero sulla scheda
  	    }
  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
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
  htim4.Init.Prescaler = 0;
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
        time_counter++;

        if (time_counter == 200) {
            data.curvature_radius_ref_m = 1.3;
            pid_steering.Iterm = 0;
            pid_steering.e_old = 0;
        }
        if (time_counter == 580) {
            data.enable = 0;
        }
    }
}

//USART2 -> ST_Link UART for DEBUG with USB
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
{
    // STERZO -> PA3 (TIM5 CH4)
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        static uint32_t ic_val1_s = 0;
        uint32_t ic_val2_s = 0;
        uint32_t diff_steering = 0;

        if (Is_First_Captured_Steering == 0)
        {
            ic_val1_s = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            Is_First_Captured_Steering = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2_s = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

            if (ic_val2_s > ic_val1_s)
                diff_steering = ic_val2_s - ic_val1_s;
            else
                diff_steering = (0xFFFFFFFF - ic_val1_s) + ic_val2_s;

            // Per lo sterzo FIR
            if (diff_steering > 600 && diff_steering < 2400) {
                steering_us = diff_steering;
                new_rc_steering = 1;
            }

            Is_First_Captured_Steering = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    //Trazione -> PA0 (TIM5 CH1)
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        static uint32_t ic_val1_t = 0;
        uint32_t ic_val2_t = 0;
        uint32_t diff_throttle = 0;

        if (Is_First_Captured_Throttle == 0)
        {
            ic_val1_t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured_Throttle = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2_t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (ic_val2_t > ic_val1_t)
                diff_throttle = ic_val2_t - ic_val1_t;
            else
                diff_throttle = (0xFFFFFFFF - ic_val1_t) + ic_val2_t;


            if (diff_throttle > 600 && diff_throttle < 2400) {
                throttle_us = diff_throttle;
                new_rc_throttle = 1;
            }

            Is_First_Captured_Throttle = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
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
