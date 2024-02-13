/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
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
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1600 // corespond à 10 cm.
#define Seuil_Dist_3 1600
#define Seuil_Dist_1 1600
#define Seuil_Dist_2 1600
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S     1000 //( pwm période = 2 ms )
#define T_200_MS  100
#define T_2000_MS 1000
#define CKp_D 100  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50

#define PWM_180 1500
#define PWM_90  2700
#define PWM_0   3900

#define TURN_SERVO_TIME 600
#define TURN_LEFT_TIME  1150
#define TURN_RIGHT_TIME 1150
#define FORWARD_50_TIME 1000

#define cptRota 2000

enum CMDE {
	START,
	STOP,
	STOP_ARRET,
	AVANT,
	ARRIERE,
	DROITE,
	GAUCHE,
	SERVO_ROTATE_0,
	SERVO_ROTATE_90,
	SERVO_ROTATE_180,
	PARK,
	MOV_PARK
};

enum MODE {
	SLEEP, ACTIF
};

enum Mesure_State {
	SERVO_ROTATE_Y,
	MESURE_Y,
	SERVO_ROTATE_X,
	MESURE_X,
	SERVO_ROTATE_Z,
	MESURE_Z,
	END
};

enum MovPark_State {
	FORWARD,
	POSITION,
	TURN_R1,
	TURN_L1,
	TURN_R2,
	TURN_L2,
	FORWARD_Y,
	FORWARD_Z,
	FORWARD_X,
	FINISH
};

enum Mean_Distance_State {
	WAIT_MESURE,
	MEAN_MESURE
};

enum Mean_Distance_State Distance_State = WAIT_MESURE;
enum MovPark_State MovPark_State = FINISH;
volatile enum CMDE CMDE;
enum Mesure_State Mesure_State = END;
volatile enum MODE Mode;

volatile unsigned char New_CMDE = 0;
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4;
volatile unsigned int Time = 0;
volatile unsigned int Tech = 0;
uint16_t adc_buffer[10];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX;

uint16_t _DirG, _DirD, CVitG, CVitD, DirD, DirG;
uint16_t _CVitD = 0;
uint16_t _CVitG = 0;
uint16_t VitD, VitG;
int16_t DistD, DistG;
int16_t DistD_old = 0;
int16_t DistG_old = 0;
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
volatile uint32_t Dist_Obst_;
volatile uint32_t Dist_Obst_cm;
volatile uint32_t Derniere_Dist_Obst_cm;

// ADDED VARIABLES
uint32_t Dist;
volatile uint32_t vbatt; //  STOCKER LA TENSION MESURÉE DE LA BATTERIE
// REPÉRAGE SPATIALE DU ROBOT
static uint32_t z0;
static uint32_t x0;
static uint32_t y0;
static uint32_t z;
static uint32_t x;
static uint32_t y;
// COMPTEURS POUR LE DÉPLACEMENT
volatile uint16_t TurnServoCounter = 0;
volatile uint16_t TurnCounter = 0;
volatile uint16_t tour_dist = 0;
static uint8_t tab_elem = 0;
// SIGNAUX DE CONTRÔLE DES ACTIONS DU ROBOT
volatile uint8_t Mesure_On   = 0;
volatile uint8_t Mesure_Park_On   = 0;
volatile uint8_t Mean         = 1;
volatile uint8_t MovPark_On   = 0;
volatile uint8_t TurnRightOn = 0;
volatile uint8_t TurnLeftOn  = 0;
// MESURE DISTANCE SONAR
volatile uint32_t Dist_Obst_cm_moy;
volatile uint32_t Dist_cm[5] = {0, 0, 0, 0, 0};
// ZIGBEE
volatile uint8_t buffer[3] = {1, 2, 3};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);
void SonarStart(void);
void ServoAngle(int16_t angle);
void MesurePOsition(void);
void MES_PARK(void);
void TurnRight(void);
void TurnLeft(void);
void MOVPARK(void);
void SonarDistance(void);
void ENVOYER(void);
void SonarStop(void);
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
  Dist_Obst = 0;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_SuspendTick(); // suppresion des Tick interrupt pour le mode sleep.
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  CMDE = STOP;
  New_CMDE = 1;
  HAL_TIM_Base_Start_IT(&htim2);  // Start IT sur font montant PWM
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  SonarStart();
  for (uint8_t i=0; i<5; i++){
	  SonarDistance();
  }

  while (1)
  {
	  Gestion_Commandes();
	  controle();
	  SonarDistance();

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
  RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void Gestion_Commandes(void) {
	enum ETAT {
		VEILLE,
		ARRET,
		AV1,
		AV2,
		AV3,
		RV1,
		RV2,
		RV3,
		DV1,
		DV2,
		DV3,
		GV1,
		GV2,
		GV3
	};
	static enum ETAT Etat = VEILLE;

	if (MovPark_On)
		MOVPARK();

	if (Mesure_Park_On)
		MES_PARK();


if (New_CMDE) {
		New_CMDE = 0;
	switch (CMDE) {
		case STOP_ARRET: {
			// STOP SANS METTRE EN VEILLE
			_CVitD = 0;
			_CVitG = 0;
			Etat   = ARRET;
			Mode   = SLEEP;
			break;
		}
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , réveil via IT BP1
			Etat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// réveil sytème grace à l'IT BP1
			Etat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG  = AVANCE;
				_DirD  = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat   = AV1;
				Mode   = ACTIF;
				break;
			}
			case AV1: {
				_DirG  = AVANCE;
				_DirD  = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat   = AV2;
				Mode   = ACTIF;
				break;
			}
			case AV2: {
				_DirG  = AVANCE;
				_DirD  = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat   = AV3;
				Mode   = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat   = AV3;
				Mode   = ACTIF;
				break;
			}
			case RV1: {
				_DirG  = RECULE;
				_DirD  = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat   = ARRET;
				Mode   = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case ARRIERE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case DROITE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case GAUCHE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}

		}
			break;
		}
		case SERVO_ROTATE_0: {
			ServoAngle(0);
			break;
		}
		case SERVO_ROTATE_90: {
			ServoAngle(90);
			break;
		}
		case SERVO_ROTATE_180: {
			ServoAngle(180);
			break;
		}
		case PARK: {
			//Mesure_On = 1;
			Mesure_Park_On = 1;
			Mesure_State = SERVO_ROTATE_Y;
			break;
		}
		case MOV_PARK: {
			MovPark_On       = 1;
			MovPark_State    = FORWARD;
			TurnServoCounter = 0;
			break;
		}
	}
}
}
void controle(void) {

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}

}

void ACS(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF )
			Etat = ACTIF;
		else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIF: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

void Calcul_Vit(void) {

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF)
			Etat = ACTIF;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIF: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);

		}
		break;
	}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F': {
			CMDE = AVANT;
			//New_CMDE = 1;
			break;
		}

		case 'B': {
			CMDE = ARRIERE;
			//New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			//New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			//New_CMDE = 1;
			break;
		}

		case 'D':{
			// disconnect bluetooth
			break;
		}
		case 'Z': {
			CMDE = SERVO_ROTATE_0;
			//New_CMDE = 1;
			break;
		}
		case 'M': {
			CMDE = SERVO_ROTATE_90;
			//New_CMDE = 1;
			break;
		}
		case 'P': {
			CMDE = SERVO_ROTATE_180;
			//New_CMDE = 1;
			break;
		}
		case 'S': {
			CMDE = STOP_ARRET;
			//New_CMDE = 1;
			break;
		}
		case 'U': {
			CMDE = PARK;
			//New_CMDE = 1;
			break;
		}
		case 'W': {
			CMDE = MOV_PARK;
			New_CMDE = 1;
			break;
		}
		default:
			New_CMDE = 1;
		}

		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	Dist_ACS_3 = adc_buffer[0] - adc_buffer[5];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[8];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[6];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[7];
	vbatt = adc_buffer[4];
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if ( htim->Instance == TIM2) {
		cpt++;
		Time++;
		Tech++;

		// Compteurs ajoutés
		TurnServoCounter++;
		tour_dist++;
		TurnCounter++;

		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE)
		CMDE = STOP;
	else
		CMDE = START;
	TOGGLE = ~TOGGLE;
	New_CMDE = 1;
}

// LED TÉMOIN BATTERIE
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		Dist_Obst = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
		Dist_Obst_ = Dist_Obst;
		Dist_Obst_cm = 645*Dist_Obst_/65535;
	}
}

void SonarStart(void){
	HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, GPIO_PIN_SET);
}

void SonarStop(void){
	HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, GPIO_PIN_RESET);
}

void ServoAngle(int16_t angle){
	switch(angle){
			case 0:{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t ) PWM_0);
				break;
			}
			case 90:{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t ) PWM_90);
				break;
			}
			case 180:{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t ) PWM_180);
				break;
			}
			default:{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t ) PWM_90);
				break;
			}
	}
}


void MesurePosition(){
	switch (Mesure_State){
		case SERVO_ROTATE_Y: {
			ServoAngle(0);
			TurnServoCounter = 0;
			Mesure_State = MESURE_Y;
			break;
		}
		case MESURE_Y: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				z = Dist_Obst_cm;
				Mesure_State = SERVO_ROTATE_X;
			}
			break;
		}
		case SERVO_ROTATE_X: {
			ServoAngle(90);
			TurnServoCounter = 0;
			Mesure_State = MESURE_X;
			break;
		}
		case MESURE_X: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				x = Dist_Obst_cm;
				Mesure_State = SERVO_ROTATE_Z;
			}
			break;
		}
		case SERVO_ROTATE_Z: {
			ServoAngle(180);
			TurnServoCounter = 0;
			Mesure_State = MESURE_Z;
			break;
		}
		case MESURE_Z: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				y = Dist_Obst_cm;
				Mesure_State = END;
			}
			break;
		}
		case END: {
			ServoAngle(90);
			Mesure_On = 0;
		}

	}
}


void MES_PARK(){
	switch (Mesure_State){
		case SERVO_ROTATE_Y: {
			ServoAngle(0);
			TurnServoCounter = 0;
			Mesure_State = MESURE_Y;
			break;
		}
		case MESURE_Y: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				z0 = Dist_Obst_cm;
				Mesure_State = SERVO_ROTATE_X;
			}
			break;
		}
		case SERVO_ROTATE_X: {
			ServoAngle(90);
			TurnServoCounter = 0;
			Mesure_State = MESURE_X;
			break;
		}
		case MESURE_X: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				x0 = Dist_Obst_cm;
				Mesure_State = SERVO_ROTATE_Z;
			}
			break;
		}
		case SERVO_ROTATE_Z: {
			ServoAngle(180);
			TurnServoCounter = 0;
			Mesure_State = MESURE_Z;
			break;
		}
		case MESURE_Z: {
			if (TurnServoCounter >= TURN_SERVO_TIME){
				y0 = Dist_Obst_cm;
				Mesure_State = END;
			}
			break;
		}
		case END: {
			ServoAngle(90);
			Mesure_Park_On = 0;
		}

	}
}


void TurnRight(){
	if (TurnCounter >= TURN_RIGHT_TIME){
		CMDE = STOP_ARRET;
		New_CMDE = 1;
		TurnRightOn = 0;
	}
	else{
		CMDE = DROITE;
		New_CMDE = 1;
	}
}

void TurnLeft(){
	if (TurnCounter >= TURN_LEFT_TIME){
		CMDE = STOP_ARRET;
		New_CMDE = 1;
		TurnLeftOn = 0;
	}
	else{
		CMDE = GAUCHE;
		New_CMDE = 1;
	}
}

void MOVPARK(){
	switch (MovPark_State){
		case FORWARD: {
			if (TurnServoCounter <= FORWARD_50_TIME){
				CMDE = AVANT;
				New_CMDE = 1;
			}
			else {
				CMDE = STOP_ARRET;
				New_CMDE = 1;
				MovPark_State = POSITION;
				Mesure_State = SERVO_ROTATE_Y;
				Mesure_On = 1;
			}
			break;
		}
		case POSITION: {
			MesurePosition();
			if (!Mesure_On){
				if (y >= y0+30){
					MovPark_State = TURN_R1;
					TurnRightOn = 1;
				}
				else {
					MovPark_State = TURN_L1;
					TurnLeftOn = 1;
				}
				TurnCounter = 0;
			}
			break;
		}
		case TURN_R1: {
			TurnRight();
			if (!TurnRightOn)
				MovPark_State = FORWARD_Y;
			break;
		}
		case TURN_R2: {
			TurnRight();
			if (!TurnRightOn)
				MovPark_State = FORWARD_X;
			break;
		}
		case TURN_L1: {
			TurnLeft();
			if (!TurnLeftOn)
				MovPark_State = FORWARD_Z;
			break;
		}
		case TURN_L2: {
			TurnLeft();
			if (!TurnLeftOn)
				MovPark_State = FORWARD_X;
			break;
		}
		case FORWARD_X: {
			CMDE = AVANT;
			New_CMDE = 1;
			if ((int)(Dist_Obst_cm) <= x0+10)
				MovPark_State = FINISH;
			break;
		}
		case FORWARD_Y: {
			CMDE = AVANT;
			New_CMDE = 1;
			if ((int)(Dist_Obst_cm) <= y0-30){
				MovPark_State = TURN_L2;
				TurnLeftOn = 1;
				TurnCounter = 300;
			}
			break;
		}
		case FORWARD_Z: {
			CMDE = AVANT;
			New_CMDE = 1;
			if ((int)(Dist_Obst_cm) <= z0+30){
				MovPark_State = TURN_R2;
				TurnRightOn = 1;
				TurnCounter = 300;
			}
			break;
		}
		case FINISH: {
			SonarStop();
			CMDE = STOP_ARRET;
			New_CMDE = 1;
			MovPark_On = 0;
			break;
		}
	}
}

void SonarDistance(){
	switch(Distance_State){
		case WAIT_MESURE:{
			Distance_State = (tour_dist >= 50) ? MEAN_MESURE : WAIT_MESURE;
			break;
		}
		case MEAN_MESURE: {
			// À VOIR ET CORRIGER
			Dist_Obst_ = Dist_Obst;
			Dist_Obst_cm = 645*Dist_Obst_/65535;
			// MOYENNE SUR 5 MESURES DE SONAR
			if(Mean){
				Dist_cm[tab_elem] = Dist_Obst_cm;
				Dist_Obst_cm_moy = (Dist_cm[0]+Dist_cm[1]+Dist_cm[2]+Dist_cm[3])/4;
				tab_elem = (tab_elem+1)%4;
			}
			Distance_State = WAIT_MESURE;
			break;
		}
	}
}

void ENVOYER()
{
	HAL_UART_Transmit(&huart1, 'a', sizeof('a'), HAL_MAX_DELAY);
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
