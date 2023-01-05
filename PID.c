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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
void pidr(void);
void pidg(void);
void pidg_arriere(void);
void pidr_arriere(void);
void stop(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// DROITE: KP 5 KI 35 ---------- GAUCHE KP 5 KI 37

float  consigne=1.5;
float  consigneg=1.5;
double erreur=0;
double erreurg=0;


//AVANT
float Kp=5;
float Ki=35;

float Kpg=5;
float Kig=37;

// ARRIERE
float kp_droite_arriere=5;
float ki_droite_arriere=35;

float  kp_gauche_arriere=5;
float  ki_gauche_arriere=37;


double cmd=0;
double cmdg=0;
double somme_erreur=0;
double somme_erreurg=0;

int rapport=0;
int rapportg=0;
float delta =0;
float deltag =0;
float erreurpred=0;
float erreurpredg=0;
float kd=0;
float kdg=0;


float vitesseM=0;
float vitesseMg=0;
float vitesseMprec=0;
float vitesseMprecg=0;
float vitesseF=0;
float vitesseFg=0;
float vitesseFprec=0;
float vitesseFprecg=0;
long ticks_droite=0;
long ticks_gauche=0;
int MOTOR_STOP=0;
int distance=0;




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2)
		ticks_droite++;
	else if(htim== &htim5)
		ticks_gauche++;


}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

if (MOTOR_STOP==0 || MOTOR_STOP==2) {
pidr();
pidg();}
else {
	pidr_arriere();
	pidg_arriere();


}


}

void pidr(){

	erreur=consigne-vitesseF;

		somme_erreur += erreur*0.001;
		cmd= Kp*erreur +Ki*somme_erreur+kd*delta;


		if(cmd<=0 ) { TIM1->CCR1=0;   }
		//else if(cmd<= 350 ) TIM1->CCR1=350;
		else if(cmd >= 12) TIM1->CCR1 = TIM1->ARR;


		rapport = (TIM1->ARR * cmd) / 12;

		TIM1->CCR1=rapport;

}

void pidr_arriere(){

	erreur=consigne-vitesseF;

		somme_erreur += erreur*0.001;

		cmd= kp_droite_arriere*erreur +ki_droite_arriere*somme_erreur;


		if(cmd<=0 ) { TIM1->CCR2=0;   }
		//else if(cmd<= 350 ) TIM1->CCR1=350;
		else if(cmd >= 12) TIM1->CCR2 = TIM1->ARR;


		rapport = (TIM1->ARR * cmd) / 12;

		TIM1->CCR2=rapport;

}

void pidg(){


erreurg=consigneg-vitesseFg;

	somme_erreurg += erreurg*0.001;
	cmdg= Kpg*erreurg +Kig*somme_erreurg+kdg*deltag;



	if(cmdg<=0 ) { TIM3->CCR1=0;   }
	//else if(cmd<= 350 ) TIM1->CCR1=350;
	else if(cmdg >= 12) TIM3->CCR1 = TIM3->ARR;


	rapportg = (TIM3->ARR * cmdg) / 12;

	TIM3->CCR1=rapportg;


}


void pidg_arriere(){


erreurg=consigneg-vitesseFg;

	somme_erreurg += erreurg*0.001;

	cmdg= kp_gauche_arriere*erreurg +ki_gauche_arriere*somme_erreurg;



	if(cmdg<=0 ) { TIM3->CCR2=0;   }
	//else if(cmd<= 350 ) TIM1->CCR1=350;
	else if(cmdg >= 12) TIM3->CCR2 = TIM3->ARR;


	rapportg = (TIM3->ARR * cmdg) / 12;

	TIM3->CCR2=rapportg;


}

void stop_avant() {

	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1 );
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1 );
	HAL_TIM_Base_Stop_IT(&htim14);

}
void stop_arriere() {

	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2 );
	HAL_TIM_Base_Stop_IT(&htim14);

}
void correction_avant(){

	if (ticks_droite !=ticks_gauche) {
					if(ticks_droite<ticks_gauche){
						HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1 );
						while(ticks_droite<ticks_gauche)
								TIM1->CCR1=TIM1->ARR*0.6;

						HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1 );
							}


					if(ticks_droite>ticks_gauche){
									HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1 );
									while(ticks_droite>ticks_gauche)
											TIM3->CCR1=TIM1->ARR*0.5;
									HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1 );
										}

				}

}
void correction_arriere(){
	if (ticks_droite !=ticks_gauche) {
							if(ticks_droite<ticks_gauche){
								HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2 );
								while(ticks_droite<ticks_gauche)
										TIM1->CCR2=TIM1->ARR*0.5;

								HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2 );
									}


							if(ticks_droite>ticks_gauche){
											HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2 );
											while(ticks_droite>ticks_gauche)
													TIM3->CCR2=TIM1->ARR*0.6;
											HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2 );
												}

						}

}

void start_arriere(){


			ticks_gauche=0;
			ticks_droite=0;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2 );
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2 );
			HAL_TIM_Base_Start_IT(&htim14);
}

void vitesse(){

	if(TIM2->CCR1==0) {vitesseM=0; vitesseF=0;}
				  	  else {
				  		  vitesseMprec=vitesseM;
				  		  vitesseM= 16000000 / ( (float)(TIM2->CCR1+1) * 1200);
				  		  if (vitesseM >= 2) {  vitesseM =vitesseMprec;}
				  		  vitesseF=0.0004998*vitesseM + 0.0004998*vitesseMprec+ 0.999*vitesseFprec;
				  		  vitesseFprec=vitesseF;

				  	  	  	}
	if(TIM5->CCR1==0) {vitesseMg=0;
				  	  	vitesseFg=0;
				  	  	  	  	  	  	  	  	  	  	  	  }
				  	  	  else  {
				  	  		  vitesseMprecg=vitesseMg;
				  	  		  vitesseMg= 16000000 / ( (float)(TIM5->CCR1+1) * 1200);
				  	  		  if (vitesseMg >= 2) {  vitesseMg =vitesseMprecg;}
				  	  		  vitesseFg=0.0004998*vitesseMg + 0.0004998*vitesseMprecg+ 0.999*vitesseFprecg;
				  	  		  vitesseFprecg=vitesseFg;

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
HAL_Delay(1000);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1 );
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1 );

HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
HAL_TIM_Base_Start_IT(&htim14);

distance= (int)1 * 5350;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// 1 metre = 5350
	  vitesse();
if(  (ticks_gauche>= distance || ticks_droite>=distance) && MOTOR_STOP==0 ) {


		stop_avant();
		HAL_Delay(200);
		correction_avant();
		HAL_Delay(1000);
		start_arriere();
		MOTOR_STOP=1;

	}

if(  (ticks_gauche>= distance || ticks_droite>=distance) && MOTOR_STOP==1   )
{


	stop_arriere();
	HAL_Delay(200);
	correction_arriere();
while(1) {

	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2 );

}
/*
ticks_gauche=0; ticks_droite=0;
MOTOR_STOP=2;
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1 );
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1 );
HAL_TIM_Base_Start_IT(&htim14);*/

}

/*
if(  (ticks_gauche>= 5350 || ticks_droite>=5350) && MOTOR_STOP==2 ) {


			stop();
			HAL_Delay(1000);
			correction_avant();
			HAL_Delay(1000);
			ticks_gauche=0;
			ticks_droite=0;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2 );
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2 );
			HAL_TIM_Base_Start_IT(&htim14);

			MOTOR_STOP=3;

	}

if(  (ticks_gauche>= 5350 || ticks_droite>=5350) && MOTOR_STOP==3   )
{


	HAL_TIM_Base_Stop_IT(&htim14);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2 );
	HAL_Delay(500);
	correction_arriere();


while(1){
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2 );

}

}*/


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 799;
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 799;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 15999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

