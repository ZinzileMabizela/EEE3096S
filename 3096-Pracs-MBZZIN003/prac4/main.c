/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 128       // Number of samples in LUT
#define TIM2CLK  8000000 // STM Clock frequency
#define F_SIGNAL 5 // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {511.500000000000, 536.795577740481, 562.029252963414, 587.139274637083, 612.064194330731, 636.743016589161, 661.115348198840, 685.121545980211, 708.702862744550, 731.801591058178, 754.361204462223, 776.326495802338, 797.643712329858, 818.260687243782, 838.126967351679, 857.193936537103, 875.414934731377, 892.745372098600, 909.142838154448, 924.567205551733, 938.980728278752, 952.348134030095, 964.636710523894, 975.816385554252, 985.859800582977, 994.742377690502, 1002.44237972218, 1008.94096348274, 1014.22222584874, 1018.27324268618, 1021.08410047802, 1022.64792058424, 1022.96087607497, 1022.02220109569, 1019.83419274138, 1016.40220543511, 1011.73463782494, 1005.84291223095, 998.741446692954, 990.447619687136, 980.981727597983, 970.366935049652, 958.629218218243, 945.797301263751, 931.902586037226, 916.979075235185, 901.063289189317, 884.194176495105, 866.413018698097, 847.763329271036, 828.290747129105, 808.042924943856, 787.069412529133, 765.421535584379, 743.152270092042, 720.316112676476, 696.968947241567, 673.167908213453, 648.971240723009, 624.438158070258, 599.628696819495, 574.603569879758, 549.424017930167, 524.151659553717, 498.848340446283, 473.575982069833, 448.396430120242, 423.371303180506, 398.561841929742, 374.028759276991, 349.832091786548, 326.031052758433, 302.683887323524, 279.847729907959, 257.578464415622, 235.930587470867, 214.957075056144, 194.709252870895, 175.236670728964, 156.586981301903, 138.805823504895, 121.936710810684, 106.020924764815, 91.0974139627743, 77.2026987362492, 64.3707817817570, 52.6330649503487, 42.0182724020169, 32.5523803128641, 24.2585533070460, 17.1570877690547, 11.2653621750647, 6.59779456488962, 3.16580725862273, 0.977798904305668, 0.0391239250293357, 0.352079415763131, 1.91589952197875, 4.72675731382473, 8.77777415126474, 14.0590365172628, 20.5576202778202, 28.2576223094977, 37.1401994170231, 47.1836144457478, 58.3632894761062, 70.6518659699047, 84.0192717212481, 98.4327944482666, 113.857161845552, 130.254627901400, 147.585065268623, 165.806063462897, 184.873032648321, 204.739312756218, 225.356287670142, 246.673504197662, 268.638795537777, 291.198408941822, 314.297137255450, 337.878454019789, 361.884651801160, 386.256983410839, 410.935805669269, 435.860725362916, 460.970747036586, 486.204422259519, 511.500000000000};

uint32_t saw_LUT[NS] = {0, 8.05511811023622, 16.1102362204724, 24.1653543307087, 32.2204724409449, 40.2755905511811, 48.3307086614173, 56.3858267716535, 64.4409448818898, 72.4960629921260, 80.5511811023622, 88.6062992125984, 96.6614173228347, 104.716535433071, 112.771653543307, 120.826771653543, 128.881889763780, 136.937007874016, 144.992125984252, 153.047244094488, 161.102362204724, 169.157480314961, 177.212598425197, 185.267716535433, 193.322834645669, 201.377952755906, 209.433070866142, 217.488188976378, 225.543307086614, 233.598425196850, 241.653543307087, 249.708661417323, 257.763779527559, 265.818897637795, 273.874015748031, 281.929133858268, 289.984251968504, 298.039370078740, 306.094488188976, 314.149606299213, 322.204724409449, 330.259842519685, 338.314960629921, 346.370078740157, 354.425196850394, 362.480314960630, 370.535433070866, 378.590551181102, 386.645669291339, 394.700787401575, 402.755905511811, 410.811023622047, 418.866141732283, 426.921259842520, 434.976377952756, 443.031496062992, 451.086614173228, 459.141732283465, 467.196850393701, 475.251968503937, 483.307086614173, 491.362204724409, 499.417322834646, 507.472440944882, 515.527559055118, 523.582677165354, 531.637795275591, 539.692913385827, 547.748031496063, 555.803149606299, 563.858267716536, 571.913385826772, 579.968503937008, 588.023622047244, 596.078740157480, 604.133858267717, 612.188976377953, 620.244094488189, 628.299212598425, 636.354330708662, 644.409448818898, 652.464566929134, 660.519685039370, 668.574803149606, 676.629921259843, 684.685039370079, 692.740157480315, 700.795275590551, 708.850393700787, 716.905511811024, 724.960629921260, 733.015748031496, 741.070866141732, 749.125984251969, 757.181102362205, 765.236220472441, 773.291338582677, 781.346456692913, 789.401574803150, 797.456692913386, 805.511811023622, 813.566929133858, 821.622047244095, 829.677165354331, 837.732283464567, 845.787401574803, 853.842519685039, 861.897637795276, 869.952755905512, 878.007874015748, 886.062992125984, 894.118110236220, 902.173228346457, 910.228346456693, 918.283464566929, 926.338582677165, 934.393700787402, 942.448818897638, 950.503937007874, 958.559055118110, 966.614173228346, 974.669291338583, 982.724409448819, 990.779527559055, 998.834645669291, 1006.88976377953, 1014.94488188976, 1023};

uint32_t triangle_LUT[NS] = {0, 16.2380952380952, 32.4761904761905, 48.7142857142857, 64.9523809523810, 81.1904761904762, 97.4285714285714, 113.666666666667, 129.904761904762, 146.142857142857, 162.380952380952, 178.619047619048, 194.857142857143, 211.095238095238, 227.333333333333, 243.571428571429, 259.809523809524, 276.047619047619, 292.285714285714, 308.523809523810, 324.761904761905, 341, 357.238095238095, 373.476190476191, 389.714285714286, 405.952380952381, 422.190476190476, 438.428571428571, 454.666666666667, 470.904761904762, 487.142857142857, 503.380952380952, 519.619047619048, 535.857142857143, 552.095238095238, 568.333333333333, 584.571428571429, 600.809523809524, 617.047619047619, 633.285714285714, 649.523809523810, 665.761904761905, 682, 698.238095238095, 714.476190476191, 730.714285714286, 746.952380952381, 763.190476190476, 779.428571428571, 795.666666666667, 811.904761904762, 828.142857142857, 844.380952380952, 860.619047619048, 876.857142857143, 893.095238095238, 909.333333333333, 925.571428571429, 941.809523809524, 958.047619047619, 974.285714285714, 990.523809523810, 1006.76190476190, 1023, 1023, 1006.76190476190, 990.523809523810, 974.285714285714, 958.047619047619, 941.809523809524, 925.571428571429, 909.333333333333, 893.095238095238, 876.857142857143, 860.619047619048, 844.380952380952, 828.142857142857, 811.904761904762, 795.666666666667, 779.428571428571, 763.190476190476, 746.952380952381, 730.714285714286, 714.476190476191, 698.238095238095, 682, 665.761904761905, 649.523809523810, 633.285714285714, 617.047619047619, 600.809523809524, 584.571428571429, 568.333333333333, 552.095238095238, 535.857142857143, 519.619047619048, 503.380952380952, 487.142857142857, 470.904761904762, 454.666666666667, 438.428571428571, 422.190476190476, 405.952380952381, 389.714285714286, 373.476190476191, 357.238095238095, 341, 324.761904761905, 308.523809523810, 292.285714285714, 276.047619047619, 259.809523809524, 243.571428571429, 227.333333333333, 211.095238095238, 194.857142857143, 178.619047619048, 162.380952380952, 146.142857142857, 129.904761904762, 113.666666666667, 97.4285714285714, 81.1904761904762, 64.9523809523810, 48.7142857142857, 32.4761904761905, 16.2380952380952, 0};

// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK/(F_SIGNAL*NS); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
uint32_t currentDebounce = 0;
int count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
  lcd_command(CLEAR);
  lcd_putstring("Sine");
  delay(3000);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()
	if(HAL_GetTick() - currentDebounce > 100){
			 count += 1;
			 if (count==4){
				 count = 1;
			 }


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1); //Disable the DMA transfer
	HAL_DMA_Abort_IT(&hdma_tim2_ch1); //Abort IT mode

	//Switch between the waveforms
		switch(count){
		case 1:
			HAL_DMA_Start_IT(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);
			lcd_command(CLEAR);
			lcd_putstring("Sawtooth");
			delay(3000);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);//Re-enable transfer
			break;
		case 2:
			HAL_DMA_Start_IT(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);
			lcd_command(CLEAR);
			lcd_putstring("Triangle");
			delay(3000);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);//Re-enable transfer
			break;
		case 3:
			HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);
			lcd_command(CLEAR);
			lcd_putstring("Sine");
			delay(3000);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);//Re-enable transfer
			break;
		}
		currentDebounce = HAL_GetTick();
	}
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
