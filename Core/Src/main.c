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
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
volatile uint8_t requested_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NOTE_A3 220
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880

typedef struct { uint16_t freq; uint16_t ms; } Note;

static void Buzzer_Play(const Note *song, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++) {
    uint16_t f = song[i].freq;
    if (f == 0U) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);   /* 休止符 */
    } else {
      uint32_t arr = (1000000UL / f) - 1U;                /* 计数时钟 1MHz (PSC=83) / f - 1 */
      __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (arr + 1U) / 2U);
    }
    HAL_Delay(song[i].ms);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);       /* 断音间隔 */
    HAL_Delay(30);
  }
}

/* 《大鱼》  */
static const Note s_bigfish[] = {
  /* 海浪无声将夜幕深深淹没 */
  {NOTE_A3,300},{NOTE_C4,300},{NOTE_C4,300},{NOTE_D4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_G4,300},{NOTE_A4,450},
  {NOTE_G4,300},{NOTE_E4,300},{NOTE_D4,500},{0,150},
  /* 漫过天空尽头的角落 */
  {NOTE_A3,300},{NOTE_C4,300},{NOTE_C4,300},{NOTE_D4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_A4,450},{NOTE_G4,500},{0,150},
  /* 大鱼在梦境的缝隙里游过 */
  {NOTE_A3,300},{NOTE_C4,300},{NOTE_C4,300},{NOTE_D4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_G4,300},{NOTE_A4,450},
  {NOTE_G4,300},{NOTE_E4,300},{NOTE_D4,500},{0,150},
  /* 凝望你沉睡的轮廓 */
  {NOTE_D4,300},{NOTE_E4,300},{NOTE_A4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_A4,300},{NOTE_G4,400},{NOTE_A4,700},{0,150},
  /* 看海天一色 听风起雨落 */
  {NOTE_A4,300},{NOTE_C5,300},{NOTE_D5,300},{NOTE_C5,300},{NOTE_A4,500},
  {NOTE_A4,300},{NOTE_A4,300},{NOTE_C5,300},{NOTE_D5,300},{NOTE_C5,500},
  /* 执子手吹散苍茫茫烟波 */
  {NOTE_D5,300},{NOTE_E5,300},{NOTE_E5,300},{NOTE_G5,300},{NOTE_A5,300},{NOTE_A5,300},{NOTE_G5,300},{NOTE_E5,300},{NOTE_D5,300},{NOTE_C5,500},{0,150},
  /* 大鱼的翅膀 已经太辽阔 */
  {NOTE_D5,300},{NOTE_C5,300},{NOTE_A4,300},{NOTE_A4,300},{NOTE_C5,300},{NOTE_D5,300},{NOTE_C5,400},{NOTE_A4,700},{0,150},
  /* 我松开时间的绳索 */
  {NOTE_D5,300},{NOTE_E5,300},{NOTE_A4,300},{NOTE_D5,300},{NOTE_E5,300},{NOTE_A4,300},{NOTE_G4,400},{NOTE_A4,700},{0,150},
  /* 怕你飞远去 怕你离我而去 */
  {NOTE_C5,400},{NOTE_B4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,400},
  {NOTE_C4,300},{NOTE_C4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,500},
  /* 更怕你永远停留在这里 */
  {NOTE_C4,300},{NOTE_A4,300},{NOTE_C5,300},{NOTE_B4,300},{NOTE_A4,300},{NOTE_G4,300},{NOTE_D4,400},{NOTE_E4,700},{0,300},
  /* 每一滴泪水都向你流淌去 */
  {NOTE_C5,300},{NOTE_B4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,400},{NOTE_C4,300},{NOTE_D4,300},{NOTE_E4,600},
  /* 倒流进天空的海底 */
  {NOTE_D4,300},{NOTE_E4,300},{NOTE_A4,300},{NOTE_D5,300},{NOTE_E5,300},{NOTE_A4,400},{NOTE_G4,300},{NOTE_A4,900},
  /* 收尾留白 */
  {0,500},
};

/* 《小星星》 */
static const Note s_twinkle[] = {
  {NOTE_C4,300},{NOTE_C4,300},{NOTE_G4,300},{NOTE_G4,300},{NOTE_A4,300},{NOTE_A4,300},{NOTE_G4,600},
  {NOTE_F4,300},{NOTE_F4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,300},{NOTE_D4,300},{NOTE_C4,600},
  {NOTE_G4,300},{NOTE_G4,300},{NOTE_F4,300},{NOTE_F4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,600},
  {NOTE_G4,300},{NOTE_G4,300},{NOTE_F4,300},{NOTE_F4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,600},
  {NOTE_C4,300},{NOTE_C4,300},{NOTE_G4,300},{NOTE_G4,300},{NOTE_A4,300},{NOTE_A4,300},{NOTE_G4,600},
  {NOTE_F4,300},{NOTE_F4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_D4,300},{NOTE_D4,300},{NOTE_C4,600},
  {0,400},
};

/* 十二 */
static void UpdateLed(uint8_t mode, uint32_t now)
{
  static uint8_t  last_mode = 0xFF;
  static uint32_t last_tick = 0;
  static int16_t  breath = 0;
  static int8_t   dir = 1;
  static uint8_t  blink_on = 0;

  if (mode != last_mode) {
    last_mode = mode;
    last_tick = now;
    breath = 0; dir = 1; blink_on = 1;
    if (mode == 0) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    if (mode == 2) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 500);
  }

  switch (mode) {
    case 0:  
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
      break;

    case 1:
      if ((now - last_tick) >= 10U) {
        last_tick = now;
        breath = (int16_t)(breath + dir * 10);
        if (breath >= 1000) { breath = 1000; dir = -1; }
        else if (breath <= 0) { breath = 0; dir = 1; }
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)breath);
      }
      break;

    case 2:
      if ((now - last_tick) >= 500U) {
        last_tick = now;
        blink_on = (uint8_t)!blink_on;
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, blink_on ? 500U : 0U);
      }
      break;

    default:
      break;
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
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_PRESCALER(&htim4, 83);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    UpdateLed(requested_mode, HAL_GetTick());
    HAL_IWDG_Refresh(&hiwdg);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == KEY_Pin) {
    static uint32_t last_key_tick = 0;
    static uint8_t  has_last_key_tick = 0;
    uint32_t now = HAL_GetTick();


    if (!has_last_key_tick || (now - last_key_tick) >= 30U) {
      has_last_key_tick = 1;
      last_key_tick = now;
      requested_mode = (requested_mode + 1U) % 3U;   /* 0 -> 1 -> 2 -> 0 */
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
