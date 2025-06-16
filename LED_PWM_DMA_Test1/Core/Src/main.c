/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_LEDS 221 //221
#define DEBOUNCE_TIME_MS 100
#define NUM_EFFECTS 5 // 4 effects, 1 using max9814, 1 for off
#define CENTER_POINT MAX_LEDS/2

// chase eff
#define CHASE_SPEED_MS 10
#define TAIL_LENGTH    30
#define CHASE_COLOR_R  0
#define CHASE_COLOR_G  255
#define CHASE_COLOR_B  200


// blink eff
#define BLINK_SPEED_MS 100

#define ADC_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
uint8_t led_data[MAX_LEDS][3];
uint16_t pwm_data[MAX_LEDS * 24 + 50];

volatile int sent_flag = 1;
volatile int adc_led_map = 0;
volatile int effect_num = 0;
uint32_t last_button_press_time = 0;

volatile uint16_t adc_buf[ADC_BUFFER_SIZE];
volatile uint16_t sound_change_speed = 0;


void setColor(int LedNum, int Green, int Red, int Blue);
void sendWs2812(void);
void clearLed(void);

uint32_t Wheel(uint8_t WheelPos);
void rainbow_step(void);
void chasing_step(void);
void blinking_step(void);

static int displayed_num_leds_each_side = 0;
static uint32_t last_decay_time = 0;
const uint32_t DECAY_INTERVAL_MS = 3;
const int DECAY_STEP = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void process_adc_data(volatile uint16_t* data_ptr, uint16_t len) {

    static int previous_raw_loudness = 0;
    static int smoothed_loudness = 0;
    static int smoothed_speed = 0;

    const uint16_t noise_floor = 250;
    const uint16_t full_scale  = 3750;          //map to 255

    // EMA filter. Higher N = more smoothing (slower response).
    // smoothed_value = (previous_smoothed_value * (N-1) + new_raw_value) / N

	const int alpha_loud = 1.25;  		// Smoothing for loudness
    const int alpha_speed = 5;     	// Smoothing for speed

    // process interval
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 1) {
        return;
    }
    last = now;

    // raw peak to peak value
    uint16_t min_raw = 4095;
    uint16_t max_raw = 0;
    for (int i = 0; i < len; i++) {
        if (data_ptr[i] < min_raw) min_raw = data_ptr[i];
        if (data_ptr[i] > max_raw) max_raw = data_ptr[i];
    }
    uint16_t current_peak_to_peak = max_raw - min_raw;

    // loudness in range with noise floor and full scale 1000-2000 -> 0-255
    uint8_t current_raw_loudness;
    if (current_peak_to_peak <= noise_floor) {
        current_raw_loudness = 0;
    } else if (current_peak_to_peak >= full_scale) {
        current_raw_loudness = 255;
    } else {
        uint32_t value_in_range = current_peak_to_peak - noise_floor;
        uint32_t active_range = full_scale - noise_floor;
        current_raw_loudness = (uint8_t)((value_in_range * 255) / active_range);
    }

    // speed (if speed is high, the LEDs drop slowly, else fast)
    int current_raw_speed = current_raw_loudness - previous_raw_loudness;
    previous_raw_loudness = current_raw_loudness;

	//EMA Smoothing
    // Loudness:
    smoothed_loudness = ((smoothed_loudness * (alpha_loud - 1)) + current_raw_loudness) / alpha_loud;

    // Speed:
    smoothed_speed = ((smoothed_speed * (alpha_speed - 1)) + current_raw_speed) / alpha_speed;

    adc_led_map = smoothed_loudness;
    sound_change_speed = smoothed_speed;
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		process_adc_data(&adc_buf[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		process_adc_data(&adc_buf[0], ADC_BUFFER_SIZE/2);
	}
}


// --- Effect Implementations ---
uint32_t Wheel(uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return (uint32_t)(((255 - WheelPos * 3) << 16) | ((WheelPos * 3) << 8) | 0); // Green -> Red -> Blue = 0
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return (uint32_t)((0 << 16) | ((255 - WheelPos * 3) << 8) | (WheelPos * 3)); // Red -> Blue -> Green = 0
    }
    WheelPos -= 170;
    return (uint32_t)(((WheelPos * 3) << 16) | (0 << 8) | (255 - WheelPos * 3)); // Blue -> Green -> Red = 0
}

void rainbow_step() {
    static uint16_t rainbow_offset = 0; // Keeps track of the rainbow state

    for(int i=0; i<MAX_LEDS; i++) {
        uint8_t hue = (uint8_t)(((i * 256) / MAX_LEDS) + rainbow_offset); //uint_8t
        uint32_t color = Wheel(hue);

        uint8_t g = (color >> 16) & 0xFF;
        uint8_t r = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        setColor(i, g, r, b);
    }
    rainbow_offset++; // Move the rainbow along
}


void adc_vu_meter_with_decay_step() {
    clearLed();

    int current_loudness_metric = adc_led_map;
    int target_num_leds_each_side = ((uint32_t)current_loudness_metric * CENTER_POINT) / 255;

    if (target_num_leds_each_side > displayed_num_leds_each_side) {
        displayed_num_leds_each_side = target_num_leds_each_side;
        last_decay_time = HAL_GetTick();
    } else {
        uint32_t now = HAL_GetTick();
        if (now - last_decay_time >= DECAY_INTERVAL_MS) {
            last_decay_time = now;
            if (displayed_num_leds_each_side > 0) {
                displayed_num_leds_each_side -= DECAY_STEP;
                if (displayed_num_leds_each_side < 0) {
                    displayed_num_leds_each_side = 0;
                }
            }
        }
        // if (target_num_leds_each_side < displayed_num_leds_each_side && displayed_num_leds_each_side > 0) {
        //     if (displayed_num_leds_each_side > target_num_leds_each_side + DECAY_STEP) {
        //
        //     }
        // }
    }
    static uint16_t rainbow_offset = 0;
    static uint32_t last = 0;
	uint32_t now = HAL_GetTick();
	if (now - last > 50) {
		rainbow_offset++;
		last = now;
	}

    if (displayed_num_leds_each_side > 0) {
        for (int i = 0; i < displayed_num_leds_each_side; i++) {
        	uint8_t hue = (i * 255 * 4) / CENTER_POINT;
			uint32_t color = Wheel(hue + rainbow_offset);
			uint8_t g_val = (color >> 16) & 0xFF;
			uint8_t r_val = (color >> 8) & 0xFF;
			uint8_t b_val = color & 0xFF;
            int led_idx_low = CENTER_POINT - 1 - i;
            int led_idx_high = CENTER_POINT + i;

            if (led_idx_low >= 0) {
                setColor(led_idx_low, g_val, r_val, b_val);
            }
            if (led_idx_high < MAX_LEDS) {
                setColor(led_idx_high, g_val, r_val, b_val);
            }
        }
    }
}

void chasing_step() {
    static int chase_pos = 0; // Current position of the chase head
    static uint32_t last_chase_update = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_chase_update >= CHASE_SPEED_MS) {
         last_chase_update = now;

         for (int i = 0; i < MAX_LEDS; i++) {
             int distance_from_head = 0;
             int in_tail = 0;

             if (chase_pos >= i) {
                 distance_from_head = chase_pos - i;
             } else {
                 distance_from_head = chase_pos + (MAX_LEDS - i);
             }

             if (distance_from_head < TAIL_LENGTH) {
                 in_tail = 1;
             }

             if (in_tail) {
                 uint8_t brightness = 255 * (TAIL_LENGTH - distance_from_head) / TAIL_LENGTH;

                 uint8_t r = (CHASE_COLOR_R * brightness) / 255;
                 uint8_t g = (CHASE_COLOR_G * brightness) / 255;
                 uint8_t b = (CHASE_COLOR_B * brightness) / 255;
                 setColor(i, g, r, b);
             } else {
                 int prev_tail_end_pos = (chase_pos - TAIL_LENGTH + MAX_LEDS) % MAX_LEDS;
                 if (i == prev_tail_end_pos) {
                    setColor(i, 0, 0, 0);
                 }

             }
         }

         chase_pos++;
         if (chase_pos >= MAX_LEDS) {
             chase_pos = 0; // Wrap around
         }
    }
}

void led_stop() {
	clearLed();
}

void blinking_step() {
    static uint8_t blink_state = 0;       // 0 = off, 1 = on
    static uint32_t last_blink_update = 0;
    static uint8_t current_blink_hue = 0;
    uint32_t now = HAL_GetTick(); //get thoi gian hien tai, tru di tgian truoc do, polling lien tuc de coi co blink hay k

    if (now - last_blink_update >= BLINK_SPEED_MS) {
        last_blink_update = now;
        blink_state = !blink_state;

        if (blink_state) {
            uint32_t color = Wheel(current_blink_hue);
            uint8_t g = (color >> 16) & 0xFF;
            uint8_t r = (color >> 8) & 0xFF;
            uint8_t b = color & 0xFF;

            for (int i = 0; i < MAX_LEDS; i++) {
                setColor(i, g, r, b);
            }
        } else {
            clearLed();
        }
    }
    current_blink_hue = (current_blink_hue + 1) % 256;
}


// ngat ngoai chuyen eff
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        uint32_t now = HAL_GetTick(); //add debounce
        if (now - last_button_press_time > DEBOUNCE_TIME_MS) {
            last_button_press_time = now;
            effect_num++;

            if (effect_num >= NUM_EFFECTS) {
                effect_num = 0;
            }
            clearLed();
            sent_flag = 1;
        }
    }
}

// dma
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM1) {
          HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
          sent_flag = 1;
      }
}


void setColor(int LedNum, int Green, int Red, int Blue) {
    if (LedNum < MAX_LEDS) { // Basic bounds check
        led_data[LedNum][0] = (uint8_t)Green;
        led_data[LedNum][1] = (uint8_t)Red;
        led_data[LedNum][2] = (uint8_t)Blue;
    }
}


void sendWs2812() {
    if (sent_flag == 0) {
        return;
    }

    sent_flag = 0; //reset
    uint32_t color = 0;
    int indx = 0;
    for (int i = 0; i < MAX_LEDS; i++) {
        color = ((uint32_t)led_data[i][0]<<16) | ((uint32_t)led_data[i][1]<<8) | ((uint32_t)led_data[i][2]);
        for (int j = 23; j >= 0; j--) {
            if (color & (1UL<<j)) {
                 pwm_data[indx] = 60; //high
            } else {
                 pwm_data[indx] = 30; //low
            }
            indx++;
        }
    }

    for (int i = 0; i < 50; i++) {
        pwm_data[indx] = 0; // low signal 50us, bao hieu ket thuc tin hieu
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwm_data, indx);

}

void clearLed() {
    for (int i = 0; i < MAX_LEDS; i++) {
        setColor(i, 0, 0, 0);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_SIZE); // start DMA for ADC
  clearLed(); // Start with LEDs off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      switch(effect_num) {
        case 0:
            rainbow_step();
            sendWs2812();
            HAL_Delay(100);
            break;
        case 1:
            chasing_step();
            sendWs2812();
			HAL_Delay(1);
            break;
        case 2:
            blinking_step();
            sendWs2812();
			HAL_Delay(1);
            break;
        case 3:
        	adc_vu_meter_with_decay_step();
        	sendWs2812();
			HAL_Delay(1);
        	break;
        case 4:
        	led_stop();
        	break;
        default:
            clearLed();
            effect_num = 0;
            break;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  // TIM1 Clock = 72MHz (APB2)
  // Target WS2812 freq = 800kHz -> Period = 1.25us
  // Timer Period (ARR+1) = Clock / Target Freq = 72MHz / 800kHz = 90
  // ARR = 90 - 1 = 89
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90 - 1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
