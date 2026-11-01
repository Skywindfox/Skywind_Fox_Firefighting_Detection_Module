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

#include <stdio.h>
#include <string.h>
#include <cJSON.h>
#include <ds18b20.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ALL_GPIO_PORT   GPIOA
#define BUZZER_GPIO_PIN   GPIO_PIN_0                   				//无源蜂鸣器
#define LED_BLUE_GPIO_PIN   GPIO_PIN_1   							//蓝色LED：代表烟雾报警，低电平点亮
#define LED_GREEN_GPIO_PIN  GPIO_PIN_2   							//绿色LED：代表温度报警，低电平点亮
#define LED_RED_GPIO_PIN    GPIO_PIN_3   							//红色LED：代表火焰报警，低电平点亮
#define FIRE_GPIO_PIN   GPIO_PIN_5									//火焰传感器,识别到火焰为低电平,默认拉高
#define SMOKE_GPIO_PIN   GPIO_PIN_6									//烟雾传感器,识别到烟雾时为低电平,默认拉高
#define TEMPREATURE_GPIO_PIN   GPIO_PIN_7							//温度传感器（DS18B20）的信号线
#define UART_RX_GPIO_PIN   GPIO_PIN_10								//UART接收端
#define UART_TX_GPIO_PIN   GPIO_PIN_9								//UART发射端

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint8_t buzzer_state = 0; 					//无源蜂鸣器状态机，0=灭，1=急促(100ms响/100ms停)，2=缓慢(500ms响/500ms停)，3=长响
volatile _Bool fire_state = 0;						//火焰传感器状态机，0为没火，1为有火
volatile _Bool smoke_state = 0;						//烟雾传感器，0为没烟，1为有烟
volatile float temperature = 0;						//温度传感器值

// 三色LED状态机（每 100ms 换相）：
// 0=灭
// 1=蓝闪(仅烟雾)  2=绿闪(仅温度)  3=红闪(仅火焰)
// 4=蓝↔绿(烟雾+温度)  5=蓝↔红(烟雾+火焰)  6=绿↔红(温度+火焰)
// 7=红→绿→蓝 轮流亮(三路全报)
volatile uint8_t led_state = 0;

ds18b20_t ds18b20; 									//ds18b20所需参数

volatile uint8_t init_err = 0;   /* 扫描返回码 */
volatile uint8_t init_dev = 0;   /* 扫描到的器件数 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void Buzzer_Set(uint8_t state);										//蜂鸣器状态设置函数声明

void LED_Set(uint8_t state);										//三色LED闪烁状态设置函数声明

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);		//定时器中断函数声明

void My_DS18B20_Init(void);											//温度传感器初始化函数定义

float Get_Temperature(void);										//温度传感器读取函数定义

void Send_Sensor_JSON(float temp, uint8_t fire, uint8_t smoke);		//UART串口发送数据函数定义

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. 启动底层心跳定时器（250us，一直跑，永不停止）
  __HAL_TIM_SET_PRESCALER(&htim3, 64 - 1);
  __HAL_TIM_SET_AUTORELOAD(&htim3, 250 - 1);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Base_Start_IT(&htim3);

  Buzzer_Set(0);				//初始化无源蜂鸣器状态
  LED_Set(0);					//初始化三色LED状态
  My_DS18B20_Init(); 			// 初始化温度传感器
  ds18b20_cnv(&ds18b20);		//开始第一次温度转换

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    /* ============================================================
	     * DS18B20 掉线自恢复
	     * 原因：My_DS18B20_Init() 只在开机时扫描一次 ROM（约 250ms 内重试）。
	     *       如果传感器上电慢/接触抖动导致这次扫描没应答，rom_id_found
	     *       会一直是 0；而 Get_Temperature() 用 ds18b20_req_read(...,0)
	     *       需要 ROM 编号，于是永远读不到温度（temperature 保持 0）。
	     * 做法：在“没有找到器件”时每 2 秒重新扫描一次；一旦扫到（dev>=1）
	     *       这段就自动不再执行，不影响正常运行的时序。
	     * ============================================================ */
	    static uint32_t last_scan_tick = 0;                 /* 上次重扫时刻 */
	    if (ow_devices(&ds18b20.ow) == 0) {                 /* 当前没找到 ROM */
	        if (HAL_GetTick() - last_scan_tick >= 2000) {   /* 每 2 秒重扫一次 */
	            last_scan_tick = HAL_GetTick();
	            ds18b20_update_rom_id(&ds18b20);            /* 重新搜索 ROM */
	            while (ds18b20_is_busy(&ds18b20));          /* 等搜索完成（几 ms） */
	        }
	    }

	  fire_state = (HAL_GPIO_ReadPin(ALL_GPIO_PORT, FIRE_GPIO_PIN) == GPIO_PIN_RESET) ? 1 : 0;		//传感器采样写入状态机
	  smoke_state = (HAL_GPIO_ReadPin(ALL_GPIO_PORT, SMOKE_GPIO_PIN) == GPIO_PIN_RESET) ? 1 : 0;
	  temperature = Get_Temperature();

	    // 2. 限制发送频率（比如 200ms 发一次，而不是死循环狂发）
	    static uint32_t last_uart_tick = 0;
	    if (HAL_GetTick() - last_uart_tick >= 200) {
	        last_uart_tick = HAL_GetTick();

	        // 这里直接调用即可，HAL_UART_Transmit 内部会自动阻塞等待发送完毕，不需要额外的 HAL_Delay
	        Send_Sensor_JSON(temperature, fire_state, smoke_state);
	    }

	    HAL_Delay(10); // 给 CPU 稍微喘口气，或者保持你之前的 20ms 也行

//	    /* ---- 报警分级：每个传感器 0=安全，1=危险 ---- */
//	    uint8_t temp_trigger  = (temperature >= 50.0f) ? 1 : 0;
//	    uint8_t trigger_count = (uint8_t)(fire_state + smoke_state + temp_trigger);
//
//	    if (trigger_count == 3) {
//	        LED_Set(7);  Buzzer_Set(3);                        /* 三路全报：三色轮闪 + 长响 */
//	    } else if (trigger_count == 2) {
//	        if (smoke_state && temp_trigger)    LED_Set(4);    /* 烟+温：蓝↔绿 */
//	        else if (smoke_state && fire_state) LED_Set(5);    /* 烟+火：蓝↔红 */
//	        else                                LED_Set(6);    /* 温+火：绿↔红 */
//	        Buzzer_Set(1);                                     /* 急促 */
//	    } else if (trigger_count == 1) {
//	        if (smoke_state)       LED_Set(1);                 /* 仅烟：蓝闪 */
//	        else if (temp_trigger) LED_Set(2);                 /* 仅温：绿闪 */
//	        else                   LED_Set(3);                 /* 仅火：红闪 */
//	        Buzzer_Set(2);                                     /* 缓慢 */
//	    } else {
//	        LED_Set(0);  Buzzer_Set(0);
//	    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 初始化 DS18B20 库（去 main 里面调用）
void My_DS18B20_Init(void) {
    ow_init_t ow_init_struct = {0};
    ow_init_struct.tim_handle = &htim2;
    ow_init_struct.gpio = ALL_GPIO_PORT;
    ow_init_struct.pin = TEMPREATURE_GPIO_PIN;
    ow_init_struct.tim_cb = HAL_TIM_PeriodElapsedCallback;
    ow_init_struct.done_cb = NULL;
    ow_init_struct.rom_id_filter = 0;

    ds18b20_init(&ds18b20, &ow_init_struct);

    /* 最多重扫 5 次，避免上电时序/接触抖动导致永久找不到器件 */
    for (uint8_t i = 0; i < 5; i++) {
        ds18b20_update_rom_id(&ds18b20);
        while (ds18b20_is_busy(&ds18b20));                 /* 等搜索真正结束 */
        init_err = (uint8_t)ds18b20_last_error(&ds18b20);  /* 这时才是真实错误 */
        init_dev = ow_devices(&ds18b20.ow);
        if (init_dev > 0) break;
        HAL_Delay(50);
    }
}

// 温度传感器读取函数
float Get_Temperature(void)
{
    static float last_temp = 0.0f;          // 上一次有效温度（℃）

    if (ds18b20_is_cnv_done(&ds18b20))
    {
        int16_t raw = DS18B20_ERROR;

        if (ow_xfer(&ds18b20.ow, DS18B20_CMD_READ, NULL, 0, 9) == OW_ERR_NONE)
        {
            while (ds18b20_is_busy(&ds18b20));
            raw = ds18b20_read_c(&ds18b20);
        }

        if (raw != DS18B20_ERROR)
        {
            last_temp = (float)raw / 100.0f;   // ℃×100 → ℃
        }
        /* else：读失败就保持上次有效值，别把 -10000 播出去 */

        ds18b20_cnv(&ds18b20);                 // 无论成败都续下一轮
    }
    return last_temp;
}

/**
 * @brief 控制蜂鸣器
 * @param state 0=关闭，1=急促，2=缓慢, 3=长响
 */
void Buzzer_Set(uint8_t state) {
    buzzer_state = state;
    if (state == 0) {
        // 立刻拉低引脚，确保不响
        HAL_GPIO_WritePin(ALL_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
    }
}

void LED_Set(uint8_t state) {
    if (state == led_state) return;      /* 状态没变就返回，别打断中断里的闪烁相位 */
    led_state = state;

    /* 先全灭 */
    HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN | LED_GREEN_GPIO_PIN | LED_RED_GPIO_PIN, GPIO_PIN_SET);

    /* 点亮第 0 相 */
    if (state == 1 || state == 4 || state == 5) {           /* 含“烟”：先亮蓝 */
        HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN, GPIO_PIN_RESET);
    } else if (state == 2 || state == 6) {                  /* 含“温”：先亮绿 */
        HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_RESET);
    } else if (state == 3 || state == 7) {                  /* 仅火 / 三色轮闪：先亮红 */
        HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
    }
}

/**
 * @brief 定时器中断回调（统一处理蜂鸣器和LED的节奏）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        static uint8_t  tick_250us = 0;

        // ============ 蜂鸣器内部变量 ============
        static uint16_t buzzer_ms = 0;
        static uint8_t  buzzer_ring = 0;
        static uint8_t  last_buzzer_state = 0;

        // ============ LED 内部变量 ============
        static uint16_t led_ms = 0;
        static uint8_t  led_phase = 0;           // 相位：0~5 计数（两相图案用 &1，三相图案用 %3）
        static uint8_t  last_led_state = 0;

        // 【状态变化检测】
        if (buzzer_state != last_buzzer_state) {
            last_buzzer_state = buzzer_state;
            buzzer_ms = 0;
            buzzer_ring = 1; // 重新触发时，从“响”开始
        }

        if (led_state != last_led_state) {
            last_led_state = led_state;
            led_ms = 0;
            led_phase = 0;

            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN | LED_GREEN_GPIO_PIN | LED_RED_GPIO_PIN, GPIO_PIN_SET);
            if (led_state == 1 || led_state == 4 || led_state == 5) {
                HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN, GPIO_PIN_RESET);
            } else if (led_state == 2 || led_state == 6) {
                HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_RESET);
            } else if (led_state == 3 || led_state == 7) {
                HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
            }
        }

        // 【高频发声】
        if (buzzer_state != 0 && buzzer_ring) {
            HAL_GPIO_TogglePin(ALL_GPIO_PORT, BUZZER_GPIO_PIN);
        }

        // 【1ms 节拍处理】250us * 4 = 1ms
        tick_250us++;
        if (tick_250us >= 4) {
            tick_250us = 0;

            // ----- 1. 处理蜂鸣器节奏 -----
            if (buzzer_state == 3) {                    /* 长响：只响不停 */
                buzzer_ms   = 0;
                buzzer_ring = 1;                        /* 让它一直翻转引脚 */
            } else if (buzzer_state == 0) {             /* 灭 */
                buzzer_ms = 0;
                buzzer_ring = 0;
                HAL_GPIO_WritePin(ALL_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
            } else {                                    /* 1=急促(100/100)，2=缓慢(500/500) */
                buzzer_ms++;
                uint16_t target = (buzzer_state == 1) ? 100 : 500;
                if (buzzer_ring && buzzer_ms >= target) {
                    buzzer_ring = 0;
                    buzzer_ms = 0;
                    HAL_GPIO_WritePin(ALL_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
                } else if (!buzzer_ring && buzzer_ms >= target) {
                    buzzer_ring = 1;
                    buzzer_ms = 0;
                }
            }

            // ----- 2. 处理 LED 节奏 -----
            if (led_state == 0) {
                led_ms = 0;
                led_phase = 0;
                /* 灭灯已经由 LED_Set(0) 做过，这里每毫秒重复写没有意义，省掉 */
            } else {
                led_ms++;
                if (led_ms >= 100) {                    // 100ms 换一相
                    led_ms = 0;
                    led_phase++;
                    if (led_phase >= 6) led_phase = 0;  // 0~5：2 相和 3 相都对齐

                    HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN | LED_GREEN_GPIO_PIN | LED_RED_GPIO_PIN, GPIO_PIN_SET);

                    if (led_state == 7) {
                        /* 三路全报：红 → 绿 → 蓝 轮流亮 */
                        if (led_phase % 3 == 0) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
                        } else if (led_phase % 3 == 1) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_RESET);
                        } else {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN, GPIO_PIN_RESET);
                        }
                    } else if ((led_phase & 1) == 0) {
                        /* ---- 第 0 相 ---- */
                        if (led_state == 1 || led_state == 4 || led_state == 5) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_BLUE_GPIO_PIN, GPIO_PIN_RESET);
                        } else if (led_state == 2 || led_state == 6) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_RESET);
                        } else if (led_state == 3) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
                        }
                    } else {
                        /* ---- 第 1 相 ---- */
                        if (led_state == 4) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_RESET);
                        } else if (led_state == 5 || led_state == 6) {
                            HAL_GPIO_WritePin(ALL_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
                        }
                        /* 1/2/3 是灭相，不写任何东西 */
                    }
                }
            }
        }
    }
    else if (htim->Instance == TIM2) {
            ow_callback(&ds18b20.ow);  // 把中断交给单总线库处理
    }
}

// 通过串口发送传感器 JSON 数据（温度保留两位小数，不用浮点 printf）
void Send_Sensor_JSON(float temp, uint8_t fire, uint8_t smoke)
{
    /* 先把温度转成"百分之一度"的整数，再用整数格式化输出两位小数：
       - 为什么：newlib-nano 默认没有浮点 printf，sprintf("%f") 会输出空串；
       - 四舍五入：正数 +0.5、负数 -0.5 后截断；
       - 负数单独取符号，避免 -0.5 被格式化成 "0.50"；
       - %02lu 保证 "27.05" 不会被打印成 "27.5"。 */
    int32_t  t100    = (int32_t)(temp * 100.0f + ((temp >= 0.0f) ? 0.5f : -0.5f));
    const char *sign = (t100 < 0) ? "-" : "";
    uint32_t abs100  = (uint32_t)((t100 < 0) ? -t100 : t100);

    char buf[192];   /* 一行 JSON 约 160 字节，192 留余量 */
    int n = snprintf(buf, sizeof(buf),
        "{\"header\":\"SENSOR_DATA\",\"data\":{"
        "\"temperature\":%s%lu.%02lu,"
        "\"int_dev\":%u,\"int_err\":%u,"					//UART数据体里的温度传感器调试字段
        "\"fire_alarm\":%u,\"smoke_alarm\":%u,"
    	"\"led_state\":%u,\"buzzer_state\":%u"
        "},\"footer\":\"END\"}\r\n",
        sign,
        (unsigned long)(abs100 / 100u),     /* 整数部分 */
        (unsigned long)(abs100 % 100u),     /* 两位小数 */
        (unsigned)ow_devices(&ds18b20.ow),  /* 调试字段，可保留 */
        (unsigned)ds18b20_last_error(&ds18b20),
        (unsigned)fire,
        (unsigned)smoke,
		(unsigned)led_state,
		(unsigned)buzzer_state);

    if (n > 0)   /* snprintf 出错(-1)就不发 */
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);  /* 100ms 超时，避免死等 */
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
