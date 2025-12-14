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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FORMAT_TEXT,
    FORMAT_CSV,
    FORMAT_JSON
} OutputFormat_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Sensor variables
int temperature = 25;
int humidity = 60;

// System state variables
volatile bool monitoring_active = false;
OutputFormat_t output_format = FORMAT_TEXT;
uint32_t interval_ms = 2000;
uint32_t last_sample_time = 0;

// UART reception variables
volatile uint8_t rx_byte = 0;
volatile char command_buffer[50];
volatile uint8_t buffer_index = 0;
volatile bool command_ready = false;

// Output buffer
char uart_tx_buffer[150];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void process_command(void);
void send_sensor_data(void);
void send_response(const char *message);
void simulate_sensors(void);
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
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Wait for system to stabilize
  HAL_Delay(100);

  // Send startup message
  send_response("\r\n\r\n=== Sensor Monitoring System ===\r\n");
  send_response("Type HELP for available commands\r\n\r\n");

  // Enable UART receive interrupt
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);

  // Initialize timestamp
  last_sample_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    // Process commands
    if (command_ready) {
      command_ready = false;
      process_command();
    }

    // Send sensor data if monitoring is active
    if (monitoring_active) {
      if ((HAL_GetTick() - last_sample_time) >= interval_ms) {
        send_sensor_data();
        last_sample_time = HAL_GetTick();
      }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Send a response message via UART
 */
void send_response(const char *message) {
    if (message != NULL) {
        HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
    }
}

/**
 * @brief Simulate sensor readings
 */
void simulate_sensors(void) {
    temperature = 20 + (HAL_GetTick() / 1000) % 16;
    humidity = 40 + (HAL_GetTick() / 500) % 41;
}

/**
 * @brief Format and send sensor data
 */
void send_sensor_data(void) {
    simulate_sensors();

    switch(output_format) {
        case FORMAT_TEXT:
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                    "Temperature: %d C, Humidity: %d %%\r\n",
                    temperature, humidity);
            break;

        case FORMAT_CSV:
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                    "%d,%d\r\n", temperature, humidity);
            break;

        case FORMAT_JSON:
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                    "{\"temp\":%d,\"hum\":%d}\r\n",
                    temperature, humidity);
            break;
    }

    send_response(uart_tx_buffer);
}

/**
 * @brief Parse and execute received command
 */
void process_command(void) {
    // Copy from volatile buffer to local buffer
    char local_buffer[50];
    uint8_t local_index = buffer_index;

    __disable_irq();
    memcpy(local_buffer, (const char*)command_buffer, local_index);
    local_buffer[local_index] = '\0';

    // Reset volatile buffer immediately
    buffer_index = 0;
    memset((void*)command_buffer, 0, sizeof(command_buffer));
    __enable_irq();

    // Remove \r and \n terminators
    for (int i = 0; i < local_index; i++) {
        if (local_buffer[i] == '\r' || local_buffer[i] == '\n') {
            local_buffer[i] = '\0';
            break;
        }
    }

    // Skip empty commands
    if (strlen(local_buffer) == 0) {
        return;
    }

    // Parse command and parameter
    char *space_pos = strchr(local_buffer, ' ');
    char *command = local_buffer;
    char *parameter = NULL;

    if (space_pos != NULL) {
        *space_pos = '\0';
        parameter = space_pos + 1;
    }

    // Execute commands
    if (strcmp(command, "READ") == 0) {
        send_sensor_data();

    } else if (strcmp(command, "START") == 0) {
        monitoring_active = true;
        last_sample_time = HAL_GetTick();
        send_response("Monitoring STARTED\r\n");

    } else if (strcmp(command, "STOP") == 0) {
        monitoring_active = false;
        send_response("Monitoring STOPPED\r\n");

    } else if (strcmp(command, "SET_INTERVAL") == 0) {
        if (parameter != NULL) {
            int seconds = atoi(parameter);
            if (seconds > 0 && seconds <= 60) {
                interval_ms = seconds * 1000;
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                        "Interval set to %d seconds\r\n", seconds);
                send_response(uart_tx_buffer);
            } else {
                send_response("ERROR: Interval must be 1-60 seconds\r\n");
            }
        } else {
            send_response("ERROR: Missing interval parameter\r\n");
        }

    } else if (strcmp(command, "FORMAT") == 0) {
        if (parameter != NULL) {
            if (strcmp(parameter, "TEXT") == 0) {
                output_format = FORMAT_TEXT;
                send_response("Format set to TEXT\r\n");
            } else if (strcmp(parameter, "CSV") == 0) {
                output_format = FORMAT_CSV;
                send_response("Format set to CSV\r\n");
            } else if (strcmp(parameter, "JSON") == 0) {
                output_format = FORMAT_JSON;
                send_response("Format set to JSON\r\n");
            } else {
                send_response("ERROR: Unknown format. Use TEXT, CSV, or JSON\r\n");
            }
        } else {
            send_response("ERROR: Missing format parameter\r\n");
        }

    } else if (strcmp(command, "HELP") == 0) {
        send_response("\r\n=== Available Commands ===\r\n");
        send_response("READ                  - Read sensor values once\r\n");
        send_response("START                 - Start continuous monitoring\r\n");
        send_response("STOP                  - Stop continuous monitoring\r\n");
        send_response("SET_INTERVAL <sec>    - Set interval (1-60 seconds)\r\n");
        send_response("FORMAT <type>         - Set format (TEXT/CSV/JSON)\r\n");
        send_response("HELP                  - Show this help\r\n");
        send_response("==========================\r\n\r\n");

    } else {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                "ERROR: Unknown command '%s'. Type HELP for commands.\r\n", command);
        send_response(uart_tx_buffer);
    }
}

/**
 * @brief UART receive complete callback (interrupt handler)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Echo received character
        if (rx_byte >= 32 && rx_byte <= 126) {
            HAL_UART_Transmit(&huart1, (uint8_t*)&rx_byte, 1, 10);
        } else if (rx_byte == '\r') {
            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 10);
        }

        // Store character in buffer
        if (buffer_index < sizeof(command_buffer) - 1) {
            command_buffer[buffer_index] = rx_byte;
            buffer_index++;

            // Trigger ONLY on \r (not on \n to avoid double processing)
            if (rx_byte == '\r') {
                command_ready = true;
            }
        } else {
            // Buffer overflow - reset
            buffer_index = 0;
            memset((void*)command_buffer, 0, sizeof(command_buffer));
            send_response("\r\nERROR: Command too long\r\n");
        }

        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
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
