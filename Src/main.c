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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include <stdbool.h>
#include <stdio.h>
#include "m95640_driver.h"
#include <string.h>
#include "semphr.h"
#include "stm32f405xx.h"
#include <stdint.h>
#include "queue.h"
#include <receiver.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AIN1_PIN    9   // PA9
#define AIN2_PIN    10  // PA10
extern UART_HandleTypeDef huart2;   // add this at top of main.c
/* ===== Sensor pins ===== */
#define VIBRATION_PORT   GPIOB
#define VIBRATION_PIN    GPIO_PIN_5    // SW420 DO -> PB5

#define IR_PORT          GPIOC
#define IR_PIN           13U           // IR DO -> PC13

/* ===== Thresholds ===== */
#define TEMP_THRESHOLD_C  30

/* ===== Shared flags ===== */
volatile bool engine_off = false;
volatile bool reason_temp = false;
volatile bool reason_vibration = false;
volatile bool reason_collision = false;
#define EEPROM_ADDRESS 0X0000
#define LED_GREEN_ON()   (GPIOC->ODR |=  (1U<<8))
#define LED_GREEN_OFF()  (GPIOC->ODR &= ~(1U<<8))
#define LED_RED_ON()     (GPIOC->ODR |=  (1U<<12))
#define LED_RED_OFF()    (GPIOC->ODR &= ~(1U<<12))

#define USERS_MAX              5
#define USER_SLOT_SIZE         8      // bytes reserved per user
#define USER_CODE_LEN          4
#define USER_SLOT_ADDR(uidx)   (0x0000 + ((uidx) * USER_SLOT_SIZE))

/* ========== IR Remote button addresses ========== */
#define IR_POWER    0x01FE48B7u
#define IR_STAR     0x01FE58A7u
#define IR_HASH     0x01FE7887u
#define IR_0        0x01FEE01Fu
#define IR_1        0x01FE50AFu
#define IR_2        0x01FED827u
#define IR_3        0x01FEF807u
#define IR_4        0x01FE30CFu
#define IR_5        0x01FEB04Fu
#define IR_6        0x01FE708Fu
#define IR_7        0x01FE00FFu
#define IR_8        0x01FEF00Fu
#define IR_9        0x01FE9867u
#define IR_A        0x01FE807Fu
#define IR_B        0x01FE40BFu
#define IR_C        0x01FEC03Fu
#define IR_D        0x01FE20DFu

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void motor_forward(void) {
    GPIOA->ODR |= (1 << AIN1_PIN);
    GPIOA->ODR &= ~(1 << AIN2_PIN);
}
void motor_stop(void) {
    GPIOA->ODR &= ~(1 << AIN1_PIN);
    GPIOA->ODR &= ~(1 << AIN2_PIN);
}
void init_motor_GPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (1 << (AIN1_PIN * 2)) | (1 << (AIN2_PIN * 2));
    GPIOA->OTYPER &= ~((1 << AIN1_PIN) | (1 << AIN2_PIN));
    GPIOA->OSPEEDR |= (3 << (AIN1_PIN * 2)) | (3 << (AIN2_PIN * 2));
}

/* ===== ADC for Temperature (PC2 = ADC1_IN12) ===== */
volatile int temperature = 0;
uint16_t adc_value;
float voltage, t1;

void adc_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (3 << (2 * 2));   // PC2 analog

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3 = 12;                  // channel 12 = PC2
    ADC1->CR1 = 0;
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON;
}
uint16_t adc_read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}
void Task_Remote(void *params);
char IR_GetKey(void);
static char wait_key_blocking(void);

/* ===================== EEPROM Helpers ====================== */
static void EEPROM_ReadUserCode(uint8_t userIdx, char out[USER_CODE_LEN+1])
{
    uint32_t addr = USER_SLOT_ADDR(userIdx);
    uint8_t buf[USER_CODE_LEN];
    EepromReadBuffer_HAL(addr, buf, USER_CODE_LEN);
    for (int i=0;i<USER_CODE_LEN;i++) out[i] = (char)buf[i];
    out[USER_CODE_LEN] = '\0';
}

static void EEPROM_WriteUserCode(uint8_t userIdx, const char code[USER_CODE_LEN])
{
    uint32_t addr = USER_SLOT_ADDR(userIdx);
    EepromWriteBuffer_HAL(addr, (uint8_t*)code, USER_CODE_LEN);
}

/* If slot is blank (all 0xFF), default it to "1234" once */
static void EEPROM_EnsureDefaults(void)
{
    for (uint8_t u=0; u<USERS_MAX; u++)
    {
        uint8_t raw[USER_CODE_LEN];
        EepromReadBuffer_HAL(USER_SLOT_ADDR(u), raw, USER_CODE_LEN);
        bool blank = true;
        for (int i=0;i<USER_CODE_LEN;i++) if (raw[i] != 0xFF) { blank = false; break; }
        if (blank) {
            const char def[USER_CODE_LEN] = {'1','2','3','4'};
            EepromWriteBuffer_HAL(USER_SLOT_ADDR(u), (uint8_t*)def, USER_CODE_LEN);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

/* ===================== LCD Helpers ====================== */
static void LCD_Clear2(void)
{
    lprint(0x80, "                ");
    lprint(0xC0, "                ");
}

static void LCD_ShowTop(const char* s)
{
    char line[17] = "                ";
    strncpy(line, s, 16);
    lprint(0x80, line);
}

static void LCD_ShowBottom(const char* s)
{
    char line[17] = "                ";
    strncpy(line, s, 16);
    lprint(0xC0, line);
}

/* ===================== IR Keypad replacement ====================== */
char IR_GetKey(void)
{
    uint32_t code;
    if (HX1838_GetCode(&code, 200) == pdPASS) {
        switch (code) {
            case IR_POWER: return 'P';
            case IR_0: return '0';
            case IR_1: return '1';
            case IR_2: return '2';
            case IR_3: return '3';
            case IR_4: return '4';
            case IR_5: return '5';
            case IR_6: return '6';
            case IR_7: return '7';
            case IR_8: return '8';
            case IR_9: return '9';
            case IR_STAR: return '*';
            case IR_HASH: return '#';
            case IR_A: return 'A';
            case IR_B: return 'B';
            case IR_C: return 'C';
            case IR_D: return 'D';
            default: return 0;
        }
    }
    return 0;
}

static char wait_key_blocking(void)
{
    char k;
    do {
        k = IR_GetKey();
        vTaskDelay(pdMS_TO_TICKS(20));
    } while (k == 0);
    return k;
}

/* ===================== Input Helpers ====================== */
static uint8_t read_user_id(void)
{
    LCD_Clear2();
    LCD_ShowTop("EnterUserID 1-5:");
    LCD_ShowBottom("Then press #");

    char id_char = 0;
    while (1)
    {
        char k = wait_key_blocking();
        if (k >= '1' && k <= '5') {
            id_char = k;
            char msg[17] = "Selected: X     ";
            msg[10] = id_char;
            LCD_ShowBottom(msg);
        } else if (k == '#') {
            if (id_char >= '1' && id_char <= '5') {
                return (uint8_t)(id_char - '1');
            }
        } else if (k == '*') {
            return 0xFF;
        }
    }
}

static bool read_code_4(char out[USER_CODE_LEN])
{
    LCD_Clear2();
    LCD_ShowTop("Security Code:");
    char shown[17] = "Code:            ";
    uint8_t idx = 0;

    while (1)
    {
        char k = wait_key_blocking();
        if (k >= '0' && k <= '9') {
            if (idx < USER_CODE_LEN) {
                out[idx++] = k;
                int pos = 6 + idx - 1;
                if (pos >= 0 && pos < 16) shown[pos] = '*';
                LCD_ShowBottom(shown);
            }
        } else if (k == '#') {
            if (idx == USER_CODE_LEN) return true;
        } else if (k == '*') {
            return false;
        }
    }
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TempTask(void *argument) {
    for (;;) {
        adc_value = adc_read();
        voltage = (adc_value / 4095.0) * 3.3;
        t1 = -50.0 * voltage;
        temperature = (int)(t1 + 107.5);

        if (temperature > TEMP_THRESHOLD_C) {
            reason_temp = true;
        } else {
            reason_temp = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ===== VibrationTask ===== */
void VibrationTask(void *argument) {
    uint8_t prev = HAL_GPIO_ReadPin(VIBRATION_PORT, VIBRATION_PIN);

    for (;;) {
        uint8_t now = HAL_GPIO_ReadPin(VIBRATION_PORT, VIBRATION_PIN);
        if (now != prev) {
            vTaskDelay(pdMS_TO_TICKS(30)); // debounce
            now = HAL_GPIO_ReadPin(VIBRATION_PORT, VIBRATION_PIN);
            if (now != prev) {
                prev = now;
                if (now == GPIO_PIN_SET) {
                    reason_vibration = true;
                    vTaskDelay(pdMS_TO_TICKS(10000));   // debounce
                } else {
                    reason_vibration = false;
                }

            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===== CollisionTask ===== */
void CollisionTask(void *argument) {
    bool last_raw = 0;
    uint32_t stable_ms = 0;

    for (;;) {
        uint8_t raw = ((IR_PORT->IDR >> IR_PIN) & 0x1U) ? 0U : 1U; // 1=obstacle
        if (raw != last_raw) {
            last_raw = raw;
            stable_ms = 0;
        } else {
            if (stable_ms < 1000) stable_ms++;
        }
        if (stable_ms == 30) {
        	if (raw) {
        	    reason_collision = true;
        	} else {
        	    reason_collision = false;
        	}

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ===== EngineTask ===== */


void send_alert(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY); // newline
}

void EngineTask(void *argument) {
    LcdInit();
    LcdFxn(0, 0x01);
    lprint(0x80, "System Ready");

    for (;;) {
        engine_off = reason_temp || reason_vibration || reason_collision;

        if (engine_off) {
            motor_stop();
            if (reason_temp) {
                lprint(0xC0, "Over Temp!      ");
                send_alert("ALERT: Over Temp!");
            }
            else if (reason_vibration) {
                lprint(0xC0, "Vibration!      ");
                send_alert("ALERT: Vibration detected!");
            }
            else if (reason_collision) {
                lprint(0xC0, "Collision!      ");
                send_alert("ALERT: Collision detected!");
            }
            else {
                lprint(0xC0, "Engine OFF      ");
                send_alert("ALERT: Engine OFF");
            }
        } else {
            motor_forward();
            lprint(0xC0, "Engine ON       ");
            send_alert("STATUS: Engine ON");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // send every 2s
    }
}

void Task_Remote(void *params)
{
    EEPROM_EnsureDefaults();
    LCD_Clear2();
    LCD_ShowTop("Press Power");
    LED_GREEN_OFF();
    LED_RED_OFF();
    while (1) {
            if (IR_GetKey() == 'P') break;   // map Power to 'P' in IR_GetKey
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        /* Once Power pressed, show Welcome */
        LCD_Clear2();
        LCD_ShowTop("Welcome");
    LCD_ShowBottom("Press * to start");
    while (1) {
        if (IR_GetKey() == '*') break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (1)
    {
        LED_GREEN_OFF();
        LED_RED_OFF();


        /* --- User ID step --- */
        uint8_t userIdx;
        while (1) {
            userIdx = read_user_id();
            if (userIdx < USERS_MAX) break;
        }

        /* --- Security Code step --- */
        char entered[USER_CODE_LEN];
        if (!read_code_4(entered)) {
            continue;
        }

        char stored[USER_CODE_LEN+1];
        EEPROM_ReadUserCode(userIdx, stored);

        if (strncmp(entered, stored, USER_CODE_LEN) == 0) {
            LCD_Clear2();
            LCD_ShowTop("Access Granted");
            LED_GREEN_ON();
            vTaskDelay(pdMS_TO_TICKS(800));
            LED_GREEN_OFF();

            while (1) {
                LCD_Clear2();
                LCD_ShowTop("Menu:");
                LCD_ShowBottom("A=ChgPwd B=Enter");
                char k = wait_key_blocking();

                if (k == 'A') {
                    LCD_Clear2();
                    LCD_ShowTop("New Security Code:");
                    char newcode[USER_CODE_LEN];
                    if (read_code_4(newcode)) {
                        EEPROM_WriteUserCode(userIdx, newcode);
                        vTaskDelay(pdMS_TO_TICKS(10));
                        LCD_Clear2();
                        LCD_ShowTop("Password Changed!");
                        vTaskDelay(pdMS_TO_TICKS(1200));
                    }
                    break;
                }
                else if (k == 'B') {
                    LCD_Clear2();
                    LCD_ShowTop("Entering System...");
                    vTaskDelay(pdMS_TO_TICKS(1200));
                    xTaskCreate(TempTask, "Temp", 256, NULL, 1, NULL);
				  xTaskCreate(VibrationTask, "Vibe", 256, NULL, 1, NULL);
				  xTaskCreate(CollisionTask, "Collision", 256, NULL, 1, NULL);
				  xTaskCreate(EngineTask, "Engine", 256, NULL, 1, NULL);
				  vTaskDelete(NULL);

                }
            }
        } else {
            LCD_Clear2();
            LCD_ShowTop("Access Denied.");
            LCD_ShowBottom("Try Again!");
            LED_RED_ON();
            vTaskDelay(pdMS_TO_TICKS(1200));
            LED_RED_OFF();
        }
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SpiEepromInit_HAL();
  adc_init();
  LcdInit();
     init_motor_GPIO();
      HX1838_Init(&htim2, GPIO_PIN_5);   // configure IR pin

      xTaskCreate(Task_Remote, "Remote", 256, NULL, 2, NULL);

//      xTaskCreate(TempTask, "Temp", 256, NULL, 1, NULL);
//      xTaskCreate(VibrationTask, "Vibe", 256, NULL, 1, NULL);
//      xTaskCreate(CollisionTask, "Collision", 256, NULL, 1, NULL);
//      xTaskCreate(EngineTask, "Engine", 256, NULL, 1, NULL);

      vTaskStartScheduler();

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
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
