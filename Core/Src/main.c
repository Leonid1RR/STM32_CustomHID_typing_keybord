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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include "usbd_customhid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
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
uint8_t report[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void keybord();
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
keybord();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void hold(uint8_t modifier){
	memset(report, 0, sizeof(report));
    report[0] = modifier;  // Модификатор
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(50);
    memset(report, 0, sizeof(report));
}
void stophold(){
	memset(report, 0, sizeof(report));
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(50);
}

// Функция для отправки одного символа (без модификаторов)
void SendKey(uint8_t keycode) {
	memset(report, 0, sizeof(report));
    report[2] = keycode;  // Клавиша в третьем байте
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(80);

    // Отпускаем клавишу
    memset(report, 0, sizeof(report));
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(80);
}

// Функция для отправки комбинации с модификатором (например, Win)
void SendCombo(uint8_t modifier, uint8_t keycode) {
	memset(report, 0, sizeof(report));
    report[0] = modifier;  // Модификатор в первом байте
    report[2] = keycode;   // Клавиша в третьем байте
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(80);

    // Отпускаем всё
    memset(report, 0, sizeof(report));
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
    HAL_Delay(80);
}

void send(const char* text) {
    for (int i = 0; text[i] != '\0'; i++) {  // Идём до конца строки
        uint8_t keycode = 0;
        char c = tolower(text[i]);  // Для упрощения сравнения

        // Преобразование символа в HID-код
        switch (c) {
            case 'a': keycode = 0x04; break;
            case 'b': keycode = 0x05; break;
            case 'c': keycode = 0x06; break;
            case 'd': keycode = 0x07; break;
            case 'e': keycode = 0x08; break;
            case 'f': keycode = 0x09; break;
            case 'g': keycode = 0x0A; break;
            case 'h': keycode = 0x0B; break;
            case 'i': keycode = 0x0C; break;
            case 'j': keycode = 0x0D; break;
            case 'k': keycode = 0x0E; break;
            case 'l': keycode = 0x0F; break;
            case 'm': keycode = 0x10; break;
            case 'n': keycode = 0x11; break;
            case 'o': keycode = 0x12; break;
            case 'p': keycode = 0x13; break;
            case 'q': keycode = 0x14; break;
            case 'r': keycode = 0x15; break;
            case 's': keycode = 0x16; break;
            case 't': keycode = 0x17; break;
            case 'u': keycode = 0x18; break;
            case 'v': keycode = 0x19; break;
            case 'w': keycode = 0x1A; break;
            case 'x': keycode = 0x1B; break;
            case 'y': keycode = 0x1C; break;
            case 'z': keycode = 0x1D; break;
            case '1': keycode = 0x1E; break;
            case '2': keycode = 0x1F; break;
            case '3': keycode = 0x20; break;
            case '4': keycode = 0x21; break;
            case '5': keycode = 0x22; break;
            case '6': keycode = 0x23; break;
            case '7': keycode = 0x24; break;
            case '8': keycode = 0x25; break;
            case '9': keycode = 0x26; break;
            case '0': keycode = 0x27; break;
            case ' ': keycode = 0x2C; break;
            case '-': keycode = 0x2D; break;
            case '=': keycode = 0x2E; break;
            case '[': keycode = 0x2F; break;
            case ']': keycode = 0x30; break;
            case '\\': keycode = 0x31; break;
            case ';': keycode = 0x33; break;
            case '\'': keycode = 0x34; break;
            case '`': keycode = 0x35; break;
            case ',': keycode = 0x36; break;
            case '.': keycode = 0x37; break;
            case '/': keycode = 0x38; break;
            default: continue;  // Пропускаем неподдерживаемые символы
        }

        // Отправка клавиши
        SendKey(keycode);
    }
}
void keybord(){
	HAL_Delay(2000);
	GPIOC->ODR |= (1 << 13);// отключаем светодиод (чтоб показать что работа началась)
	// Нажимаем Win + R
	SendCombo(0x08, 0x15);  // 0x08 = Win, 0x15 = R

	//"cmd" + Enter
	send("cmd");
	SendKey(0x28);  // Enter
	HAL_Delay(500);// задержка, чтобы успеть открыть cmd
	send("notepad");
	SendKey(0x28);  // Enter
	HAL_Delay(500);// задержка, чтобы успеть открыть notepad
	SendCombo(0x04, 0x02);
	HAL_Delay(500);
		memset(report, 0, sizeof(report));
		report[0] = 0x06;
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
		HAL_Delay(80);
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, 8);
		HAL_Delay(80);
		memset(report, 0, sizeof(report));
	send("ghbdtn");
	GPIOC->ODR &= ~ (1 << 13);// включаем светодиод (работа завершилась)


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
