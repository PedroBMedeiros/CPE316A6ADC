
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
void ADC_init(void);
void ADC_ISR(void);
uint32_t calculateMinimum(uint16_t array[]);
uint32_t calculateMaximum(uint16_t array[]);
uint32_t calculateAverage(uint16_t array[]);

#define arraySize 20

// global variables
uint8_t ISRFlag = 1;
uint16_t sampleValue = 0;

void uint32_to_string(uint32_t value, char* voltageString) {
	// value received ranges from 0 to 3300
	uint32_t valueOne = 0;
	uint32_t valueTwo = 0;
	uint32_t valueThree = 0;
	//uint32_t valueFour = 0;

	valueOne = value/1000;
	valueTwo = (value - (valueOne*1000))/100;
	valueThree = (value - (valueOne*1000) - (valueTwo*100))/10;
	//valueFour = (value - (valueOne*1000) - (valueTwo*100) - (valueThree*10));

	voltageString[0] = valueOne + 48;
	voltageString[2] = valueTwo + 48;
	voltageString[3] = valueThree + 48;

}

void delay(volatile uint32_t count) {
    while (count--) {
        __NOP();  // No operation, just wait
    }
}


int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	// Initialize //

	NVIC_EnableIRQ(USART2_IRQn);
	USART2_init();
	ADC_init();


	uint8_t count = 0;
	uint16_t arrayValues[arraySize];
	uint32_t minimumVal = 0;
	uint32_t maximumVal = 0;
	uint32_t averageVal = 0;
	char averagePrint[5] = {48, '.', 48, 48, '\0'};
	char maxPrint[5] = {48, '.', 48, 48, '\0'};
	char minPrint[5] = {48, '.', 48, 48, '\0'};

	USART_ESC_Print("J"); // clear terminal
	USART_ESC_Print("0;0H"); // move cursor home
	USART_ESC_Print("0m"); // turn off character attributes
	USART2_Print("Average Value: ");

	while (1) {
		averageVal = ADC1->DR;
		ADC1->CR |= ADC_CR_ADSTART;
		if (ISRFlag == 1) {
			if (count < arraySize) {
				arrayValues[count] = sampleValue;
				++count;
			} else {
				minimumVal = 0.80566*calculateMinimum(arrayValues);
				maximumVal = 0.80566*calculateMaximum(arrayValues);
				averageVal = 0.80566*calculateAverage(arrayValues);

				uint32_to_string(averageVal, averagePrint);
				uint32_to_string(minimumVal, minPrint);
				uint32_to_string(maximumVal, maxPrint);

				USART_ESC_Print("J"); // clear terminal
				USART_ESC_Print("0;0H"); // move cursor home
				USART_ESC_Print("0m"); // turn off character attributes
				USART2_Print("Average Value: ");
				USART2_Print(averagePrint);
				USART2_Print(" V");
				USART_ESC_Print("0;0H"); // move cursor home
				USART_ESC_Print("1B");
				USART2_Print("Maximum Value: ");
				USART2_Print(maxPrint);
				USART2_Print(" V");
				USART_ESC_Print("0;0H"); // move cursor home
				USART_ESC_Print("2B");
				USART2_Print("Minimum Value: ");
				USART2_Print(minPrint);
				USART2_Print(" V");
				count = 0;
				delay(2000000);
			}
			ISRFlag = 0;
			ADC1->CR |= ADC_CR_ADSTART;
		}
	}
}


void ADC_init(void) {
	// Relevant pages: Datasheet 172//
	/// ADC CONFIGURATION ///
	// Enable clock (1) & synchronous mode (01) [251, 608] //
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	ADC123_COMMON->CCR |= ADC_CCR_CKMODE_0;
	// Power ADC (0) / Voltage regulator (1) [583] //
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	ADC1->CR |= ADC_CR_ADVREGEN;
	for(uint32_t i = 0; i < 50000; i++);		//Wait for 20 us
	// Single ended mode for channel 5 PA0 (0) (0) [585, 604 & Data 73] //
	ADC1->CR &= ~ADC_CR_ADEN;					//Disables ADC
	ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5;
	// Calibrate ADC (0) (1) [583] //
	ADC1->CR &= ~ADC_CR_ADCALDIF;
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0);		//Wait for ADCAL to be 0
	// Enable ADC (1) () [580, 585] //
	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) != 1);	//Wait for ADRDY to be 1
	// Sampling time of 2.5 (000) [593] //
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);
	ADC1->SMPR1 |= (4 << ADC_SMPR1_SMP5_Pos);
	// Configure sequence for channel 5 [597] //
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);
	// Configure resolution / data alignment //
	// Single conversion mode, 12-bit, right aligned
	ADC1->CFGR = 0;
	// Configure interrupts //
	ADC1->IER |= ADC_IER_EOCIE;  // Enable end of conversion interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn);   // Enable ADC interrupt in NVIC
	/// GPIOA CONFIGURATION ///
	// Enable GPIOA clock (1) [251] //
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Analog mode (11) [304] //
	GPIOA->MODER &= ~GPIO_MODER_MODE0;
	GPIOA->MODER |= GPIO_MODER_MODE0;
	// Push pull (0) [305] //
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;
	// Very high (11) [305] //
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;
	// No pull up, pull down (00) [305] //
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;
	// Connect to analog (1) [311] //
	GPIOA->ASCR |= GPIO_ASCR_ASC0;
	// start a conversion
	//ADC1->CR |= ADC_CR_ADSTART;
}


void ADC1_2_IRQHandler(void) {
	// Check if the end-of-conversion (EOC) flag is set
	if (ADC1->ISR & ADC_ISR_EOC) {
		// Read the conversion result and store it in the global variable
		sampleValue = ADC1->DR;
		// Set the global flag to indicate that the result is ready
		ISRFlag = 1;
		// Clear the EOC flag by writing 1 (to clear it)
		ADC1->ISR |= ADC_ISR_EOC;
	}
}

uint32_t calculateMinimum(uint16_t array[]) {
	uint32_t minimumValue = array[0];
	for (uint8_t i = 1; i < arraySize; ++i) {
		if (minimumValue > array[i]) {
			minimumValue = array[i];
		}
	}
	return minimumValue;
}

uint32_t calculateMaximum(uint16_t array[]) {
	uint32_t maximumValue = array[0];
	for (uint8_t i = 1; i < arraySize; ++i) {
		if (maximumValue < array[i]) {
			maximumValue = array[i];
		}
	}
	return maximumValue;
}

uint32_t calculateAverage(uint16_t array[]) {
	uint32_t totalSum = 0;
	uint32_t averageValue = 0;
	for (uint8_t i = 0; i < arraySize; ++i) {
		totalSum += array[i];
	}
	averageValue = totalSum / arraySize;
	return averageValue;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
