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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PAUSE_PIN GPIOC, GPIO_PIN_13
#define CHL_SELECT_PIN GPIOC, GPIO_PIN_14
#define HSE_IN_PIN GPIOH, GPIO_PIN_0
#define HSE_OUT_PIN GPIOH, GPIO_PIN_1
#define TIM5_CH1_PIN GPIOA, GPIO_PIN_0
#define TIM5_CH2_PIN GPIOA, GPIO_PIN_1
#define ENCODE2_PIN GPIOA, GPIO_PIN_2
#define WAVEIN1_PIN GPIOA, GPIO_PIN_3
#define WAVEIN2_PIN GPIOA, GPIO_PIN_4
#define LCD_SCK_PIN GPIOA, GPIO_PIN_5
#define LCD_MISO_PIN GPIOA, GPIO_PIN_6
#define LCD_MOSI_PIN GPIOA, GPIO_PIN_7
#define LCD_DCRS_PIN GPIOC, GPIO_PIN_4
#define LCD_RESET_PIN GPIOC, GPIO_PIN_5
#define LCD_CS_PIN GPIOB, GPIO_PIN_0
#define SD_SCK_PIN GPIOB, GPIO_PIN_13
#define SD_MISO_PIN GPIOB, GPIO_PIN_14
#define SD_MOSI_PIN GPIOB, GPIO_PIN_15
#define SD_CS_PIN GPIOC, GPIO_PIN_6
#define USART1_TX_PIN GPIOA, GPIO_PIN_9
#define USART1_RX_PIN GPIOA, GPIO_PIN_10
#define USB_DM_PIN GPIOA, GPIO_PIN_11
#define USB_DP_PIN GPIOA, GPIO_PIN_12
#define SWD_IO_PIN GPIOA, GPIO_PIN_13
#define SWD_CLK_PIN GPIOA, GPIO_PIN_14
#define TIM2_CH1_PIN GPIOA, GPIO_PIN_15
#define ENCODE1_PIN GPIOC, GPIO_PIN_11
#define TIM2_CH2_PIN GPIOB, GPIO_PIN_3
#define SCALE_AXIS_PIN GPIOB, GPIO_PIN_4
#define I2C1_SCL_PIN GPIOB, GPIO_PIN_6
#define I2C1_SDA_PIN GPIOB, GPIO_PIN_7

#define ADC_12BIT 4095.0f
#define VRef 3.3f //V
#define ATTENUATION_FACTOR 51.0f //51:1
#define OP_AMP_GAIN 4.0f //Gain of OP-AMP
#define ADC_BUFFER_SIZE 4096 //Size of ADC DMA buffer
#define TIM3_CLK 84000000 //Timer 3 Clock Frequency

#define X_DIVISIONS 20.0f // 20 X divisions
#define Y_DIVISIONS 10.0f // +/- 10 Y divisions
//50Hz input min, 25KHz input max
//At least 10 Waveforms, meaning 2 div per wavefront
//Min 1 Waveform, meaning 20 div per wavefront
//Max T(1/50) * (1/2) = 0.01s, Min T(1/25K) * 1/20 = 2e-6s = 0.000002
static const float xdiv_Levels[] = {0.000002f, 0.0000025f, 0.000003f, 0.000004f, 0.000005f, 0.000008f, 0.00001f, 0.000015f, 0.00002f, 0.000025f, 0.00003f, 0.00004f, 0.00005f, 0.00008f, 0.0001f, 0.00015f, 0.0002f, 0.00025f, 0.0003f, 0.0004f, 0.0005f, 0.0008f, 0.001f, 0.0015f, 0.002f, 0.0025f, 0.003f, 0.004f, 0.005f, 0.008f, 0.01f};
//Min is roughly resolution, Max is 20v Peak / no. divisions = 20/10 = 2V
static const float ydiv_Levels[] = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.08f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.8f, 1.0f, 1.2f, 1.5f, 1.6f, 2.0f};
#define COUNT_PER_STEP 4 //Count increase on encoder timer per encoder step

typedef enum {
  SCOPE_ARMED = 0,
  SCOPE_TRIGGERED,
  SCOPE_WAITING
} scope_state_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define XDIV_COUNT (sizeof(xdiv_Levels)/sizeof(xdiv_Levels[0]))
#define YDIV_COUNT (sizeof(ydiv_Levels)/sizeof(ydiv_Levels[0]))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  float BIAS_VOLTAGE = 1.65f; //Bias Voltage
  volatile uint8_t CHL_Select_State = 0; //Channel Select Button State
  volatile uint8_t Pause_State = 0; //Pause Button State
  volatile uint8_t Auto_Scale_Y_State = 0; //Auto Scale Y State
  volatile uint8_t Scale_Axis_State = 0; //Scale Axis Button State 0->x, 1->y
  volatile float Trig_Voltage = 0.0f; //Set Initial Trigger Voltage
  volatile uint8_t Trig_Flag = 0;
  volatile uint16_t Trig_ADC_Value = 2048;
  volatile scope_state_t scope_state = SCOPE_ARMED;
  float Sample_Rate; //Hz
  float Sample_period; //s
  uint8_t Screen_Change_Flag = 1; //Flag to indicate screen change
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void VBias_Init(uint16_t *adc_buffer);
void Set_Axis_Div(float *buffer, float *xDIV, float *yDIV, uint16_t length);
void Set_Trig_Value(void);
void ADC_Convert(uint16_t *adc_input, float *Vin, uint16_t DMA_index);
float VmaxABS(float *buffer, uint16_t length);
float Auto_Scale_Y(float *buffer, uint16_t length, int *ylocation);
void ADC_SetTrigger(void);
void DWT_Init(void);
uint32_t Micros(void);

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3); //Start Timer 3 for ADC trigger
  static uint16_t adc_buffer[ADC_BUFFER_SIZE]; //ADC DMA buffer
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE); //Start ADC with DMA
  HAL_Delay(100); //Delay for ADC to stabilise and fill buffer
  VBias_Init(adc_buffer); //Initialise Bias Voltage 

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Start Encoder 1
  __HAL_TIM_SET_COUNTER(&htim2, 0x80000000); //Set Encoder 1 counter to middle value
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); //Start Encoder 2
  __HAL_TIM_SET_COUNTER(&htim5, 0x80000000); //Set Encoder 2 counter to middle value

  static float Vin_buffer[ADC_BUFFER_SIZE]; //Scaled Voltage buffer
  ADC_Convert(adc_buffer, Vin_buffer, ADC_BUFFER_SIZE);

  yDIV = 0.5f; //Initial Y Division setting
  xDIV = 0.0002f; //Initial X Division setting
  Set_Axis_Div(Vin_buffer, &xDIV, &yDIV, ADC_BUFFER_SIZE); //Initialise Axis Division setting
  Set_Trig_Value(); //Initialise Trigger Value setting

  ILI9341_Init();
  ILI9341_Clear(BLACK);
  ILI9341_ClearUI();
  ILI9341_UpdateVals(xDIV, yDIV, Trig_Voltage);
  ILI9341_DrawAxis(Y_DIVISIONS, X_DIVISIONS);

  static float plot_buffer[ADC_BUFFER_SIZE/2];

  DWT_Init(); //Initialize DWT for Micros() function
  uint32_t Last_Trig_Time = Micros(); 

  uint16_t DMA_Index = 0; //Get current DMA index
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t Total_Display_Time = (uint32_t)(xDIV * X_DIVISIONS * 1e6f); //Total time (us) displayed on screen
    if (Trig_Flag) {
      Trig_Flag = 0;
      HAL_ADC_Stop_DMA(&hadc1); //Stop ADC DMA
      DMA_Index = ADC_BUFFER_SIZE - hdma_adc1.Instance->NDTR; //Get current DMA index
      ADC_Convert(adc_buffer, Vin_buffer, DMA_Index); //Convert ADC values to Voltages
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE); //Restart ADC DMA
      Last_Trig_Time = Micros();
      scope_state = SCOPE_WAITING;
    }
    if (scope_state == SCOPE_WAITING) {
      if ((Micros() - Last_Trig_Time) >= Total_Display_Time*1.5f) {
        volatile uint32_t temp_void = hadc1.Instance->DR; //Read DR to clear potential overrun flag
        (void)temp_void;
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
        scope_state = SCOPE_ARMED;
      }
    }

	  if (!CHL_Select_State){
		  for (uint16_t i=0; i<DMA_Index/2; i++){
			  plot_buffer[i] = Vin_buffer[2*i];
		  }
	  }
	  else {
		  for (uint16_t i=0; i<DMA_Index/2; i++){
			  plot_buffer[i] = Vin_buffer[2*i+1];
		  }
	  }

    
    Wavefront_Index = Find_Rising_Edge(plot_buffer, DMA_Index/2); //Find Rising Edge for Triggering
    if (Wavefront_Index >= DMA_Index/2) {
      Wavefront_Index = 0; //No Trigger Found, set to 0
    }
    static float plot_volts[ADC_BUFFER_SIZE/2];
    for (uint16_t i=0; i<(DMA_Index/2)-Wavefront_Index; i++) {
      plot_volts[i] = plot_buffer[Wavefront_Index + i];
    }

	  Set_Axis_Div(plot_volts, &xDIV, &yDIV, (DMA_Index/2)-Wavefront_Index); //Update Axis Division setting
	  Set_Trig_Value(); //Update Trigger Value setting
    if (Screen_Change_Flag) {
      Screen_Change_Flag = 0;
      ILI9341_UpdateVals(xDIV, yDIV, Trig_Voltage);
    }
	  if (!Pause_State){
      ILI9341_ClearWaveform();
		  ILI9341_DrawAxis(Y_DIVISIONS, X_DIVISIONS);
		  ILI9341_PlotWaveform(plot_volts,xDIV,yDIV,RED);
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
    switch(GPIO_Pin)
    {
        case GPIO_PIN_14: // PC14 -- CHL_Select Button
            // handle PC14 interrupt
        	CHL_Select_State = !CHL_Select_State;
        	ADC_SetTrigger();
            break;
        case GPIO_PIN_13: // PC13 -- Pause Button
            // handle PC13 interrupt
        	Pause_State = !Pause_State;
            break;
        case GPIO_PIN_11: // PC11 -- Encoder 1 Button
            // handle PC11 interrupt
        	Auto_Scale_Y_State = 1;
            break;
        case GPIO_PIN_4:  // PB4 -- Scale Axis Button
            // handle PB4 interrupt
        	Scale_Axis_State = !Scale_Axis_State;
            break;
        case GPIO_PIN_2:  // PA2 -- Encoder 2 Button
            // handle PA2 interrupt
        	Trig_Voltage = 0.0f; //Reset Trigger Voltage
            break;
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
		//Trigger
    if (scope_state == SCOPE_ARMED) {
		    Trig_Flag = 1;
        scope_state = SCOPE_TRIGGERED;
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
		  }
	  }
  return;
}

void VBias_Init(uint16_t *adc_buffer) {
	//Initialise Bias Voltage
	uint32_t adc_sum = 0;
	for (int i=0; i<ADC_BUFFER_SIZE; i++) {
		adc_sum += adc_buffer[i];
	}
	BIAS_VOLTAGE = ((float)adc_sum / ADC_BUFFER_SIZE) * (VRef / ADC_12BIT);
	return;
}

void Set_Axis_Div(float *buffer, float *xDIV, float *yDIV, uint16_t length) {
	static int ylocation = 4; //Initial Y division index
  static int xlocation = 16; //Initial X division index
  if (Auto_Scale_Y_State) {
		*yDIV = Auto_Scale_Y(buffer, length, &ylocation); //Auto Scale Y if enabled
		Auto_Scale_Y_State = 0; //Reset Auto Scale Y State
    Screen_Change_Flag = 1;
    return;
	}
  static uint32_t TIM2_Count = 0;
	uint32_t TIM2_New_Count = __HAL_TIM_GET_COUNTER(&htim2);
	if (TIM2_Count == 0) {
		TIM2_Count = __HAL_TIM_GET_COUNTER(&htim2);

	}
	else if (TIM2_New_Count != TIM2_Count) {
		int steps = ((int32_t)TIM2_New_Count - (int32_t)TIM2_Count) / COUNT_PER_STEP;
		TIM2_Count = TIM2_New_Count;
		if (Scale_Axis_State == 0) {
			//X-Axis Scaling
			//Adjust Time/Div based on steps
			xlocation += steps;
			if (xlocation >= (XDIV_COUNT - 1)) {xlocation = XDIV_COUNT - 1;}
			else if (xlocation < 0) {xlocation = 0;}
			*xDIV = xdiv_Levels[xlocation];
      Screen_Change_Flag = 1;
      float Total_Display_Time = (*xDIV) * X_DIVISIONS; //Total time (s) displayed on screen
      Sample_period = Total_Display_Time / (ADC_BUFFER_SIZE/4.0f); //Calculate Sample Period based on 2 channels and 2x bufferring
      Sample_Rate = (uint32_t)(1.0f / Sample_period); //Calculate Sample Rate
      Scope_SetSampleRate(Sample_Rate); //Set Sample Rate
		}
		else {
			//Y-Axis Scaling
			//Adjust V/Div based on steps
			ylocation += steps;
			if (ylocation >= (YDIV_COUNT - 1)) {ylocation = YDIV_COUNT - 1;}
			else if (ylocation < 0) {ylocation = 0;}
			*yDIV = ydiv_Levels[ylocation];
      Screen_Change_Flag = 1;
		}
	}
}

void Set_Trig_Value(void) {
	static uint32_t TIM5_Count = 0;
	uint32_t TIM5_New_Count = __HAL_TIM_GET_COUNTER(&htim5);
	if (TIM5_Count == 0) {
		TIM5_Count = __HAL_TIM_GET_COUNTER(&htim5);
	}
	else if (TIM5_New_Count != TIM5_Count) {
		int steps = ((int32_t)TIM5_New_Count - (int32_t)TIM5_Count) / 4;
		TIM5_Count = TIM5_New_Count;
		float Temp_Trig_Voltage = Trig_Voltage + (float)steps*0.01f;
		if (Temp_Trig_Voltage > 20.0f) {Trig_Voltage = 20.0f;}
		else if (Temp_Trig_Voltage < -20.0f) {Trig_Voltage = -20.0f;}
		else {Trig_Voltage = Temp_Trig_Voltage;}

		float Trig_Voltage_Adj = (Trig_Voltage / ATTENUATION_FACTOR * OP_AMP_GAIN) + BIAS_VOLTAGE;
		Trig_ADC_Value = (uint16_t)(Trig_Voltage_Adj * (ADC_12BIT/VRef));
    if (Trig_ADC_Value > 4095) {Trig_ADC_Value = 4095;}
		ADC_SetTrigger();
    Screen_Change_Flag = 1;
	}
}

void ADC_Convert(uint16_t *adc_input, float *Vin, uint16_t DMA_index) {
	//Scale ADC input based on schematic design
	//ADC input range: 0-3.3V (0-4095), VRef = 3.3V
	//Scope input range: +/-20v
	//Attenuation factor: 51:1
	//Bias: 3.3v/2 = 1.65v
	//OP-AMP Gain = 4
	//Vin = (Input-Bias)*(Attenuation Factor)/(OP-AMP Gain)
	for (int i=0; i<DMA_index; i++){
		float Vadc = (float)adc_input[i] * (VRef/ADC_12BIT); //Convert ADC value to voltage
		Vin[i] = (Vadc - BIAS_VOLTAGE) * (ATTENUATION_FACTOR) / (OP_AMP_GAIN); //Multiply by attenuation factor and divide by OP-AMP gain
	}
}

float VmaxABS(float *buffer, uint16_t length) {
	float max = fabsf(buffer[0]);
	for (int i=1; i<length; i++) {
		if (fabsf(buffer[i]) > max) {
			max = fabsf(buffer[i]);
		}
	}
	return max;
}

float Auto_Scale_Y(float *buffer, uint16_t length, int *ylocation) {
	float Vpp = 2* VmaxABS(buffer, length);
	//Screen Height is 240 pixels
	//Usable height = 200 pixels (20 pixels margin top and bottom)
	//Scale such that Vpp fits in 200 pixels
	//Similarly +/- 10 divisions vertically from #define Y_DIVISIONS
	//Max V/div = 20V/10 = 2V/div
	//Min V/div = 0.1V/10 = 0.01V/div //Roughly Corresponds to accuracy of ADC
	float V_div = Vpp / (2.0f * (float)Y_DIVISIONS); //Volts per division
	for (uint8_t i=0; i<YDIV_COUNT; i++) {
		if (V_div <= ydiv_Levels[i]) {
			*ylocation = i;
			return ydiv_Levels[i];
		}
	}
	return ydiv_Levels[YDIV_COUNT - 1];
}

void ADC_SetTrigger(void) {

	uint32_t CHANNEL;
	if (!CHL_Select_State) {CHANNEL = ADC_CHANNEL_3;}
	else {CHANNEL = ADC_CHANNEL_4;}

	ADC_AnalogWDGConfigTypeDef awdConfig;
	awdConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	awdConfig.Channel = CHANNEL;
	awdConfig.ITMode = ENABLE;
	awdConfig.HighThreshold = Trig_ADC_Value;
	awdConfig.LowThreshold  = 0; // keep low
	HAL_ADC_AnalogWDGConfig(&hadc1, &awdConfig);
}

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable the trace and debug block
  DWT->CYCCNT = 0; // Reset the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter
}

uint32_t Micros(void) {
  return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

uint16_t Find_Rising_Edge(float *buffer, uint16_t DMA_Index) {
  for (uint16_t i = 1; i < DMA_Index; i++) {
    if (buffer[i - 1] < Trig_Voltage && buffer[i] >= Trig_Voltage) {
      return i;
    }
  }
  return 0; // No rising edge found
}

void Scope_SetSampleRate(uint32_t fs) {
  if (fs<1) {fs = 1;}
  uint32_t arr = (TIM3_CLK / fs) - 1;

  __HAL_TIM_DISABLE(&htim3);
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_ENABLE(&htim3);
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
