/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <float.h>
#include <stdio.h>  // For debugging output if needed
#include <stdlib.h>
#define  ARM_MATH_CM
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Estimated value
    float p; // Estimation error covariance
    float k; // Adaptive Kalman gain
} KalmanFilter;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

// Declare the external assembly function for Kalman filter update
extern int kalmanFilter_assembly(KalmanFilter* kf, float measurement);


//for testing c code purpose----------------------------------------------------------------



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int kalmanFilter_update(KalmanFilter* kf, float measurement) {
	if (kf->p > FLT_MAX - kf->q) {//p > Max-q
		return 1;
	}
    // Prediction update
    kf->p += kf->q;
	if (fabs(kf->p + kf->r) <= FLT_EPSILON) { // handle div by 0
		return 1;
	}
    // Compute the Kalman gain
    kf->k = kf->p / (kf->p + kf->r);
	if (isinf(kf->k)) { //handle overflow
		return 1;
	}
    // Update the estimate
    kf->x += kf->k * (measurement - kf->x);
	if (isinf(kf->x)) { //handle overflow
		return 1;
	}
    // Update the error covariance
    kf->p *= (1 - kf->k);

	if (isinf(kf->p)) { //handle overflow
		return 1;
	}

	return 0;
}

int Kalmanfilter(float* inputArray, float* outputArray, KalmanFilter* kstate, int length) {
	for(int i=0; i<length; i++) {
		float measurement = inputArray[i];
		kalmanFilter_update(kstate, measurement);
		outputArray[i] = kstate->x;
	}

	return 1;
}


int Kalmanfilter_asm(float* inputArray, float* outputArray, KalmanFilter* kstate, int length) {
	int condition = 0;
	//loop through input measurement array
	for(int i=0; i<length; i++) {
		float measurement = inputArray[i];
		kalmanFilter_assembly(kstate, measurement);
		outputArray[i] = kstate->x;

	}

	return condition;
}

//void calculateDifferenceArray(float* InputArray, float* OutputArray, float* differenceArray, int Length) {
//	for(int i = 0; i < Length; i++) {
//		// element-wise subtraction
//		differenceArray[i] = InputArray[i] - OutputArray[i];
//	}
//	return; // void
//}



int kalmanFilter_CMSIS_update(KalmanFilter* kf, float measurement) {
	//p=p+q
	if (kf->p > FLT_MAX - kf->q) {
		return -1;
	}

	arm_add_f32(&(kf->p), &(kf->q), &(kf->p), 1);

	//k=p/(p+r)
	float temp = 0;
	arm_add_f32(&(kf->p), &(kf->r), &temp, 1); //adding p+r and storing in temp
	if (fabs(temp) <= FLT_EPSILON) {// Handle the division by zero or small number
		return -1;
	}
	temp = 1/temp;	//reciprocating temp to use the CMSIS multiply function
	arm_mult_f32(&(kf->p), &temp, &(kf->k), 1);

	//x=x+k*(measurement-x)
	if (isinf(kf->k)) {// Handle the overflow or invalid result
		return -1;
	}
	temp = 0;
	arm_sub_f32(&measurement, &(kf->x), &temp, 1); //subtracting measurement-x and storing in temp
	arm_mult_f32(&(kf->k), &temp, &temp, 1); //restoring the k*temp into temp
	arm_add_f32(&(kf->x), &temp, &(kf->x), 1); //adding x+temp to update the x value
	if (isinf(kf->x)) {// Handle the overflow or invalid result
		kf->x = -1.111;
		return -1;
	}

	//p=(1-k)*p
//	float one = 1.0f;
	temp = 0;
	arm_mult_f32(&(kf->k), &(kf->p), &temp, 1); //multiplying temp with k and updating k
	arm_sub_f32(&(kf->p), &temp, &(kf->p), 1); //subtracting 1-k and storing in temp
	if (isinf(kf->p)) {// Handle the overflow or invalid result
		return -1;
	}

	return 0;

}

int kalmanFilter_CMSIS(float* inputArray, float* outputArray, KalmanFilter* kstate, int length) {
	for(int i=0; i<length; i++) {
		float measurement = inputArray[i];
		kalmanFilter_CMSIS_update(kstate, measurement);
		outputArray[i] = kstate->x;
	}

	return 1;
}

float subtraction(float* inputArray, float* outputArray, float* resultArray, int length) {
	if(inputArray == 0 || outputArray == 0) {
		return -1;
	}
	for(int i=0; i<length; i++) {
		float result = outputArray[i] - inputArray[i];
		if (result < 0){
			resultArray[i] = -result;
		} else {
			resultArray[i] = result;
		}
	}
	return 1;
}

float avg(float* inputArray, int length) {
	float sum = 0;
	float average = 0;

	if(inputArray == 0) {
		return -1;
	}
	for(int i=0; i<length; i++) {
		sum += inputArray[i];
	}

	average = sum/(float)length;
	return average;
}


float std_dev(float* inputArray, int length) {
	float standard_deviation = 0;
	float sum = 0;
	float meanValue = avg(inputArray, length);

	if(inputArray == 0) {
		return -1;
	}


	for(int i=0; i<length; i++) {
		sum += pow((inputArray[i]-meanValue), 2);
	}
	standard_deviation = sqrt(sum/(length-1));

	return standard_deviation;

}

float correlation(float* inputArray, float* outputArray, float* correlationArray, int length) {

	for (int i = 0; i < 2*length-1; i++) {
		correlationArray[i] = 0.0;
	    }


	for(int i=0; i<length-1; i++){
		int x_start = length - 1 - i;
		int y_start = 0;
		for(int j=0;j<length;j++){
			correlationArray[2* length - 2- i] += inputArray[x_start] * outputArray[y_start];
			correlationArray[i] += inputArray[y_start]*outputArray[x_start];
			x_start++;
			y_start++;
			if(x_start == length || y_start == length){
				break;
			}
		}

	}
	for(int k=0;k<length;k++){
		correlationArray[length-1] += inputArray[k]*outputArray[k];
	}


	return 1;

}

float convolution(float* inputArray, float* outputArray, float* convolutionArray, int length) {

    if(inputArray == 0 || outputArray == 0) {
    			return -1;
    		}
	for(int i=0; i<(2*length-1); i++){
		convolutionArray[i] = 0;

		for (int j=0; j<length; j++){
			if(i-j>=0 && i-j<length){
				convolutionArray[i] += inputArray[j] * outputArray[i-j];
			}
		}
	}

	return 1;
}

float subtraction_CMSIS(float* inputArray, float* outputArray, float* resultArray, int length) {
	arm_sub_f32(outputArray, inputArray, resultArray, length);
	arm_abs_f32(resultArray, resultArray, length);
	return 1;
}

float avg_CMSIS(float* inputArray, int length) {
	float avgValue = 0;
	arm_mean_f32(inputArray, length, &avgValue);
	return avgValue;
}

float std_dev_CMSIS(float* inputArray, int length) {
	float std_devValue = 0;
	arm_std_f32(inputArray, length, &std_devValue);
	return std_devValue;
}

float correlation_CMSIS(float* inputArray, float* outputArray, float* resultArray, int length) {
	arm_correlate_f32 (inputArray, length, outputArray, length, resultArray);
	return 1;
}

float convolution_CMSIS(float* inputArray, float* outputArray, float* resultArray, int length) {
	arm_conv_f32 (inputArray, length, outputArray, length, resultArray);
	return 1;
}


void processedData_CMSIS(float* inputArray, float* outputArray, float* finalArray, float* correlation, float* convolution, int length, float* avg_value, float* std_devValue) {
	subtraction_CMSIS(inputArray, outputArray, finalArray, length);
	*avg_value = avg_CMSIS(finalArray, length);
	*std_devValue = std_dev_CMSIS(finalArray, length);
	correlation_CMSIS(inputArray, outputArray, correlation, length);
	convolution_CMSIS(inputArray, outputArray, convolution, length);
}


void processedData_C(float* inputArray, float* outputArray, float* finalArray, float* correlation_result, float* convolution_result, int length, float* avg_value, float* std_devValue) {


	subtraction(inputArray, outputArray, finalArray, length);
	*avg_value = avg(finalArray, length);
	*std_devValue = std_dev(finalArray, length);
	correlation(inputArray, outputArray, correlation_result, length);
	convolution(inputArray, outputArray, convolution_result, length);

}
//for testing c code purpose----------------------------------------------------------------


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//
	float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
	                    10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
	                    9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
	                    9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
	                    10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
	                    10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
	                    10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
	                    10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
	                    9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
	                    10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
	                    10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
	                    10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
	                    10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
	                    10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
	                    9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
	                    10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
	                    9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
	                    10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
	                    9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
	                    10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
	                    9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
	                    10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
	                    9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
	                    9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
	                    10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
	                    9.5799256668};


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
  /* USER CODE BEGIN 2 */

//  // Call the Kalman filter update function (in assembly) // assembly entry____________________________________
//  kalmanFilter_assembly(&kf, measurement);
  KalmanFilter kf1 = {
        .q = 0.1f,  // initial values
        .r = 0.1f,
        .x = 5.0f,
        .p = 0.1f,
        .k = 0.0f
    };
  KalmanFilter kf2 = {
        .q = 0.1f,  // initial values
        .r = 0.1f,
        .x = 5.0f,
        .p = 0.1f,
        .k = 0.0f
    };
  KalmanFilter kf3 = {
        .q = 0.1f,  // initial values
        .r = 0.1f,
        .x = 5.0f,
        .p = 0.1f,
        .k = 0.0f
    };


//  float TEST_ARRAY[] = {0.0f, 2.0f, 3.0f, 4.0f, 5.0f};  // Array of measurement values
//  int length = 5;
////  float outputArr1[5];
//
//  kalmanFilter_CMSIS(measurements, outputArr, &kf, num_measurements);

//  testing with the given testcase
//  int length = sizeof(TEST_ARRAY)/sizeof(TEST_ARRAY[0]);
//
  int length = 101;
  float outputArr1[length];
  Kalmanfilter(TEST_ARRAY, outputArr1, &kf1, length); 	//(float* InputArray, float* OutputArray, KalmanFilter* kstate, int Length)

  float outputArr2[length];
  kalmanFilter_CMSIS(TEST_ARRAY, outputArr2, &kf2, length);

  float outputArr3[length];
  Kalmanfilter_asm(TEST_ARRAY, outputArr3, &kf3, length);

//
  float finalArray_CMSIS[length];
  float finalArray_C[length];
  float correlation_CMSIS[length*2-1];
  float correlation_C[length*2-1];
  float convolution_CMSIS[length*2-1];
  float convolution_C[length*2-1];
  float avg_value_CMSIS = 0;
  float avg_value_C = 0;
  float std_devValue_CMSIS= 0;
  float std_devValue_C = 0;
  processedData_CMSIS(TEST_ARRAY, outputArr1, finalArray_CMSIS, correlation_CMSIS,convolution_CMSIS, length, &avg_value_CMSIS, &std_devValue_CMSIS);
  processedData_C(TEST_ARRAY, outputArr1, finalArray_C, correlation_C, convolution_C, length, &avg_value_C, &std_devValue_C);


//  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_Delay(1000);
    // You can add code here for continuous processing
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  /* User can add their own implementation to report the HAL error return state */
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
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
