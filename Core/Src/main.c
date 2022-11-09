/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "crc.h"
#include "dma.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ai_datatypes_defines.h"
#include "ai_platform.h"

#include "network_data.h"
#include "network.h"

#include "sine_model.h"
#include "sine_model_data.h"

#include <math.h>
#include "string.h"
#include <stdio.h>
#include <assert.h>

#include "images.h"

#include "w25qxx.h"
#include "w25qxxConf.h"

#include "..\\tjpg\\tjpgd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Target  Space

#define serialPortDebug huart1
#define serialPortCAM huart3

#define shrdPortFM hspi1
#define localPortFM hspi4

#define debugPort huart1
#define serialCAM huart3
#define N_BPP (3 - JD_FORMAT)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int compression_ratio = 0;


uint8_t SR=0x77;
uint8_t start=0x11;

uint16_t i=0;
uint16_t iLoop=0;
uint16_t jLoop=0;
uint16_t loop=0;

uint32_t adrs=0;

uint8_t WREN  =0x06;
uint8_t WRDI  =0x04;
uint8_t RDSR  =0x05;
uint8_t WRSR  =0x01;
uint8_t READ_3ByteAdd  =0x03;
uint8_t READ_4ByteAdd  =0x13;
uint8_t WRITE_3ByteAdd =0x02;
uint8_t WRITE_4ByteAdd =0x12;

uint8_t ERASE =0x20;
uint8_t SECTORERASE_3ByteAdd =0xD8;
uint8_t SECTORERASE_4ByteAdd =0xDC;
uint8_t GETDATA =0x00;

int *pDataByte;
uint8_t StringLength=0;
char txString [50];
char Rx_indx, Rx_Buffer[100], Transfer_cplt;
char Rx_dataDBG[2]={0x11, 0x11};
char Rx_dataCAM[2]={0x33, 0x33};
uint8_t commandCAM=0x29;

#define csDelay 10
#define tiniestDelay 10
#define tinyDelay    100
#define fareDelay    2000


uint8_t handshakeCAM = 0;
uint8_t ACKtoCAM = 0x28;

uint8_t colored = 0x31;
uint8_t black = 0x32;
uint8_t error = 0x33;

uint8_t width_ack = 0x11;
uint8_t height_ack = 0x22;

uint8_t CAM_reso = 0;
uint8_t CAM_reso_W = 0;
uint8_t CAM_reso_H = 0;

uint8_t Earth_ICU = 0x31;
uint8_t Space_ICU = 0x32;
uint8_t Error_ICU = 0x33;

int Width = 0;
int Height = 0;
int Multiplier = 0;

int break_loop = 0;
int count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void rgb2gray_uint8_T(uint8_t* Img, int channels, int width, int height, int compression, uint8_t result[76800]);
static void main_dataconverter(double Umat[14688], uint8_t* image, double imgdat[35]);
uint16_t in_func (JDEC* jd, uint8_t* buff, uint16_t nbyte);
uint16_t out_func (JDEC* jd, void* bitmap, JRECT* rect);
static void prepocess(uint8_t* Img, float result[19200*2]);



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}
GETCHAR_PROTOTYPE
{
uint8_t ch = 0;

/* Clear the Overrun flag just before receiving the first character */
__HAL_UART_CLEAR_OREFLAG(&huart1);
/* Wait for reception of a character on the USART RX line and echo this

character on console */
HAL_UART_Receive (&debugPort, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
HAL_UART_Transmit(&debugPort, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    uint32_t fp;          																							/* File pointer for input function */
    uint8_t *fbuf;     																								/* Pointer to the frame buffer for output function */
    uint16_t wfbuf;    																								/* Width of the frame buffer [pix] */
} IODEV;

uint16_t in_func (JDEC* jd, uint8_t* buff, uint16_t nbyte)
{
    IODEV *dev = (IODEV*)jd->device;   																				/* Device identifier for the session (5th argument of jd_prepare function) */
    int i;

    if (buff) {
    	W25qxx_ReadBytes( buff, dev->fp, nbyte );

    	dev->fp += nbyte;

    	StringLength=sprintf(txString,"Value from FM %02X and Address %04u \r\n",*(buff+nbyte-1),dev->fp-1);
    	HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);

        return nbyte;
    } else {
    	/* Remove bytes from input stream */
    	dev->fp += nbyte;
        return nbyte;
    }
}

uint16_t out_func (JDEC* jd, void* bitmap, JRECT* rect)
{
    IODEV *dev = (IODEV*)jd->device;
    uint8_t *src, *dst;
    uint16_t y, bws, bwd;


    /* Copy the decompressed RGB rectangular to the frame buffer (assuming RGB888 cfg) */
    src = (uint8_t*)bitmap;
    dst = dev->fbuf + N_BPP * (rect->top * dev->wfbuf + rect->left);  													/* Left-top of destination rectangular */
    bws = N_BPP * (rect->right - rect->left + 1);     																	/* Width of source rectangular [byte] */
    bwd = N_BPP * dev->wfbuf;                         																	/* Width of frame buffer [byte] */
    for (y = rect->top; y <= rect->bottom; y++) {
        memcpy(dst, src, bws);   																					/* Copy a line */
        src += bws; dst += bwd;  																					/* Next line */
    }

    return 1;    																									/* Continue to decompress */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  //(void)argc;
	  //(void)argv;
	  void *work;       																								/* Pointer to the decompressor work area */
	  JDEC jdec;        																								/* Decompression object */
	  JRESULT res;      																								/* Result code of TJpgDec API */
	  IODEV devid;      																								/* User defined device identifier */
	  int i;
	  int image_size = 0;

	  double predict;
	  uint8_t SR = 0x24;

	  size_t sz_work = 5100;

	  char buf[50];
	  int buf_len = 0;

	  ai_error ai_err;
	  ai_error ai_err_ICU;


	  ai_i32 nbatch;
	  ai_i32 nbatch_ICU;

	  uint32_t timestamp;
	  uint32_t timestamp_ICU;

	  uint8_t prediction = 0xFF;

	  float y_val;

	  // Chunk of memory used to hold intermediate values for neural network
	  ai_u8 activations[AI_SINE_MODEL_DATA_ACTIVATIONS_SIZE];

	  // Buffers used to store input and output tensors
	  ai_i8 in_data[AI_SINE_MODEL_IN_1_SIZE_BYTES];
	  ai_i8 out_data[AI_SINE_MODEL_OUT_1_SIZE_BYTES];

	  // Pointer to our model
	  ai_handle sine_model = AI_HANDLE_NULL;

	  // Initialize wrapper structs that hold pointers to data and info about the
	  // data (tensor height, width, channels)
	  ai_buffer ai_input[AI_SINE_MODEL_IN_NUM] = AI_SINE_MODEL_IN;
	  ai_buffer ai_output[AI_SINE_MODEL_OUT_NUM] = AI_SINE_MODEL_OUT;


	  // Set working memory and get weights/biases from model
	  ai_network_params ai_params = {
	    AI_SINE_MODEL_DATA_WEIGHTS(ai_sine_model_data_weights_get()),
	    AI_SINE_MODEL_DATA_ACTIVATIONS(activations)
	  };



	  // Chunk of memory used to hold intermediate values for neural network
	  ai_u8 activations_ICU[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

	  // Buffers used to store input and output tensors
	  ai_i8 image[AI_NETWORK_IN_1_SIZE_BYTES];
	  ai_i8 out_data_ICU[AI_NETWORK_OUT_1_SIZE_BYTES];

	  // Pointer to our model
	  ai_handle icu_tflite = AI_HANDLE_NULL;

	  // Initialize wrapper structs that hold pointers to data and info about the
	  // data (tensor height, width, channels)
	  ai_buffer ai_input_ICU[AI_NETWORK_IN_NUM] = AI_NETWORK_IN;
	  ai_buffer ai_output_ICU[AI_NETWORK_OUT_NUM] = AI_NETWORK_OUT;

	  // Set working memory and get weights/biases from model
//	  ai_network_params ai_params_ICU = AI_NETWORK_PARAMS_INIT(AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()), AI_NETWORK_DATA_ACTIVATIONS(activations_ICU));
	  ai_network_params ai_params_ICU = {
			  AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
			  AI_NETWORK_DATA_ACTIVATIONS(activations_ICU)
	  };


	  		ai_input_ICU[0].n_batches = 1;
	  		ai_input_ICU[0].data = AI_HANDLE_PTR(image);
	  		ai_output_ICU[0].n_batches = 1;
	  		ai_output_ICU[0].data = AI_HANDLE_PTR(out_data_ICU);

	  		//End of ICU part 2


	  	  // Set pointers wrapper structs to our data buffers
	  	  ai_input[0].n_batches = 1;
	  	  ai_input[0].data = AI_HANDLE_PTR(in_data);
	  	  ai_output[0].n_batches = 1;
	  	  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_CRC_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  W25qxx_Init();

  printf("GRSS Satellite .. ICU code \r\n--------------------------\r\n");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Control the sharedFM, SET=STM32 RESET=ATMEGA
  buf_len = sprintf(buf, "\r\n\r\nSTM32 X-Cube-AI\r\n");
  HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);


  // Create instance of neural network
  ai_err = ai_sine_model_create(&sine_model, AI_SINE_MODEL_DATA_CONFIG);
  if (ai_err.type != AI_ERROR_NONE)
  {
    buf_len = sprintf(buf, "Error: could not create NN instance\r\n");
    HAL_UART_Transmit(&debugPort, (uint8_t *)buf, buf_len, 100);
    while(1);
  }
  else
  {
	    buf_len = sprintf(buf, "NN instance created! \r\n");
	    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
  }

  // Initialize neural network
  if (!ai_sine_model_init(sine_model, &ai_params))
  {
    buf_len = sprintf(buf, "Error: could not initialize NN\r\n");
    HAL_UART_Transmit(&debugPort, (uint8_t *)buf, buf_len, 100);
    while(1);
  }
  else
  {
	    buf_len = sprintf(buf, "NN initialized\r\n");
	    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
  }


  // Create instance of neural network
  ai_err_ICU = ai_network_create(&icu_tflite, AI_NETWORK_DATA_CONFIG);
  if (ai_err_ICU.type != AI_ERROR_NONE)
  {
    buf_len = sprintf(buf, "Error: could not create NN instance\r\n");
    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
    while(1);
  }
  else
  {
	    buf_len = sprintf(buf, "NN instance created! \r\n");
	    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
  }

  // Initialize neural network
  if (!ai_network_init(icu_tflite, &ai_params_ICU))
  {
    buf_len = sprintf(buf, "Error: could not initialize NN\r\n");
    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
    while(1);
  }
  else
  {
	    buf_len = sprintf(buf, "NN initialized\r\n");
	    HAL_UART_Transmit(&debugPort, (uint8_t *) &buf, buf_len, 100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	char readBytes[5000]="ABC";
  	W25qxx_EraseBlock(0*0x10000); HAL_Delay(1000);
  	W25qxx_ReadBytes( readBytes, 0, sizeof(Target)%sizeof(readBytes) );	HAL_Delay(10);
  	W25qxx_WriteBlock(Target, 0*0x10000, 0, sizeof(Target)); HAL_Delay(1000);

  while (1)
  {
		printf("Loop \r\n");

		work = (void*)malloc(sz_work);
	  	count++;

	  	if (count == 1) {
	  		W25qxx_ReadBytes( readBytes, 0*0x10000, sizeof(Target) );
//	  		W25qxx_ReadBytes( readBytes, 1*0x10000, sizeof(Space) );
//	  		W25qxx_ReadBytes( readBytes, 2*0x10000, sizeof(Earth) );
	  	}

		  // Fill input buffer (use test value)
			for (uint32_t i = 0; i < AI_SINE_MODEL_IN_1_SIZE; i++)
			{
			  ((ai_float *)in_data)[i] = (ai_float)1.0f;
			}

			// Get current timestamp
			timestamp = htim13.Instance->CNT;

			// Perform inference
			nbatch = ai_sine_model_run(sine_model, &ai_input[0], &ai_output[0]);
			if (nbatch != 1) {
			  buf_len = sprintf(buf, "Error: could not run inference\r\n");
			  HAL_UART_Transmit(&debugPort, (uint8_t *)buf, buf_len, 100);
			}

			// Read output (predicted y) of neural network
			y_val = ((float *)out_data)[0];

			// Print output of neural network along with inference time (microseconds)
			buf_len = sprintf(buf,
							  "Output: %f | Duration: %lu\r\n",
							  y_val,
							  htim13.Instance->CNT - timestamp);
			HAL_UART_Transmit(&debugPort, (uint8_t *)buf, buf_len, 100);

			// Wait before doing it again
			HAL_Delay(500);


	  	handshakeCAM = 0;



			StringLength=sprintf(txString,"\r\n\nImage Processing Starts Now\r\n");
			HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);

			HAL_Delay(1000);
			devid.fp=0;
//			devid.fp= Space;
			devid.fp= count%3 * 0x10000;

			StringLength=sprintf(txString,"\r\n");
			HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);

//			display_bulk_4ByteAdd_SharedFM(0x00000000, 3500);

			res = jd_prepare(&jdec, in_func, work, sz_work, &devid);													/* Prepare to decompress */
			if (res == JDR_OK)
			{																					/* Ready to decompress. Image info is available here. */
				StringLength=sprintf(txString,"\r\n\nOriginal image size is %u x %u X 3.\r\n%u Bytes of work area is used.\r\n\n", jdec.width, jdec.height, sz_work - jdec.sz_pool);
				HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);

				image_size = N_BPP  * jdec.width * jdec.height;
				devid.fbuf = (uint8_t*)malloc(image_size); /* Create frame buffer for output image */
		        devid.wfbuf = jdec.width;

				Width = jdec.width;
				Height = jdec.height;

				StringLength=sprintf(txString,"\r\n\nPreparation for Decompression - Success\r\n");
				HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);

				res = jd_decomp(&jdec, out_func, compression_ratio);   																/* Start to decompress with 1-1 scaling*/
				if (res == JDR_OK) {
					StringLength=sprintf(txString,"Decompression - Success\r\n");
					HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);
				}
				else{
					StringLength=sprintf(txString,"%d - Decompression - Failed\r\n",res);
					HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);
				}
			}
			else{
			  StringLength=sprintf(txString,"\r\n\n %d - Preparation for Decompression - Failed\r\n",res);
			  HAL_UART_Transmit(&debugPort, (uint8_t *) &txString, StringLength, 100);
			}

			HAL_Delay(1000);




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	free(work);
	free(devid.fbuf);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
