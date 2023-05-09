/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * 
  *   This code was developed in collaboration with the following people:
  *   - Jorge Askur Vázquez Fernández
  *   - Andrés Sarellano Acevedo
  *   - Izel María Ávila Rodríguez
  * 
  * The code inside this file is meant to be run at an STM32H745 NUCLEO Board
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//  Analog to digital converter handler
ADC_HandleTypeDef hadc3;
//  CAN bus handler
FDCAN_HandleTypeDef hfdcan1;
//  PWM timer handler
TIM_HandleTypeDef htim2;
//  STM32H745 UART3 handler
UART_HandleTypeDef huart3;
//  CAN Filter Configuration
FDCAN_FilterTypeDef sFilterConfig;
//  CAN_Tx Header
FDCAN_TxHeaderTypeDef TxHeader;
//  CAN_Rx Header
FDCAN_RxHeaderTypeDef RxHeader;
UART_HandleTypeDef huart3;
//  Global Variable for displaying Tank's pressure
uint8_t pressureVal;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN Boot_Mode_Sequence_0 */
    int32_t timeout;
  /* USER CODE END Boot_Mode_Sequence_0 */

  /* USER CODE BEGIN Boot_Mode_Sequence_1 */
    /* Wait until CPU2 boots and enters in stop mode or timeout*/
    timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
    if ( timeout < 0 )
    {
    Error_Handler();
    }
  /* USER CODE END Boot_Mode_Sequence_1 */
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();
  /* USER CODE BEGIN Boot_Mode_Sequence_2 */
  /* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
  HSEM notification */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /*Take HSEM */
  HAL_HSEM_FastTake(HSEM_ID_0);
  /*Release HSEM in order to notify the CPU2(CM4)*/
  HAL_HSEM_Release(HSEM_ID_0,0);
  /* wait until CPU2 wakes up from stop mode */
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
    Error_Handler();
  }
  /* USER CODE END Boot_Mode_Sequence_2 */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_FDCAN1_Init();
    MX_ADC3_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /* USER CODE BEGIN WHILE */ /*AAO+*/
    printf("Hello World\n\r");

    //	Message data
    uint8_t TxData[8] = {0x1D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t TxData2[8] = {0x0A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    //  Setting default pressure setpoint
    uint8_t Setpoint = 0;
    //  Reading header
    uint8_t RxData[8];
    uint8_t pressureVal;
    int32_t DC = 65535;

    double e = 0.0, e_1=0.0, u=0.0, u_1=0.0;
    double kp, ti, q0, q1;

    void PID_Initialize( void ){
        double T = 1.59524;            //sample rate
        //Complete here with the identified
        //parameters of the plant
      double k = 1;
      double tao = 14.17;
      double theta = 18.45;

      //Calculate the coefficients for
      //the discrete PID controller
      kp = (0.9 * tao) / (k * theta);
      ti = 3.333333 * theta;
      q0 = kp + ((kp * T)/(2.0 * ti));
      q1 = ((kp * T)/(2.0 * ti)) - kp;
    }

    double PID_Discrete( double yM , uint8_t setpoint){
          double R = setpoint;            //set-point
          e = R - yM;            //calculate the error
          u = u_1 + q0*e + q1*e_1;    //discrete PID controller
          //Saturate the controller with upper and lower limits
          if(u >= 100) u = 20;
          if (yM == 0) u = 20; //ressure value
          if(u <= 0) u = 0;          //minimum pressure value
          e_1 = e;
          u_1 = u;
          return u;
        }

    /**
     * 	Esta función activa el solenoide y el compresor dependiendo de la cantidad
     * 	de presión que se desea tener en el tanque.
     *	@param uint8_t setpoint  Es la Presión que se desea alcanzar.
    *
    * */

    void adjustPressure( uint8_t setpoint)
    {

      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);
      printf("DC: %d\n\r", DC);

      double u = PID_Discrete(pressureVal, setpoint);
      printf("PID: %2.2lf\n\r", u);


      if (pressureVal > setpoint - 5 && pressureVal < setpoint)
      {
        DC = (100-u)/100 * 65535;

        printf("Compresor  Encendido \n\r");
        printf("DC con PID: %d\n\r", DC);
        TIM2->CCR1 = DC;

      } else if (pressureVal < setpoint){
        DC = 65535;
        TIM2->CCR1 = DC;
      } else if(pressureVal > setpoint) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
        printf("Se apago el solenoide\n\r");

        TIM2->CCR1 = DC;
        printf("Se activo el solenoide\n\r");
      } else {

        DC = 51000;
        TIM2->CCR1 = DC;

      }

    }

    /**
     * 	Esta función le asigna el valor de la presión deseada a la variable Setpoint para recibirse
     * 	en la función adjustPressure para activar el sistema de control.
     *
     *	@param uint8_t RData[8]  Es un arreglo de 8 bytes, es el mensaje SPN recibido del CAN cuyo primer
    *		elemento contiene el valorde los datos.
    *	@return uint8_t pressureVal		Es el valor de presión que se desea tener en el tanque,
    *		permanecerá la presión previa si no se recibe nada del CAN.
    *
    * */
    uint8_t setSetpoint(uint8_t RData[8])
    {
      uint8_t setpoint;
      RxHeader.Identifier = 0x10FEF9A3;
      //	Identifier 0x10FEF9A3 -> Pressure Setpoint
      //	Identifier 0x18FEEEA4 -> Meassured Pressure Front Right Tire
      if(RxHeader.Identifier == 0x10FEF9A3)
      {

        //	Cambiamos el Identifier para mandar el mensaje bajo dicha directiva
        TxHeader.Identifier = 0x18FEEEA4;
        //	El primer valor de la transmisión es el valor de presión deseado, por lo que será el setpoint
        setpoint = RData[0];
        printf("Cambio de Setpoint: %d  \n\r", setpoint);
        return setpoint;
      }
      return setpoint;

    }

    /**
     * 	Esta función actualiza el valor del arreglo de transmisión para transmitir por el CAN
     *	@param uint8_t TxData[8]  Es un arreglo de 8 bytes, es el mensaje SPN recibido del CAN cuyo primer
    *		elemento contiene el valor de los datos.
    *
    * */
    uint8_t transmitPressure(uint8_t TxData[8], uint8_t pressureVal)
    {
      //Definir parametros iniciales
      //(HAY QUE CALIBRARLO)
      uint16_t pressureZero = 2100;
      uint16_t pressureMax = 16380;

      //	Definimos el Identifier para transmitir la presión actual
      TxHeader.Identifier = 0x18FEEEA4;

      //	Inicializamos el ADC
      HAL_ADC_Start(&hadc3);
      //	Preguntamos si se esta recibiendo algo
      HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
      //	Guardamos el valor obtenido
      uint16_t raw = HAL_ADC_GetValue(&hadc3);

      //	Se descartan los dos LSB para eliminar algo de ruido
      raw = raw >> 2;
      raw = raw << 2;

      //	Imprimir valor de resolucion obtenido
      printf("Raw = %d \n\r", raw);
      //	Convertir a PSI
      pressureVal = ((raw - pressureZero)*100)/ (pressureMax - pressureZero);
      //	Imprimir el valor de pressureVal final
      printf("Pressure = %d PSI \n\r", pressureVal);
      return pressureVal;
    }

    RxHeader.Identifier = 0x10FEF9A3;
    PID_Initialize();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
    while (1)
    {
        /*	Transmisión de datos	*/
        if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
        {
          printf("aux sent\n\r");
          Error_Handler();
        }
        //	Significant delay to show the functionality on a slow pace.
        //HAL_Delay(500);

        printf("\nMessage sent\r");


        /*	Recepción de datos	*/
      while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK);

      //HAL_Delay(10);
      printf("CAN ID: 0x%.8lX \n\r", RxHeader.Identifier);

      printf("CAN Message: \n\r");
      int dataSize = RxHeader.DataLength >>  16;
      for (int i = 0; i < dataSize; i ++)
      {
          printf(" %x", RxData[i]);
        }
      printf("\n\r");

      //	Si se recibe un valor del Dashboard para actualizar la presión, el Setpoint se actualiza.
      Setpoint =  setSetpoint(RxData);
      //	Ajustamos la presión del tanque con el setpoint presentado
      adjustPressure(Setpoint);
      //	Actualización de los datos para mandar al NodeMCU
      pressureVal = transmitPressure(RxData, pressureVal);
      //	Actualizamos el SPN para mandar el dato deseado
      TxData[0] = pressureVal;

      HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
      //HAL_Delay(100); //AAO

    }
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_14B_OPT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x3F;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */


  /*AAO+*/
    /* Configure Rx filter */
     sFilterConfig.IdType = FDCAN_EXTENDED_ID;
     sFilterConfig.FilterIndex = 0;
     sFilterConfig.FilterType = FDCAN_FILTER_MASK;
     sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
     sFilterConfig.FilterID1 = 0x300;//	Ponemos 3f8
     sFilterConfig.FilterID2 = 0x7FF;


     /* Configure global filter to reject all non-matching frames */
     //HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);



     if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
       {
          /* Filter configuration Error */
          Error_Handler();
       }
      /* Start the FDCAN module */
     if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {}
          /* Start Error */
     if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){}
          /* Notification Error */

      /* Configure Tx buffer message */
     TxHeader.Identifier = 0x18FEEEA4; // 111
     TxHeader.IdType = FDCAN_EXTENDED_ID;
     TxHeader.TxFrameType = FDCAN_DATA_FRAME;
     TxHeader.DataLength = FDCAN_DLC_BYTES_8; //CAMB
     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
     TxHeader.BitRateSwitch = FDCAN_BRS_ON;
     TxHeader.FDFormat = FDCAN_FD_CAN;
     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
     TxHeader.MessageMarker = 0x00;
    /*AAO-*/

     RxHeader.IdType = FDCAN_EXTENDED_ID;
     RxHeader.RxFrameType = FDCAN_DATA_FRAME;
     RxHeader.DataLength = FDCAN_DLC_BYTES_8;




  /* USER CODE END FDCAN1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EnablePWM_GPIO_Port, EnablePWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EnablePWM_Pin */
  GPIO_InitStruct.Pin = EnablePWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EnablePWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
