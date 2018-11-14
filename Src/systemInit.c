/**
 * @file
 *
 * @brief System initialisation
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Defines -------------------------------------------------------------------*/
#define GYRO 0
#define ACC 1

/* Function prototypes -------------------------------------------------------*/
int read_data(int address, int type);
void write_data(uint8_t address, uint8_t data, uint8_t type);

/* External variables --------------------------------------------------------*/
  
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

TIM_OC_InitTypeDef sConfigOC;

/**
 * @brief System clock configuration
 * 
 * @detail Initialises and configures the system clock.
 */
/* System Clock Configuration */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief SPI2 Initialisation
 * 
 * @detail Initialises and configures Serial Peripheral Interface 2.
 */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi2);

}

/**
 * @brief TIM1 Initialisation
 * 
 * @detail Initialises and configures Timer 1.
 */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/**
 * @brief TIM2 Initialisation
 * 
 * @detail Initialises and configures Timer 2.
 */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;


  //SystemCoreClock; // expression has no effect
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //Sets channel to the PWM-signal
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);  
}

/**
 * @brief TIM3 Initialisation
 * 
 * @detail Initialises and configures Timer 3.
 */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);

}
  
/**
 * @brief TIM4 Initialisation
 * 
 * @detail Initialises and configures Timer 4.
 */ 
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 17;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);

}
  
/**
 * @brief TIM8 Initialisation
 * 
 * @detail Initialises and configures Timer 8.
 */
void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim8);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1);

}

/**
 * @brief USART2 Initialisation
 * 
 * @detail Initialises and configures Universal Synchronous and Asynchronous
 * Receiver-Transmitter 2.
 */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

} 

/**
 * @brief USART3 Initialisation
 * 
 * @detail Initialises and configures Universal Synchronous and Asynchronous
 * Receiver-Transmitter 3.
 */
void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart3);
  

}  

/**
 * @brief USB Initialisation
 * 
 * @detail Initialises and configures USB.
 */
void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  HAL_PCD_Init(&hpcd_USB_FS);

} 
  
/**
 * @brief GPIO Initialisation
 * 
 * @detail Configures GPIO pins as
 * - Analog
 * - Input
 * - Output
 * - EVENT_OUT
 * - EXTI
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : INT2_XM_Pin INT_G_Pin DRDY_G_Pin INT1_XM_Pin */
  GPIO_InitStruct.Pin = INT2_XM_Pin|INT_G_Pin|DRDY_G_Pin|INT1_XM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ACC_MAGN_Pin CS_GYRO_Pin CS_BARO_Pin DEN_GYRO_Pin */
  GPIO_InitStruct.Pin = CS_ACC_MAGN_Pin|CS_GYRO_Pin|CS_BARO_Pin|DEN_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief LIS3DH accelerometer initialisation
 * 
 * @detail Initialises the accelerometer control registers. This initialisation
 * is specific to the project and should not be changed.
 *
 * @see http://www.st.com/en/mems-and-sensors/lis3dh.html
 */
void lis3dh_Init(void)
{
  //lis3dh init variables
  uint8_t register_Value;       //Used to store the information sent to the register     
  uint8_t reg_Address;          //Used to store address of used register and rw and ms bit
  uint8_t status_Reg[2] = {0x00, 0x00};
  
  /*
  To start an SPI transmission, a specific pin must be set to 0.
  The MCU has the problem with having these pins set as 0 as default.
  This can confuse the sensor during the first transmission.
  To solve this, we start a transmission and try to read a specific register
  that always contain the same value.
  We know that the confusion has subsided when we receive the correct value
  from the sensor.
  */
  reg_Address = 0x8F;
  while(status_Reg[1] != 0x33)
  {
    HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
    HAL_SPI_TransmitReceive(&hspi2, &reg_Address, (uint8_t *)status_Reg, 2, 1000); 
    HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
   // printf("%i\n", status_Reg[1]);
  }
  
  //CTRL_REG1
  reg_Address = 0x20;                      //rw:0, ms:0 adress: 0x20
  register_Value = 0x77;             //ODR[7:4]: 400Hz, LPen[3]: HIGH/NORMAL, ZXYen[2:0]: 1 (enable output)
  write_data(reg_Address, register_Value, ACC);
    
  //CTRL_REG2
  reg_Address = 0x21;                      //rw:0, ms:0 adress: 0x21
  register_Value = 0x00;             // Highpass filter[7:0]: 0 (disable)
  write_data(reg_Address, register_Value, ACC);
  
  //CTRL_REG3
  reg_Address = 0x22;                      //rw:0, ms:0 adress: 0x22
  register_Value = 0x00;             // Interrupts[7:0]: 0 (disable)
  write_data(reg_Address, register_Value, ACC);
  
  //CTRL_REG4
  reg_Address = 0x23;                      //rw:0, ms:0 adress: 0x23
  register_Value = 0x68;             //BDU[7]:0 (continouse update), BLE[6]:1 (MSB @ lower address), FS[5:4]:10 (+-8g), HR[3]:1 (high res), ST[2:1]:00 (self-test disable), SIM[0]:0 (4wire interface). 
  write_data(reg_Address, register_Value, ACC);
  
  //CTRL_REG5
  reg_Address = 0x24;                      //rw:0, ms:0 adress: 0x24
  register_Value = 0x00;             //reboot memory, FIFO disable, int not latched
  write_data(reg_Address, register_Value, ACC);

  
  //CTRL_REG6
  reg_Address = 0x25;                      //rw:0, ms:0 adress: 0x25
  register_Value = 0x00;             //click int disable, all disable, active high
  write_data(reg_Address, register_Value, ACC);
  
  
  //REFERENCE
  reg_Address = 0x26;                      //rw:0, ms:0 adress: 0x26
  register_Value = 0x00;             //0 (no interrupt generation.
  write_data(reg_Address, register_Value, ACC);
  
  
  //INT1_THS  
  reg_Address = 0x32;                      //rw:0, ms:0 adress: 0x32
  register_Value = 0x00;             //interrrupt threshold
  write_data(reg_Address, register_Value, ACC);

  
  //INT1_DUR 
  reg_Address = 0x33;                      //rw:0, ms:0 adress: 0x33
  register_Value = 0x00;             //duration of interrupts
  write_data(reg_Address, register_Value, ACC);
  
  //INT1_CFG 
  reg_Address = 0x38;                      //rw:0, ms:0 adress: 0x38
  register_Value = 0x00;             //properties of interrupts
  write_data(reg_Address, register_Value, ACC);
  
}

/**
 * @brief L3GD20H gyroscope initialisation
 * 
 * @detail Initialises the gyroscope sensor by sending data to the sensor
 * control registers. This code has been specifically written for this project
 * and should not be changed.
 * 
 * @see http://www.st.com/en/mems-and-sensors/l3gd20h.html
 */
void l3gd20h_Init(void)
{
  //lis3dh init variables
  uint8_t register_Value;       //Used to store the information sent to the register     
  uint8_t reg_Address;          //Used to store address of used register and rw and ms bit
  uint8_t status_Reg[2] = {0x00, 0x00};
  
  /*
  To start an SPI transmission, a specific pin must be set to 0.
  The MCU has the problem with having these pins set as 0 as default.
  This can confuse the sensor during the first transmission.
  To solve this, we start a transmission and try to read a specific register
  that always contain the same value.
  We know that the confusion has subsided when we receive the correct value
  from the sensor.
  */
  reg_Address = 0x8F;
  while(status_Reg[1] != 0xD7)
  {
    HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &reg_Address, (uint8_t *)status_Reg, 2, 1000); 
    HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
   // printf("%i\n", status_Reg[1]);
  }
  
  //CTRL_REG1
  reg_Address = 0x20;                      //rw:0, ms:0 adress: 0x20
  register_Value = 0xBF;                   //1011 1111. DR[7:6]10 (400Hz) BV[5:4]11 (110Hz cut-off f) PD[3]1 (chip enable) ZXYen[0:2]111 (axes enabled)
  write_data(reg_Address, register_Value, GYRO);
  
  //CTRL_REG2
  reg_Address = 0x21;                      //rw:0, ms:0 adress: 0x21
  register_Value = 0x00;                   //High-pass filter. edge trigger, sens trigger disable.
  write_data(reg_Address, register_Value, GYRO);
  
  //CTRL_REG3
  reg_Address = 0x22;                      //rw:0, ms:0 adress: 0x22
  register_Value = 0x00;                   //Interrupts disables.(Interrupt pin push pull)
  write_data(reg_Address, register_Value, GYRO);
  
  //CTRL_REG4
  reg_Address = 0x23;                      //rw:0, ms:0 adress: 0x23
  register_Value = 0x60;                   //0100 0100. BDU[7]:0 (Continuouse update), BLE[6]:1 (Data MSB @ lower address), FS[5:4]:10 (degrees per second 2000dps), IMP[3]:0 (sens latch disable), ST[2:1]:00 (selft test enable. Normal mode), SIM[0]: (4 wire SPI interface)
  write_data(reg_Address, register_Value, GYRO);
  
  //CTRL_REG5
  reg_Address = 0x24;                      //rw:0, ms:0 adress: 0x24
  register_Value = 0x00;                   //0000 0000, Normal mode reboot, FIFO disable, High pass filter disabled, 
  write_data(reg_Address, register_Value, GYRO);
  
}

/**
 * @brief Write data to SPI2
 * 
 * @param uint8_t adress to module
 * @param uint8_t data put at address
 * @param uint8_t type, if writing to gyro or accelerometer
 *
 * @detail Helper function to write data to SPI for the accelerometer and
 * gyroscope.
 */
void write_data(uint8_t address, uint8_t data, uint8_t type)
{
  // Activate the sensor in question
  if (type == GYRO)
  {
    HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_RESET);
  }
  else if (type == ACC)
  {
    HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_RESET);
  }

  uint8_t ADDRESS = address;
  //printf("Address value: %d \n", ADDRESS);//test

  uint8_t DATA = data;
  //printf("Data value: %d \n", DATA);//test

  // Send data
  HAL_SPI_Transmit(&hspi2, &ADDRESS, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &DATA, 1, 1000);
  
  // Disable sensors
  HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_SET);
}

/**
 * @brief System initialisation
 * 
 * @detail This function initiates the code generated from cubeMX.
 */
void systemInit(void){
/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
  
        //initialization of PWM-signal
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  
  /* Sensor Init Code */
  
  /* Setup sensor register */
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  
  l3gd20h_Init();  /*Init gyro*/
  lis3dh_Init();   /*Init accel*/ 

    /* End sensor Init Code */
}