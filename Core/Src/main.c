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
#include "MR_MAX30009.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* MAX30009 INT katkestus seab selle lipu; FIFO loetakse põhitsüklis, mitte katkestuse sees. */
extern volatile bool max30009_int;

/* UART käsud kogutakse bait-haaval kuni reavahetuseni. */
uint8_t uart_rx_byte;
uint8_t uart_cmd_buf[32];
uint8_t uart_cmd_idx = 0;
volatile bool uart_cmd_ready = false;

/* Hetkel aktiivne mõõtekonfiguratsioon: sageduse, ergutusvoolu ja võimenduse indeksid. */
uint8_t current_freq_idx = 52;
uint8_t current_stim_idx = 12;
uint8_t current_gain_idx = 0;

/* RAW sagedusskaneeringu parameetrid; tegelik skaneerimine tehakse main-loop'is. */
volatile bool    scan_running   = false;
volatile uint8_t scan_stim_idx  = 12;
volatile uint8_t scan_gain_idx  = 0;
volatile uint8_t scan_freq_start = 0;
volatile uint8_t scan_freq_end   = 52;

char tx_buf[128];
/* Suurem saatmispuhver mitme RAW rea korraga UART-i edastamiseks.
   Globaalne puhver vähendab stack'i kasutust ja UART kutsungite arvu. */
static char batch_buf[32 * 40];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void process_command(char *cmd);
static inline void uart_send(const char *buf, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* printf() suunatakse UART1 peale, et debug- ja mõõtetulemused jõuaksid arvutisse. */
int _write(int fd, char *ptr, int len)
{
	HAL_StatusTypeDef hstatus;
	if (fd == 1 || fd == 2)
	{
		hstatus = HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return len;
		else
			return -1;
	}
	return -1;
}

/* Väike abifunktsioon otseseks UART saatmiseks ilma printf vorminduseta. */
static inline void uart_send(const char *buf, uint16_t len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

/* Arvutab MAX30009 RAW real/imag väärtustest impedantsi hinnangu.
   K teisendab ADC ühikud pingeks; vool ja võimendus võetakse indeksite järgi tabelitest. */
void calculate_impedance(int32_t real_adc, int32_t imag_adc,
                         uint8_t stim_idx, uint8_t gain_idx)
{
    const double currents[16] = {
        1.6e-8, 3.2e-8, 8.0e-8, 1.6e-7, 3.2e-7, 6.4e-7,
        1.6e-6, 3.2e-6, 6.4e-6, 1.28e-5, 3.2e-5, 6.4e-5,
        1.28e-4, 2.56e-4, 6.4e-4, 1.28e-3
    };
    const double gains[4] = {1.0, 2.0, 5.0, 10.0};

    /* Kasutatav skaleerimiskonstant sõltub seadistusest/OSR-ist ja on määratud katsete põhjal. */
    double K = 110437.0;

    double Z_real = (double)real_adc  / (K * currents[stim_idx] * gains[gain_idx]);
    double Z_imag = (double)imag_adc  / (K * currents[stim_idx] * gains[gain_idx]);
    double Z_mag  = sqrt(Z_real*Z_real + Z_imag*Z_imag);
    double Z_phase = atan2(Z_imag, Z_real) * 57.2958;

    printf("|Z|=%.1f Ohm, R=%.1f, X=%.1f, Phase=%.2f deg\r\n",
           Z_mag, Z_real, Z_imag, Z_phase);

    float raw_mag = sqrt(real_adc*real_adc + imag_adc*imag_adc);
    printf("MAG = %.2f\r\n", raw_mag);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Algseis: MAX30009 initsialiseeritakse, seatakse vaikimisi sagedus/vool
     ning mõõtmine käivitatakse, et süsteem oleks kohe UART käskudeks valmis. */
  printf("\n\n\n-------------\nStarting\n");
  MAX30009_init(true);
  MAX30009_SetFreq(52, true);
  MAX30009_SetStimulusIndex(12, true);
  MAX30009_start_measurement(true);

  HAL_Delay(10);
  /* Kontrolllugemised kinnitavad, et MAX30009 on SPI kaudu vastamas ja seadistus jõudis registritesse. */
  uint8_t test_bioz1 = MAX30009_regRead(0x20);
  uint8_t test_sys   = MAX30009_regRead(0x11);
  uint8_t test_int_en = MAX30009_regRead(0x80);
  printf("CHECK: BIOZ1:0x%02X SYS:0x%02X INT_EN:0x%02X\r\n",
         test_bioz1, test_sys, test_int_en);
  uint8_t part_id = MAX30009_regRead(0xFF);
  printf("Part ID: 0x%02X\r\n", part_id);

  /* UART vastuvõtt käivitatakse katkestusega: iga saabuv bait töödeldakse callback'is. */
  HAL_StatusTypeDef uart_status = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  printf("UART IT status: %d\r\n", uart_status);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Põhitsükkel teeb ainult kolm asja: töötleb UART käske, teeb RAW skaneeringu
	     või loeb üksikmõõtmise tulemuse, kui MAX30009 INT lipp on aktiivne. */
	  if (uart_cmd_ready)
		 {
			 uart_cmd_ready = false;
			 process_command((char*)uart_cmd_buf);
			 uart_cmd_idx = 0;
		 }

	  if (scan_running) {
	      scan_running = false;

	      /* Sagedusindeksite vastavus hertsides; indeks saadetakse CSV RAW reas koos mõõtetulemusega. */
	      static const uint32_t freq_hz[60] = {
	          808960,722944,649216,581632,499712,468992,
	          420864,377856,338944,304128,272896,249856,
	          245248,220160,199936,176896,158976,143104,
	          131072,114944,99968,93056,82944,82048,
	          75008,66944,60032,54016,49984,43008,
	          41024,38976,35008,30976,28032,24992,
	          23008,20000,18016,16000,15008,14016,
	          13008,12000,11008,10000,9008,8000,
	          7008,6000,5000,4000,2000,1000,
	          500,250,125,64,32,16
	      };

	      /* Skaneering kasutab käsu hetkel aktiivset ergutusvoolu ja võimendust. */
	      MAX30009_SetStimulusIndex(scan_stim_idx, false);
	      MAX30009_SetBiozGain(scan_gain_idx, false);

	      printf("SCAN_START,%u,%u,%u,%u\r\n",
	             scan_stim_idx, scan_gain_idx,
	             scan_freq_start, scan_freq_end);

	      /* Iga sageduse kohta kogutakse kuni 100 RAW proovi ja saadetakse need arvutisse. */
	      for (uint8_t f = scan_freq_start; f <= scan_freq_end; f++) {
	          uint16_t samples_collected = 0;
	          uint32_t timeout_start;
	          int len;
	          uint16_t settle_ms;

	          /* Sageduse vahetamisel peatatakse mõõtmine, muudetakse MAX30009 seadistus
	             ja käivitatakse mõõteahel uuesti, et FIFO-sse ei jääks vana sageduse andmeid. */
	          MAX30009_stop_measurement(false);
	          HAL_Delay(20);

	          MAX30009_SetFreq(f, false);

	          max30009_int = false;

	          /* start_measurement() aktiveerib BioZ ahela, puhastab FIFO/status lipud
	             ja seadistab FIFO läve, mille täitumisel tekib INT signaal. */
	          MAX30009_start_measurement(false);

	          /* Pärast sageduse muutmist antakse analoogahelale stabiliseerumisaeg. */
	          if (f <= 30)           settle_ms = 50;   // kõrgemad sagedused
	          else if (f <= 51)      settle_ms = 200;  // keskvahemik
	          else                   settle_ms = 400;  // 2 kHz ja madalam

	          HAL_Delay(settle_ms);

	          /* INT ja status puhastatakse vahetult enne kogumise algust, et arvestus
	             algaks ainult stabiliseerumise järel tekkinud proovidest. */
	          MAX30009_regWrite(0x0E, 0x1A);
	          (void)MAX30009_regRead(0x00);
	          max30009_int = false;
	          timeout_start = HAL_GetTick();

	          while (samples_collected < 100) {
	              /* Timeout väldib lõputut ootamist, kui INT signaali või FIFO andmeid ei tule. */
	              if ((HAL_GetTick() - timeout_start) > 30000U) {
	                  MAX30009_stop_measurement(false);
	                  max30009_int = false;

	                  len = snprintf(tx_buf, sizeof(tx_buf),
	                                 "TIMEOUT,%u,%u\r\n", f, samples_collected);
	                  uart_send(tx_buf, len);
	                  break;
	              }

	              if (max30009_int) {
	                  max30009_int = false;

	                  /* INT tähendab, et FIFO lävi on täitunud; loetakse üks proovipakk. */
	                  struct raw_impedance samples[MAX30009_SAMPLE_COUNT_RAW];
	                  struct MAX30009_status status;
	                  uint8_t n = MAX30009_FifoReadSamples(samples, MAX30009_SAMPLE_COUNT_RAW, &status, false);

	                  /* Status- ja proovilipud kodeeritakse ühte flags välja, et PC-poolne analüüs
	                     saaks eristada ülevoolu, alavoolu, lead-off'i, markerit ja sample errorit. */
	                  uint8_t status_flags = 0;
	                  if (status.idrv_ovr)  status_flags |= 0x01;
	                  if (status.code_ovr)  status_flags |= 0x02;
	                  if (status.code_und)  status_flags |= 0x04;
	                  if (status.leadsoff)  status_flags |= 0x08;

	                  /* RAW read kogutakse esmalt batch_buf'i ja saadetakse ühe UART kutsungiga. */
	                  int batch_len = 0;
	                  for (uint8_t s = 0; s < n && samples_collected < 100; s++) {
	                      uint8_t flags = status_flags;
	                      if (samples[s].error)   flags |= 0x10;
	                      if (samples[s].marker)  flags |= 0x20;

	                      batch_len += snprintf(batch_buf + batch_len,
	                                            (int)sizeof(batch_buf) - batch_len,
	                                            /* CSV formaat: RAW,freq_idx,freq_hz,sample_no,real,imag,flags */
	                                            "RAW,%u,%lu,%u,%ld,%ld,%u\r\n",
	                                            f,
	                                            (unsigned long)freq_hz[f],
	                                            samples_collected,
	                                            (long)samples[s].realbits,
	                                            (long)samples[s].imagbits,
	                                            flags);
	                      samples_collected++;
	                  }
	                  if (batch_len > 0) {
	                      uart_send(batch_buf, (uint16_t)batch_len);
	                  }

	                  timeout_start = HAL_GetTick();
	              }
	          }

	          MAX30009_stop_measurement(false);
	          max30009_int = false;

	          /* Märgirida lihtsustab arvutis andmete jaotamist sageduste kaupa. */
	          len = snprintf(tx_buf, sizeof(tx_buf),
	                         "FREQ_DONE,%u,%u\r\n", f, samples_collected);
	          uart_send(tx_buf, len);
	      }

	      MAX30009_stop_measurement(false);
	      max30009_int = false;
	      printf("SCAN_END\r\n");
	  }
	  else if (max30009_int) {
	      /* Tavarežiimis loetakse FIFO keskmistatud tulemus ja prinditakse impedantsi hinnang. */
	      max30009_int = false;
	      struct MAX30009_status raw = MAX30009_FifoReadRaw(false);
	      if (raw.real_adc_avg != 0 || raw.imag_adc_avg != 0) {
	          calculate_impedance(raw.real_adc_avg, raw.imag_adc_avg,
	                             current_stim_idx, current_gain_idx);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 töötab master-režiimis ja suhtleb MAX30009 registrite/FIFO-ga. */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* USART1 on PC-side andme- ja käsuliides; suur baudrate võimaldab RAW skaneeringu andmeid kiiresti edastada. */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* CS hoitakse vaikimisi kõrgel; aktiivseks SPI tehinguks viiakse see draiveris madalaks. */
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Z_CS_Pin */
  GPIO_InitStruct.Pin = Z_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Z_CS_GPIO_Port, &GPIO_InitStruct);

  /* MAX30009 INT sisend tekitab EXTI katkestuse, kui FIFO/status vajab lugemist. */
  /*Configure GPIO pin : MAX30009_INT_Pin */
  GPIO_InitStruct.Pin = MAX30009_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAX30009_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	/* Katkestuses ei loeta SPI-d; seatakse ainult lipp, et töö teha turvaliselt main-loop'is. */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		if (GPIO_Pin == MAX30009_INT_Pin)
			max30009_int = true;
	}

	/* UART callback kogub käsu kuni '\n' märgini ja käivitab järgmise baidi vastuvõtu. */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	    if (huart->Instance == USART1)
	    {
	    	if (uart_rx_byte == '\n')
	    	{
	    	    uart_cmd_buf[uart_cmd_idx] = '\0';
	    	    if (uart_cmd_idx > 0) {
	    	        uart_cmd_ready = true;
	    	    }
	    	}
	    	else if (uart_rx_byte == '\r')
	    	{
	    	    /* CR ignoreeritakse, et toetada nii \n kui ka \r\n rea lõppe. */
	    	}
	    	else if (uart_cmd_idx < 31)
	    	{
	    	    uart_cmd_buf[uart_cmd_idx++] = uart_rx_byte;
	    	}
	        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	    }
	}

	/* Tekstipõhine UART käsuliides mõõteseadistuse muutmiseks ja RAW andmete kogumiseks.
	   Peamised käsud: F=sagedus, I=vool, G=võimendus, S/P=start/stop, R=RAW skaneering, ?=olek. */
	void process_command(char *cmd)
	{
	    while (*cmd == ' ' || *cmd == '\r' || *cmd == '\n') cmd++;

	    uint8_t val1, val2;
	    int parsed;

	    /* Enamik seadistuskäske loeb registri enne ja pärast kirjutust,
	       et UART väljundist oleks näha, kas kirjutus MAX30009-sse õnnestus. */

	    if (cmd[0] == 'F') {
	        parsed = atoi(cmd + 1);
	        if (parsed < 0 || parsed > 59) {
	            printf("ERR: freq index must be 0..59\r\n");
	            return;
	        }
	        MAX30009_stop_measurement(false);
	        /* SetFreq muudab registrit 0x20 (BIOZ_CFG_1, ADC_OSR+DAC_OSR)
	         * ja PLL registrid 0x17-0x1A. */
	        uint8_t pre20 = MAX30009_regRead(0x20);
	        uint8_t pre17 = MAX30009_regRead(0x17);
	        current_freq_idx = (uint8_t)parsed;
	        MAX30009_SetFreq(current_freq_idx, false);
	        max30009_int = false;
	        uint8_t post20 = MAX30009_regRead(0x20);
	        uint8_t r17 = MAX30009_regRead(0x17);
	        uint8_t r18 = MAX30009_regRead(0x18);
	        uint8_t r19 = MAX30009_regRead(0x19);
	        printf("SET F=%u  Reg0x20: pre=0x%02X -> post=0x%02X  "
	               "PLL: pre0x17=0x%02X -> post: 0x17=0x%02X 0x18=0x%02X 0x19=0x%02X\r\n",
	               current_freq_idx, pre20, post20, pre17, r17, r18, r19);
	    }
	    else if (cmd[0] == 'I') {
	        parsed = atoi(cmd + 1);
	        if (parsed < 0 || parsed > 15) {
	            printf("ERR: stimulus index must be 0..15\r\n");
	            return;
	        }
	        /* SetStimulusIndex kirjutab registrisse 0x22 (BIOZ_CFG_3) */
	        uint8_t pre22 = MAX30009_regRead(0x22);
	        uint8_t pre25 = MAX30009_regRead(0x25);
	        current_stim_idx = (uint8_t)parsed;
	        MAX30009_SetStimulusIndex(current_stim_idx, false);

	        uint8_t amp_rge;
	        if      (current_stim_idx <= 3)  amp_rge = 0;
	        else if (current_stim_idx <= 7)  amp_rge = 1;
	        else if (current_stim_idx <= 11) amp_rge = 2;
	        else                             amp_rge = 3;
	        MAX30009_SetBiozAmpRange(amp_rge, false);

	        uint8_t post22 = MAX30009_regRead(0x22);
	        uint8_t post25 = MAX30009_regRead(0x25);
	        printf("SET I=%u (AmpRange=%u auto)  Reg0x22: pre=0x%02X -> post=0x%02X  "
	               "Reg0x25: pre=0x%02X -> post=0x%02X\r\n",
	               current_stim_idx, amp_rge, pre22, post22, pre25, post25);
	    }
	    else if (cmd[0] == 'G') {
	        parsed = atoi(cmd + 1);
	        if (parsed < 0 || parsed > 3) {
	            printf("ERR: gain index must be 0..3\r\n");
	            return;
	        }
	        /* SetBiozGain muudab registrit 0x24 (BIOZ_CFG_5) bittides [1:0] */
	        uint8_t pre = MAX30009_regRead(0x24);
	        current_gain_idx = (uint8_t)parsed;
	        MAX30009_SetBiozGain(current_gain_idx, false);
	        uint8_t post = MAX30009_regRead(0x24);
	        printf("SET G=%u  Reg0x24: pre=0x%02X -> post=0x%02X\r\n",
	               current_gain_idx, pre, post);
	    }
	    else if (cmd[0] == 'H') {
	        val1 = atoi(cmd + 1);
	        /* SetAnalogHPF muudab registrit 0x24 (BIOZ_CFG_5) bittides [7:4] */
	        uint8_t pre = MAX30009_regRead(0x24);
	        MAX30009_SetAnalogHPF(val1, false);
	        uint8_t post = MAX30009_regRead(0x24);
	        printf("SET AHPF=%d  Reg0x24: pre=0x%02X -> post=0x%02X\r\n",
	               val1, pre, post);
	    }
	    else if (cmd[0] == 'D') {
	        if (strlen(cmd) < 6) {
	            printf("ERR: D command format is D<dhpf> <dlpf> (e.g. D1 2)\r\n");
	            return;
	        }
	        val1 = atoi(&cmd[2]);
	        val2 = atoi(&cmd[5]);
	        /* SetDigitalFilter muudab registrit 0x21 (BIOZ_CFG_2) */
	        uint8_t pre = MAX30009_regRead(0x21);
	        MAX30009_SetDigitalFilter(val1, val2, false);
	        uint8_t post = MAX30009_regRead(0x21);
	        printf("SET DHPF=%d DLPF=%d  Reg0x21: pre=0x%02X -> post=0x%02X\r\n",
	               val1, val2, pre, post);
	    }
	    else if (cmd[0] == 'A') {
	        /* Amp Range (0..3), käsitsi ülekirjutamiseks (I käsk ka seab selle automaatselt) */
	        val1 = atoi(cmd + 1);
	        if (val1 > 3) {
	            printf("ERR: AmpRange must be 0..3\r\n");
	            return;
	        }
	        /* SetBiozAmpRange muudab registrit 0x25 (BIOZ_CFG_6) */
	        uint8_t pre = MAX30009_regRead(0x25);
	        MAX30009_SetBiozAmpRange(val1, false);
	        uint8_t post = MAX30009_regRead(0x25);
	        printf("SET AmpRange=%u  Reg0x25: pre=0x%02X -> post=0x%02X\r\n",
	               val1, pre, post);
	    }
	    else if (cmd[0] == 'W') {
	        /* Amp BW (0..3) */
	        val1 = atoi(cmd + 1);
	        if (val1 > 3) {
	            printf("ERR: AmpBW must be 0..3\r\n");
	            return;
	        }
	        uint8_t pre = MAX30009_regRead(0x25);
	        MAX30009_SetBiozAmpBW(val1, false);
	        uint8_t post = MAX30009_regRead(0x25);
	        printf("SET AmpBW=%u  Reg0x25: pre=0x%02X -> post=0x%02X\r\n",
	               val1, pre, post);
	    }
	    else if (cmd[0] == 'C') {
	        /* DC Restore (0/1) */
	        val1 = atoi(cmd + 1);
	        if (val1 > 1) {
	            printf("ERR: DCRestore must be 0 or 1\r\n");
	            return;
	        }
	        uint8_t pre = MAX30009_regRead(0x25);
	        MAX30009_SetBiozDCrestore(val1, false);
	        uint8_t post = MAX30009_regRead(0x25);
	        printf("SET DCRestore=%u  Reg0x25: pre=0x%02X -> post=0x%02X\r\n",
	               val1, pre, post);
	    }
	    else if (cmd[0] == 'E') {
	        /* Ext Cap (0/1) */
	        val1 = atoi(cmd + 1);
	        if (val1 > 1) {
	            printf("ERR: ExtCap must be 0 or 1\r\n");
	            return;
	        }
	        uint8_t pre = MAX30009_regRead(0x25);
	        MAX30009_SetBiozExtCap(val1, false);
	        uint8_t post = MAX30009_regRead(0x25);
	        printf("SET ExtCap=%u  Reg0x25: pre=0x%02X -> post=0x%02X\r\n",
	               val1, pre, post);
	    }
	    else if (cmd[0] == 'Q') {
	        /* Q<hex_addr>  -> loe suvaline register debugiks, nt Q24 */
	        char *p = cmd + 1;
	        while (*p == ' ') p++;
	        if (!*p) {
	            printf("ERR: Q command needs register address in hex (e.g. Q24)\r\n");
	            return;
	        }
	        int addr = (int)strtol(p, NULL, 16);
	        if (addr < 0 || addr > 0xFF) {
	            printf("ERR: register address out of range\r\n");
	            return;
	        }
	        uint8_t v = MAX30009_regRead((uint8_t)addr);
	        printf("READ Reg0x%02X = 0x%02X (%u)\r\n", (uint8_t)addr, v, v);
	    }
	    else if (cmd[0] == 'X') {
	        /* X <hex_addr> <hex_val>  -> kirjuta registrisse, nt: X 28 08
	           Kasutatakse datasheeti kalibreerimismeetodi käsitsi-juhtimiseks
	           (BIOZ_DRV_RESET, BIOZ_Q_CLK_PHASE, BIOZ_I_CLK_PHASE jne).
	           Märkus: Kasutame X-käsku, sest W on AmpBW jaoks juba kasutusel. */
	        char *p = cmd + 1;
	        while (*p == ' ') p++;
	        if (!*p) {
	            printf("ERR: X command needs <hex_addr> <hex_val> (e.g. X 28 08)\r\n");
	            return;
	        }
	        int addr = (int)strtol(p, &p, 16);
	        if (addr < 0 || addr > 0xFF) {
	            printf("ERR: register address out of range\r\n");
	            return;
	        }
	        while (*p == ' ') p++;
	        if (!*p) {
	            printf("ERR: X command needs <hex_val> after addr\r\n");
	            return;
	        }
	        int val = (int)strtol(p, NULL, 16);
	        if (val < 0 || val > 0xFF) {
	            printf("ERR: register value out of range\r\n");
	            return;
	        }
	        uint8_t pre = MAX30009_regRead((uint8_t)addr);
	        MAX30009_regWrite((uint8_t)addr, (uint8_t)val);
	        uint8_t post = MAX30009_regRead((uint8_t)addr);
	        printf("WRITE Reg0x%02X: pre=0x%02X -> post=0x%02X (req=0x%02X)\r\n",
	               (uint8_t)addr, pre, post, (uint8_t)val);
	    }
	    else if (cmd[0] == 'S') {
	        MAX30009_stop_measurement(false);
	        HAL_Delay(50);
	        max30009_int = false;

	        MAX30009_start_measurement(false);
	        printf("MEASUREMENT started\r\n");
	    }
	    else if (cmd[0] == 'P') {
	        MAX30009_stop_measurement(false);
	        printf("MEASUREMENT stopped\r\n");
	    }
	    else if (cmd[0] == 'R') {
	        /* RAW skaneering ei käivitu callback'is, vaid seatakse lipp;
	           tegelik kogumine toimub põhitsüklis. */
	        scan_stim_idx = current_stim_idx;
	        scan_gain_idx = current_gain_idx;
	        scan_freq_start = 0;
	        scan_freq_end   = 52;

	        char *p = cmd + 1;
	        while (*p == ' ') p++;
	        if (*p) {
	            int a = atoi(p);
	            if (a >= 0 && a <= 59) scan_freq_start = (uint8_t)a;
	            while (*p && *p != ' ') p++;
	            while (*p == ' ') p++;
	            if (*p) {
	                int b = atoi(p);
	                if (b >= a && b <= 59) scan_freq_end = (uint8_t)b;
	            }
	        }
	        scan_running = true;
	        printf("RAW scan armed: stim=%u gain=%u freq=%u..%u\r\n",
	               scan_stim_idx, scan_gain_idx, scan_freq_start, scan_freq_end);
	    }
	    else if (cmd[0] == '?') {
	        printf("Config: F=%u I=%u G=%u\r\n",
	               current_freq_idx, current_stim_idx, current_gain_idx);
	        /* Dump kõik olulised registrid, et kasutaja näeks hetke olekut */
	        uint8_t r20 = MAX30009_regRead(0x20);  // BIOZ_CFG_1 (OSR)
	        uint8_t r21 = MAX30009_regRead(0x21);  // BIOZ_CFG_2 (DHPF, DLPF)
	        uint8_t r22 = MAX30009_regRead(0x22);  // BIOZ_CFG_3 (stim mag)
	        uint8_t r23 = MAX30009_regRead(0x23);  // BIOZ_CFG_4
	        uint8_t r24 = MAX30009_regRead(0x24);  // BIOZ_CFG_5 (AHPF, gain)
	        uint8_t r25 = MAX30009_regRead(0x25);  // BIOZ_CFG_6 (AmpRange, AmpBW, DCR, ExtCap)
	        uint8_t r28 = MAX30009_regRead(0x28);  // BIOZ_CFG_7
	        printf("Registers: 0x20=0x%02X 0x21=0x%02X 0x22=0x%02X 0x23=0x%02X "
	               "0x24=0x%02X 0x25=0x%02X 0x28=0x%02X\r\n",
	               r20, r21, r22, r23, r24, r25, r28);
	    }
	    else if (cmd[0] == '#') {
	        /* Hash/help — lühike käsulist */
	    	printf("Commands: F<idx> I<idx> G<idx> H<idx> D<dhpf> <dlpf> "
	    	           "A<rge> W<bw> C<0/1> E<0/1> Q<hex_addr> "
	    	           "X<hex_addr> <hex_val> S P R[a [b]] ?\r\n");
	    }
	    else {
	        printf("Unknown command: %s\r\n", cmd);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
