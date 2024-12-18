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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mongoose.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNIT_ID 0x01
#define COIL_OUT_BLOCK_SIZE 128
#define COIL_IN_BLOCK_SIZE 128
#define REG_IN_BLOCK_SIZE 100
#define REG_OUT_BLOCK_SIZE 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t counter = 1;
struct mg_connection* conn = NULL;
bool connected = false;
uint8_t modbus_coil_out_mem[COIL_OUT_BLOCK_SIZE];
const uint8_t modbus_coil_in_mem[COIL_IN_BLOCK_SIZE] =
{
  0x49,
  0xF1,
  0xAA,
  0x96,
  0xB0,
  0x72,
  0xC9,
  0x15,
  0x47,
  0x01
};
const uint16_t modbus_reg_in_mem[REG_IN_BLOCK_SIZE] =
{
  0xFFDC,
  0x3F1D,
  0xC840,
  0x59E6,
  0x8B0F,
  0xF723,
  0x2CA9,
  0xD15E,
  0x6478,
  0x90F1
};
uint16_t modbus_reg_out_mem[REG_OUT_BLOCK_SIZE] =
{
  0xA3F2,
  0x5B7C,
  0xC1D4,
  0x8E09,
  0xF4A6,
  0x23E1,
  0x9B7F,
  0x4D92,
  0x6AC8,
  0xEF3B
};

typedef enum
{
  ILLEGAL_FUNCTION = 0x01,
  ILLEGAL_DATA_ADDRESS = 0x02,
  ILLEGAL_DATA_VALUE = 0x03,
  SLAVE_DEVICE_FAILURE = 0x04,
  SLAVE_DEVICE_BUSY  = 0x06
} ModbusExceptionCode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void modbus_exception(ModbusExceptionCode error_code)
{
  MG_ERROR(("MODBUS EXCEPTION : %d", error_code));
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 1000);
  return len;
}

/*uint64_t mg_millis(void)
{
  return HAL_GetTick();
}*/

bool glue_modbus_read_coil(uint8_t func, uint16_t start, uint8_t bit_position, bool *coil_state)
{
  if(func == 1)//coil
  {
      if(start > COIL_OUT_BLOCK_SIZE)
      {
          //MG_ERROR(("INVALID COIL ADDRESS"));
          modbus_exception(ILLEGAL_DATA_ADDRESS);
          return 0;
      }
      *coil_state = (bool)((modbus_coil_in_mem[start] >> bit_position) & 0x01);
      return 1;
  }
  else if(func == 2)//discrete input
  {
      if(start > COIL_IN_BLOCK_SIZE)
      {

          modbus_exception(ILLEGAL_DATA_ADDRESS);
          return 0;
      }
     *coil_state = (bool)((modbus_coil_in_mem[start] >> bit_position) & 0x01);
      return 1;
  }
  else return 0;
}

bool glue_modbus_write_coil(uint16_t start, uint8_t bit_position, bool coil_state)
{
  if(start > COIL_OUT_BLOCK_SIZE)
  {
    MG_ERROR(("INVALID COIL ADDRESS"));
    return 0;
  }
  if(coil_state == 1)
  {
       modbus_coil_out_mem[start] |= (1 << bit_position);
  }
  else
  {
       modbus_coil_out_mem[start] &= ~(1 << bit_position);
  }
  return 1;
}

bool glue_modbus_write_reg(uint16_t start, uint16_t value)
{
    if(start > REG_OUT_BLOCK_SIZE)
    {
      MG_ERROR(("INVALID REG ADDRESS"));
      //modbus_exception(ILLEGAL_DATA_ADDRESS);
      return 0;
    }
    modbus_reg_out_mem[start] = value;
    return 1;


}

bool glue_modbus_read_reg(uint8_t func, uint16_t start, uint16_t* value)
{

  switch(func)
  {
    case 3://read holding register
      if(start > REG_IN_BLOCK_SIZE )
      {
      modbus_exception(ILLEGAL_DATA_ADDRESS);
      return 0;
      }
      * value = modbus_reg_out_mem[start];
      return 1;
    case 4://read input register
      if(start > REG_OUT_BLOCK_SIZE )
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        return 0;
      }
     * value = modbus_reg_in_mem[start];
      return 1;
    default:
     modbus_exception(ILLEGAL_FUNCTION);
     return 0;

  }
}

static void handle_modbus_pdu(struct mg_connection *c, uint8_t *buf,
                              size_t len) {
  MG_DEBUG(("Received PDU %p len %lu, hexdump:", buf, len));
  mg_hexdump(buf, len);
  // size_t hdr_size = 8, max_data_size = sizeof(response) - hdr_size;
  if (len < 12) {
    MG_ERROR(("PDU too small"));
  } else {
    uint8_t func = buf[7];  // Function
    bool success = false;
    size_t response_len = 0;
    uint8_t response[260] = {0};
    memcpy(response, buf, 8);
    uint16_t tid = mg_ntohs(*(uint16_t *) &buf[0]);  // Transaction ID
    uint16_t pid = mg_ntohs(*(uint16_t *) &buf[2]);  // Protocol ID
    uint16_t len = mg_ntohs(*(uint16_t *) &buf[4]);  // PDU length
    uint8_t uid = buf[6];                            // Unit identifier
    if (func == 6) {  // write single holding register
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t value = mg_ntohs(*(uint16_t *) &buf[10]);
      success = glue_modbus_write_reg(start, value);
      if(success == false)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;

      }
      *(uint16_t *) &response[8] = mg_htons(start);
      *(uint16_t *) &response[10] = mg_htons(value);
      response_len = 12;
      MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
    } else if (func == 16) {  // Write multiple
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t num = mg_ntohs(*(uint16_t *) &buf[10]);
      if(num > 123)
      {
        modbus_exception(ILLEGAL_DATA_VALUE);
        success = false;
        goto error_check;
      }
      uint16_t i, *data = (uint16_t *) &buf[13];
      if ((size_t) (num * 2 + 10) < sizeof(response)) {
        for (i = 0; i < num; i++) {
          success =
              glue_modbus_write_reg((uint16_t) (start + i), mg_htons(data[i]));
          if (success == false)
          {
             modbus_exception(ILLEGAL_DATA_ADDRESS);
             response[8] = ILLEGAL_DATA_ADDRESS;
             goto error_check;
          }
        }
        *(uint16_t *) &response[8] = mg_htons(start);
        *(uint16_t *) &response[10] = mg_htons(num);
        response_len = 12;
        MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
      }
    } else if (func == 3 || func == 4) {  // Read multiple
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t num = mg_ntohs(*(uint16_t *) &buf[10]);
      if ((size_t) (num * 2 + 9) < sizeof(response)) {
        uint16_t i, val, *data = (uint16_t *) &response[9];
        for (i = 0; i < num; i++) {
          success = glue_modbus_read_reg(func,(uint16_t) (start + i), &val);
          if (success == false) break;
          data[i] = mg_htons(val);
        }
        response[8] = (uint8_t) (num * 2);
        response_len = 9 + response[8];
        MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
      }
    }
    else if(func == 5)//write single coil
    {
      response_len = 12;
      success = true;
      response[8] = buf[8];//copy the address bytes in the response buffer
      response[9] = buf[9];
      response[10] = buf[10];
      response[11] = buf[11];
     uint16_t start_byte = (uint16_t)(buf[8] << 8 |  buf[9]);
      if(start_byte > COIL_OUT_BLOCK_SIZE)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        response[8] = ILLEGAL_DATA_ADDRESS;
        success = false;
        goto error_check;
      }

      uint16_t byte_loc = start_byte / 8;
      uint8_t bit_pos = start_byte % 8;
      if(buf[10] == 0xFF && buf[11] == 0x00)
      {
          success &= glue_modbus_write_coil(byte_loc, bit_pos, true);
      }
      else if(buf[10] == 0x00 && buf[11] == 0x00)
      {
          success &= glue_modbus_write_coil(byte_loc, bit_pos, false);
      }
      else
      {
         MG_INFO(("Invalid coil value, command ignored"));

      }
    }
    else if(func == 15)//write multiple coils
    {
      success = true;
      uint16_t start_byte = (uint16_t)(buf[8] << 8 | buf[9]);

      uint16_t byte_loc = start_byte / 8;
      uint8_t bit_pos = start_byte % 8;
      uint16_t coils_count = (uint16_t)((buf[10] << 8) | buf[11]);
      uint16_t buffer_start_index = 13;//coils value starts at index 13
      uint8_t shift_index = 0;//for comparing with the buffer;
      response[8] = buf[8];//copy the address bytes in the response buffer
      response[9] = buf[9];
      response[10] = buf[10];
      response[11] = buf[11];
      response_len = 12;
      if((start_byte  + coils_count / 8)> COIL_OUT_BLOCK_SIZE)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        success = false;
         response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;
      }
      if(coils_count > 1968)
      {
        success = false;
        modbus_exception(ILLEGAL_DATA_VALUE);
        response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;
      }
      for(uint16_t i = 0; i < coils_count; i++)
      {
        if(((buf[buffer_start_index] >> shift_index) & 0x01) == 1)
        {
            success &= glue_modbus_write_coil(byte_loc, bit_pos, true);
        }
        else
        {
            success &= glue_modbus_write_coil(byte_loc, bit_pos, false);
        }
        bit_pos ++;
        shift_index++;
        if(bit_pos == 8)
        {
          bit_pos = 0;
          byte_loc++;
        }
        if(shift_index == 8)
        {
          shift_index = 0;
          buffer_start_index++;
        }
      }

    }
    else if(func == 1 || func == 2)//read coil or input
    {
        success = true;
        uint16_t coil_addr  = (uint16_t)((buf[8] << 8) | buf[9]);//
        uint8_t bit_pos = coil_addr % 8;
        uint16_t byte_num = coil_addr / 8;
        bool coil_bit_value = false;
        uint16_t coils_count = (uint16_t)((buf[10] << 8) | buf[11]);//
        if(coils_count > 2000)
        {
          modbus_exception(ILLEGAL_DATA_VALUE);
          success = false;
          response[8] = ILLEGAL_DATA_VALUE;
          goto error_check;
        }

        // uint16_t buffer_start_index = 13;
        uint8_t response_bit_index = 0 ;
        uint16_t resp_index = 9;
        response[8] = (coils_count / 8) + (bool)((coils_count % 8) > 0);
        if(response[8] > COIL_IN_BLOCK_SIZE)
        {
          modbus_exception(ILLEGAL_DATA_VALUE);
          success = false;
          response[8] = ILLEGAL_DATA_VALUE;
          goto error_check;
        }
        for(uint16_t i = 0; i < coils_count; i++)
        {
          if(glue_modbus_read_coil(func,byte_num, bit_pos, &coil_bit_value) == true)
          {
            if(coil_bit_value == true)
            {
              response[resp_index] |= (1 << response_bit_index);
            }
          }
          else
          {
            success = false;
            modbus_exception(ILLEGAL_DATA_VALUE);
            response[8] = ILLEGAL_DATA_VALUE;
            goto error_check;
          }
          bit_pos ++;
          response_bit_index++;
          if(bit_pos == 8)
          {
            bit_pos = 0;
            byte_num++;
          }
          if(response_bit_index == 8)
          {
            response_bit_index = 0;
            resp_index++;
          }
        }
         response_len = 9 +resp_index;
    }
error_check :
    if (success == false) {
      response_len = 9;
      response[7] |= 0x80;
      //response[8] = 4;  // Server Device Failure
    }
    *(uint16_t *) &response[4] = mg_htons((uint16_t) (response_len - 6));
    MG_DEBUG(("Sending PDU response %lu:", response_len));
    mg_hexdump(response, response_len);
    mg_send(c, response, response_len);
  }
}

void modbus_ev_handler(struct mg_connection *c, int ev, void *ev_data) {

  if (ev == MG_EV_READ) {
    uint16_t len;

    if (c->recv.len < 7) return;  // Less than minimum length, buffer more
    len = mg_ntohs(*(uint16_t *) &c->recv.buf[4]);  // PDU length
    MG_INFO(("Got %lu, expecting %lu", c->recv.len, len + 6));
    if (c->recv.len < len + 6U) return;          // Partial frame, buffer more
    if(c->recv.buf[6] != UNIT_ID)
    {
      MG_INFO(("UNIT ID NOT MATCHING"));
      return;
    }
    handle_modbus_pdu(c, c->recv.buf, len + 6);  // Parse PDU and call user
    mg_iobuf_del(&c->recv, 0, len + 6U);         // Delete received PDU
  }
  (void) ev_data;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void web_fn(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG)
  {
    mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "Hello, %s\nCounter = %d", "mongoose", counter++);
  }
}

void log_fn(char ch, void *param)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	struct mg_mgr mgr;

	mg_log_set(MG_LL_DEBUG);

	mg_log_set_fn(log_fn, NULL);

	mg_mgr_init(&mgr);

	  // Initialise Mongoose network stack
	struct mg_tcpip_driver_stm32f_data driver_data = {
		.mdc_cr = 4,
		.phy_addr = 0
	};

	// set mac address using UID of micro
	// first 3 bytes are ST Micro's Vendor ID
	uint8_t mac[] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};

	uint32_t uid = HAL_GetUIDw0();

	mac[3] = (uid >> 16) & 0xFF;
	mac[4] = (uid >> 8) & 0xFF;
	mac[5] = uid & 0xFF;

	struct mg_tcpip_if mif = {
		.ip = mg_htonl(MG_U32(192, 168, 68, 44)),
		.mask = mg_htonl(MG_U32(255, 255, 252, 0)),
		.gw = mg_htonl(MG_U32(192, 168, 68, 1)),
		.driver = &mg_tcpip_driver_stm32f,
		.driver_data = &driver_data
	};

	memcpy(&mif.mac, mac, 6);

	mg_tcpip_init(&mgr, &mif);
	mg_listen(&mgr, "tcp://0.0.0.0:502", modbus_ev_handler, NULL);
  /* Infinite loop */
  for(;;)
  {
	  mg_mgr_poll(&mgr, 1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
