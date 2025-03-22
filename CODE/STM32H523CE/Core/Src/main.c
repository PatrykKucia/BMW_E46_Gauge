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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TURN_RAPID        0x80
#define TURN_RIGHT        0x40
#define TURN_LEFT         0x20
#define FOG_REAR          0x10
#define FOG_FRONT         0x08
#define BEAM_HIGH         0x04
#define BEAM_LOW          0x02
#define PARKING           0x01
#define BUFFER_SIZE 128  // Rozmiar bufora odbiorczego
#define CCM_LIC_PLATE     0x80
#define CCM_TURN_RIGHT    0x40
#define CCM_TURN_LEFT     0x20
#define CCM_FOG_REAR      0x10
#define CCM_FOG_FRONT     0x08
#define CCM_HIGH_BEAM     0x04
#define CCM_LOW_BEAM      0x02
#define CCM_PARKING       0x01

#define CCM_REVERSE       0x20
#define INDICATORS        0x04
#define CCM_BRAKE         0x02

#define FOG_REAR_SWITCH   0x40
#define KOMBI_LOW_LEFT    0x20
#define KOMBI_LOW_RIGHT   0x10
#define KOMBI_BRAKE_LEFT  0x02

#define MIN_SPEED 0        // 0 km/h
#define MAX_SPEED 255      // 255 km/h
#define MIN_FREQ 100       // 100 Hz
#define MAX_FREQ 1770000   // 1770 kHz
#define PCLK1_FREQ 250000000 // 250 MHz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader_DME1;
FDCAN_TxHeaderTypeDef RxHeader2;
uint8_t               TxData_DME1[8]= {0xff, 0xff, 0xff, 0x1e, 0x55, 0x66, 0x77, 0x88};
uint8_t               RxData2[8];
uint32_t              TxMailbox;
#define RX_BUFFER_SIZE 25600


uint8_t rxDatar;  // Pojedynczy bajt do odbioru

#define UART_TIMEOUT 500  // Timeout dla UART w ms
#define WIFI_SSID "PLAY_Swiatlowod_19A1"
#define WIFI_PASS "t8Xv9auf7Z#D"
#define BUFFER_SIZE 256  // Zapasowy bufor na dane
uint8_t uartRxBuffer[256]; // Bufor dla odbieranych danych
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ICACHE_Init(void);
#define MAX_PACKET_SIZE 128
/* USER CODE BEGIN PFP */
typedef struct {
    uint32_t time;
    char car[4];
    uint16_t flags;
    char gear;
    char plid;
    float speed;
    float rpm;
    float turbo;
    float engTemp;
    float fuel;
    float oilPressure;
    float oilTemp;
    uint32_t dashLights;
    uint32_t showLights;
    float throttle;
    float brake;
    float clutch;
    char display1[16];
    char display2[16];
    int id;
} xxx;

// Bufor na dane odbierane z ESP32

char rxBuffer[BUFFER_SIZE];
uint8_t packetBuffer[MAX_PACKET_SIZE];
volatile int rxIndex = 0;
int packetLength = 0;
bool packetReady = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
    STATE_SEARCHING,  // Szukanie nagłówka "+IPD"
    STATE_READING_LEN, // Odczyt długości pakietu
    STATE_READING_DATA // Odczyt danych
} ParserState;

ParserState state = STATE_SEARCHING;
uint16_t packet_length = 0;  // Długość pakietu
uint16_t data_index = 0;     // Indeks w buforze danych
uint8_t packet_buffer[256];  // Bufor na dane pakietu
void ProcessByte(uint8_t byte) {
    static uint8_t ipd_header_index = 0;
    static const uint8_t ipd_header[] = {0x2B, 0x49, 0x50, 0x44};  // "+IPD" w hex

    switch (state) {
        case STATE_SEARCHING:
            if (byte == ipd_header[ipd_header_index]) {
                ipd_header_index++;
                if (ipd_header_index == sizeof(ipd_header)) {
                    // Znaleziono pełny nagłówek +IPD
                    state = STATE_READING_LEN;
                    ipd_header_index = 0;
                    packet_length = 0;
                }
            } else {
                ipd_header_index = (byte == ipd_header[0]) ? 1 : 0;
            }
            break;

        case STATE_READING_LEN:
            if (byte >= '0' && byte <= '9') {
                packet_length = packet_length * 10 + (byte - '0');
                if (packet_length >= BUFFER_SIZE) {
                    state = STATE_SEARCHING;
                    packet_length = 0;
                }
            } else if (byte == ':') {
                state = STATE_READING_DATA;
                data_index = 0;
            } else {
                state = STATE_SEARCHING;
                packet_length = 0;
            }
            break;

        case STATE_READING_DATA:
            if (data_index < packet_length && data_index < BUFFER_SIZE) {
                packet_buffer[data_index++] = byte;
            }

            if (data_index >= packet_length) {
                // Otrzymano pełny pakiet
                if (packet_length >= sizeof(xxx)) {
                    xxx received_packet;
                    memcpy(&received_packet, packet_buffer, sizeof(xxx));

                    // Wypisanie wartości
                    printf("Speed: %.2f m/s\n", received_packet.speed);
                    printf("RPM: %.2f\n", received_packet.rpm);
                    printf("Throttle: %.2f\n", received_packet.throttle);
                    printf("Brake: %.2f\n", received_packet.brake);
                    printf("Clutch: %.2f\n", received_packet.clutch);
                    printf("Gear: %d\n", received_packet.gear);
                    printf("Turbo: %.2f BAR\n", received_packet.turbo);
                    printf("Engine Temp: %.2f C\n", received_packet.engTemp);
                }

                state = STATE_SEARCHING;
            }
            break;
    }
}

void ESP32_SendCommand(const char* command) {
    HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Końcówka komendy AT
    HAL_Delay(100);  // Czekaj na odpowiedź
}

// Funkcja do odbierania danych z ESP32
void ESP32_ReceiveData(uint8_t* buffer, uint16_t size) {
    HAL_UART_Receive(&huart1, buffer, size, HAL_MAX_DELAY);
}

// Funkcja do parsowania danych UDP
void ParseUDPPacket(uint8_t* data, xxx* packet) {
    memcpy(packet, data, sizeof(xxx));  // Skopiuj dane do struktury
}
//int __io_putchar(int ch) //function used to print() in usart
//{
//  if (ch == '\n') {
//    __io_putchar('\r');
//
//  }
//
//  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
//
//  return 1;
//}
void Set_PWM_Frequency(uint16_t speed_kmh) {
    // Poprawiona interpolacja częstotliwości
    uint32_t freq = 100 + ((1700 - 100) * speed_kmh) / 250;

    uint32_t arr_value, psc_value;

    if (freq < 3800) {
        psc_value = (250000000 / (65536 * freq));
        if (psc_value > 65535) psc_value = 65535;  // PSC nie może przekroczyć 16 bitów
        arr_value = (250000000 / ((psc_value + 1) * freq)) - 1;
    } else {
        psc_value = 0;
        arr_value = (250000000 / freq) - 1;
    }

    if (arr_value > 65535) arr_value = 65535;  // Ograniczenie ARR do 16 bitów
    //printf("Speed: %d km/h, Freq: %lu Hz, PSC: %lu, ARR: %lu\n", speed_kmh, freq, psc_value, arr_value);

    __HAL_TIM_SET_PRESCALER(&htim1, psc_value);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr_value);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // W niektórych przypadkach wymagane jest wygenerowanie zdarzenia aktualizacji
    //__HAL_TIM_GENERATE_EVENT(&htim1, TIM_EVENTSOURCE_UPDATE);
}

void PWM_UpdateFrequency(TIM_HandleTypeDef *htim, uint8_t speed) {
    uint32_t frequency = PWM_frequency(speed);

    // Obliczanie prescalera
    uint32_t prescaler = 1;
    while (frequency < PCLK1_FREQ / 256 && prescaler < 256) {
        prescaler *= 2;
    }

    // Obliczanie ARR (auto-reload) na podstawie częstotliwości PWM i preskalera
    uint32_t arr = (PCLK1_FREQ / (frequency * prescaler)) - 1;

    // Ustawienie wartości prescalera i ARR
    __HAL_TIM_SET_PRESCALER(htim, prescaler - 1);
     __HAL_TIM_SET_AUTORELOAD(htim, arr);
     __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, arr);  // Ustawienie wypełnienia 50%
 }
xxx packet;
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
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
 // HAL_UART_Receive_IT(&huart1, &rxDatar, 1);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer[rxIndex], 1);
  MX_USART2_UART_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  //initESP();
  //TIM1->CCR1 = 50;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Konfiguracja ESP32 za pomocą komend AT
     ESP32_SendCommand("AT+RST");  // Resetuj ESP32
    // HAL_Delay(1000);
    // ESP32_SendCommand("AT+CWMODE=1");  // Ustaw tryb stacji (klient Wi-Fi)
    // HAL_Delay(1000);
    // ESP32_SendCommand("AT+CWJAP=\"PLAY_Swiatlowod_19A1\",\"t8Xv9auf7Z#D\"");  // Połącz z Wi-Fi
     HAL_Delay(5000);
     ESP32_SendCommand("AT+CIPSTART=\"UDP\",\"0.0.0.0\",12345,12345,2");  // Ustaw tryb UDP
     HAL_Delay(1000);
     //ESP32_SendCommand("AT+CIPRECVMODE=1");  // Włącz tryb odbioru danych
     //HAL_Delay(1000);
     xxx received_packet;


  //HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0);
  //HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

  // Konfiguracja wiadomości



	 TxData_DME1[0] = 0x19 ;                     // LV_SWI_IGK=1, LV_F_N_ENG=0, LV_ACK_TCS=0, LV_ERR_GC=1, SF_TQD=1
	 TxData_DME1[1] = 0x4C;      // TQI_TQR_CAN = 75%
	 uint16_t engineSpeed = (uint16_t)(3000 / 0.15625);
	 TxData_DME1[2] = 0xD0; // N_ENG LSB
	 TxData_DME1[3] = (uint8_t)(engineSpeed >> 8);   // N_ENG MSB
	 TxData_DME1[4] = (uint8_t)(60 / 0.390625);      // TQI_CAN = 60%
	 TxData_DME1[5] = (uint8_t)(5 / 0.390625);       // TQ_LOSS_CAN = 5%
	 TxData_DME1[6] = 0b11000000;                   // ERR_AMT_CAN bits
	 TxData_DME1[7] = (uint8_t)(80 / 0.390625);      // TQI_MAF_CAN = 80%

  TxHeader_DME1.Identifier = 0x316;  // ID ramki
  TxHeader_DME1.IdType = FDCAN_STANDARD_ID;
  TxHeader_DME1.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader_DME1.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader_DME1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader_DME1.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader_DME1.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader_DME1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;



  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x316;
  sFilterConfig.FilterID2 = 0x1FFFFFFF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
      /* Filter configuration Error */
      printf("[CAN] Unable to configure!\n");
  }
  if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
  {
   Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  printf("starting\n");
  HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
  //HAL_GPIO_WritePin(GPIO_PIN_3, GPIOB, GPIO_PIN_RESET);
  uint8_t piecpiec[]     = {0x55};

  uint8_t a[]     = {  0x3f, 0x0b, 0xBF ,0x0c, 0x00, 0x00, 0x00,0x00, 0x00 ,0x00, 0x01, 0x06 };
  uint8_t turnRight[]     = {0xD0, 0x07, 0xBF, 0x5B, 0x43, 0x83, 0x2E, 0x3F, 0xE2, '\n'};
  uint8_t turnLeft[]      = {0xD0, 0x07, 0xBF, 0x5B, 0x23, 0x83, 0x0E, 0x3F, 0xA2};
  uint8_t hazardLights[]  = {0xD0, 0x07, 0xBF, 0x5B, 0x63, 0x83, 0x0E, 0x3F, 0xE2};
  uint8_t highBeam1[] = {0xd0, 0x07, 0xbf, 0x5b, 0x07, 0x83, 0x0a, 0x3f, 0x82, '\n'};  // Zakończone LF (Line Feed)
  uint8_t highBeam2[] = {0xd0, 0x07, 0xbf, 0x5b, 0x07, 0x83, 0x0a, 0x3f, 0x82, '\r'};  // Zakończone CR (Carriage Return)
  uint8_t highBeam3[] = {0xd0, 0x07, 0xbf, 0x5b, 0x07, 0x83, 0x0a, 0x3f, 0x82, '\r', '\n'}; // Zakończone CRLF (Carriage Return + Line Feed)
  uint8_t highBeam4[] = {0xd0, 0x07, 0xbf, 0x5b, 0x07, 0x83, 0x0a, 0x3f, 0x82, '\r', '\n'};
  uint8_t testing[] = {0xd0, 0x07, 0xbf, 0x5b, 0x01 , 0xC9 , 0x02 , 0x02 , 0xFB, '\n'}; // Zakończone NULL (znak końca ciągu w stylu C)
  uint8_t byte_before_newline = highBeam4[8];  // 0x82
  uint8_t new_byte = (byte_before_newline << 1); // Przesuwamy 0x82 w lewo o 1 bit, aby zrobić miejsce na bit LOW

  // Wstawiamy nowy bajt do tablicy
  highBeam4[8] = new_byte;  // Zaktualizowana wartość 0x82 -> 0x04
  uint8_t highBeam5[] = {0xd0, 0x07, 0xbf, 0x5b, 0x07, 0x83, 0x0a, 0x3f, 0x82, 0xFF}; // Zakończone 0xFF (często używane w niektórych protokołach)
  uint8_t stopTurning[]   = {0xD0, 0x07, 0xBF, 0x5B, 0x03, 0x83, 0x0A, 0x3F, 0x86};
  uint8_t lcdTurnOff[]    = {0x30, 0x19, 0x80, 0x1A, 0x30, 0x00, 0x20, 0x20, 0x20,
                             0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
                             0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x83, 0x80, 0x04,
                             0x30, 0x1B, 0x00, 0x8F};

  uint8_t turnRight_noChecksum[] = {0xD0, 0x07, 0xBF, 0x5B, 0x40, 0x40, 0x32, 0x40};
  uint8_t turnLeft_noChecksum[]  = {0xD0, 0x07, 0xBF, 0x5B, 0x23, 0x83, 0x0E, 0x3F};
  uint8_t turnRight_XOR[9];
  uint8_t turnLeft_XOR[9];
  uint8_t calculateXORChecksum(uint8_t *data, uint16_t length) {
      uint8_t checksum = 0;
      for (uint16_t i = 0; i < length; i++) {
          checksum ^= data[i];
      }
      return checksum;
  }
  uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < length; i++) {
          checksum ^= data[i];
      }
      return checksum;
  }
  void send_cluster_request(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
      uint8_t frame[9];
      frame[0] = 0xD0;
      frame[1] = 0x07; // Długość wiadomości
      frame[2] = 0xBF;
      frame[3] = 0x5B; // ID Cluster Indicators Request (0x5A)
      frame[4] = byte1;
      frame[5] = byte2;
      frame[6] = byte3;
      frame[7] = byte4;
      frame[8] = calculate_checksum(frame, 8); // Checksum XOR
 	  HAL_UART_Transmit(&huart2, frame, sizeof(frame), 500);
  }
  void send_lin_frame(uint8_t identifier, uint8_t *data, uint8_t data_length) {
      if (data_length > 8) return;  // Maksymalnie 8 bajtów danych w polu Data Field

      uint8_t frame[11];  // Maksymalny rozmiar ramki: 1 bajt ID + 8 bajtów danych + 1 bajt sumy kontrolnej
      frame[0] = identifier;

      // Kopiowanie danych
      for (uint8_t i = 0; i < data_length; i++) {
          frame[1 + i] = data[i];
      }

      // Obliczanie sumy kontrolnej
      uint8_t checksum = calculateXORChecksum(frame, 1 + data_length);
      frame[1 + data_length] = checksum;

      // Wysyłanie pola Break
      __HAL_UART_SEND_REQ(&huart1, UART_SENDBREAK_REQUEST);
      HAL_Delay(1);  // Krótka przerwa po Break

      // Wysyłanie pola Sync
      uint8_t sync = 0x55;
      HAL_UART_Transmit(&huart1, &sync, 1, 100);

      // Wysyłanie pola Identifier, Data i Checksum
      HAL_UART_Transmit(&huart1, frame, 2 + data_length, 100);
  }

  void prepareChecksumFrames() {
      memcpy(turnRight_XOR, turnRight_noChecksum, 8);
      turnRight_XOR[8] = calculateXORChecksum(turnRight_noChecksum, 8);

      memcpy(turnLeft_XOR, turnLeft_noChecksum, 8);
      turnLeft_XOR[8] = calculateXORChecksum(turnLeft_noChecksum, 8);
  }
  // Funkcja wysyłania danych przez UART
  void sendFrame(uint8_t *frame, uint16_t size) {
	  HAL_LIN_SendBreak(&huart2);
	  HAL_UART_Transmit(&huart2, piecpiec, sizeof(piecpiec), 100);
      HAL_UART_Transmit(&huart2, frame, size, 100);
  }

 // HAL_UART_Transmit(&huart2, turnRight, sizeof(turnRight), 500);
  //HAL_UART_Transmit(&huart2, frame, sizeof(frame), HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart2, frame2, sizeof(frame), HAL_MAX_DELAY);
  uint8_t rxData[1]; // Bufor na 1 bajt danych
  uint16_t speed = 0;
	    int8_t direction = 1; // 1 = rośnie, -1 = maleje


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  uint8_t byte;
//	          if (HAL_UART_Receive(&huart1, &byte, 1, 300) == HAL_OK) {
//	              ProcessByte(byte);
//	          }
//	  ESP32_ReceiveData(rx_buffer, sizeof(rx_buffer));
//
//	          // Sprawdź, czy odebrano pakiet UDP
//	          if (strstr((char*)rx_buffer, "+IPD") != NULL) {
//	              // Parsuj dane UDP do struktury
//	              ParseUDPPacket(rx_buffer, &received_packet);
//
//	              // Przetwarzaj odebrane dane
//	              printf("Speed: %.2f m/s\n", received_packet.speed);
//	              printf("RPM: %.2f\n", received_packet.rpm);
//	              printf("Throttle: %.2f\n", received_packet.throttle);
//	              printf("Brake: %.2f\n", received_packet.brake);
//	              printf("Clutch: %.2f\n", received_packet.clutch);
//	              printf("Gear: %d\n", received_packet.gear);
//	              printf("Turbo: %.2f BAR\n", received_packet.turbo);
//	              printf("Engine Temp: %.2f C\n", received_packet.engTemp);
//	              printf("Fuel: %.2f\n", received_packet.fuel);
//	              printf("Oil Temp: %.2f C\n", received_packet.oilTemp);
//	              printf("Dash Lights: %u\n", received_packet.dashLights);
//	              printf("Show Lights: %u\n", received_packet.showLights);
//	          }
//
//	          HAL_Delay(100);  // Czekaj przed kolejnym odczytem
	         int speed_kph = packet.speed * 3.6;

	        Set_PWM_Frequency(speed_kph);
//	        HAL_Delay(1);
//
//	        speed += direction;
//	        if (speed >= 250) {
//	            direction = -1; // Odwracamy kierunek
//	        }
//	        else if (speed <= 0) {
//	       	            direction = 1; // Odwracamy kierunek
//	       	        }

//	  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
//	      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_DME1, TxData_DME1) != HAL_OK) {
//	          printf("Błąd wysyłania wiadomości\n");
//	          Error_Handler();
//	      }
//	  } else {
//		  CheckCANErrors();
//	 //x     printf("Bufor nadawczy pełny, nie można dodać wiadomości\n");
//	  }
//	   CheckCANErrors();

	   char at_response[64]; // Bufor na odpowiedź
	//   HAL_UART_Transmit(&huart1, (uint8_t*)"AT\r\n", 4, HAL_MAX_DELAY);
	  // HAL_UART_Receive(&huart1, (uint8_t*)at_response, sizeof(at_response), 500);

//	   if (strstr(at_response, "OK") != NULL) {
//	       HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET); // Zapal LED
//	   } else {
//	       HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET); // Zgaś LED
//	   }

	  // HAL_UART_Transmit(&huart2, frame, sizeof(frame), HAL_MAX_DELAY);
	 //  if (HAL_UART_Receive(&huart2, rxData, 1, HAL_MAX_DELAY) == HAL_OK) {
	  //            // Odebrano dane — wyślij je z powrotem przez USART
	//	   printf("Odebrano: 0x%02X (%c)\r\n", rxData[0], rxData[0]);
	   //       }
	 //  HAL_LIN_SendBreak(&huart2);
	  // HAL_UART_Transmit(&huart2, frame, sizeof(frame), 100);

	   //HAL_UART_Transmit(&huart2, frame2, sizeof(frame), HAL_MAX_DELAY);
	  // printf("working\n");
	   // HAL_UART_Transmit(&huart2, piecpiec		, sizeof(piecpiec), 100);
	   uint8_t lf = 0x0A;  // LF (Line Feed) w ASCII
	     // HAL_Delay(1000);  // Poczekaj 1 sekundę

		  //HAL_LIN_SendBreak(&huart2);
		  //HAL_UART_Transmit(&huart2, highBeam1, sizeof(highBeam1), 100);
		    // HAL_Delay(1000);  // Poczekaj 1 sekundę
		//  HAL_UART_Transmit(&huart2, highBeam2, sizeof(highBeam2), 100);
		   //  HAL_Delay(1000);  // Poczekaj 1 sekundę
		  //HAL_UART_Transmit(&huart2, highBeam3, sizeof(highBeam3), 100);
		 //    HAL_Delay(1000);  // Poczekaj 1 sekundę

		//  HAL_UART_Transmit(&huart2, piecpiec, sizeof(piecpiec), 100);
		 // HAL_Delay(100);
		 // HAL_UART_Transmit(&huart2, turnRight, sizeof(turnRight), 100);
		 // HAL_Delay(500);
		 //HAL_UART_Transmit(&huart2, testing, sizeof(testing), 500);
		// send_cluster_request(0x1A, 0x12, 0x00, 0x00);

		// send_cluster_request(0x7, 0x00, 0xff, 0xff);
		 //    HAL_Delay(1000);  // Poczekaj 1 sekundę

		 // HAL_UART_Transmit(&huart2, highBeam5, sizeof(highBeam5), 100);
		 //Set_PWM_Frequency(100);
		 // HAL_Delay(10);  // Poczekaj chwilę, aby USART mógł zakończyć transmisję

		  //HAL_UART_Transmit(&huart2, &lf, 1, HAL_MAX_DELAY);  // Wysyłanie LF

	      uint8_t turn_left_data[] = {0xD0 ,0x08 ,0xBF ,0x5B ,0x40 ,0x00 ,0x04 ,0x00 ,0x00 ,0x78};
	      uint8_t identifier = 0x5A;  // Przykładowy identyfikator ramki
	      //Run_PWM_SpeedLoop;

	      //send_lin_frame(identifier, turn_left_data, sizeof(turn_left_data));
	  // HAL_Delay(200);
	   //HAL_LIN_SendBreak(&huart2);
	   //HAL_UART_Transmit(&huart2, lin_frame, sizeof(lin_frame), 100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60808CD3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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
}
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(K_BUS_SLP_GPIO_Port, K_BUS_SLP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Backlight_STM_Pin|Batt_Charge_Light_STM_Pin|Oil_Iight_STM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Brake_fluid_light_STM_Pin|TRCVR_MODE_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ABS_STM_Pin|Parking_Brake_STM_Pin|Brake_Wear_Sens_STM_Pin|Coolant_level_Sens_STM_Pin
                          |D2_Pin|D1_Pin|Fuel_HVC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Washer_Fluid_Lvl_STM_GPIO_Port, Washer_Fluid_Lvl_STM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : K_BUS_SLP_Pin */
  GPIO_InitStruct.Pin = K_BUS_SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(K_BUS_SLP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Backlight_STM_Pin */
  GPIO_InitStruct.Pin = Backlight_STM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Backlight_STM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Batt_Charge_Light_STM_Pin Oil_Iight_STM_Pin Brake_fluid_light_STM_Pin TRCVR_MODE_Pin
                           D3_Pin */
  GPIO_InitStruct.Pin = Batt_Charge_Light_STM_Pin|Oil_Iight_STM_Pin|Brake_fluid_light_STM_Pin|TRCVR_MODE_Pin
                          |D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ABS_STM_Pin Parking_Brake_STM_Pin Brake_Wear_Sens_STM_Pin Coolant_level_Sens_STM_Pin
                           Washer_Fluid_Lvl_STM_Pin D2_Pin D1_Pin Fuel_HVC_Pin */
  GPIO_InitStruct.Pin = ABS_STM_Pin|Parking_Brake_STM_Pin|Brake_Wear_Sens_STM_Pin|Coolant_level_Sens_STM_Pin
                          |Washer_Fluid_Lvl_STM_Pin|D2_Pin|D1_Pin|Fuel_HVC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
    	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
    	    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader2, RxData2) == HAL_OK) {
    	       // printf("Odebrano ramkę: ID = 0x%03X, Dane = ", RxHeader2.Identifier);
    	        for (int i = 0; i < RxHeader2.DataLength; i++) {
    	            //printf("%02X ", RxData2[i]);
    	        }
    	       // printf("\n");
    	    }
    	}
    }

void CheckCANErrors() {
    uint32_t error = HAL_FDCAN_GetError(&hfdcan1);

    if (error == HAL_FDCAN_ERROR_NONE) {
      //  printf("Brak błędów CAN\n");
    } else {
        if (error & HAL_FDCAN_ERROR_TIMEOUT) {
            printf(" HAL_FDCAN_ERROR_TIMEOUT\n");
        }
        if (error & HAL_FDCAN_ERROR_NOT_INITIALIZED) {
            printf(" HAL_FDCAN_ERROR_NOT_INITIALIZED\n");
        }
        if (error & HAL_FDCAN_ERROR_NOT_READY) {
            printf(" HAL_FDCAN_ERROR_NOT_READY\n");
        }
        if (error & HAL_FDCAN_ERROR_NOT_STARTED) {
            printf(" HAL_FDCAN_ERROR_NOT_STARTED\n");
        }
        if (error & HAL_FDCAN_ERROR_NOT_SUPPORTED) {
            printf(" HAL_FDCAN_ERROR_NOT_SUPPORTED\n");
        }
        if (error & HAL_FDCAN_ERROR_PARAM) {
            printf("HAL_FDCAN_ERROR_PARAM\n");
        }
        if (error & HAL_FDCAN_ERROR_PENDING) {
            printf("  HAL_FDCAN_ERROR_PENDING\n");
        }
        if (error & HAL_FDCAN_ERROR_RAM_ACCESS) {
            printf("  HAL_FDCAN_ERROR_RAM_ACCESS\n");
        }
        if (error & HAL_FDCAN_ERROR_PROTOCOL_ARBT) {
            printf(" HAL_FDCAN_ERROR_PROTOCOL_ARBT\n");
        }
        if (error & HAL_FDCAN_ERROR_PROTOCOL_DATA) {
                 printf(" HAL_FDCAN_ERROR_PROTOCOL_DATA\n");
          }
        if (error & HAL_FDCAN_ERROR_RESERVED_AREA) {
                        printf(" HAL_FDCAN_ERROR_RESERVED_AREA\n");
                 }
	   if (error & HAL_FDCAN_ERROR_FIFO_EMPTY) {
		   printf("HAL_FDCAN_ERROR_FIFO_EMPTY\n");
	   }
	   if (error & HAL_FDCAN_ERROR_FIFO_FULL) {
		   printf("  HAL_FDCAN_ERROR_FIFO_FULL\n");
	   }
	   if (error & HAL_FDCAN_ERROR_LOG_OVERFLOW) {
		   printf("  HAL_FDCAN_ERROR_LOG_OVERFLOW\n");
	   }
	   if (error & HAL_FDCAN_ERROR_RAM_WDG) {
		   printf(" HAL_FDCAN_ERROR_RAM_WDG\n");
    }
}}

void ReceiveCANFrame() {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        printf("Odebrano ramkę ID: 0x%X\n", rxHeader.Identifier);
        for (int i = 0; i < rxHeader.DataLength >> 16; i++) {
            printf("Dane[%d]: 0x%X\n", i, rxData[i]);
        }
    } else {
        printf("Brak ramek do odebrania\n");
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1) {
    HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
  }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	  HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
//	  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
//    if (huart->Instance == USART1) {
//       // if (rxDatar == '\n' || rxIndex >= RX_BUFFER_SIZE - 1) {
//            //rxBuffer[rxIndex] = '\0';  // Zakończ string
//    	rxBuffer[rxIndex++] = rxDatar;
//           // UART1_Print((char*)rxBuffer);  // Wyświetl na terminalu
//            //rxIndex = 0;  // Reset bufora
//        //} else {
//            //  // Zapisz bajt do bufora
//        //}
//        HAL_UART_Receive_IT(&huart1, &rxDatar, 1);  // Restart odbioru
//    }
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART1) {  // Sprawdzamy, czy przerwanie pochodzi z USART1
//    		  HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
//        if (rxIndex < BUFFER_SIZE - 1) {
//            rxIndex++;
//        }
//        else{
//        rxBuffer[rxIndex] = '\0';  // Zakończenie stringa
//        rxIndex = 0;  // Reset bufora po przetworzeniu
//        }
//
//        char* start = strstr(rxBuffer, "+IPD,");//rawdza, czy w buforze znajduje się ciąg +IPD, (nagłówek ESP32).
//
//        if (start) {
//        	  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
//            int dataLength = atoi(start + 5);  // Pobranie długości pakietu (za IPD)
//            char* dataStart = strchr(start, ':');// szuka pierwszego : po +IPD,.
//            if (dataStart && dataLength > 0) {
//                dataStart++;
//                parseData(dataStart, dataLength);
//                rxIndex = 0;  // Reset bufora po przetworzeniu
//            }
//        }
//        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer[rxIndex], 1);  // Restart odbioru
//    }
//}
//void parseData(const char* data, int length) {
//    if (length < sizeof(xxx)) {
//        printf("Błąd: Zbyt mało danych\n");
//        return;
//    }
//    xxx packet;
//    memcpy(&packet, data, sizeof(xxx));//memcpy kopiuje bajty z jednego miejsca w pamięci do drugiego. W tym przypadku kopiuje bajty z data do packet
//
//    printf("Car: %s\n", packet.car);
//    printf("Speed: %.2f m/s\n", packet.speed);
//    printf("RPM: %.2f\n", packet.rpm);
//    printf("Throttle: %.2f\n", packet.throttle);
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	 HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
        if (rxIndex < BUFFER_SIZE - 1) {
            rxBuffer[rxIndex++] = huart->Instance->RDR;  // Odczytaj dane z rejestru danych UART
        } else {
            rxBuffer[rxIndex] = '\0';  // Zakończenie stringa
            rxIndex = 0;  // Reset bufora po przetworzeniu
        }

        char* start = strstr((char*)rxBuffer, "+IPD,");  // Sprawdź, czy w buforze znajduje się ciąg +IPD,

        if (start) {
            packetLength = atoi(start + 5);  // Pobranie długości pakietu (za IPD)
            char* dataStart = strchr(start, ':');  // Szuka pierwszego : po +IPD,

            if (dataStart && packetLength > 0) {
                dataStart++;
                if (packetLength <= MAX_PACKET_SIZE) {
                    memcpy(packetBuffer, dataStart, packetLength);  // Kopiuj dane do packetBuffer
                    packetReady = true;  // Ustaw flagę, że pakiet jest gotowy do przetworzenia
                }
                rxIndex = 0;  // Reset bufora po przetworzeniu
            }
        }

        if (packetReady) {
            parseData((char*)packetBuffer, packetLength);  // Parsuj dane
            packetReady = false;  // Zresetuj flagę
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxBuffer[rxIndex], 1);  // Restart odbioru
    }
}

void parseData(const char* data, int length) {
    if (length < sizeof(xxx)) {
        printf("Błąd: Zbyt mało danych\n");
        return;
    }

    memcpy(&packet, data, sizeof(xxx));  // Kopiuj dane do struktury
    HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
//    printf("Car: %s\n", packet.car);
//    printf("Speed: %.2f m/s\n", packet.speed);
//    printf("RPM: %.2f\n", packet.rpm);
//    printf("Throttle: %.2f\n", packet.throttle);
}
void UART1_Print(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Nowa linia
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, SET);
	CheckCANErrors();
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
