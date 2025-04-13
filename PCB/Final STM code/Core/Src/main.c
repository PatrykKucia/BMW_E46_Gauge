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
#include "fdcan.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t time;// time in milliseconds (to check order) // N/A,
    char car[4];// Car name // N/A, fixed value of "beam"
    uint16_t flags;
    char gear;// Reverse:0, Neutral:1, First:2...
    char plid;// Unique ID of viewed player (0 = none) // N/A,
    float speed; // M/S
    float rpm;// RPM
    float turbo; // BAR
    float engTemp;// C
    float fuel;// 0 to 1
    float oilPressure;// BAR // N/A, hardcoded to 0
    float oilTemp;// C
    uint32_t dashLights;// Dash lights available (see DL_x below)
    uint32_t showLights;// Dash lights currently switched on
    float throttle;// 0 to 1
    float brake; // 0 to 1
    float clutch;// 0 to 1
    char display1[16];// Usually Fuel // N/A, hardcoded to ""
    char display2[16];// Usually Settings // N/A, hardcoded to ""
    int id; // optional - only if OutGauge ID is specified
} FrameData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// K BUS
#define LM      						0xd0
#define Broadcast        			    0xbf
#define Cluster_Indicators_Request      0x5a
#define Door_Lid_Status        			0x7a

// Byte 1 (MSB)
#define TURN_RAPID        0x80
#define TURN_RIGHT        0x40
#define TURN_LEFT         0x20
#define FOG_REAR          0x10
#define FOG_FRONT         0x08
#define BEAM_HIGH         0x04
#define BEAM_LOW          0x02
#define PARKING           0x01
// Byte 2
#define CCM_LIC_PLATE     0x80
#define CCM_TURN_RIGHT    0x40
#define CCM_TURN_LEFT     0x20
#define CCM_FOG_REAR      0x10
#define CCM_FOG_FRONT     0x08
#define CCM_HIGH_BEAM     0x04
#define CCM_LOW_BEAM      0x02
#define CCM_PARKING       0x01
// Byte 3
#define CCM_REVERSE       0x20
#define INDICATORS        0x04
#define CCM_BRAKE         0x02
// Byte 4
#define FOG_REAR_SWITCH   0x40

// outgauge protocol
// OG_x - Bity dla flags
#define OG_SHIFT    (1 << 0)   // key N/A
#define OG_CTRL     (1 << 1)   // key N/A
#define OG_TURBO    (1 << 13)  // Wskaźnik turbo
#define OG_KM       (1 << 14)  // if not set - user prefers MILES
#define OG_BAR      (1 << 15)  // if not set - user prefers PSI

// DL_x - Bity dla dashLights i showLights
#define DL_SHIFT        (1 << 0)  // shift light
#define DL_FULLBEAM     (1 << 1)  // full beam
#define DL_HANDBRAKE    (1 << 2)  // handbrake
#define DL_PITSPEED     (1 << 3)  // pit speed limiter // N/A
#define DL_TC           (1 << 4)  // tc active or switched off
#define DL_SIGNAL_L     (1 << 5)  // left turn signal
#define DL_SIGNAL_R     (1 << 6)  // right turn signal
#define DL_SIGNAL_ANY   (1 << 7)  // shared turn signal // N/A
#define DL_OILWARN      (1 << 8)  // oil pressure warning
#define DL_BATTERY      (1 << 9)  // battery warning
#define DL_ABS          (1 << 10) // abs active or switched off
#define DL_SPARE        (1 << 11) // N/A
//

#define RX_BUFFER_SIZE 128
#define FRAME_SIZE 96
#define HEADER "+IPD"
#define MAX_SPEED 270      // 255 km/h
#define MIN_FREQ 25      // 100 Hz
#define MAX_FREQ 1770  // 1770 kHz

// MCP4662 defines
#define MCP4662_ADDR_WRITE  (0x2C << 1)  // Adres I²C + bit zapisu (0)
#define MCP4662_ADDR_READ   (0x2C << 1 | 1)  // Adres I²C + bit odczytu (1)
#define TCON_REGISTER       0x04
#define R_TOTAL       5000
#define R_MIN         75
#define R_STEP        19.3

#define VOLATILE_WIPER_0  0x00
#define VOLATILE_WIPER_1  0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t UartBuffer[RX_BUFFER_SIZE];  // Bufor odbiorczy
uint8_t FrameBuffer[FRAME_SIZE];     // Bufor na pełną ramkę
uint8_t headerMatch = 0;             // Stan detekcji nagłówka
uint8_t frameIndex = 0;              // Indeks do zapisu danych ramki
bool frameReady = false;

FrameData frame;

float speed = 0;

bool isTurboActive;
bool isMetric;
bool prefersBar;

bool isShiftLightOn;
bool isFullBeam;
bool isHandbrakeOn;
bool isTractionCtrl;
bool isABSActive;
bool isOilWarning;
bool isBatteryWarning;
bool isLeftSignal;
bool isRightSignal;
uint16_t alive_counter;
uint8_t frame1F3_counter = 0;
static float previous_fuel = 0;
static uint32_t previous_time = 0;
static uint16_t mpgloop = 0;
static uint16_t prev_rpm = 0;
static uint32_t rpm_integral = 0; // Do śledzenia historii
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetTCON(I2C_HandleTypeDef *hi2c, uint16_t value) {
    uint8_t data[3];

    // Rejestr TCON (0x04), Write = 00
    data[0] = (0x04 << 4) | 0x00;

    // 9-bitowa wartość TCON
    value &= 0x01FF;
    data[2] = (value >> 8) & 0x01; // MSB (1 bit)
    data[1] = value & 0xFF;        // LSB (8 bitów)

    // Wysłanie komendy + 2 bajtów wartości
    if (HAL_I2C_Master_Transmit(hi2c, (0x2C << 1), data, 3, HAL_MAX_DELAY) != HAL_OK) {
        printf("⚠️ Błąd zapisu do TCON!\n");
    }
}

uint16_t MCP4662_ReadTCON(I2C_HandleTypeDef *hi2c) {
    uint8_t command_byte = (0x04 << 4) | 0x0C; // Rejestr TCON, CC = 11 (Read)
    uint8_t data[2] = {0};

    // Wysłanie bajtu komendy
    if (HAL_I2C_Master_Transmit(hi2c, (0x2C << 1), &command_byte, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // Kod błędu
    }

    // Odczyt danych (9-bitowa wartość, ale zwracamy 16-bitową zmienną)
    if (HAL_I2C_Master_Receive(hi2c, (0x2C << 1) | 1, data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // Kod błędu
    }

    // Połączenie danych (TCON ma 9 bitów, więc maskujemy)
    uint16_t tcon_value = ((data[0] << 8) | data[1]) & 0x01FF;

    return tcon_value;
}

int __io_putchar(int ch) //function used to print() in usart
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}
void Set_PWM_Frequency(uint16_t speed_kmh) {
    uint32_t freq = MIN_FREQ + ((MAX_FREQ - MIN_FREQ) * speed_kmh) / MAX_SPEED;

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
    //__HAL_TIM_SET_COUNTER(&htim1, 0);

}
uint16_t ReadWiper(I2C_HandleTypeDef *hi2c, uint8_t wiper_reg) {
    uint8_t command_byte = (wiper_reg << 4) | 0x0C; // CC = 11 (Read)
    uint8_t data[2] = {0};

    // Wysłanie bajtu komendy
    if (HAL_I2C_Master_Transmit(hi2c, (0x2C << 1), &command_byte, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // Kod błędu
    }

    // Odczyt 10-bitowego wiper value
    if (HAL_I2C_Master_Receive(hi2c, (0x2C << 1) | 1, data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // Kod błędu
    }

    uint16_t wiper_value = ((data[0] << 8) | data[1]) & 0x03FF; // 10-bitowy wynik
    return wiper_value;
}

void wiper_command(I2C_HandleTypeDef *hi2c, uint8_t wiper, uint8_t command) {
    uint8_t cmd;

    if (wiper == 0) {
        cmd = command;  // Dla Wiper 0 (komendy 0x04 lub 0x08)
    } else if (wiper == 1) {
        cmd = command | 0x10; // Dodajemy bit 4, aby przełączyć na Wiper 1
    } else {
        return; // Nieprawidłowy wybór wipera
    }

    HAL_I2C_Master_Transmit(hi2c, MCP4662_ADDR_WRITE, &cmd, 1, HAL_MAX_DELAY);
}

void ESP32_SendCommand(const char* command) {
    HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Końcówka komendy AT
    HAL_Delay(100);  // Czekaj na odpowiedź
}


void I2C_Scan()
{
    printf("Skanowanie I2C...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 100) == HAL_OK)
        {
            printf("Znaleziono urządzenie na adresie: 0x%X\r\n", addr);
        }
    }
}

void InitAnalogIndicators(){

	 HAL_GPIO_WritePin(K_BUS_SLP_GPIO_Port, K_BUS_SLP_Pin, SET); //turn off k-bus tranciver sleep mode

	 HAL_GPIO_WritePin(WASHER_FLU_LVL_GPIO_Port, WASHER_FLU_LVL_Pin, SET); // SET to off
	 HAL_GPIO_WritePin(COOLANT_LVL_SENS_GPIO_Port, COOLANT_LVL_SENS_Pin, SET); //SET to off
	 HAL_GPIO_WritePin(BRAKE_WEAR_SENS_GPIO_Port, BRAKE_WEAR_SENS_Pin, SET);//SET to off
	 HAL_GPIO_WritePin(PARKING_BRAKE_GPIO_Port, PARKING_BRAKE_Pin, RESET);//SET to off temp
	 HAL_GPIO_WritePin(ABS_GPIO_Port, ABS_Pin, SET);//SET to off temp
	 HAL_GPIO_WritePin(BRAKE_FLU_LIGHT_GPIO_Port, BRAKE_FLU_LIGHT_Pin, SET);//SET to off
	 HAL_GPIO_WritePin(OIL_LIGHT_GPIO_Port, OIL_LIGHT_Pin, RESET);//RESET to off temp
	 HAL_GPIO_WritePin(BATT_CHARGE_LIGHT_GPIO_Port, BATT_CHARGE_LIGHT_Pin, RESET);//RESET to off temp

	 //HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, RESET);//RESET to off ------------NOT WORKING
	 modify_can_frame_byte(FRAME_316, 0, 0x05);
	 modify_can_frame_byte(FRAME_316, 1, 0x14);
	 modify_can_frame_byte(FRAME_316, 4, 0x14);
	 modify_can_frame_byte(FRAME_316, 5, 0x17);
	 modify_can_frame_byte(FRAME_316, 6, 0x0);
	 modify_can_frame_byte(FRAME_316, 7, 0x16);


	 modify_can_frame_byte(FRAME_153, 0, 0x00); // brak błędu //06 //ff to turn on
	 modify_can_frame_byte(FRAME_153, 1, 0x00); // brak aktywnej interwencji DSC
	 modify_can_frame_byte(FRAME_153, 2, 0x00); // zero flag
	 modify_can_frame_byte(FRAME_153, 3, 0x00); // zero błędów
	 modify_can_frame_byte(FRAME_153, 4, 0x00); // zero
	 modify_can_frame_byte(FRAME_153, 5, 0x00); // zero
	 modify_can_frame_byte(FRAME_153, 6, 0x00); // brak błędu
	 modify_can_frame_byte(FRAME_153, 7, 0x00); // suma kontrolna lub pusty


	 modify_can_frame_byte(FRAME_545, 4, 0x7E);  // Unused byte
	 modify_can_frame_byte(FRAME_545, 5, 0x10);  // Unused byte
	 modify_can_frame_byte(FRAME_545, 6, 0x00);  // Unused byte
	 modify_can_frame_byte(FRAME_545, 7, 0x18);  // Unused byte


	 modify_can_frame_byte(FRAME_329, 0, 0x07);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 2, 0xB2);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 3, 0x19);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 4, 0x00);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 5, 0xEE);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 6, 0x00);  // Unused byte
	 modify_can_frame_byte(FRAME_329, 7, 0x00);  // Unused byte

	 modify_can_frame_byte(FRAME_1F3, 0, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 1, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 2, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 3, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 4, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 5, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 6, 0x00);
	 modify_can_frame_byte(FRAME_1F3, 7, 0x00);


}
uint16_t calculate_fuel_consumption(float throttle, uint8_t gear) {
    if (throttle <= 0.0f) return 0x0000; // Brak spalania, jeśli pedał gazu nie jest wciśnięty
    if (gear == 0) return 0xFFFF; // Maksymalne spalanie na biegu jałowym

    float max_consumption = 20.0f; // Maksymalne spalanie w l/100 km
    float gear_factor = 1.0f + (gear - 1) * 0.5f; // Im wyższy bieg, tym niższe spalanie
    float fuel_consumption = (throttle * max_consumption);// / gear_factor;

    // Ograniczenie spalania do maksimum
    if (fuel_consumption > max_consumption) {
        fuel_consumption = max_consumption;
    }

    // Skalowanie do 0xFFFF
    uint16_t fuel_scaled = (uint16_t)((fuel_consumption / max_consumption) * 0xFFFF);
    return fuel_scaled;
}


void parse_frame(uint8_t *buffer) {
    uint8_t offset = 0;

    memcpy(&frame.time, &buffer[offset], sizeof(frame.time));
    offset += sizeof(frame.time);

    memcpy(&frame.car, &buffer[offset], sizeof(frame.car));
    offset += sizeof(frame.car);

    memcpy(&frame.flags, &buffer[offset], sizeof(frame.flags));
    offset += sizeof(frame.flags);

    memcpy(&frame.gear, &buffer[offset], sizeof(frame.gear));
    offset += sizeof(frame.gear);

    memcpy(&frame.plid, &buffer[offset], sizeof(frame.plid));
    offset += sizeof(frame.plid);

    memcpy(&frame.speed, &buffer[offset], sizeof(frame.speed));
    offset += sizeof(frame.speed);

    memcpy(&frame.rpm, &buffer[offset], sizeof(frame.rpm));
    offset += sizeof(frame.rpm);

    memcpy(&frame.turbo, &buffer[offset], sizeof(frame.turbo));
    offset += sizeof(frame.turbo);

    memcpy(&frame.engTemp, &buffer[offset], sizeof(frame.engTemp));
    offset += sizeof(frame.engTemp);

    memcpy(&frame.fuel, &buffer[offset], sizeof(frame.fuel));
    offset += sizeof(frame.fuel);

    memcpy(&frame.oilPressure, &buffer[offset], sizeof(frame.oilPressure));
    offset += sizeof(frame.oilPressure);

    memcpy(&frame.oilTemp, &buffer[offset], sizeof(frame.oilTemp));
    offset += sizeof(frame.oilTemp);

    memcpy(&frame.dashLights, &buffer[offset], sizeof(frame.dashLights));
    offset += sizeof(frame.dashLights);

    memcpy(&frame.showLights, &buffer[offset], sizeof(frame.showLights));
    offset += sizeof(frame.showLights);

    memcpy(&frame.throttle, &buffer[offset], sizeof(frame.throttle));
    offset += sizeof(frame.throttle);

    memcpy(&frame.brake, &buffer[offset], sizeof(frame.brake));
    offset += sizeof(frame.brake);

    memcpy(&frame.clutch, &buffer[offset], sizeof(frame.clutch));
    offset += sizeof(frame.clutch);

    memcpy(&frame.display1, &buffer[offset], sizeof(frame.display1));
    offset += sizeof(frame.display1);

    memcpy(&frame.display2, &buffer[offset], sizeof(frame.display2));
    offset += sizeof(frame.display2);

    memcpy(&frame.id, &buffer[offset], sizeof(frame.id));

    isTurboActive = frame.flags & OG_TURBO;
    isMetric = frame.flags & OG_KM;
    prefersBar = frame.flags & OG_BAR;

    isShiftLightOn = frame.showLights & DL_SHIFT;
    isFullBeam = frame.showLights & DL_FULLBEAM;
    isHandbrakeOn = frame.showLights & DL_HANDBRAKE;
    isTractionCtrl = frame.showLights & DL_TC;
    isABSActive = frame.showLights & DL_ABS;
    isOilWarning = frame.showLights & DL_OILWARN;
    isBatteryWarning = frame.showLights & DL_BATTERY;
    isLeftSignal = frame.showLights & DL_SIGNAL_L;
    isRightSignal = frame.showLights & DL_SIGNAL_R;



    HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);  // Diagnostyka
//	if (frame.throttle == 0x00) frame.throttle = 0x01; // Minimalna wartość to 0x01
//	 	 modify_can_frame_byte(FRAME_329, 5, frame.throttle * 256.0f);
//
//    uint16_t fuel_value = calculate_fuel_consumption(frame.throttle, frame.gear);
//    uint8_t fuel_lsb = fuel_value & 0xFF;
//    uint8_t fuel_msb = (fuel_value >> 8) & 0xFF;
//
//    modify_can_frame_byte(FRAME_545, 1, fuel_lsb);
//    modify_can_frame_byte(FRAME_545, 2, fuel_msb);
}
void Modify_Values(){
	speed = frame.speed * 3.6;
    Set_PWM_Frequency(speed);

    uint16_t hexValue_RPM = (uint16_t)(frame.rpm / 0.15625);  // Rzutowanie na uint16_t
    uint8_t lsb = hexValue_RPM & 0xFF;  // Pobranie 8 najmłodszych bitów
    uint8_t msb = (hexValue_RPM >> 8) & 0xFF;  // Pobranie 8 najbardziej znaczących bitów

    uint8_t hexValue_temperature = ((frame.engTemp + 48.0) / 0.75) ;

    modify_can_frame_byte(FRAME_316, 2, lsb);  // Modyfikacja bajtu w ramce CAN
    modify_can_frame_byte(FRAME_316, 3, msb);  // Modyfikacja bajtu w ramce CAN
    modify_can_frame_byte(FRAME_329, 1, hexValue_temperature);

    if(frame.engTemp >= 129)
    {
    	modify_can_frame_bit(FRAME_545, 3, 3, 1);
    }
    else
    {
    	modify_can_frame_bit(FRAME_545, 3, 3, 0);
    }

//    if (mpgloop == 0xFFFF) {
//        mpgloop = 0x0;
//    } else {
//        mpgloop += frame.rpm / 130;
//    }
//    modify_can_frame_byte(FRAME_545, 1, mpgloop & 0xFF);  // Lower byte of mpgloop
//    modify_can_frame_byte(FRAME_545, 2, mpgloop >> 8);    // Higher byte of mpgloop
    if (mpgloop == 0xFFFF) {
        mpgloop = 0x0;
    } else {
        // Oblicz zmianę RPM (pochodną)
        int16_t rpm_delta = frame.rpm - prev_rpm;

        // Część dynamiczna - reaguje na zmiany obrotów
        int16_t dynamic_component = 0;
        if (rpm_delta > 0) {
            dynamic_component = rpm_delta * 3; // Silna reakcja na przyspieszenie
        } else if (rpm_delta < 0) {
            dynamic_component = rpm_delta; // Słabsza reakcja na zwalnianie
        }

        // Część statyczna - zależy od aktualnych obrotów
        uint16_t static_component = frame.rpm / 50; // Dostosuj dzielnik

        // Suma obu komponentów
        int32_t new_mpgloop = mpgloop + dynamic_component + static_component;

        // Jeśli pedał gazu puszczony - szybkie zmniejszanie
        if (frame.throttle <= 0.2) {
            new_mpgloop = 0; // Szybkie zmniejszanie new_mpgloop * 0.7
        }

        // Ograniczenia wartości
        new_mpgloop = (new_mpgloop < 0) ? 0 : new_mpgloop;
        mpgloop = (new_mpgloop > 0xFFFF) ? 0xFFFF : (uint16_t)new_mpgloop;

        prev_rpm = frame.rpm;
    }

    modify_can_frame_byte(FRAME_545, 1, mpgloop & 0xFF);
    modify_can_frame_byte(FRAME_545, 2, mpgloop >> 8);

    modify_can_frame_byte(FRAME_545, 1, mpgloop & 0xFF);
    modify_can_frame_byte(FRAME_545, 2, mpgloop >> 8);
    if(isTractionCtrl)
    {
	 modify_can_frame_byte(FRAME_153, 0, 0x06); // brak błędu //06 //ff to turn on
	 modify_can_frame_byte(FRAME_153, 1, 0xff); // brak aktywnej interwencji DSC
    }
    else
    {
   	 modify_can_frame_byte(FRAME_153, 0, 0x00); // brak błędu //06 //ff to turn on
   	 modify_can_frame_byte(FRAME_153, 1, 0x00); // brak aktywnej interwencji DSC
    }
    }

uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void Send_KBUS_frame(uint8_t Source_ID, uint8_t Dest_ID, uint8_t command, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t size )
{
	uint8_t frame[10];
	//uint8_t lenght;

	frame[0] = Source_ID; //LM 0xd0 → Broadcast 0xbf
	//frame[1] = lenght;
	frame[2] = Dest_ID;
	frame[3] = command;
	frame[4] = Byte1;
	frame[5] = Byte2;
	frame[6] = Byte3;
	frame[7] = Byte4;
	frame[8] = 0;

	//lenght = sizeof(frame) - 2;
	frame[1] = size; //lenght
	frame[8] = calculate_checksum(frame, size+1);
	frame[9] = '\n';

	HAL_UART_Transmit(&huart2, frame, sizeof(frame), 100);
}


void process_frame(void) {
    if (frameReady) {
        parse_frame(FrameBuffer);
        frameReady = false;
    }
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
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  ESP32_SendCommand("AT+RST");  // Resetuj ESP32
 // HAL_Delay(1000);
 // ESP32_SendCommand("AT+CWMODE=1");  // Ustaw tryb stacji (klient Wi-Fi)
 // HAL_Delay(1000);
 // ESP32_SendCommand("AT+CWJAP=\"PLAY_Swiatlowod_19A1\",\"t8Xv9auf7Z#D\"");  // Wi-Fi //AT+CWJAP="PLAY_Swiatlowod_19A1","t8Xv9auf7Z#D" - if connection lost/MCU replaced
  HAL_Delay(5000);
  ESP32_SendCommand("AT+CIPSTART=\"UDP\",\"0.0.0.0\",12345,12345,2");  // Ustaw tryb UDP
  HAL_Delay(1000);
  HAL_UART_Receive_DMA(&huart1, UartBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(5);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);

  InitCANFrames();
  InitAnalogIndicators();

  uint8_t increasing0 = 1, increasing1 = 1; // Flagi dla obu wiperów
  uint8_t reset_cmd = 0x06; // Komenda General Call Reset
//HAL_I2C_Master_Transmit(&hi2c1, 0x00, &reset_cmd, 1, HAL_MAX_DELAY);
HAL_Delay(10);
SetTCON(&hi2c1, 0xbb );  // Podłącza piny A, W, B dla obu wiperów  //0x34 //0xBB - HW ON -A OFF  //33 - HW OFF
HAL_Delay(10);



   // uint8_t set_wiper[2] = {0x00, 0xc8}; // Środkowa wartość (128)
  //  HAL_I2C_Master_Transmit(&hi2c1, MCP4662_ADDR_WRITE, set_wiper, 2, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 process_frame();
	 Modify_Values();
////////////////////////
     uint16_t wiper0 = ReadWiper(&hi2c1, VOLATILE_WIPER_0); // Wiper 0
     uint16_t wiper1 = ReadWiper(&hi2c1, VOLATILE_WIPER_1); // Wiper 1
     uint16_t tcon = MCP4662_ReadTCON(&hi2c1);  // TCON

//     printf("Wiper 0 value: %d\n", wiper0);
//     printf("Wiper 1 value: %d\n", wiper1);
//     printf("TCON: 0x%03X\n", tcon);

     //WriteWiper(&hi2c1, VOLATILE_WIPER_0, 0x80);
     //WriteWiper(&hi2c1, VOLATILE_WIPER_1, 0x80);
//
//      Interpretacja bitów:
         uint8_t GCEN = (tcon >> 8) & 0x01;  // Bit 8 (GCEN)
         uint8_t R1HW = (tcon >> 7) & 0x01;  // Bit 7
         uint8_t R1A  = (tcon >> 6) & 0x01;  // Bit 6
         uint8_t R1W  = (tcon >> 5) & 0x01;  // Bit 5
         uint8_t R1B  = (tcon >> 4) & 0x01;  // Bit 4
         uint8_t R0HW = (tcon >> 3) & 0x01;  // Bit 3
         uint8_t R0A  = (tcon >> 2) & 0x01;  // Bit 2
         uint8_t R0W  = (tcon >> 1) & 0x01;  // Bit 1
         uint8_t R0B  = (tcon >> 0) & 0x01;  // Bit 0
//
//         printf("TCON: 0x%04X\n", tcon);
//         printf("GCEN: %d\n", GCEN);
//         printf("R1HW: %d, R1A: %d, R1W: %d, R1B: %d\n", R1HW, R1A, R1W, R1B);
//         printf("R0HW: %d, R0A: %d, R0W: %d, R0B: %d\n", R0HW, R0A, R0W, R0B);

//         wiper_command(&hi2c1, 0, 0x44);
//
//         // Przykład: Włączenie GCEN i ustawienie R1A=1, R0W=1
//         uint16_t all_ones_tcon = 0x01FF;
//         uint16_t new_tcon_value = 0x0181;
 //        MCP4662_WriteTCON(&hi2c1, new_tcon_value);
//////////////////////////////

//
////     uint8_t enable_tcon[2] = {0x04, 0b00001100}; // Włącz wszystkie połączenia
////     HAL_I2C_Master_Transmit(&hi2c1, MCP4662_ADDR, enable_tcon, 2, HAL_MAX_DELAY);
//
//     uint8_t enable_tcon[2] = {0x04, 0x1FF};  // Wszystkie połączenia aktywne
//     HAL_I2C_Master_Transmit(&hi2c1, MCP4662_ADDR, enable_tcon, 2, HAL_MAX_DELAY);
//
//
//     uint8_t read_tcon[1] = {0x04};
//     uint8_t tcon_value[2] = {0};
//
//     HAL_I2C_Master_Transmit(&hi2c1, MCP4662_ADDR, read_tcon, 1, HAL_MAX_DELAY);
//     HAL_I2C_Master_Receive(&hi2c1, MCP4662_ADDR, tcon_value, 2, HAL_MAX_DELAY);
//
//     uint16_t tcon = (tcon_value[0] << 8 | tcon_value[1]) & 0x1FF;
//     printf("TCON value: 0x%03X\n", tcon);

//     uint8_t save_to_eeprom[2] = {0x20, 0x80}; // Zapisz do EEPROM
//     HAL_I2C_Master_Transmit(&hi2c1, MCP4662_ADDR, save_to_eeprom, 2, HAL_MAX_DELAY);

////////////////////////////////
//      {
//         // *** WIPER 0 ***
//         if (increasing0) {
//             wiper_command(&hi2c1, 0, 0x04); // Inkrementacja Wiper 0
//             wiper0++;
//             if (wiper0 >= 256) increasing0 = 0;
//         } else {
//             wiper_command(&hi2c1, 0, 0x08); // Dekrementacja Wiper 0
//             wiper0--;
//             if (wiper0 == 0) increasing0 = 1;
//         }
//
//         // *** WIPER 1 ***
//         if (increasing1) {
//             wiper_command(&hi2c1, 1, 0x04); // Inkrementacja Wiper 1
//             wiper1++;
//             if (wiper1 >= 256) increasing1 = 0;
//         } else {
//             wiper_command(&hi2c1, 1, 0x08); // Dekrementacja Wiper 1
//             wiper1--;
//             if (wiper1 == 0) increasing1 = 1;
//
//         }
//
//
//         //HAL_Delay(50); // Czekaj dla stabilności
//     }
//     HAL_Delay(250); // Opóźnienie dla stabilnego działania
/////////////////////////////////////

         uint16_t target0 = (uint16_t)(0.01 * 255);//0.1 - 570ohm/off  0=1/4   0.01= ~half 0.03-half
         uint16_t target1 = (uint16_t)(0.01 * 255);

             if (wiper0 < target0) {
                 wiper_command(&hi2c1, 0, 0x04); // Inkrementacja
                 wiper0++;
             } else if (wiper0 > target0) {
                 wiper_command(&hi2c1, 0, 0x08); // Dekrementacja
                 wiper0--;
             }

             if (wiper1 < target1) {
                 wiper_command(&hi2c1, 1, 0x04); // Inkrementacja
                 wiper1++;
             } else if (wiper1 > target1) {
                 wiper_command(&hi2c1, 1, 0x08); // Dekrementacja
                 wiper1--;
             }

//////////////////////////////////////

   //  I2C_Scan();
     //SetResistance(500);
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t byte; // Przechowywany bajt
    HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
    if (huart->Instance == USART1) {
        byte = UartBuffer[0]; // Odbieramy bajt po bajcie

        // Sprawdzamy nagłówek "+IPD"
        if (headerMatch < 4) {
            if (byte == HEADER[headerMatch]) {
                headerMatch++;
            } else {
                headerMatch = 0; // Reset, jeśli się nie zgadza
            }
        } else if (headerMatch == 4) {
            // Oczekujemy na długość ramki, pomijamy "xx:"
            if (byte == ':') {
                headerMatch++; // Przechodzimy do odbioru danych
                frameIndex = 0;
            }
        } else {
            // Odbiór danych ramki
            if (frameIndex < 96 ) {
                FrameBuffer[frameIndex++] = byte;
            }

            if (frameIndex >= 96 ) {
                frameReady = true;  // Pełna ramka odebrana
                process_frame();
                headerMatch = 0;    // Reset detekcji nagłówka
                frameIndex = 0;     // Reset indeksu bufora ramki
            }}
        HAL_UART_Receive_DMA(&huart1, UartBuffer, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	 if (htim == &htim2)
	 {
	     SendCANFrame(FRAME_316);  // Wysyła ramkę o ID 0x316
	     SendCANFrame(FRAME_329);  // Wysyła ramkę o ID 0x329
	     SendCANFrame(FRAME_545);  // Wysyła ramkę o ID 0x545
	 }
	 if (htim == &htim4)
	 {
	     SendCANFrame(FRAME_153);

	     frame1F3_counter++;
	     if (frame1F3_counter >= 2) {
	         SendCANFrame(FRAME_1F3);
	         frame1F3_counter = 0;
	     }
	 }
	 if (htim == &htim3)
	 {
		bool anyConditionMet = false;
		uint8_t command = 0x00;  // Domyślnie brak sygnału
		if (isFullBeam) {
			command = BEAM_LOW | PARKING | BEAM_HIGH;
			anyConditionMet = true;
		}
		if (isLeftSignal) {
			command = command | TURN_LEFT;
			anyConditionMet = true;
		}
		if (isRightSignal) {
			command = command | TURN_RIGHT;
			anyConditionMet = true;
		}
		if (!anyConditionMet) {
			Send_KBUS_frame(LM, Broadcast, 0x5B, 0x00, 0x00, 0x00, 0x00, 0x07);//, 0x83, 0x0a, 0x3f
		}
		Send_KBUS_frame(LM, Broadcast, 0x5B, command, 0x00, 0x00, 0x00, 0x07);

		if (isHandbrakeOn) {
			HAL_GPIO_WritePin(PARKING_BRAKE_GPIO_Port, PARKING_BRAKE_Pin, SET);//SET to off temp
		}else
			HAL_GPIO_WritePin(PARKING_BRAKE_GPIO_Port, PARKING_BRAKE_Pin, RESET);//SET to off temp
		if (isOilWarning) {
			HAL_GPIO_WritePin(OIL_LIGHT_GPIO_Port, OIL_LIGHT_Pin, SET); //SET on
		}else
			HAL_GPIO_WritePin(OIL_LIGHT_GPIO_Port, OIL_LIGHT_Pin, RESET); //SET to off temp
		if (isBatteryWarning) {
			HAL_GPIO_WritePin(BATT_CHARGE_LIGHT_GPIO_Port, BATT_CHARGE_LIGHT_Pin, SET);
		}else
			HAL_GPIO_WritePin(BATT_CHARGE_LIGHT_GPIO_Port, BATT_CHARGE_LIGHT_Pin, RESET);
		if (isABSActive) {
			HAL_GPIO_WritePin(ABS_GPIO_Port, ABS_Pin, RESET);
		}else
			HAL_GPIO_WritePin(ABS_GPIO_Port, ABS_Pin, SET);
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);  // Diagnostyka

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
