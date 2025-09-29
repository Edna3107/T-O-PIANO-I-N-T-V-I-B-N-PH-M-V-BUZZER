/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Piano 4x4 + LCD I2C + Buzzer PWM (PA8/TIM1_CH1) - FIXED
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ==== Dùng l?i handle/hàm init do CubeMX sinh trong i2c.c & tim.c ==== */
#include "i2c.h"
#include "tim.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;   // TIM1 (PWM @ PA8)
extern TIM_HandleTypeDef htim2;   // TIM2 (1ms tick)

/* ===================== LCD I2C (PCF8574 backpack) ===================== */
#define LCD_ADDR        (0x27 << 1)   // d?i sang (0x3F<<1) n?u LCD không hi?n
#define LCD_BL          0x08
#define LCD_EN          0x04
#define LCD_RW          0x02
#define LCD_RS          0x01

static void LCD_I2C_Write(uint8_t d){ HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &d, 1, 50); }
static void LCD_Pulse(uint8_t d){ LCD_I2C_Write(d | LCD_EN); HAL_Delay(1); LCD_I2C_Write(d & ~LCD_EN); HAL_Delay(1); }
static void LCD_Send4(uint8_t hi, uint8_t rs){
  uint8_t x = (hi & 0xF0) | LCD_BL | (rs ? LCD_RS : 0);
  LCD_Pulse(x);
}
static void LCD_Send(uint8_t val, uint8_t rs){
  LCD_Send4(val & 0xF0, rs);
  LCD_Send4((val<<4) & 0xF0, rs);
}
static void LCD_Cmd(uint8_t c){ LCD_Send(c,0); }
static void LCD_Data(uint8_t d){ LCD_Send(d,1); }
static void LCD_Init(void){
  HAL_Delay(50);
  LCD_Send4(0x30,0); HAL_Delay(5);
  LCD_Send4(0x30,0); HAL_Delay(5);
  LCD_Send4(0x30,0); HAL_Delay(5);
  LCD_Send4(0x20,0); HAL_Delay(5);    // 4-bit
  LCD_Cmd(0x28);                      // 2 lines, 5x8
  LCD_Cmd(0x0C);                      // display ON, cursor OFF
  LCD_Cmd(0x01); HAL_Delay(2);        // clear
  LCD_Cmd(0x06);                      // entry mode
}
static void LCD_SetCursor(uint8_t row, uint8_t col){
  static const uint8_t off[2] = {0x00,0x40};
  LCD_Cmd(0x80 | (off[row&1] + (col & 0x0F)));
}
static void LCD_Print(const char* s){ while(*s) LCD_Data((uint8_t)*s++); }

/* ===================== Keypad wiring (CHU?N TÊN R1..R4, C1..C4) ===================== */
/* C1..C4 -> PA0..PA3 (OUTPUT, m?c d?nh HIGH)
   R1..R4 -> PA4..PA7 (INPUT PULL-UP) */
#define C1_PORT GPIOA
#define C2_PORT GPIOA
#define C3_PORT GPIOA
#define C4_PORT GPIOA
#define R1_PORT GPIOA
#define R2_PORT GPIOA
#define R3_PORT GPIOA
#define R4_PORT GPIOA

#define C1_PIN  GPIO_PIN_0
#define C2_PIN  GPIO_PIN_1
#define C3_PIN  GPIO_PIN_2
#define C4_PIN  GPIO_PIN_3
#define R1_PIN  GPIO_PIN_4
#define R2_PIN  GPIO_PIN_5
#define R3_PIN  GPIO_PIN_6
#define R4_PIN  GPIO_PIN_7

static void KEYPAD_AllColsHigh(void){
  HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);
}
static void KEYPAD_SetActiveColIdx(uint8_t idx){ // idx = 0..3 tuong ?ng C1..C4
  KEYPAD_AllColsHigh();
  switch(idx){
    case 0: HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_RESET); break;
    case 1: HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_RESET); break;
    case 2: HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_RESET); break;
    case 3: HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_RESET); break;
  }
}
/* Tr? v? R=1..4, C=1..4 (1-based) */
static int8_t KEYPAD_ScanOnce(uint8_t *pR, uint8_t *pC){
  for(uint8_t cidx=0; cidx<4; cidx++){
    KEYPAD_SetActiveColIdx(cidx);
    HAL_Delay(1); // settle
    if(HAL_GPIO_ReadPin(R1_PORT, R1_PIN)==GPIO_PIN_RESET){*pR=1; *pC=cidx+1; return 1;}
    if(HAL_GPIO_ReadPin(R2_PORT, R2_PIN)==GPIO_PIN_RESET){*pR=2; *pC=cidx+1; return 1;}
    if(HAL_GPIO_ReadPin(R3_PORT, R3_PIN)==GPIO_PIN_RESET){*pR=3; *pC=cidx+1; return 1;}
    if(HAL_GPIO_ReadPin(R4_PORT, R4_PIN)==GPIO_PIN_RESET){*pR=4; *pC=cidx+1; return 1;}
  }
  return 0;
}
static int8_t KEYPAD_GetKey(uint8_t *pR, uint8_t *pC){
  if(KEYPAD_ScanOnce(pR,pC)){
    HAL_Delay(8); // debounce
    uint8_t rr,cc;
    if(KEYPAD_ScanOnce(&rr,&cc) && rr==*pR && cc==*pC) return 1;
  }
  return 0;
}

/* ===================== Notes ===================== */
typedef struct { const char* name; float freq; } note_t;
/* [R1..R4][C1..C4] -> dùng index 0..3 n?i b? nên tr? 1 khi truy c?p */
static const note_t note_map[4][4] = {
  /* R1 (trên cùng)  */ { {"Do", 523.25f}, {"Re", 587.33f}, {"Mi", 659.25f}, {"Fa", 698.46f} }, // C5 D5 E5 F5
  /* R2              */ { {"Sol",783.99f}, {"La",880.00f}, {"Si",987.77f}, {"Do",1046.50f} },   // G5 A5 B5 C6
  /* R3              */ { {"Re",1174.66f}, {"Mi",1318.51f}, {"Fa",1396.91f},{"Sol",1567.98f} }, // D6 E6 F6 G6
  /* R4 (du?i cùng)  */ { {"La",1760.00f}, {"Si",1975.53f}, {"Do",2093.00f},{"Re",2349.32f} }   // A6 B6 C7 D7
};

/* ===================== Ticker ===================== */
volatile uint32_t g_ms = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM2) g_ms++;
}

/* ===================== Prototypes ===================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* ===================== Tone (TIM1_CH1 @ PA8) ===================== */
static void Tone_Start(float f_hz){
  if(f_hz < 1.0f){
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    return;
  }
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

  /* Timer clock = 1 MHz (HSI 8MHz / (PSC+1) = 8MHz/8 = 1MHz) */
  uint32_t arr = (uint32_t)((1000000.0f / f_hz) + 0.5f) - 1;
  if(arr < 1) arr = 1;
  if(arr > 65535) arr = 65535; // 16-bit timer limit

  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr+1)/2);

  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
static void Tone_Stop(void){
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

/* ===================== Test Buzzer ===================== */
static void Test_Buzzer(void){
  LCD_SetCursor(0,0); LCD_Print("Testing Buzzer..");
  Tone_Start(2000.0f);
  HAL_Delay(500);
  Tone_Stop();
  LCD_SetCursor(1,0); LCD_Print("Test Complete!");
  HAL_Delay(1000);
}

/* ===================== MAIN ===================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();      // keypad + PA8 AF_PP
  MX_I2C1_Init();      // t? i2c.c
  MX_TIM1_Init();      // t? tim.c (PSC=7, ARR=999, CH1 PWM)
  MX_TIM2_Init();      // t? tim.c (PSC=799, ARR=9) -> 1ms

  __HAL_TIM_MOE_ENABLE(&htim1);

  LCD_Init();
  Test_Buzzer();

  LCD_Cmd(0x01); HAL_Delay(2);
  LCD_SetCursor(0,0); LCD_Print("Piano 4x4 Ready");
  LCD_SetCursor(1,0); LCD_Print("Press a key...");

  HAL_TIM_Base_Start_IT(&htim2);   // 1ms ticker

  while (1)
  {
    uint8_t R,C; // 1..4
    if(KEYPAD_GetKey(&R,&C)){
      const note_t nt = note_map[R-1][C-1];

      // LCD show
      LCD_Cmd(0x01); HAL_Delay(2);
      LCD_SetCursor(0,0); LCD_Print("Note: "); LCD_Print(nt.name);
      char buf[16];
      snprintf(buf,sizeof(buf),"%.0f Hz R%uC%u", nt.freq, R, C);
      LCD_SetCursor(1,0); LCD_Print(buf);

      // Play while pressed (max 2s)
      uint32_t t0 = g_ms;
      Tone_Start(nt.freq);

      while((g_ms - t0) < 2000){
        uint8_t rr,cc;
        if(!KEYPAD_ScanOnce(&rr,&cc) || rr!=R || cc!=C) break; // nh? phím / khác phím
        HAL_Delay(10);
      }
      Tone_Stop();

      // anti-repeat: d?i nh? h?n
      while(1){
        uint8_t rr,cc;
        if(!KEYPAD_ScanOnce(&rr,&cc)) break;
        HAL_Delay(10);
      }
    }
    HAL_Delay(10);
  }
}

/* ===================== Clock: HSI 8MHz ===================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};

  o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&o);

  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV1;
  c.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0);
}

/* ===================== GPIO (kèm PA8 AF_PP) ===================== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  // Keypad columns PA0..PA3 -> Output PP, default HIGH
  g.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);

  // Keypad rows PA4..PA7 -> Input Pull-Up
  g.Pin  = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);

  // Buzzer PA8 -> AF Push-Pull (TIM1_CH1)
  g.Pin   = GPIO_PIN_8;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
}

/* ===================== Error Handler ===================== */
void Error_Handler(void){ __disable_irq(); while (1) {} }
