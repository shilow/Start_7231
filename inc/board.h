#pragma once
#ifndef BOARD_H
#define BOARD_H

#include "stm8s.h"
#include "event-system.h"

/* BH1750 */
/* No active state */
#define BH1750_POWER_DOWN           0x00
/* Waiting for measurement command */
#define BH1750_POWER_ON             0x01
/* Reset only illuminance data register. Not accepted in POWER_DOWN mode */
#define BH1750_RESET                0x07

/* Continuous measurement. 1.0 lx resolution. Integration time is 120..180ms. */
#define BH1750_CONTINUOUS_HIGH_RES_MODE   0x10
/* Continuous measurement. 0.5 lx resolution. Integration time is 120..180ms. */
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2 0x11
/* Continuous measurement. 4.0 lx resolution. Integration time is 16...24ms.  */
#define BH1750_CONTINUOUS_LOW_RES_MODE    0x13

/* One-time measurement (Power down after measurement). 1.0 lx resolution. Integration time is 120..180ms. */
#define BH1750_ONE_TIME_HIGH_RES_MODE     0x20
/* One-time measurement (Power down after measurement). 0.5 lx resolution. Integration time is 120..180ms. */
#define BH1750_ONE_TIME_HIGH_RES_MODE_2   0x21
/* One-time measurement (Power down after measurement). 4.0 lx resolution. Integration time is 16...24ms.  */
#define BH1750_ONE_TIME_LOW_RES_MODE      0x23

/* High Bit of changing Measurement Time. */
#define BH1750_MEASUREMENT_TIME_H   0x40
/* Low  Bit of changing Measurement Time. */
#define BH1750_MEASUREMENT_TIME_L   0x60

/* Default Integration/Measurement time = 69  */
#define BH1750_MTREG_DEFAULT        0x45
/* Min.    Integration/Measurement time = 31  */
#define BH1750_MTREG_MIN            0x1F
/* Max.    Integration/Measurement time = 254 */
#define BH1750_MTREG_MAX            0xFE

/* Measurement Accuracy. Used for result adjustment/calibration (min - 0.96, max - 1.44, typ - 1.2) */
#define BH1750_MEASUREMENT_ACCURACY 1.2

/* Device Address on i2c serial bus when address pin LOW */
#define BH1750_DEFAULT_I2CADDR      0x23
/* Device Address on i2c serial bus when address pin HIGH */
#define BH1750_SECOND_I2CADDR       0x5C
/* i2c slave address of the BH1750 chip */
#define BH1750_I2C_WRADDR           0x46
#define BH1750_I2C_RDADDR           0x47

/* Пищалка / TIM2_CH1 */
#define BEEP_PIN        GPIO_PIN_4
#define BEEP_ON         TIM2->CR1 |= TIM2_CR1_CEN
#define BEEP_OFF        TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN)
#define BEEP_1KHZ       TIM2->PSCR = TIM2_PRESCALER_16
#define BEEP_2KHZ       TIM2->PSCR = TIM2_PRESCALER_8
#define BEEP_4KHZ       TIM2->PSCR = TIM2_PRESCALER_4
#define TIM2_PERIOD     1000
#define TIM2_PRESCALER  TIM2_PRESCALER_16

/* Часы реального времени */
#define RTC_INT_PORT  (GPIO_TypeDef *)GPIOC
#define RTC_INT_PIN   GPIO_PIN_7

/* Кнопки */
#define BTN_NUM       (u8)9
#define BTN_ROW_NUM   (u8)3
#define BTN_ROW_PORT  (GPIO_TypeDef *)GPIOA
#define BTN_ROW_PINS  (GPIO_Pin_TypeDef)(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define BTN_ROW1_PIN  GPIO_PIN_2
#define BTN_ROW2_PIN  GPIO_PIN_1
#define BTN_ROW3_PIN  GPIO_PIN_3
#define BTN_STATUS    (uint8_t)(GPIOA->IDR & (uint8_t)BTN_ROW_PINS)

/* период обработки кнопок = периоду обновления индикаторов = 20 мс
 * все временные константы кнопок в этих тиках, т.е. кратны 20 мс. */
#define BTN_TIME_PRESSED  (u8)5
#define BTN_TIME_HOLDED   (u8)25
#define BTN_TIME_REPEATED (u8)10
#define BTN_TIME_PAUSE    (u8)5

/* Индикаторы */
#define IND_NUM       (u8)4

#define IND_C_PINS    (GPIO_Pin_TypeDef)(GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6)
#define IND_C1_PIN    GPIO_PIN_6
#define IND_C2_PIN    GPIO_PIN_5
#define IND_C3_PIN    GPIO_PIN_3
#define IND_C4_PIN    GPIO_PIN_2

#define BTN_COL1      GPIO_PIN_5
#define BTN_COL2      GPIO_PIN_3
#define BTN_COL3      GPIO_PIN_2
#define BTN_COLS      (GPIO_Pin_TypeDef)(GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2)

#define IND_ALL_OFF   GPIOD->ODR = 0
#define IND_ON(pin)   GPIOD->ODR = pin
#define IND_C1_ON     GPIOD->ODR = IND_C1_PIN
#define IND_C2_ON     GPIOD->ODR = IND_C2_PIN
#define IND_C3_ON     GPIOD->ODR = IND_C3_PIN
#define IND_C4_ON     GPIOD->ODR = IND_C4_PIN

#define IND_SPI_PORT  (GPIO_TypeDef *)GPIOC
#define IND_SPI_PINS  (GPIO_Pin_TypeDef)(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6)
#define IND_SPI_LATCH GPIO_PIN_4
#define IND_SPI_SCK   GPIO_PIN_5
#define IND_SPI_DO    GPIO_PIN_6
#define LATCH_DOWN    GPIOC->ODR &= ~IND_SPI_LATCH
#define LATCH_UP      GPIOC->ODR |= IND_SPI_LATCH

/* LED Bright control TIM1_CH3 */
#define IND_PWM_PORT    (GPIO_TypeDef *)GPIOC
#define IND_PWM_PIN     GPIO_PIN_3
#define TIM1_PERIOD     1000
#define TIM1_PRESCALER  63

typedef enum {
  com1  = (uint8_t)IND_C1_PIN,
  com2  = (uint8_t)IND_C2_PIN,
  com3  = (uint8_t)IND_C3_PIN,
  com4  = (uint8_t)IND_C4_PIN
} ind_com_t;

/* led indicator segment definition */
/*
 A
F B
 G
E C
 D P
*/
#define IND_SEG_A     0x10
#define IND_SEG_B     0x08
#define IND_SEG_C     0x04
#define IND_SEG_D     0x02
#define IND_SEG_E     0x01
#define IND_SEG_F     0x40
#define IND_SEG_G     0x80
#define IND_SEG_H     0x20

typedef enum {
  sym0      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C | IND_SEG_D | IND_SEG_E | IND_SEG_F),
  sym1      = (uint8_t)(IND_SEG_B | IND_SEG_C),
  sym2      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_G | IND_SEG_E | IND_SEG_D),
  sym3      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C | IND_SEG_D | IND_SEG_G),
  sym4      = (uint8_t)(IND_SEG_F | IND_SEG_G | IND_SEG_B | IND_SEG_C),
  sym5      = (uint8_t)(IND_SEG_A | IND_SEG_F | IND_SEG_G | IND_SEG_C | IND_SEG_D),
  sym6      = (uint8_t)(IND_SEG_A | IND_SEG_G | IND_SEG_C | IND_SEG_D | IND_SEG_E | IND_SEG_F),
  sym7      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C),
  sym8      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C | IND_SEG_D | IND_SEG_E | IND_SEG_F | IND_SEG_G),
  sym9      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C | IND_SEG_D | IND_SEG_G | IND_SEG_F),
  symA      = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_C | IND_SEG_E | IND_SEG_F | IND_SEG_G),
  symb      = (uint8_t)(IND_SEG_F | IND_SEG_E | IND_SEG_C | IND_SEG_D | IND_SEG_G),
  symc      = (uint8_t)(IND_SEG_D | IND_SEG_E | IND_SEG_G),
  symC      = (uint8_t)(IND_SEG_A | IND_SEG_D | IND_SEG_E | IND_SEG_F ),
  symd      = (uint8_t)(IND_SEG_B | IND_SEG_C | IND_SEG_D | IND_SEG_E | IND_SEG_G),
  symE      = (uint8_t)(IND_SEG_A | IND_SEG_D | IND_SEG_E | IND_SEG_F | IND_SEG_G),
  symF      = (uint8_t)(IND_SEG_A | IND_SEG_E | IND_SEG_F | IND_SEG_G),
  symOff    = (uint8_t)(IND_SEG_C | IND_SEG_D | IND_SEG_E | IND_SEG_G),
  symOn     = (uint8_t)(IND_SEG_A | IND_SEG_B | IND_SEG_F | IND_SEG_G),
  symDP     = (uint8_t)IND_SEG_H,
  symMinus  = (uint8_t)IND_SEG_G,
  symn      = (uint8_t)(IND_SEG_E | IND_SEG_C | IND_SEG_G),
  symr      = (uint8_t)(IND_SEG_E | IND_SEG_G),
  symS      = (uint8_t)(IND_SEG_A | IND_SEG_D | IND_SEG_F | IND_SEG_G | IND_SEG_C),
  symu      = (uint8_t)(IND_SEG_E | IND_SEG_C | IND_SEG_D),
  symBlank  = (uint8_t)0x00
} ind_symb_t;

typedef struct {
  ind_symb_t Value[IND_NUM];
  uint8_t Index;
} indicator_t;

typedef enum {
  btnH  = 0x01,
  btnM  = 0x02,
  btnK  = 0x04,
  btnC  = 0x08,
  btnA1 = 0x10,
  btnA2 = 0x20,
  btnT  = 0x40,
  btnS  = 0x80,
  btnO  = 0x100
} btn_val_t;

typedef struct {
  uint8_t     time;
  es_event_t  event;
  uint8_t     pin;
} btn_t;

typedef struct {
  u8 RTC_Int : 1;
  u8 Ind_New : 1;
  u8 Cntd_En : 1;
  u8 SunTime : 1;
  u8 Beeper  : 1;
  u8 BH1750  : 1;
  u8 NewCal  : 1;
} _flag_t;

typedef enum {
  AC1 = 0x00,
  AC2 = 0x01
} ac_num_t;

typedef struct {
  u8 Enabled   : 1;
  u8 Monday    : 1;
  u8 Tuesday   : 1;
  u8 Wednesday : 1;
  u8 Thursday  : 1;
  u8 Friday    : 1;
  u8 Saturday  : 1;
  u8 Sunday    : 1;
} ac_flag_t;

typedef struct {
  uint8_t   Hour;
  uint8_t   Minute;
  ac_flag_t Flag;
} alarm_clock_t;

typedef struct {
  uint8_t Hour;
  uint8_t Minute;
  uint8_t Second;
} cntdown_t;

#endif /* BOARD_H */
