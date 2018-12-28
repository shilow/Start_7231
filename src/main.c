/**
  ******************************************************************************
  * @file    main.c
  * @author  Vladimir N. Shilov <shilow@ukr.net>
  * @version for 4 single 1.2" 7-segment led indicators
  * @date    28-Dec-2018
  * @brief   Start 7231
  ******************************************************************************
  * @attention
  *
  * Wall Clock "Start 7231".
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "board.h"
#include "rtos.h"
#include "event-system.h"
#include "i2c.h"
#include "ds3231.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define STORE_TIMEOUT_MS        5000
#define SHOW_SUBTIME_MS         10000
/* Private macro -------------------------------------------------------------*/
#define PIN_SET(port, pin)      port->ODR |= (uint8_t)pin
#define PIN_RESET(port, pin)    port->ODR &= (uint8_t)(~pin)
#define PIN_TOGGLE(port, pin)   port->ODR ^= (uint8_t)pin

/* EEPROM variables ----------------------------------------------------------*/
#define EEP1_ADDR   FLASH_DATA_START_PHYSICAL_ADDRESS
#pragma location=EEP1_ADDR
__no_init alarm_clock_t EEP_AC1;
#pragma required=EEP_AC1

#define EEP2_ADDR   (uint32_t)(EEP1_ADDR + sizeof(EEP_AC1))
#pragma location=EEP2_ADDR
__no_init alarm_clock_t EEP_AC2;
#pragma required=EEP_AC2

#define EEP3_ADDR   (uint32_t)(EEP2_ADDR + sizeof(EEP_AC2))

/* Private variables ---------------------------------------------------------*/
__IO _flag_t Flag;
__IO indicator_t Indicator;

static es_event_t Event;
static ds3231_t RTC;
static alarm_clock_t AlarmClock[2];
static cntdown_t CountDown;
static btn_t Button[BTN_NUM] = {
  // time, event, pin
  {0, evButtonHPressed, 0},
  {0, evButtonMPressed, 0},
  {0, evButtonKPressed, 0},
  {0, evButtonCPressed, 0},
  {0, evButtonA1Pressed, 0},
  {0, evButtonA2Pressed, 0},
  {0, evButtonTPressed, 0},
  {0, evButtonSPressed, 0},
  {0, evButtonOPressed, 0},
};
static t_i2c_status i2c_status;

const ind_symb_t hex2Sym[16] = {
  sym0, sym1, sym2, sym3, sym4, sym5, sym6, sym7,
  sym8, sym9, symA, symb, symC, symd, symE, symF
};
const ind_com_t pos[4] = {com1, com2, com3, com4};
const uint16_t light[11] =
 {0, 11, 30, 64, 115, 188, 288, 417, 581, 782, 1001};

/* Private function prototypes -----------------------------------------------*/
static void Board_Init(void);
static void refreshIndicators(void);
static void btnScan(void);
static void btnProcess(void);
static void blankColon(void);
static void storeAC1EEP(void);
static void storeAC2EEP(void);
static void toShowTime(void);
static void brightControl(void);
static void beepAC1(void);
static void beepAC2(void);
static void beepCD(void);

/* MAIN function -------------------------------------------------------------*/
void main(void)
{
  Board_Init();

  Flag.Ind_New = 0;
  Flag.RTC_Int = 0;
  Flag.Beeper = 0;
  Flag.Cntd_En = 0;
  Flag.NewCal = 0;

  Indicator.Index = 0;

  AlarmClock[AC1] = EEP_AC1;
  AlarmClock[AC2] = EEP_AC2;

  /* Initialize modules */
  ES_Init(stShowTime);
  RTOS_Init();
  i2c_master_init();
  DS3231_Init();

  i2c_status = i2c_wr_byte(BH1750_I2C_WRADDR, BH1750_CONTINUOUS_HIGH_RES_MODE);
  if (i2c_status == I2C_SUCCESS) {
    Flag.BH1750 = 1;
  } else {
    Flag.BH1750 = 0;
  }

  /* Start tasks */
  RTOS_SetTask(refreshIndicators, 0, 5);
  RTOS_SetTask(brightControl, 0, 1000);

  DS3231_ReadAll(&RTC);
  showTime();

  BEEP_ON;
  Delay(125);
  BEEP_OFF;
  Delay(65);
  BEEP_ON;
  Delay(125);
  BEEP_OFF;

  /* Infinite loop */
  do {
    /** Process flags */
    if (Flag.RTC_Int == 1) {
      Flag.RTC_Int = 0;
      ES_PlaceEvent(evRTCinterrupt);

      /* Process time */
      u8 ss = bcd2bin(RTC.Sec);
      ss ++;
      RTC.Sec = bin2bcd(ss);
      if (ss > 59) { // end of minute
        if (RTC.Min == 0x59) { // end of hour
          if (RTC.Hr == 0x23) { //end of day
            DS3231_ReadAll(&RTC); // start of new day
          } else {
            DS3231_ReadTime(&RTC);
          }
          // start of new hour
          /* Переход на летнее время */
          if ((RTC.Mon == 3) && (RTC.WD == 7) && (RTC.Hr == 3) && (Flag.SunTime == 0)) {
            if ((RTC.Day + 7) > 31) {
              RTC.Hr = 4;
              DS3231_WriteHHMM(&RTC);
              Flag.SunTime = 1;
            }
          }
          /* Переход на зимнее время */
          if ((RTC.Mon == 10) && (RTC.WD == 7) && (RTC.Hr == 4) && (Flag.SunTime == 1)) {
            if ((RTC.Day + 7) > 31) {
              RTC.Hr = 3;
              DS3231_WriteHHMM(&RTC);
              Flag.SunTime = 0;
            }
          }
        } else { // inside of hour
          DS3231_ReadMMSS(&RTC);
        }
      }

      /* Alarm Clocks */
      if ((AlarmClock[AC1].Flag.Enabled != 0) && (RTC.Sec == 0)) {
        if ((AlarmClock[AC1].Hour == RTC.Hr) && (AlarmClock[AC1].Minute == RTC.Min)) {
          Flag.Beeper = 1;
          beepAC1();
        }
      } // end of AC1

      if ((AlarmClock[AC2].Flag.Enabled != 0) && (RTC.Sec == 0)) {
        if ((AlarmClock[AC2].Hour == RTC.Hr) && (AlarmClock[AC2].Minute == RTC.Min)) {
          Flag.Beeper = 1;
          beepAC2();
        }
      } // end of AC2

      /* count down */
      if (Flag.Cntd_En != 0) {
        u8 cd_ss = bcd2bin(CountDown.Second);
        u8 cd_mm = bcd2bin(CountDown.Minute);
        u8 cd_hh = bcd2bin(CountDown.Hour);

        if (cd_ss != 0) {
          cd_ss --;
        } else {
          cd_ss = 59;
          if (cd_mm != 0) {
            cd_mm --;
          } else {
            cd_mm = 59;
            if (cd_hh != 0) {
              cd_hh --;
            } else {
              Flag.Cntd_En = 0;
            }
          }
        }

        if (Flag.Cntd_En == 0) {
          finishCountdown();
        } else {
          CountDown.Second = bin2bcd(cd_ss);
          CountDown.Minute = bin2bcd(cd_mm);
          CountDown.Hour = bin2bcd(cd_hh);
        }

      } // end count down

    } // end Flag.RTC == 1

    if (Flag.Ind_New == 1) {
      Flag.Ind_New = 0;
    }

    /** Rotate task scheduler */
    RTOS_DispatchTask();

    /** Rotate even dispatcher */
    Event = ES_GetEvent();
    if (Event) {
      ES_Dispatch(Event);
    }

    /** Sleep for interrupt */
    wfi();

  } while (1);

}

/* Private functions ---------------------------------------------------------*/
static void Board_Init(void){
  /** Clock */
  /* Initialization of the clock to 16MHz */
  CLK->CKDIVR = 0x00;

  /* Disable clock of unused peripherial */
  CLK->PCKENR1 = (uint8_t)(~CLK_PCKENR1_UART1);
  CLK->PCKENR2 = (uint8_t)(~(CLK_PCKENR2_CAN | CLK_PCKENR2_ADC | CLK_PCKENR2_AWU));

  /** GPIO */
  /* GPIOA - PA1-PA3 button pull-up inputs */
  GPIOA->DDR = 0; // input
  GPIOA->CR1 = (u8)BTN_ROW_PINS; // pull-up
  GPIOA->CR2 = 0; // no interrupt

  /* GPIOC - PC3 TIM1_CH3 PWM, PC4-PC6 SPI, PC7 RTC Interrupt */
  GPIOC->ODR = ~IND_SPI_PINS; // 0 to pins
  GPIOC->DDR = IND_SPI_PINS; // output
  GPIOC->CR1 = IND_SPI_PINS; // push-pull
  GPIOC->CR2 = IND_SPI_PINS; // fast mode

  GPIOC->DDR &= ~RTC_INT_PIN; // input
  GPIOC->CR1 |= RTC_INT_PIN; // pull-up
  GPIOC->CR2 |= RTC_INT_PIN; // interrupt enable
  EXTI->CR1 |= (uint8_t)((uint8_t)(EXTI_SENSITIVITY_FALL_ONLY) << 4);

  /* GPIOD - PD2 PD3 PD5 PD6 led cathodes, PD4 TIM2_CH1 BEEP */
  GPIOD->ODR = 0; // 0 to pins
  GPIOD->DDR = IND_C_PINS | BEEP_PIN; // output
  GPIOD->CR1 = IND_C_PINS; // push-pull
  GPIOD->CR2 = IND_C_PINS; // fast mode

  /** SPI */
  /* Frame Format, BaudRate, Clock Polarity and Phase configuration, Master */
  SPI->CR1 = (uint8_t)(SPI_MODE_MASTER);

  /* Data direction configuration: BDM, BDOE and RXONLY bits */
  SPI->CR2 = (uint8_t)(0x40 | 0x02 | 0x01);

  /* Enable the SPI peripheral*/
  SPI->CR1 |= SPI_CR1_SPE;

  /** TIM1 - LED Bright control */
  /* Set the Autoreload value */
  TIM1->ARRH = (uint8_t)(TIM1_PERIOD >> 8);
  TIM1->ARRL = (uint8_t)(TIM1_PERIOD);

  /* Set the Prescaler value */
  TIM1->PSCRH = (uint8_t)(TIM1_PRESCALER >> 8);
  TIM1->PSCRL = (uint8_t)(TIM1_PRESCALER);

  /* Select the Counter Mode */
  TIM1->CR1 = (uint8_t)(0x10 | 0x08);

  /* Set the Output State & Set the Output Polarity */
  TIM1->CCER2 |= (uint8_t)(TIM1_CCER2_CC3E | TIM1_CCER2_CC3P);

  /* Set the Output Compare Mode */
  TIM1->CCMR3 = TIM1_OCMODE_PWM1;

  /* Set the Pulse value 50% */
  TIM1->CCR3H = (uint8_t)((TIM1_PERIOD/2) >> 8);
  TIM1->CCR3L = (uint8_t)(TIM1_PERIOD/2);

  /* TIM1 Enable */
  TIM1->CR1 |= TIM1_CR1_CEN;

  /*TIM1 Ctrl PWM Outputs Enable */
  TIM1->BKR |= TIM1_BKR_MOE;

  /** TIM2 - Beeper */
  /* - TIM2CLK = 16 MHz
    - TIM2 counter clock = TIM2CLK / TIM2_PRESCALER = 16 MHz/16 = 1 MHz */
  /* Set the Autoreload value */
  TIM2->ARRH = (uint8_t)(TIM2_PERIOD >> 8) ;
  TIM2->ARRL = (uint8_t)(TIM2_PERIOD);

  /* Set the Prescaler value */
  TIM2->PSCR = (uint8_t)(TIM2_PRESCALER);

  /* Generate an update event to reload the Prescaler value immediately */
  TIM2->EGR = (uint8_t)0x01; // TIM2_EventSource_Update

  /* Set the PWM Mode */
  TIM2->CCMR1 = (uint8_t)0x70; // TIM2_OCMode_PWM2 on channel 1

  /* Set the Output State Enable, Output Polarity Low */
  TIM2->CCER1 = TIM2_CCER1_CC1E;// |TIM2_CCER1_CC1P;

  /* Enables TIM2 peripheral Preload register on ARR. */
  TIM2->CR1 = TIM2_CR1_ARPE;

  /* TIM2 enable counter */
  //TIM2->CR1 |= TIM2_CR1_CEN;

  /* Set the 50% PWM Duty */
  TIM2->CCR1H = (uint8_t)((TIM2_PERIOD/2) >> 8);
  TIM2->CCR1L = (uint8_t)(TIM2_PERIOD/2);
}

static void beepAC1(void) {
  static uint8_t c = 0;

  switch (c) {
  case 0:
    c = 1;
    BEEP_2KHZ;
    BEEP_ON;
    RTOS_SetTask(beepAC1, 150, 0);
    break;

  case 1:
    c = 2;
    BEEP_1KHZ;
    BEEP_OFF;
    RTOS_SetTask(beepAC1, 75, 0);
    break;

  case 2:
    c = 3;
    BEEP_2KHZ;
    BEEP_ON;
    RTOS_SetTask(beepAC1, 100, 0);
    break;

  case 3:
    c = 0;
    BEEP_1KHZ;
    BEEP_OFF;
    RTOS_SetTask(beepAC1, 675, 0);
    break;

  default:
    BEEP_1KHZ;
    BEEP_OFF;
    c = 0;
    break;
  }
}

static void beepAC2(void) {
  static uint8_t c = 0;

  switch (c) {
  case 0:
    c = 1;
    BEEP_2KHZ;
    BEEP_ON;
    RTOS_SetTask(beepAC2, 100, 0);
    break;

  case 1:
    c = 2;
    BEEP_1KHZ;
    BEEP_OFF;
    RTOS_SetTask(beepAC2, 75, 0);
    break;

  case 2:
    c = 3;
    BEEP_2KHZ;
    BEEP_ON;
    RTOS_SetTask(beepAC2, 150, 0);
    break;

  case 3:
    c = 0;
    BEEP_1KHZ;
    BEEP_OFF;
    RTOS_SetTask(beepAC2, 675, 0);
    break;

  default:
    BEEP_1KHZ;
    BEEP_OFF;
    c = 0;
    break;
  }
}

static void beepCD(void) {
  static uint8_t c = 0;

  switch (c) {
  case 0:
    c = 1;
    BEEP_4KHZ;
    BEEP_ON;
    RTOS_SetTask(beepCD, 250, 0);
    break;

  case 1:
    c = 0;
    BEEP_1KHZ;
    BEEP_OFF;
    RTOS_SetTask(beepCD, 750, 0);
    break;

  default:
    BEEP_1KHZ;
    BEEP_OFF;
    c = 0;
    break;
  }
}

/* ES functions --------------------------------------------------------------*/
void showTime(void) {
  // hours and top dot
  if (RTC.Hr >= 0x10) {
    Indicator.Value[0] = (ind_symb_t)(hex2Sym[RTC.Hr >> 4] | symDP);
  } else {
    Indicator.Value[0] = symDP;
  }
  Indicator.Value[1] = hex2Sym[RTC.Hr & 0x0F];
  // minutes and bottom dot
  Indicator.Value[2] = (ind_symb_t)(hex2Sym[RTC.Min >> 4] | symDP);
  if ((AlarmClock[AC1].Flag.Enabled == 0) && ((AlarmClock[AC2].Flag.Enabled == 0))) {
    Indicator.Value[3] = hex2Sym[RTC.Min & 0x0F];
  } else {
    Indicator.Value[3] = (ind_symb_t)(hex2Sym[RTC.Min & 0x0F] | symDP);
  }

  RTOS_SetTask(blankColon, 500, 0);
}

void showSeconds(void) {
  // minutes and top dot
  Indicator.Value[0] = (ind_symb_t)(hex2Sym[RTC.Min >> 4] | symDP);
  Indicator.Value[1] = hex2Sym[RTC.Min & 0x0F];
  // seconds and bottom dot
  Indicator.Value[2] = (ind_symb_t)(hex2Sym[RTC.Sec >> 4] | symDP);
  Indicator.Value[3] = hex2Sym[RTC.Sec & 0x0F];
}

/**
 * @brief Show Countdown.
 */
void showCountdown(void) {
  if ((CountDown.Hour == 0) && (Flag.Cntd_En != 0)) {
    // minutes and top dot
    Indicator.Value[0] = (ind_symb_t)(hex2Sym[CountDown.Minute >> 4] | symDP);
    Indicator.Value[1] = hex2Sym[CountDown.Minute & 0x0F];
    // seconds and bottom dot
    Indicator.Value[2] = (ind_symb_t)(hex2Sym[CountDown.Second >> 4] | symDP);
    Indicator.Value[3] = hex2Sym[CountDown.Second & 0x0F];
  } else {
    // hours and top dot
    if (CountDown.Hour >= 0x10) {
      Indicator.Value[0] = (ind_symb_t)(hex2Sym[CountDown.Hour >> 4] | symDP);
    } else {
      Indicator.Value[0] = symDP;
    }
    Indicator.Value[1] = hex2Sym[CountDown.Hour & 0x0F];
    // minutes and bottom dot
    Indicator.Value[2] = (ind_symb_t)(hex2Sym[CountDown.Minute >> 4] | symDP);
    Indicator.Value[3] = hex2Sym[CountDown.Minute & 0x0F];

  }

  if (Flag.Cntd_En != 0) {
    Indicator.Value[3] |= symDP;
    if (CountDown.Hour != 0) {
      RTOS_SetTask(blankColon, 500, 0);
    }
  }
}

void finishCountdown(void) {
  CountDown.Hour = 0;
  CountDown.Minute = 0;
  CountDown.Second = 0;
  Flag.Beeper = 1;
  beepCD();
}

void showAlarm1(void) {
  // hours and top dot
  if (AlarmClock[AC1].Hour >= 0x10) {
    Indicator.Value[0] = (ind_symb_t)(hex2Sym[AlarmClock[AC1].Hour >> 4] | symDP);
  } else {
    Indicator.Value[0] = symDP;
  }
  Indicator.Value[1] = hex2Sym[AlarmClock[AC1].Hour & 0x0F];
  // minutes and bottom dot
  Indicator.Value[2] = (ind_symb_t)(hex2Sym[AlarmClock[AC1].Minute >> 4] | symDP);
  if (AlarmClock[AC1].Flag.Enabled == 0) {
    Indicator.Value[3] = hex2Sym[AlarmClock[AC1].Minute & 0x0F];
  } else {
    Indicator.Value[3] = (ind_symb_t)(hex2Sym[AlarmClock[AC1].Minute & 0x0F] | symDP);
  }
}

void showAlarm2(void) {
  // hours and top dot
  if (AlarmClock[AC2].Hour >= 0x10) {
    Indicator.Value[0] = (ind_symb_t)(hex2Sym[AlarmClock[AC2].Hour >> 4] | symDP);
  } else {
    Indicator.Value[0] = symDP;
  }
  Indicator.Value[1] = hex2Sym[AlarmClock[AC2].Hour & 0x0F];
  // minutes and bottom dot
  Indicator.Value[2] = (ind_symb_t)(hex2Sym[AlarmClock[AC2].Minute >> 4] | symDP);
  if (AlarmClock[AC2].Flag.Enabled == 0) {
    Indicator.Value[3] = hex2Sym[AlarmClock[AC2].Minute & 0x0F];
  } else {
    Indicator.Value[3] = (ind_symb_t)(hex2Sym[AlarmClock[AC2].Minute & 0x0F] | symDP);
  }
}

void toggleCountdown(void) {
  if (Flag.Cntd_En == 0) {
    Flag.Cntd_En = 1;
  } else {
    Flag.Cntd_En = 0;
  }
}

void toggleAC1(void) {
  if (AlarmClock[AC1].Flag.Enabled == 0) {
    AlarmClock[AC1].Flag.Enabled = 1;
  } else {
    AlarmClock[AC1].Flag.Enabled = 0;
  }
  RTOS_SetTask(storeAC1EEP, STORE_TIMEOUT_MS, 0);
}

void toggleAC2(void) {
  if (AlarmClock[AC2].Flag.Enabled == 0) {
    AlarmClock[AC2].Flag.Enabled = 1;
  } else {
    AlarmClock[AC2].Flag.Enabled = 0;
  }
  RTOS_SetTask(storeAC2EEP, STORE_TIMEOUT_MS, 0);
}

void timeIncHH(void) {
  u8 hh = bcd2bin(RTC.Hr);
  if (hh < 23) {
    hh ++;
  } else {
    hh = 0;
  }
  RTC.Hr = bin2bcd(hh);
  DS3231_WriteHHMM(&RTC);
}

void timeIncMM(void) {
  u8 mm = bcd2bin(RTC.Min);
  if (mm < 59) {
    mm ++;
  } else {
    mm = 0;
  }
  RTC.Min = bin2bcd(mm);
  DS3231_WriteHHMM(&RTC);
}

void timeKorrect(void) {
  if (RTC.Sec > 0x30) {
    RTC.Min ++;
    if (RTC.Min > 0x59) {
      RTC.Min = 0;
    }
  }
  RTC.Sec = 0;
  DS3231_WriteSS(&RTC);
}

void cntdwnIncHH(void) {
  u8 hh = bcd2bin(CountDown.Hour);
  if (hh < 99) {
    hh ++;
  } else {
    hh = 0;
  }
  CountDown.Hour = bin2bcd(hh);
}

void cntdwnIncMM(void) {
  u8 mm = bcd2bin(CountDown.Minute);
  if (mm < 59) {
    mm ++;
  } else {
    mm = 0;
  }
  CountDown.Minute = bin2bcd(mm);
}

void alarm1IncHH(void) {
  u8 hh = bcd2bin(AlarmClock[AC1].Hour);
  if (hh < 23) {
    hh ++;
  } else {
    hh = 0;
  }
  AlarmClock[AC1].Hour = bin2bcd(hh);
  RTOS_SetTask(storeAC1EEP, STORE_TIMEOUT_MS, 0);
}

void alarm1IncMM(void) {
  u8 mm = bcd2bin(AlarmClock[AC1].Minute);
  if (mm < 59) {
    mm ++;
  } else {
    mm = 0;
  }
  AlarmClock[AC1].Minute = bin2bcd(mm);
  RTOS_SetTask(storeAC1EEP, STORE_TIMEOUT_MS, 0);
}

void alarm2IncHH(void) {
  u8 hh = bcd2bin(AlarmClock[AC2].Hour);
  if (hh < 23) {
    hh ++;
  } else {
    hh = 0;
  }
  AlarmClock[AC2].Hour = bin2bcd(hh);
  RTOS_SetTask(storeAC2EEP, STORE_TIMEOUT_MS, 0);
}

void alarm2IncMM(void) {
  u8 mm = bcd2bin(AlarmClock[AC2].Minute);
  if (mm < 59) {
    mm ++;
  } else {
    mm = 0;
  }
  AlarmClock[AC2].Minute = bin2bcd(mm);
  RTOS_SetTask(storeAC2EEP, STORE_TIMEOUT_MS, 0);
}


/* RTOS functions ------------------------------------------------------------*/
/**
  * @brief  Out next symbol to indicators trough SPI.
  * @param  : None
  * @retval : None
  */
static void refreshIndicators(void) {

  if (Indicator.Index < (IND_NUM-1)) {
    Indicator.Index ++;
  } else {
    Indicator.Index = 0;
  }

  SPI->DR = Indicator.Value[Indicator.Index];

  IND_ALL_OFF;

  /* Wait for transfer complete */
  while (SPI->SR & SPI_FLAG_BSY) {
    nop();
  }

  TIM1->CR1 &= ~TIM1_CR1_CEN;
  LATCH_DOWN;

  LATCH_UP;
  uint8_t tmp = pos[Indicator.Index];
  IND_ON(tmp);
  TIM1->CR1 |= TIM1_CR1_CEN;

  if (Indicator.Index == 0) {
    RTOS_SetTask(btnProcess, 1, 0);
  } else {
    RTOS_SetTask(btnScan, 1, 0);
  }

  Flag.Ind_New = 1;
}

/**
 * @brief Control LED bright level
 */
void brightControl(void) {
  uint8_t buf[2];
  i2c_status = i2c_rd_bytes(BH1750_I2C_RDADDR, buf, 2);
  if (i2c_status == I2C_SUCCESS) {
    if (buf[0] > 0x00) {
      TIM1->CCR3H = (uint8_t)(light[8] >> 8);
      TIM1->CCR3L = (uint8_t)(light[8]);
    } else {
      if (buf[1] < 0x08) {
        TIM1->CCR3H = (uint8_t)(light[4] >> 8);
        TIM1->CCR3L = (uint8_t)(light[4]);
      } else {
        TIM1->CCR3H = (uint8_t)(light[7] >> 8);
        TIM1->CCR3L = (uint8_t)(light[7]);
      }
    }
  }
}

/**
  * @brief  Сканируем текущий ряд кнопок.
  * @param  : Column number - 1, 2 or 3.
  * @retval : None
  */
static void btnScan(void) {
  uint8_t column = Indicator.Index - 1;
  column *= 3;

  uint8_t pins = BTN_STATUS;

  if ((pins & BTN_ROW1_PIN) == 0) {
    Button[column].pin = 1;
  }
  column ++;
  if ((pins & BTN_ROW2_PIN) == 0 ) {
    Button[column].pin = 1;
  }
  column ++;
  if ((pins & BTN_ROW3_PIN) == 0 ) {
    Button[column].pin = 1;
  }
}

/**
  * @brief  Обработка ранее отсканированных кнопок.
  * @param  : None
  * @retval : None
  */
static void btnProcess(void) {
  /* if Freeze button pressed when Beeped - stop Beep */
  if ((Flag.Beeper != 0) && (Button[8].pin != 0)) {
    Flag.Beeper = 0;
    Button[8].pin = 0;
    BEEP_OFF;
    RTOS_DeleteTask(beepAC1);
    RTOS_DeleteTask(beepAC2);
    RTOS_DeleteTask(beepCD);
  }

  uint8_t i;
  for (i=0; i<BTN_NUM; i++) {
    if (Button[i].pin != 0) {
      // button pressed
      if (Button[i].pin == 1) {
        Button[i].pin = 0xFF;
        Button[i].time ++;
        if (Button[i].time > BTN_TIME_HOLDED) {
          Button[i].time -= BTN_TIME_REPEATED;
          ES_PlaceEvent(Button[i].event); // button pressed auto repeat
        }
      } else {
        // button released
        if (Button[i].time >= BTN_TIME_PRESSED) {
          ES_PlaceEvent(Button[i].event); // process short press.
        }
        Button[i].time = 0;
        Button[i].pin = 0;
      }
    } /* end (pin != 0) */
  } /* end FOR */
}

/**
 * Shutdown colon
 */
static void blankColon(void) {
  Indicator.Value[0] &= ~symDP;
  Indicator.Value[2] &= ~symDP;
}

void showWDay(void) {
  Indicator.Value[0] = symBlank;
  Indicator.Value[1] = hex2Sym[RTC.WD];
  Indicator.Value[2] = symBlank;
  Indicator.Value[3] = symBlank;
  RTOS_SetTask(toShowTime, SHOW_SUBTIME_MS, 0);
}

void showDDMM(void) {
  Indicator.Value[0] = hex2Sym[RTC.Day >> 4];
  Indicator.Value[1] = (ind_symb_t)(hex2Sym[RTC.Day & 0x0F] | symDP);
  Indicator.Value[2] = hex2Sym[RTC.Mon >> 4];
  Indicator.Value[3] = hex2Sym[RTC.Mon & 0x0F];
  RTOS_SetTask(toShowTime, SHOW_SUBTIME_MS, 0);
}

void showYear(void) {
  Indicator.Value[0] = sym2;
  Indicator.Value[1] = sym0;
  Indicator.Value[2] = hex2Sym[RTC.Year >> 4];
  Indicator.Value[3] = hex2Sym[RTC.Year & 0x0F];
  RTOS_SetTask(toShowTime, SHOW_SUBTIME_MS, 0);
}

void showSuntime(void) {
  Indicator.Value[0] = symS;
  Indicator.Value[1] = symu;
  Indicator.Value[2] = symn;
  if (Flag.SunTime == 0) {
    Indicator.Value[3] = symOff;
  } else {
    Indicator.Value[3] = symOn;
  }
  RTOS_SetTask(toShowTime, SHOW_SUBTIME_MS, 0);
}

void calWDInc(void) {
  if (RTC.WD < 7) {
    RTC.WD ++;
  } else {
    RTC.WD = 1;
  }
  Flag.NewCal = 1;
}

void calWDDec(void) {
  if (RTC.WD > 1) {
    RTC.WD --;
  } else {
    RTC.WD = 7;
  }
  Flag.NewCal = 1;
}

void calDDInc(void) {
  u8 dd = bcd2bin(RTC.Day);
  if (dd < 31) {
    dd ++;
  } else {
    dd = 1;
  }
  RTC.Day = bin2bcd(dd);
  Flag.NewCal = 1;
}

void calMMInc(void) {
  u8 mm = bcd2bin(RTC.Mon);
  if (mm < 12) {
    mm ++;
  } else {
    mm = 1;
  }
  RTC.Mon = bin2bcd(mm);
  Flag.NewCal = 1;
}

void calYearInc(void) {
  u8 yy = bcd2bin(RTC.Year);
  if (yy < 99) {
    yy ++;
  } else {
    yy = 0;
  }
  RTC.Year = bin2bcd(yy);
  Flag.NewCal = 1;
}

void calYearDec(void) {
  u8 yy = bcd2bin(RTC.Year);
  if (yy > 0) {
    yy --;
  } else {
    yy = 99;
  }
  Flag.NewCal = 1;
  RTC.Year = bin2bcd(yy);
}

void calToggleSuntime(void) {
  if (Flag.SunTime == 0) {
    Flag.SunTime = 1;
  } else {
    Flag.SunTime = 0;
  }
  Flag.NewCal = 1;
}

static void toShowTime(void) {
  if (Flag.NewCal != 0) {
    Flag.NewCal = 0;
    DS3231_WriteCalendar(&RTC);
  }
  ES_SetState(stShowTime);
  showTime();
}


/**
 * @brief Read level of light from BH1750
 */
void showLightLevel(void) {
  uint8_t buf[2];
  if (Flag.BH1750 == 1) {
  i2c_status = i2c_rd_bytes(BH1750_I2C_RDADDR, buf, 2);
    if (i2c_status == I2C_SUCCESS) {
      Indicator.Value[0] = hex2Sym[buf[0] >> 4];
      Indicator.Value[1] = hex2Sym[buf[0] & 0x0F];
      Indicator.Value[2] = hex2Sym[buf[1] >> 4];
      Indicator.Value[3] = hex2Sym[buf[1] & 0x0F];
    } else {
      Indicator.Value[0] = symE;
      Indicator.Value[1] = symr;
      Indicator.Value[2] = symr;
      Indicator.Value[3] = symBlank;
    }
  } else {
    Indicator.Value[0] = symn;
    Indicator.Value[1] = sym0;
    Indicator.Value[2] = symn;
    Indicator.Value[3] = symE;
  }
}

/**
 * @brief Store AlarmClock 1 to EEPROM
 */
static void storeAC1EEP(void) {
  disableInterrupts();

  /* Unlock Data Flash */
  FLASH->DUKR = (uint8_t)0xAE;
  FLASH->DUKR = (uint8_t)0x56;

  /* wait for eeprom ready */
  while (!(FLASH->IAPSR & FLASH_IAPSR_DUL));

  EEP_AC1 = AlarmClock[AC1];

  /* Lock Flash */
  FLASH->IAPSR &= (uint8_t)0xF7;

  enableInterrupts();
}

/**
 * @brief Store AlarmClock 2 to EEPROM
 */
static void storeAC2EEP(void) {
  disableInterrupts();

  /* Unlock Data Flash */
  FLASH->DUKR = (uint8_t)0xAE;
  FLASH->DUKR = (uint8_t)0x56;

  /* wait for eeprom ready */
  while (!(FLASH->IAPSR & FLASH_IAPSR_DUL));

  EEP_AC2 = AlarmClock[AC2];

  /* Lock Flash */
  FLASH->IAPSR &= (uint8_t)0xF7;

  enableInterrupts();
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  Indicator.Value[0] = symA;
  Indicator.Value[1] = symS;
  Indicator.Value[2] = symS;
  Indicator.Value[3] = symE;

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) Vladimir N. Shilov *****END OF FILE****/
