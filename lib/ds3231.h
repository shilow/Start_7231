#pragma once
#ifndef DS3231_H
#define DS3231_H

#include "stm8s.h"

// i2c slave address of the DS3231 chip
#define DS3231_I2C_WRADDR           0xD0
#define DS3231_I2C_RDADDR           0xD1

// timekeeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_CALENDAR_ADDR        0x03
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_A1IE     0x01
#define DS3231_A2IE     0x02
#define DS3231_INTCN    0x04
#define DS3231_RS1	    0x08
#define DS3231_RS2	    0x10
#define DS3231_CONV	    0x20
#define DS3231_BBSQW	  0x40
#define DS3231_OSC_OSC	0x80

// square-wave output frequency
#define DS3231_1HZ	    0x00
#define DS3231_1024HZ	  0x01
#define DS3231_4096HZ	  0x02
#define DS3231_8192HZ	  0x03

// status register bits
#define DS3231_A1F      0x01
#define DS3231_A2F      0x02
#define DS3231_OSF      0x80

/**
 * @brief   Clock sructure.
 * @note    structure of ds3231 data.
 */
typedef struct {
  /**
   * @brief  Seconds
   */
  uint8_t Sec;
  /**
   * @brief  Minutes
   */
  uint8_t Min;
  /**
   * @brief  Hours
   */
  uint8_t Hr;
  /**
   * @brief  Week Day
   */
  uint8_t WD;
  /**
   * @brief  Day of Month
   */
  uint8_t Day;
  /**
   * @brief  Month
   */
  uint8_t Mon;
  /**
   * @brief  Year
   */
  uint8_t Year;
} ds3231_t;

void DS3231_Init(void);
void DS3231_ReadAll(ds3231_t * data);
void DS3231_ReadTime(ds3231_t * data);
void DS3231_ReadMMSS(ds3231_t * data);
void DS3231_ReadCalendar(ds3231_t * data);
void DS3231_WriteAll(ds3231_t * data);
void DS3231_WriteSS(ds3231_t * data);
void DS3231_WriteHHMM(ds3231_t * data);
void DS3231_WriteCalendar(ds3231_t * data);

uint8_t bcd2bin(uint8_t bcd);
uint8_t bin2bcd(uint8_t bin);

#endif // DS3231_H
