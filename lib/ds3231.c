#include "i2c.h"
#include "board.h"
#include "ds3231.h"

static uint8_t buf[8];

/**
 * @brief Инициализация RTC
 */
void DS3231_Init(void) {
  buf[0] = (DS3231_CONV | DS3231_1HZ);
  i2c_wr_reg(DS3231_I2C_WRADDR, DS3231_CONTROL_ADDR, buf, 1);
}

/**
 * @brief Чтение времени и календаря
 */
void DS3231_ReadAll(ds3231_t * data) {
  i2c_rd_reg(DS3231_I2C_RDADDR, DS3231_TIME_CAL_ADDR, buf, 7);
  data->Sec  = buf[0];
  data->Min  = buf[1];
  data->Hr   = buf[2];
  data->WD   = buf[3];
  data->Day  = buf[4];
  data->Mon  = buf[5];
  data->Year = buf[6];
}

/**
 * @brief Чтение времени
 */
void DS3231_ReadTime(ds3231_t * data) {
  i2c_rd_reg(DS3231_I2C_RDADDR, DS3231_TIME_CAL_ADDR, buf, 3);
  data->Sec  = buf[0];
  data->Min  = buf[1];
  data->Hr   = buf[2];
}

/**
 * @brief Чтение минут и секунд
 */
void DS3231_ReadMMSS(ds3231_t * data) {
  i2c_rd_reg(DS3231_I2C_RDADDR, DS3231_TIME_CAL_ADDR, buf, 2);
  data->Sec  = buf[0];
  data->Min  = buf[1];
}

/**
 * @brief Чтение календаря
 */
void DS3231_ReadCalendar(ds3231_t * data) {
  i2c_rd_reg(DS3231_I2C_RDADDR, DS3231_CALENDAR_ADDR, buf, 4);
  data->WD   = buf[0];
  data->Day  = buf[1];
  data->Mon  = buf[2];
  data->Year = buf[3];
}

/**
 * @brief Запись времени и календаря
 */
void DS3231_WriteAll(ds3231_t * data) {
  buf[0] = data->Sec;
  buf[1] = data->Min;
  buf[2] = data->Hr;
  buf[3] = data->WD;
  buf[4] = data->Day;
  buf[5] = data->Mon;
  buf[6] = data->Year;

  i2c_wr_reg(DS3231_I2C_WRADDR, DS3231_TIME_CAL_ADDR, buf, 7);
}

/**
 * @brief Запись времени
 */
void DS3231_WriteTime(ds3231_t * data) {
  buf[0] = data->Sec;
  buf[1] = data->Min;
  buf[2] = data->Hr;

  i2c_wr_reg(DS3231_I2C_WRADDR, DS3231_TIME_CAL_ADDR, buf, 3);
}

/**
 * @brief Запись времени только секунды
 */
void DS3231_WriteSS(ds3231_t * data) {
  buf[0] = data->Sec;

  i2c_wr_reg(DS3231_I2C_WRADDR, DS3231_TIME_CAL_ADDR, buf, 1);
}

/**
 * @brief Запись времени только минуты и часы
 */
void DS3231_WriteHHMM(ds3231_t * data) {
  buf[0] = data->Min;
  buf[1] = data->Hr;

  i2c_wr_reg(DS3231_I2C_WRADDR, (DS3231_TIME_CAL_ADDR+1), buf, 2);
}

/**
 * @brief Запись календаря
 */
void DS3231_WriteCalendar(ds3231_t * data) {
  buf[0] = data->WD;
  buf[1] = data->Day;
  buf[2] = data->Mon;
  buf[3] = data->Year;

  i2c_wr_reg(DS3231_I2C_WRADDR, DS3231_CALENDAR_ADDR, buf, 4);
}

/**
 * @brief Convert BCD value to Binary
 */
uint8_t bcd2bin(uint8_t bcd)
{
    return (10 * (bcd >> 4) + (bcd & 0x0f));
}

/**
 * @brief Convert Binary value to BCD
 */
uint8_t bin2bcd(uint8_t bin)
{
    return (((bin / 10 ) << 4) | (bin % 10));
}
