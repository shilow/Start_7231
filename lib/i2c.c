/******************************************************************************
* Драйвер модуля I2C микроконтроллера STM8S003F
*
* Автор: Осипов Андрей
* Дата:  17 июля 2014
* URL:   http://hamlab.net/mcu/stm8/i2c.html
******************************************************************************/

#include "i2c.h"

/* Таймаут ожидания события I2C */
extern __IO uint8_t I2C_timeout;

/* Ожидание наступления события event в течении времени timeout в мс */
#define wait_event(event, timeout) I2C_timeout = timeout;\
                                   while(event && I2C_timeout);\
                                   if(I2C_timeout == 0) return I2C_TIMEOUT;


/**
 * @brief Инициализация I2C интерфейса
 */
void i2c_master_init(void) {
  /* Настройка GPIO SDA SCL - HiZ, Open drain, Fast */
  GPIOB->DDR |= (GPIO_PIN_4 | GPIO_PIN_5);
  GPIOB->ODR |= (GPIO_PIN_4 | GPIO_PIN_5);
  GPIOB->CR2 |= (GPIO_PIN_5 | GPIO_PIN_5);

  //Частота тактирования периферии MHz
  I2C->FREQR = (uint8_t)(F_MASTER_MHZ);

  //Отключаем I2C
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);

#ifdef I2C_FAST
  /* Set Maximum Rise Time: 300ns max in Fast Mode
    = [300ns/(1/F_MASTER_HZ)]+1
    = [(F_MASTER_MHZ * 3)/10]+1
    = ((16 * 3 )/ 10 +) 1 = 5 */
  I2C->TRISER = 5;

  /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
  //ccr = (uint16_t) 1; //(F_MASTER_HZ / (25 * F_I2C_HZ)); // = 16000 / (25 * 400) = 1,6 = 1

  /* Write CCR with new calculated value */
  I2C->CCRL = 1;
  I2C->CCRH = (uint8_t)(I2C_CCRH_FS | I2C_CCRH_DUTY);
#else
//  uint16_t ccr;
  //CCR = Fmaster/2*Fiic = 16MHz/2*100kHz = 80
//  ccr = (uint16_t)(F_MASTER_HZ / (2 * F_I2C_HZ));

  //Set Maximum Rise Time: 1000ns max in Standard Mode
  //= [1000ns/(1/InputClockFrequencyMHz.10e6)]+1
  //= InputClockFrequencyMHz+1
  I2C->TRISER = 17; //(uint8_t)(F_MASTER_MHZ + 1);

  /* Write CCR with new calculated value */
  I2C->CCRL = 80; //ccr;
  I2C->CCRH = 0; //(uint8_t)((uint8_t)((uint8_t)(ccr >> 8) & I2C_CCRH_CCR));
#endif // I2C_FAST

  //Включаем I2C
  I2C->CR1 |= I2C_CR1_PE;

  //Разрешаем подтверждение в конце посылки
  I2C->CR2 |= I2C_CR2_ACK;
}

/**
 * @brief Запись регистра slave-устройства
 */
t_i2c_status i2c_wr_reg(uint8_t address, uint8_t reg_addr, \
                        const uint8_t * data, uint8_t length) {

  //Включаем I2C
  I2C->CR1 |= I2C_CR1_PE;

  //Ждем освобождения шины I2C
  wait_event((I2C->SR3 & I2C_SR3_BUSY), 10);

  //Генерация СТАРТ-посылки
  I2C->CR2 |= I2C_CR2_START;
  //Ждем установки бита SB
  wait_event(!(I2C->SR1 & I2C_SR1_SB), 2);


  //Записываем в регистр данных адрес ведомого устройства
  I2C->DR = address & 0xFE;
  //Ждем подтверждения передачи адреса
  wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);
  //Очистка бита ADDR чтением регистра SR3
  I2C->SR3;


  //Ждем освобождения регистра данных
  wait_event(!(I2C->SR1 & I2C_SR1_TXE), 2);
  //Отправляем адрес регистра
  I2C->DR = reg_addr;

  //Отправка данных
  while(length --) {
    //Ждем освобождения регистра данных
    wait_event(!(I2C->SR1 & I2C_SR1_TXE), 2);
    //Отправляем адрес регистра
    I2C->DR = *data++;
  }

  //Ловим момент, когда DR освободился и данные попали в сдвиговый регистр
  wait_event(!((I2C->SR1 & I2C_SR1_TXE) && (I2C->SR1 & I2C_SR1_BTF)), 2);

  //Посылаем СТОП-посылку
  I2C->CR2 |= I2C_CR2_STOP;
  //Ждем выполнения условия СТОП
  wait_event((I2C->CR2 & I2C_CR2_STOP), 2);
  //Отключаем I2C
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);

  return I2C_SUCCESS;
}

/**
 * @brief Запись байта в slave-устройство
 */
t_i2c_status i2c_wr_byte(uint8_t address, uint8_t byte) {

  //Включаем I2C
  I2C->CR1 |= I2C_CR1_PE;

  //Ждем освобождения шины I2C
  wait_event((I2C->SR3 & I2C_SR3_BUSY), 10);

  //Генерация СТАРТ-посылки
  I2C->CR2 |= I2C_CR2_START;
  //Ждем установки бита SB
  wait_event(!(I2C->SR1 & I2C_SR1_SB), 2);


  //Записываем в регистр данных адрес ведомого устройства
  I2C->DR = address & 0xFE;
  //Ждем подтверждения передачи адреса
  wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);
  //Очистка бита ADDR чтением регистра SR3
  I2C->SR3;


  //Ждем освобождения регистра данных
  wait_event(!(I2C->SR1 & I2C_SR1_TXE), 2);
  //Отправляем байт
  I2C->DR = byte;

  //Ловим момент, когда DR освободился и данные попали в сдвиговый регистр
  wait_event(!((I2C->SR1 & I2C_SR1_TXE) && (I2C->SR1 & I2C_SR1_BTF)), 2);

  //Посылаем СТОП-посылку
  I2C->CR2 |= I2C_CR2_STOP;
  //Ждем выполнения условия СТОП
  wait_event((I2C->CR2 & I2C_CR2_STOP), 2);
  //Отключаем I2C
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);

  return I2C_SUCCESS;
}

/**
 * @brief Чтение регистра slave-устройства
 * @note Start -> Slave Addr -> Reg. addr -> Restart -> Slave Addr <- data ... -> Stop
 */
t_i2c_status i2c_rd_reg(uint8_t address, uint8_t reg_addr, \
                        uint8_t * data, uint8_t length) {

  //Включаем I2C
  I2C->CR1 |= I2C_CR1_PE;

  //Ждем освобождения шины I2C
  wait_event((I2C->SR3 & I2C_SR3_BUSY), 10);

  //Разрешаем подтверждение в конце посылки
  I2C->CR2 |= I2C_CR2_ACK;

  //Генерация СТАРТ-посылки
  I2C->CR2 |= I2C_CR2_START;
  //Ждем установки бита SB
  wait_event(!(I2C->SR1 & I2C_SR1_SB), 2);

  //Записываем в регистр данных адрес ведомого устройства
  I2C->DR = address & 0xFE;
  //Ждем подтверждения передачи адреса
  wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);
  //Очистка бита ADDR чтением регистра SR3
  I2C->SR3;

  //Ждем освобождения регистра данных RD
  wait_event(!(I2C->SR1 & I2C_SR1_TXE), 2);

  //Передаем адрес регистра slave-устройства, который хотим прочитать
  I2C->DR = reg_addr;
  //Ловим момент, когда DR освободился и данные попали в сдвиговый регистр
  wait_event(!((I2C->SR1 & I2C_SR1_TXE) && (I2C->SR1 & I2C_SR1_BTF)), 2);

  //Генерация СТАРТ-посылки (рестарт)
  I2C->CR2 |= I2C_CR2_START;
  //Ждем установки бита SB
  wait_event(!(I2C->SR1 & I2C_SR1_SB), 2);

  //Записываем в регистр данных адрес ведомого устройства и переходим
  //в режим чтения (установкой младшего бита в 1)
  I2C->DR = address | 0x01;

  //Дальше алгоритм зависит от количества принимаемых байт
  //N=1
  if(length == 1){
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= ~I2C_CR2_ACK;
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);

    //Заплатка из Errata
    disableInterrupts();
    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;

    //Устанавлием бит STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Заплатка из Errata
    enableInterrupts();

    //Ждем прихода данных в RD
    wait_event(!(I2C->SR1 & I2C_SR1_RXNE), 2);

    //Читаем принятый байт
    *data = I2C->DR;
  }
  //N=2
  else if(length == 2){
    //Бит который разрешает NACK на следующем принятом байте
    I2C->CR2 |= I2C_CR2_POS;
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);
    //Заплатка из Errata
    disableInterrupts();
    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    //Заплатка из Errata
    enableInterrupts();
    //Ждем момента, когда первый байт окажется в DR,
    //а второй в сдвиговом регистре
    wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);

    //Заплатка из Errata
    disableInterrupts();
    //Устанавлием бит STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Читаем принятые байты
    *data++ = I2C->DR;
    //Заплатка из Errata
    enableInterrupts();
    *data = I2C->DR;
  }
  //N>2
  else if(length > 2){
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);

    //Заплатка из Errata
    disableInterrupts();

    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;

    //Заплатка из Errata
    enableInterrupts();

    I2C_timeout = 10; // 10 ms ???
    while (length-- > 3 && I2C_timeout) {
      //Ожидаем появления данных в DR и сдвиговом регистре
      wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);
      //Читаем принятый байт из DR
      *(data++) = I2C->DR;
    }
    //Время таймаута вышло
    if(!I2C_timeout) {
      //Отключаем I2C
      I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);
      return I2C_TIMEOUT;
    }

    //Осталось принять 3 последних байта
    //Ждем, когда в DR окажется N-2 байт, а в сдвиговом регистре
    //окажется N-1 байт
    wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    //Заплатка из Errata
    disableInterrupts();
    //Читаем N-2 байт из RD, тем самым позволяя принять в сдвиговый
    //регистр байт N, но теперь в конце приема отправится посылка NACK
    *(data++) = I2C->DR;
    //Посылка STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Читаем N-1 байт
    *(data++) = I2C->DR;
    //Заплатка из Errata
    enableInterrupts();
    //Ждем, когда N-й байт попадет в DR из сдвигового регистра
    wait_event(!(I2C->SR1 & I2C_SR1_RXNE), 2);
    //Читаем N байт
    *(data++) = I2C->DR;
  }

  //Ждем отправки СТОП посылки
  wait_event((I2C->CR2 & I2C_CR2_STOP), 2);
  //Сбрасывает бит POS, если вдруг он был установлен
  I2C->CR2 &= ~I2C_CR2_POS;

  //Отключаем I2C
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);
  return I2C_SUCCESS;
}

/**
 * @brief Чтение байт из slave-устройства
 * @note Start -> Slave Addr <- data ... -> Stop
 */
t_i2c_status i2c_rd_bytes(uint8_t address, uint8_t * data, uint8_t length) {

  //Включаем I2C
  I2C->CR1 |= I2C_CR1_PE;

  //Ждем освобождения шины I2C
  wait_event((I2C->SR3 & I2C_SR3_BUSY), 10);

  //Разрешаем подтверждение в конце посылки
  I2C->CR2 |= I2C_CR2_ACK;

  //Генерация СТАРТ-посылки
  I2C->CR2 |= I2C_CR2_START;
  //Ждем установки бита SB
  wait_event(!(I2C->SR1 & I2C_SR1_SB), 2);

  //Записываем в регистр данных адрес ведомого устройства
  I2C->DR = address;

  //Дальше алгоритм зависит от количества принимаемых байт
  //N=1
  if(length == 1){
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= ~I2C_CR2_ACK;
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);

    //Заплатка из Errata
    disableInterrupts();
    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;

    //Устанавлием бит STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Заплатка из Errata
    enableInterrupts();

    //Ждем прихода данных в RD
    wait_event(!(I2C->SR1 & I2C_SR1_RXNE), 2);

    //Читаем принятый байт
    *data = I2C->DR;
  }
  //N=2
  else if(length == 2){
    //Бит который разрешает NACK на следующем принятом байте
    I2C->CR2 |= I2C_CR2_POS;
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);
    //Заплатка из Errata
    disableInterrupts();
    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    //Заплатка из Errata
    enableInterrupts();
    //Ждем момента, когда первый байт окажется в DR,
    //а второй в сдвиговом регистре
    wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);

    //Заплатка из Errata
    disableInterrupts();
    //Устанавлием бит STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Читаем принятые байты
    *data++ = I2C->DR;
    //Заплатка из Errata
    enableInterrupts();
    *data = I2C->DR;
  }
  //N>2
  else if(length > 2){
    //Ждем подтверждения передачи адреса
    wait_event(!(I2C->SR1 & I2C_SR1_ADDR), 2);

    //Заплатка из Errata
    disableInterrupts();

    //Очистка бита ADDR чтением регистра SR3
    I2C->SR3;

    //Заплатка из Errata
    enableInterrupts();

    I2C_timeout = 10; // 10 ms ???
    while (length-- > 3 && I2C_timeout) {
      //Ожидаем появления данных в DR и сдвиговом регистре
      wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);
      //Читаем принятый байт из DR
      *(data++) = I2C->DR;
    }
    //Время таймаута вышло
    if(!I2C_timeout) {
      //Отключаем I2C
      I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);
      return I2C_TIMEOUT;
    }

    //Осталось принять 3 последних байта
    //Ждем, когда в DR окажется N-2 байт, а в сдвиговом регистре
    //окажется N-1 байт
    wait_event(!(I2C->SR1 & I2C_SR1_BTF), 2);
    //Запрещаем подтверждение в конце посылки
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    //Заплатка из Errata
    disableInterrupts();
    //Читаем N-2 байт из RD, тем самым позволяя принять в сдвиговый
    //регистр байт N, но теперь в конце приема отправится посылка NACK
    *(data++) = I2C->DR;
    //Посылка STOP
    I2C->CR2 |= I2C_CR2_STOP;
    //Читаем N-1 байт
    *(data++) = I2C->DR;
    //Заплатка из Errata
    enableInterrupts();
    //Ждем, когда N-й байт попадет в DR из сдвигового регистра
    wait_event(!(I2C->SR1 & I2C_SR1_RXNE), 2);
    //Читаем N байт
    *(data++) = I2C->DR;
  }

  //Ждем отправки СТОП посылки
  wait_event((I2C->CR2 & I2C_CR2_STOP), 2);
  //Сбрасывает бит POS, если вдруг он был установлен
  I2C->CR2 &= ~I2C_CR2_POS;

  //Отключаем I2C
  I2C->CR1 &= (uint8_t)(~I2C_CR1_PE);
  return I2C_SUCCESS;
}
