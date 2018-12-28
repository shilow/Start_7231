#pragma once
#ifndef I2C_H
#define I2C_H

#include "stm8s.h"

#define I2C_FAST        1
#define F_MASTER_MHZ    16UL
#define F_MASTER_HZ     16000000UL
#ifdef I2C_FAST
//400 кГц
#define F_I2C_HZ        400000UL
#else
//100 кГц
#define F_I2C_HZ        100000UL
#endif // I2C_FAST

//Результат выполнения операции с i2c
typedef enum {
    I2C_SUCCESS = 0,
    I2C_TIMEOUT,
    I2C_ERROR
} t_i2c_status;

// Инициализация I2C интерфейса
extern void i2c_master_init(void);

// Запись регистра slave-устройства
extern t_i2c_status  i2c_wr_reg(uint8_t address, uint8_t reg_addr, \
                                const uint8_t * data, uint8_t length);

extern t_i2c_status  i2c_wr_byte(uint8_t address, uint8_t byte);

// Чтение регистра slave-устройства
extern t_i2c_status  i2c_rd_reg(uint8_t address, uint8_t reg_addr, \
                                uint8_t * data, uint8_t length);

extern t_i2c_status  i2c_rd_bytes(uint8_t address, uint8_t * data, uint8_t length);
#endif // I2C_H
