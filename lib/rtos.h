#pragma once
#ifndef __RTOS_H
#define __RTOS_H

/******************************************************************************************
 * За основу взят планировщик задач с сайта ChipEnable.ru                                 *
 * http://chipenable.ru/index.php/programming-avr/item/110-planirovschik.html             *
 *                                                                                        *
 * Доработал Шибанов Владимир aka KontAr                                                  *
 * Дата: 26.03.2014                                                                       *
 *                                                                                        *
 * Изменения:                                                                             *
 * - добавлен однократный вызов задачи                                                    *
 * - добавлено удаление задачи по имени                                                   *
 * - при повторном добавлении задачи обновляются ее переменные                            *
 * - добавлен указатель на "хвост" списка                                                 *
 * - функции РТОС скорректированы с учетом "хвоста"                                       *
 *                                                                                        *
 * 18.05.2015, Shilov V.N. <shilow@ukr.net>                                               *
 * - скрестил с timing_delay от ST                                                        *
 * - перенёс сюда обработчик прерываний для уменьшения оверхеда                           *
 *                                                                                        *
 ******************************************************************************************/

#include "stm8s.h"

// Количество задач
#define MAX_TASKS	9

#define  ENABLE_INTERRUPT enableInterrupts()
#define DISABLE_INTERRUPT disableInterrupts()

/**
 * Структура задачи
 */
typedef struct task
{
   void (*pFunc) (void); // указатель на функцию
   uint16_t delay;       // задержка перед первым запуском задачи
   uint16_t period;      // период запуска задачи
   uint8_t run;          // флаг готовности задачи к запуску
} task;

/**
 * Переменные
 */
#define USE_RTOS  1

/**
 * Прототипы фукнций
 */
void RTOS_Init (void);
void RTOS_SetTask (void (*taskFunc)(void), uint16_t taskDelay, uint16_t taskPeriod);
void RTOS_DeleteTask (void (*taskFunc)(void));
void RTOS_DispatchTask (void);

void Delay(__IO uint16_t nTime);

#endif /* __RTOS_H */
