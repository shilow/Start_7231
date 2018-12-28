//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru
//  Target(s)...: любой микроконтроллер mega
//  Compiler....: IAR 5.11A
//  Description.: Заготовка для событийной системы на таблицах
//  Data........: 30.09.12
//
//***************************************************************************

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once
#ifndef EVENT_SYSTEM_H
#define EVENT_SYSTEM_H

#include "stm8s.h"
#include "list_event.h"

/* вместимость буфера очереди событий */
#define SIZE_BUF 16

void ES_Init(es_state_t init_state);    //инициализация
es_state_t ES_GetState(void);           //взять код состояния
void ES_SetState(es_state_t new_state); //установить код состояния
es_event_t ES_GetEvent(void);           //взять код события
void ES_PlaceEvent(es_event_t event);   //разместить событие
void ES_Dispatch(es_event_t event);     //вызов диспетчера

/**
 * ES function prototypes
 */
void showTime(void);
void showSeconds(void);
void showCountdown(void);
void showAlarm1(void);
void showAlarm2(void);
void timeIncHH(void);
void timeIncMM(void);
void timeKorrect(void);
void cntdwnIncHH(void);
void cntdwnIncMM(void);
void alarm1IncHH(void);
void alarm1IncMM(void);
void alarm2IncHH(void);
void alarm2IncMM(void);
void toggleCountdown(void);
void toggleAC1(void);
void toggleAC2(void);
void finishCountdown(void);
void showLightLevel(void);
void showWDay(void);
void showDDMM(void);
void showYear(void);
void showSuntime(void);
void calWDInc(void);
void calWDDec(void);
void calDDInc(void);
void calMMInc(void);
void calYearInc(void);
void calYearDec(void);
void calToggleSuntime(void);

#endif // EVENT_SYSTEM_H
