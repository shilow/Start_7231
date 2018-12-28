//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru
//  Target(s)...: любой микроконтроллер mega
//  Compiler....: IAR 5.11A
//  Description.: Заготовка для событийной системы на таблицах
//  Data........: 30.09.12
//
//***************************************************************************
#include "event-system.h"
#include "board.h"

/* функция-заглушка */
static void EmptyFunc(void);

/* кольцевой буфер */
static volatile es_event_t cycleBuf[SIZE_BUF];
static volatile uint8_t tailBuf = 0;
static volatile uint8_t headBuf = 0;
static volatile uint8_t countBuf = 0;

static volatile es_state_t _State;

extern __IO indicator_t Indicator;
extern const ind_symb_t hex2Sym[16];

typedef struct {
    es_state_t startState;
    es_state_t endState;
    es_event_t startEvent;
    es_event_t endEvent;
    es_state_t nextstate;
    void (*pStateFunc1)(void);
    void (*pStateFunc2)(void);
} table_state_t;

/** таблица состояний */
const table_state_t table[] = {
/* STATE from   STATE to     EVENT from       EVENT to         NEXT STATE   STATE_FUNC1   STATE_FUNC2 */
  {stShowTime,    stShowTime,    evRTCinterrupt, evRTCinterrupt, stNoChange, showTime,    EmptyFunc},

  {stShowSeconds, stShowSeconds, evRTCinterrupt, evRTCinterrupt, stNoChange, showSeconds, EmptyFunc},
  {stShowCntdwn,  stShowCntdwn,  evRTCinterrupt, evRTCinterrupt, stNoChange, showCountdown, EmptyFunc},

  {stShowTime, stShowSeconds, evButtonKPressed, evButtonKPressed, stNoChange, timeKorrect, EmptyFunc},
  //{stShowTime, stShowTime, evButtonOPressed, evButtonOPressed, stFreeze,   showTime,    EmptyFunc},
  {stShowTime, stShowTime, evButtonHPressed, evButtonHPressed, stNoChange, timeIncHH, showTime},
  {stShowTime, stShowTime, evButtonMPressed, evButtonMPressed, stNoChange, timeIncMM, showTime},

  // show calendar
  {stShowTime, stShowTime, evButtonTPressed, evButtonTPressed, stShowWDay, showWDay, EmptyFunc},
  {stShowWDay, stShowWDay, evButtonTPressed, evButtonTPressed, stShowDDMM, showDDMM, EmptyFunc},
  {stShowDDMM, stShowDDMM, evButtonTPressed, evButtonTPressed, stShowYear, showYear, EmptyFunc},
  {stShowYear, stShowYear, evButtonTPressed, evButtonTPressed, stShowSuntime, showSuntime, EmptyFunc},
  {stShowSuntime, stShowSuntime, evButtonTPressed, evButtonTPressed, stShowTime, showTime, EmptyFunc},
  // set calendar
  {stShowWDay, stShowWDay, evButtonHPressed, evButtonHPressed, stNoChange, calWDInc, showWDay},
  {stShowWDay, stShowWDay, evButtonMPressed, evButtonMPressed, stNoChange, calWDDec, showWDay},
  {stShowDDMM, stShowDDMM, evButtonHPressed, evButtonHPressed, stNoChange, calDDInc, showDDMM},
  {stShowDDMM, stShowDDMM, evButtonMPressed, evButtonMPressed, stNoChange, calMMInc, showDDMM},
  {stShowYear, stShowYear, evButtonHPressed, evButtonHPressed, stNoChange, calYearInc, showYear},
  {stShowYear, stShowYear, evButtonMPressed, evButtonMPressed, stNoChange, calYearDec, showYear},
  {stShowSuntime, stShowSuntime, evButtonHPressed, evButtonHPressed, stNoChange, calToggleSuntime, showSuntime},
  {stShowSuntime, stShowSuntime, evButtonMPressed, evButtonMPressed, stNoChange, calToggleSuntime, showSuntime},

  {stShowCntdwn, stShowCntdwn, evButtonCPressed, evButtonCPressed, stNoChange, toggleCountdown, showCountdown},
  {stShowCntdwn, stShowCntdwn, evButtonHPressed, evButtonHPressed, stNoChange, cntdwnIncHH, showCountdown},
  {stShowCntdwn, stShowCntdwn, evButtonMPressed, evButtonMPressed, stNoChange, cntdwnIncMM, showCountdown},

  {stShowAlarm1, stShowAlarm1, evButtonA1Pressed, evButtonA1Pressed, stNoChange, toggleAC1, showAlarm1},
  {stShowAlarm1, stShowAlarm1, evButtonHPressed,  evButtonHPressed,  stNoChange, alarm1IncHH, showAlarm1},
  {stShowAlarm1, stShowAlarm1, evButtonMPressed,  evButtonMPressed,  stNoChange, alarm1IncMM, showAlarm1},

  {stShowAlarm2, stShowAlarm2, evButtonA2Pressed, evButtonA2Pressed, stNoChange, toggleAC2, showAlarm2},
  {stShowAlarm2, stShowAlarm2, evButtonHPressed,  evButtonHPressed,  stNoChange, alarm2IncHH, showAlarm2},
  {stShowAlarm2, stShowAlarm2, evButtonMPressed,  evButtonMPressed,  stNoChange, alarm2IncMM, showAlarm2},

  {stShowSeconds, stFreeze, evButtonTPressed, evButtonTPressed, stShowTime, showTime, EmptyFunc},
  {stNoChange, stFreeze, evButtonSPressed, evButtonSPressed, stShowSeconds, showSeconds, EmptyFunc},
  {stNoChange, stFreeze, evButtonCPressed, evButtonCPressed, stShowCntdwn, showCountdown, EmptyFunc},
  {stNoChange, stFreeze, evButtonA1Pressed, evButtonA1Pressed, stShowAlarm1, showAlarm1, EmptyFunc},
  {stNoChange, stFreeze, evButtonA2Pressed, evButtonA2Pressed, stShowAlarm2, showAlarm2, EmptyFunc},

  //{stFreeze,   stFreeze,   evButtonOPressed, evButtonOPressed, stNoChange, showTime,    EmptyFunc},
  {stNoChange, stLastState, evButtonOPressed, evButtonOPressed, stFreeze,   showLightLevel, EmptyFunc},
  {stFreeze,   stFreeze,    evRTCinterrupt,   evRTCinterrupt,   stNoChange, showLightLevel, EmptyFunc},

  /* обязательная пустая строка таблицы */
  {stNoChange, stNoChange, eventNull, eventNull, stNoChange, EmptyFunc, EmptyFunc}
};

/**
  * @brief  Take event.
  * @param  None
  * @retval Event
  */
es_event_t ES_GetEvent(void)
{
  es_event_t event;
  if (countBuf > 0){
    event = cycleBuf[headBuf];
    countBuf--;
    headBuf = (headBuf + 1) & (SIZE_BUF - 1);
    return event;
  }
  return eventNull;
}

/**
  * @brief  Place event.
  * @param  Event
  * @retval None
  */
void ES_PlaceEvent(es_event_t event)
{
  if (countBuf < SIZE_BUF){
      cycleBuf[tailBuf] = event;
      tailBuf = (tailBuf + 1) & (SIZE_BUF - 1);
      countBuf++;
  /* сигнализация переполнения буфера событий */
  } else {
      Indicator.Value[0] = symMinus;
      Indicator.Value[1] = symE;
      Indicator.Value[2] = symS;
      Indicator.Value[3] = symMinus;
      while(1);
  }
}

/**
  * @brief  Initialize event system.
  * @param  Start Event
  * @retval None
  */
void ES_Init(es_state_t init_state)
{
  tailBuf = 0;
  headBuf = 0;
  countBuf = 0;
  _State = init_state;
}

/**
  * @brief  Fake function.
  * @param  None
  * @retval None
  */
static void EmptyFunc(void)
{
}

/**
  * @brief  Dispatcher of event system.
  * @param  Event
  * @retval None
  */
void ES_Dispatch(es_event_t event)
{
    void (*pStateFunc1)(void);
    void (*pStateFunc2)(void);
    uint8_t i;

    pStateFunc1 = NULL;
    pStateFunc2 = NULL;

    //определяем следующее состояние
    for (i=0; table[i].startEvent || table[i].endEvent; i++)
    {
        //если текущее состояние попадает в диапазон
        if ((_State >= table[i].startState)&&(_State <= table[i].endState)){
          //если поступившее событие попадает в диапазон
          if((event >= table[i].startEvent) && (event <= table[i].endEvent)){
            //меняем состояние если требуется
            if (table[i].nextstate != stNoChange)
              _State = table[i].nextstate;
            pStateFunc1 = table[i].pStateFunc1;
            pStateFunc2 = table[i].pStateFunc2;

            break;
          }
        }
    }
    if (pStateFunc1) pStateFunc1();
    if (pStateFunc2) pStateFunc2();
}

/**
  * @brief  Return current state code.
  * @param  None
  * @retval Event
  */
es_state_t ES_GetState(void) {
  return _State;
}

/**
  * @brief  Set current state to given code .
  * @param  Event
  * @retval None
  */
void ES_SetState(es_state_t new_state) {
  _State = new_state;
}
