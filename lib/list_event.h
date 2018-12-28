#pragma once
#ifndef EVENT_LIST_H
#define EVENT_LIST_H

/* коды событий */
typedef enum {
  eventNull = 0x00,
  evRTCinterrupt,
  evButtonHPressed,
  evButtonMPressed,
  evButtonKPressed,
  evButtonCPressed,
  evButtonA1Pressed,
  evButtonA2Pressed,
  evButtonTPressed,
  evButtonSPressed,
  evButtonOPressed,
  evMenuWDT
} es_event_t;

/* коды состояний */
typedef enum {
  stNoChange = 0x00,
  stShowTime,
  stShowSeconds,
  stShowCntdwn,
  stShowAlarm1,
  stShowAlarm2,
  stFreeze,
  stShowWDay,
  stShowDDMM,
  stShowYear,
  stShowSuntime,
  // end
  stLastState
} es_state_t;

#ifndef NULL
  #define NULL ((void*)0)
#endif

#endif //EVENT_LIST_H
