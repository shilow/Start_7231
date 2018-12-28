#include "rtos.h"
#include "stm8s_it.h"
#include "board.h"

/* Private define ------------------------------------------------------------*/
#define TIM4_PERIOD  (uint8_t)124

/**
 * Переменные модуля
 */
static __IO task TaskArray[MAX_TASKS];	// очередь задач
static __IO uint8_t arrayTail;         // "хвост" очереди
static __IO uint16_t TimingDelay;
__IO uint8_t I2C_timeout;
extern __IO indicator_t Indicator;
extern const ind_symb_t hex2Sym[16];

/**
 * @brief Инициализация РТОС, время тика - 1 мс
 */
inline void RTOS_Init(void)
{
  /*
  TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
     clock used is 16 MHz / 128 = 125 000 Hz
   - With 125 000 Hz we can generate time base:
     max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
     min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
   - In this example we need to generate a time base equal to 1 ms
     so TIM4_PERIOD = (0.001 * 125000 - 1) = 124
   */

  /* Time base configuration. Set the Prescaler value */
  TIM4->PSCR = (uint8_t)(TIM4_PRESCALER_128);
  /* Set the Autoreload value */
  TIM4->ARR = (uint8_t)(TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4->SR1 = (uint8_t)(~TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4->IER |= (uint8_t)TIM4_IT_UPDATE;

  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4->CR1 |= (uint8_t)TIM4_CR1_CEN;

  /* "хвост" в 0 */
  arrayTail = 0;
}

/**
 * @brief Добавление задачи в список
 */
void RTOS_SetTask (void (*taskFunc)(void), uint16_t taskDelay, uint16_t taskPeriod)
{
   uint8_t i;

   if(!taskFunc) return;

   for(i = 0; i < arrayTail; i++)                     // поиск задачи в текущем списке
   {
      if(TaskArray[i].pFunc == taskFunc)              // если нашли, то обновляем переменные
      {
         DISABLE_INTERRUPT;

         TaskArray[i].delay  = taskDelay;
         TaskArray[i].period = taskPeriod;
         TaskArray[i].run    = 0;

         ENABLE_INTERRUPT;
         return;                                      // обновив, выходим
      }
   }

   if (arrayTail < MAX_TASKS)                         // если такой задачи в списке нет
   {                                                  // и есть место,то добавляем
      DISABLE_INTERRUPT;

      TaskArray[arrayTail].pFunc  = taskFunc;
      TaskArray[arrayTail].delay  = taskDelay;
      TaskArray[arrayTail].period = taskPeriod;
      TaskArray[arrayTail].run    = 0;

      arrayTail++;                                    // увеличиваем "хвост"

      ENABLE_INTERRUPT;
    /* сигнализация переполнения буфера задач */
    } else {
      Indicator.Value[0] = symMinus;
      Indicator.Value[1] = sym0;
      Indicator.Value[2] = symS;
      Indicator.Value[3] = symMinus;
      while(1);
    }
}

/**
 * @brief Удаление задачи из списка
 */
void RTOS_DeleteTask (void (*taskFunc)(void))
{
   uint8_t i;

   for (i=0; i<arrayTail; i++)                        // проходим по списку задач
   {
      if(TaskArray[i].pFunc == taskFunc)              // если задача в списке найдена
      {

         DISABLE_INTERRUPT;
         if(i != (arrayTail - 1))                     // переносим последнюю задачу
         {                                            // на место удаляемой
            TaskArray[i] = TaskArray[arrayTail - 1];
         }
         arrayTail--;                                 // уменьшаем указатель "хвоста"
         ENABLE_INTERRUPT;
         return;
      }
   }
}

/**
 * @brief Диспетчер РТОС, вызывается в main
 */
void RTOS_DispatchTask(void)
{
   uint8_t i;
   void (*function) (void);

   for (i=0; i<arrayTail; i++)                        // проходим по списку задач
   {
      if (TaskArray[i].run == 1)                      // если флаг на выполнение взведен,
      {                                               // запоминаем задачу, т.к. во
         function = TaskArray[i].pFunc;               // время выполнения может
                                                      // измениться индекс
         if(TaskArray[i].period == 0)
         {                                            // если период равен 0
            RTOS_DeleteTask(TaskArray[i].pFunc);      // удаляем задачу из списка,
         } else {
            TaskArray[i].run = 0;                     // иначе снимаем флаг запуска
            if(!TaskArray[i].delay)                   // если задача не изменила задержку
            {                                         // задаем ее
               TaskArray[i].delay = TaskArray[i].period-1;
            }                                         // задача для себя может сделать паузу
         }
         (*function)();                               // выполняем задачу
      }
   }
}

/**
 * Таймерная служба РТОС (прерывание аппаратного таймера) - в обработчике прерывания от таймера
 */

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint16_t nTime)
{
  TimingDelay = nTime;
  while (TimingDelay != 0) {
      if (TimingDelay > 2) {
        RTOS_DispatchTask();
      }
      // здесь можно спать и ждать прерывание
      wfi();
  }
}

/**
  * @brief TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler,23)
{
   /* Cleat Interrupt Pending bit */
   TIM4->SR1 = (uint8_t)(~(uint8_t)TIM4_IT_UPDATE);

   /* I2C timeout */
   if (I2C_timeout > 0) {
    I2C_timeout --;
   }

   /* TimingDelay_Decrement() */
   if (TimingDelay > 0) {
      TimingDelay --;
   }

   /* RTOS_Timer() */
   uint8_t i;
   for (i=0; i<arrayTail; i++) {       // проходим по списку задач
      if  (TaskArray[i].delay == 0) {  // если время до выполнения истекло
         TaskArray[i].run = 1;         // взводим флаг запуска,
      } else {
         TaskArray[i].delay--;         // иначе уменьшаем время
      }
   }

}
