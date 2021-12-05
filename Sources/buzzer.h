#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "board.h"

typedef enum
{
  BUZZER_RINGING,
  BUZZER_CLOSED,
  
} Buzzer_State_t;

extern volatile unsigned int Buzzer_ToggleTimeout;
extern bool Buzzer_RingEnableFlag;
extern bool Buzzer_RingingFlag;

void Buzzer_Init(void);
void Buzzer_Open(void);
void Buzzer_Close(void);
void Buzzer_Toggle(void);
Buzzer_State_t Buzzer_GetState(void);

#endif
