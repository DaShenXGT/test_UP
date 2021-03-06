#include <hidef.h>	
#include <mc9s12hy64.h>
#include "buzzer.h"

//????????Ƶ??
volatile unsigned int Buzzer_ToggleTimeout;
//?????򿪱?־
bool Buzzer_RingEnableFlag = false;
bool Buzzer_RingingFlag = false;

/**********************************************************************
@ Func:  ????????ʼ??
@ Brief: ?????????ƶ˿?ΪPP4????????????Ϊ??Դ
**********************************************************************/
void Buzzer_Init()
{
  DDRP |= (1 << 4);
  PTP &= ~(1 << 4);
}

/**********************************************************************
@ Func:  ??????????
**********************************************************************/
void Buzzer_Open()
{
  PTP |= (1 << 4);
  Buzzer_RingingFlag = true;
}

/**********************************************************************
@ Func:  ???????ر?
**********************************************************************/
void Buzzer_Close()
{
  PTP &= ~(1 << 4);
  Buzzer_RingingFlag = false;
}

/**********************************************************************
@ Func:  ???????????򿪡??ر?
**********************************************************************/
void Buzzer_Toggle()
{
  if(Buzzer_RingingFlag)
  {
    Buzzer_Close();
  }
  else
  {
    Buzzer_Open();
  }
}

/**********************************************************************
@ Func:  ??ȡ??????״̬
**********************************************************************/
Buzzer_State_t Buzzer_GetState()
{
  if(PTP & (1 << 4))
  {
    return BUZZER_RINGING;
  }
  
  return BUZZER_CLOSED;
}