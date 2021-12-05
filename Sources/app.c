           #include <hidef.h>	
#include <mc9s12hy64.h>
#include "board.h"
#include "app.h"
#include "cpu.h"
#include "time.h"
#include "atd.h"
#include "digital.h"
#include "display.h"
#include "vehicle.h"
#include "flash.h"
#include "smc_motor.h"
#include "led.h"
#include "tim1_capture.h"
#include "uart.h"
#include "CAN.h"
#include "buzzer.h"
#include "lcd.h"
#include "update.h"
#include "eeprom.h"

extern Elock_State_t Elock_State;
extern unsigned long CAN_FaultBitMap;

static void sysPowerEnable(void);
static void sysPowerDisable(void);

/*****************************************************************************************
@ Func: Main
*****************************************************************************************/
void main(void)
{
	DisableInterrupts;					
  Cofigure_Cpu_Clk();       //ϵͳʱ�ӡ����Ź���ʼ��
  EnableInterrupts;
  
  //��ϵͳ�ȶ�֮�����ø�DC-DCʹ�����ţ���ֹ�ص���ʱ������źŹض��ӳٵ��µ�Ƭ���ٴθ�λ
  sysPowerDisable();

	TIM0_Init();              //��ʱ��0��ʼ��
	TIM1_Init();	            //��ʱ��1��ʼ��
	
	
	DelayMS(100);             //�ǳ���Ҫ��������ʹ��DC-DC��������ǰ��Ҫ�㹻��ʱ����ֹ�ص��������ƽ��������Ǳ��ٴ��Լ죬��ʱ�õ��˶�ʱ��
	                          //�жϣ�������Ҫ�ŵ���ʱ����ʼ������
//����EEPROM��ʼ��
  EEPROM_IIC_Init();	
  Config_Ditigal_Io();      //����������IO��ʼ����74HC165����IO��������������IO��
	
  LCD_GPIOInit();           //LCD����IO��ʼ��
	LCD_Config();
	
  LED_IOInit();             //LED��������IO��ʼ��
  
	Config_Motor_io();        //���������ʼ��
	
  Buzzer_Init();            //����������IO��ʼ��
  
  Init_Atd_Paramter();      //ADת����ʼ��

	FLASH_Init();
	
	Mscan_Initial();          //CAN��ʼ��
	Can_Par_Init(); 
	#ifdef OUT_WDT
	En_out_WatchDog();        //ʹ���ⲿ���Ź�  
	#endif
	//DelayMS(100);             //�ǳ���Ҫ��������ʹ��DC-DC��������ǰ��Ҫ�㹻��ʱ����ֹ�ص��������ƽ��������Ǳ��ٴ��Լ�
	
  //ʹ�ܹ�������
  sysPowerEnable();  
		  
  //ϵͳ�Լ�
  System_SelfCheckWhenElockOn();
  
  tim1_ioc1_6_input_capture_init();

#ifdef WDT_ON
  __RESET_WATCHDOG();
#endif

#ifdef OUT_WDT
  Feed_WatchDog();
#endif

  for (;;)
  {
#ifdef WDT_ON
		__RESET_WATCHDOG();
#endif

#ifdef OUT_WDT
		Feed_WatchDog();
#endif

		timeEventsProc();
		
		systemTasksProc();

		//���Լ���ʱ3�룬�˴��������ж���ʱ����if(!can_task.can_star_time)
		CAN_TaskProcess();
		
		//ϵͳ����������
		//Update_CmdCheck();     
	}
}

/*****************************************************************************************
@ Func: ϵͳ��Դ��
*****************************************************************************************/
static void sysPowerEnable(void)
{
  DDRP |= (1 << 2);
  PTP |= (1 << 2);
}

/*****************************************************************************************
@ Func: ϵͳ��Դ�ر�
*****************************************************************************************/
static void sysPowerDisable(void)
{
  DDRP |= (1 << 2);
  PTP &= ~(1 << 2);
}

