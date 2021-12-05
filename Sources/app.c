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
  Cofigure_Cpu_Clk();       //系统时钟、看门狗初始化
  EnableInterrupts;
  
  //待系统稳定之后再置高DC-DC使能引脚，防止关电锁时因电锁信号关断延迟导致单片机再次复位
  sysPowerDisable();

	TIM0_Init();              //定时器0初始化
	TIM1_Init();	            //定时器1初始化
	
	
	DelayMS(100);             //非常重要！！！在使能DC-DC控制引脚前需要足够延时，防止关电锁后因电平缓降造成仪表再次自检，延时用到了定时器
	                          //中断，所以需要放到定时器初始化后面
//增加EEPROM初始化
  EEPROM_IIC_Init();	
  Config_Ditigal_Io();      //开关量输入IO初始化（74HC165控制IO、电锁开关输入IO）
	
  LCD_GPIOInit();           //LCD控制IO初始化
	LCD_Config();
	
  LED_IOInit();             //LED驱动控制IO初始化
  
	Config_Motor_io();        //电机驱动初始化
	
  Buzzer_Init();            //蜂鸣器控制IO初始化
  
  Init_Atd_Paramter();      //AD转换初始化

	FLASH_Init();
	
	Mscan_Initial();          //CAN初始化
	Can_Par_Init(); 
	#ifdef OUT_WDT
	En_out_WatchDog();        //使能外部看门狗  
	#endif
	//DelayMS(100);             //非常重要！！！在使能DC-DC控制引脚前需要足够延时，防止关电锁后因电平缓降造成仪表再次自检
	
  //使能供电引脚
  sysPowerEnable();  
		  
  //系统自检
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

		//因自检用时3秒，此处不必再判断延时——if(!can_task.can_star_time)
		CAN_TaskProcess();
		
		//系统升级命令检测
		//Update_CmdCheck();     
	}
}

/*****************************************************************************************
@ Func: 系统电源打开
*****************************************************************************************/
static void sysPowerEnable(void)
{
  DDRP |= (1 << 2);
  PTP |= (1 << 2);
}

/*****************************************************************************************
@ Func: 系统电源关闭
*****************************************************************************************/
static void sysPowerDisable(void)
{
  DDRP |= (1 << 2);
  PTP &= ~(1 << 2);
}

