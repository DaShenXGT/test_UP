#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "board.h"
#include "vehicle.h"
#include "digital.h"
#include "uart.h"
#include "lcd.h"
#include "atd.h"

#ifdef	CAN_COMM
#include "can.h"
#endif

#include "time.h"

unsigned int	SysTimer;

unsigned long	jiffies;

unsigned int	time_500ms;
unsigned int	time_1s; 
unsigned char	time_status;
unsigned long	digital_time;

//AD转换定时器
unsigned long	atd_time;

//小时计
unsigned int	working_time_h;
unsigned int	working_time_l; 
unsigned long	working_hours;
unsigned long	working_time;

unsigned char	time_6m;
unsigned char	time_1m;
vehicle_time	munich_time;


extern	queue_list		systemTask;
extern	Uint8_t			atd_delay_time;
extern	engineSpeedParams_t	engineSpeedParams;
extern	vehicle_warning	systemState;
extern Uint32_t CAN_FaultBitMap;
extern unsigned int CAN_EngineSpeedRecvTimeout; 
extern unsigned int CAN_CoolingLiquidTempRecvTimeout;
extern unsigned int CAN_UreaLevelRecvTimeout;
extern Elock_State_t Elock_State;
unsigned char   slow_flicher_flag=0;
/************************************************
*name: 		Init_Time_Params
*************************************************/
void Init_Time_Params(void)
{
	time_500ms = 0;
	time_1s = 0;
	time_status = 0;
	digital_time = 0;
	atd_time = 0;

	munich_time.enging_on_time = 1000;
  slow_flicher_flag=0;
}

/************************************************************************
@ Func: 初始化TIM0
@ Brief: CH4用于提供系统时基 CH0用于调度Motor0，CH2用于调度Motor1，
         CH5用于调度Motor4
************************************************************************/
void TIM0_Init(void)
{	
	TIM0_TIOS |= 0x10;  			    // set IOC0_4 to be output compare ..... use for 1ms system tick.
  TIM0_TIE_C4I = 1;   			    // enable IOC0_4 interrupt
	TIM0_TSCR2_PR = 3;            // BUSCLK / 8
	TIM0_OCPD |= (1 << 4);
	TIM0_TC4 = TIM0_TCNT + _1ms;  // SET INTERRUPT RATE for General Purpose System Tic
	TIM0_TSCR1 |= 1 << 5;
	
  //Set TIM0 Ch0 Ch2 Ch5 output compare
  TIM0_TIOS |= ((1 << 0) | (1 << 2) | (1 << 5));
	//Set TIM0 Ch0 Ch2 Ch5 No output compare action on the timer output signal
	TIM0_TCTL2 &= ~((3 << 0) + (3 << 4));	               
	TIM0_TCTL1 &= ~(3 << 2);
	//TIM0 CH0 CH2 CH5 Capture disabled
	TIM0_TCTL4 &= ~((3 << 0) + (3 << 4));
	TIM0_TCTL3 &= ~(3 << 2);
	//Timer Enable. Timer Stops While in Freeze Mode
	TIM0_TSCR1 |= (1 << 7) + (1 << 5);
	//Disables the timer channel port. 
	TIM0_OCPD |= (1 << 0) + (1 << 2) + (1 << 5);
	//Set Timer Input Capture/Output Compare Register for the first time
	TIM0_TC0 = TIM0_TCNT + 10000;
	TIM0_TC2 = TIM0_TCNT + 10000;
  TIM0_TC5 = TIM0_TCNT + 10000;
  //Enable CH0 CH2 CH5 Interrupt
  TIM0_TIE |= (1 << 0) + (1 << 2) + (1 << 5);   
}

/************************************************************************
@ Func:  TIM1初始化
@ Brief: CH0用于调度Motor2，CH2用于调度Motor3，CH6用于车速脉冲捕获
************************************************************************/
void TIM1_Init()
{
  //Timer Stops While in Freeze Mode
  TIM1_TSCR1 |= 1 << 5;
  
  //set IOC1_0 and IOC1_2 to be output compares.
  TIM1_TIOS  = 0x05;
  //Set TIM1 Ch0 Ch2 No output compare action on the timer output signal
  TIM1_TCTL2 &= ~((3 << 0) + (3 << 4));
  //TIM1 CH0 CH2 Capture disabled
  TIM1_TCTL4 &= ~((3 << 0) + (3 << 4));
  //Timer Enable. Timer Stops While in Freeze Mode
  TIM1_TSCR1 = 0xa0;  
  //Bus Clock / 4 = 4MHz
  TIM1_TSCR2 = 3;
  //Disables the timer channel port.
  TIM1_OCPD |= (1 << 0) + (1 << 2);
	//Set Timer Input Capture/Output Compare Register for the first time
  TIM1_TC0 = TIM1_TCNT + 10000;  
  TIM1_TC2 = TIM1_TCNT + 10000;  
  //Enable TIM1 CH0 CH2 Interrupt
  TIM1_TIE_C0I = 1;   
  TIM1_TIE_C2I = 1;
  
  //TIM1_CH6用于脉冲信号捕获
	TIM1_TIOS_IOS6 &= 0xBF;			//select ch6 is input capture
	TIM1_PACTL = 0x20;
	TIM1_TCTL3 = 0x10;					//rising deges only
	TIM1_TIE_C6I = 1;                            
	
	//溢出中断使能
	TIM1_TSCR2 |= (1 << 7);  
}

/**********************************************************************************************
@ Func:  毫秒延时
@ Brief: delay <-->延时毫秒数，最大为65535
**********************************************************************************************/
void DelayMS(Uint16_t delay)
{
	SysTimer = 0;
	while(SysTimer < delay)
	  ;
}  

/**********************************************************************************************
@ Func:  定时事件超时检测
@ Brief；当到达预定时长，置相应事件标志
**********************************************************************************************/
void timeEventsProc(void)
{
  Uint8_t	time_mi = 0;

  //------------500MS定时-------------//
  if(_pasti(time_500ms) >= VALUE_500MS)
  {
    time_500ms = jiffies;
    
    //蜂鸣器、LED闪烁控制    
    if(systemState.b.flicher_bit)
    {	
        systemState.b.flicher_bit = 0;
        
        if(systemState.b.buzz_bit)
        {	
          Buzzer_Close();
        }
    }
    else
    {	
        systemState.b.flicher_bit = 1;
        
        if(systemState.b.buzz_bit)
        {
          Buzzer_Open();
        }
    }
           
    //车速处理
    calculating_speed();
    
    //DTC故障切换计时
    if(CAN_FaultDisplayCtrl.refreshTimeout)
    {
      --CAN_FaultDisplayCtrl.refreshTimeout;
    }
    
    systemTask.b.Led_active = TRUE;
  }
  
  //---------------1秒定时--------------//
  if(time_status & TIME1S_VALID)
  {
    time_status &= ~TIME1S_VALID;
    //增加慢闪标志  by_徐光同 2021/4/26
    if (slow_flicher_flag == 0)
    {
      slow_flicher_flag = 1;
    }
    else
    {
      slow_flicher_flag --;
    }
    //驻车制动检测延时
    if(park_delay_time != 0)
    {
      park_delay_time--;
    }
    
    //CAN发送机工作时间请求超时计数器
    if(CAN_RecvEngineHourReqAckTimeout < 65535)
    {
      ++CAN_RecvEngineHourReqAckTimeout;
    }

    //CAN GPS报警超时计数器
    if(can_data.warning_time != 0)
    { 
      can_data.warning_time--;
    }
    
    if(can_data.durationFor51DaysWarning)
    {
      --can_data.durationFor51DaysWarning;
    }
    
    if(can_data.durationFor58DaysWarning)
    {
      --can_data.durationFor58DaysWarning;
    }
    
    //递减CAN故障存在时间
    for(time_mi = 0; time_mi < CAN_FAULT_RECORD_MAX_NUM; time_mi++)
    {
      if(CAN_FaultRecordBuffer[time_mi].error_time != 0)
      {
          CAN_FaultRecordBuffer[time_mi].error_time--;
      }
    }
  }
    
  //AD转换定时
  if(_pasti(atd_time) >= ATD_TIME)
  {
    systemTask.b.Atd_active = TRUE;
    atd_time = jiffies;
  }

  //数字量处理定时
  if(_pasti(digital_time) >= DITIGAL_TIME)
  {
    systemTask.b.Ditigal_active = TRUE;
    digital_time = jiffies;
  }
  
  //LCD任务定时
  if(_pasti(LCD_DisplayCtrl.taskPeriod) >= LCD_TASK_PERIOD)
  {
    LCD_DisplayCtrl.taskPeriod = jiffies;
    systemTask.b.Lcd_active = true;
  }
  
  //LCD小时计显示，沙漏闪烁频率为0.4Hz
  if(_pasti(LCD_HourglassCtrl.blinkTimeout) >= LCD_HOURGLASS_BLINK_PERIOD)
  {
    LCD_HourglassCtrl.blinkTimeout = jiffies;
    LCD_HourglassCtrl.refreshFlag = true;
    
    if(LCD_GLASSHOUR_SHOW == LCD_HourglassCtrl.displayState)
    {
      LCD_HourglassCtrl.displayState = LCD_GLASSHOUR_HIDDEN;
    }
    else
    {
      LCD_HourglassCtrl.displayState = LCD_GLASSHOUR_SHOW;
    }
  }
}

#pragma CODE_SEG NON_BANKED
/************************************************************************
@ Func:  TIM0通道4中断服务函数
************************************************************************/
__interrupt VectorNumber_Vtim0ch4 void _isr_SystemTick_Interrupt (void)
{ 
  TIM0_TFLG1 = 0x10;	                      // clear interrupt flag 
  TIM0_TC4 = TIM0_TC4 + _1ms;

  jiffies++;                                // create main system timer 

  //用于延时函数
  SysTimer++;
	
	//发动机启停时间计数器，该变量不为0表示发动机正在启停
	if(munich_time.enging_on_time != 0)
	{
		munich_time.enging_on_time--;
	}
		
	//ATD转换超时计数器
	atd_delay_time++;
	
#ifdef	CAN_COMM
  //CAN经过500MS后开始工作
  if(can_task.can_star_time)
  {
    can_task.can_star_time--;
  }

	if(!can_task.can_star_time)
	{
		if(can_task.can_time0 != 0)
			can_task.can_time0--;
		
		if(can_task.can_time1 != 0)
			can_task.can_time1--;
		
		if(can_task.can_time2 != 0)
		{
		  --can_task.can_time2;
		}
		
		if(can_task.can_time3 != 0)
		{
		  --can_task.can_time3;
		}
		
		if(can_task.can_time4 != 0)
		{
		  --can_task.can_time4;
		}
				
		if(can_task.can_wait_time != 0)
		{
			can_task.can_wait_time--;
		}
	}
#endif

	//当递减到0时请求发动机时间
	if(CAN_ReqWorkingTimeCnt > 0)
	{
	  --CAN_ReqWorkingTimeCnt;
	}
	
	//当递减到0时请求发动机总油耗
	if(CAN_ReqOilCnt > 0)
	{
	  --CAN_ReqOilCnt;
	}

  //1秒定时
  if(time_1s > 0)
  {
  	time_1s--;
  }
	else if(0 == time_1s)
	{	
		time_1s = VALUE_1000MS;
		time_status |= TIME1S_VALID;
	} 
	
	//变速油压低超时计数器
	if(Vehicle_TransOilPressureLowFlag)
	{
	  if(Vehicle_TransOilPressureLowCnt < 65535)
	  {
	    ++Vehicle_TransOilPressureLowCnt;
	  } 
	}
	
	if(Vehicle_TransOilPressureHighFlag)
	{
	  if(Vehicle_TransOilPressureHighCnt < 65535)
	  {
  	  ++Vehicle_TransOilPressureHighCnt;
	  }
	}
	
	//发动机转速接收超时计数器
	if(CAN_EngineSpeedRecvTimeout && (ELOCK_ON == Elock_State))
	{
	  --CAN_EngineSpeedRecvTimeout;
	}
	
	//冷却液温接收超时计数器
	if(CAN_CoolingLiquidTempRecvTimeout && (ELOCK_ON == Elock_State))
	{
	  --CAN_CoolingLiquidTempRecvTimeout;
	}

  //尿素液位接收超时计数器	
	if(CAN_UreaLevelRecvTimeout && (ELOCK_ON == Elock_State))
	{
	  --CAN_UreaLevelRecvTimeout;
	}
	
	//LCD速度刷新
	if(LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout < 65535)
	{
  	++LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout;
	}
	
	if(LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout < 65535)
	{
	  ++LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout;
	}

}

#pragma CODE_SEG DEFAULT 


