#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "board.h"
#include "time.h"
//#include "flash.h"
#include "vehicle.h"
#include "test_mode.h"
#include "can.h"
#include "digital.h"
#include "smc_motor.h"
#include "time.h"

//Motor0——油位表 Motor1——水温表 Motor2——油温表 Motor3——转速表 Motor4——尿素表
//新修改为：Motor0——转速表 Motor1——水温表 Motor2——油温表 Motor3——尿素液位表 Motor4——燃油液位表
motorparams Motor0, Motor1, Motor2, Motor3,Motor4;
motorparams *mtr0, *mtr1, *mtr2, *mtr3, *mtr4;
Uint8_t	motor_status = 0;

#pragma CONST_SEG DEFAULT


//M0~M3微步驱动脉冲                              
const int Motor_MicroStepDrivePulse[24] = { 265,  512,  724,  886,  989,  1014,  989,  886,  724,  512,  265, 0, 
                                           -265, -512, -724, -886, -989, -1014, -989, -886, -724, -512, -265, 0,};
//M0~M3微步驱动脉冲（相位滞后π/3）
const int Motor_MircoStepDrivePusleDelay60Degree[24] = {-724, -512, -265, 0,  265,  512,  724,  886,  989,  1014,  989, 886,  
                                                         724,  512,  265, 0, -265, -512, -724, -886, -989, -1014, -989, -886,};
                                                    
//M4微步驱动脉冲                              
const int Motor4_MicroStepDrivePulse[24] = { 66,  128,  182,  221,  247,  253,  247,  221,  182,  128,  66, 0, 
                                            -66, -128, -182, -221, -247, -253, -247, -221, -182, -128, -66, 0};
//M4微步驱动脉冲（相位滞后π/3）
const int Motor4_MircoStepDrivePulseDelay60Degree[24] = {-182, -128, -66, 0,  66,   128,  182,  221,  247,  253,  247,  221,
                                                          182,  128,  66, 0, -66,  -128, -182, -221, -247, -253, -247, -221};
extern	Uint8_t		motor_status;
extern const unsigned char CAN_WorkingTimePGN[3];
extern const unsigned char CAN_DeviceInfoBuffer[8];

/******************************************************************************
@ Func: 定时器初始化
******************************************************************************/
void Init_TIMx(void)
{
	// Set up Timer Prescaler based on Desired BUSCLK and TIMERCLK
#if ( (BUSCLK / TIMERCLK) == 1)
	#define TIMERPRESCALER 0
#endif  

#if ( (BUSCLK / TIMERCLK) == 2)
 	#define TIMERPRESCALER 1
#endif  

#if ( (BUSCLK / TIMERCLK) == 4)
 	#define TIMERPRESCALER 2
#endif  

#if ( (BUSCLK / TIMERCLK) == 8)
	#define TIMERPRESCALER 3
#endif  

#if ( (BUSCLK / TIMERCLK) == 16)
	#define TIMERPRESCALER 4
#endif  

    TIM0_TIOS  = 0x05;		//1,相应通道设置为对比输出-rookie_lxp
	TIM0_TCTL2 = 0x00;	// make sure IOC0_0 and IOC0_2 are set up for SW compare only - no action on pin.
	TIM0_TCTL4 = 0xFF;	// set up IOC0_0 - IOC0_3 to capture on either edge.  This could be used for stall detection on M0 and M1

    TIM1_TIOS  = 0x05;		//1,相应通道设置为对比输出-rookie_lxp
	TIM1_TCTL2 = 0x00;	// make sure IOC1_0 and IOC1_2 are set up for SW compare only - no action on pin.
	TIM1_TCTL4 = 0xFF;	// set up IOC1_0 - IOC1_3 to capture on either edge.  This could be used for stall detection on M0 and M1

    TIM0_TSCR2 |= TIMERPRESCALER;	//TIM0_CLK==bus clock/8 = 4MHz---rookie_lxp
	TIM1_TSCR2 = TIMERPRESCALER;	//TIM1_CLK==bus clock/8 = 4MHz---rookie_lxp

    TIM0_TSCR1 = 0xa0;  // enable timer module - stop timer in freeze mode - for DEBUG purposes
    TIM1_TSCR1 = 0xa0;  // enable timer module - stop timer in freeze mode - for DEBUG purposes
    
    TIM0_TIE_C0I = 1;   // enable IOC0_0 interrupts - used to schedule M0 movement
    TIM0_TIE_C2I = 1;	  // enable IOC0_2 interrupts - used to schedule M1 movement

    TIM1_TIE_C0I = 1;   // enable IOC1_0 interrupts - used to schedule M2 movement
    TIM1_TIE_C2I = 1;	  // enable IOC1_2 interrupts - used to schedule M3 movement

	TIM0_OCPD_OCPD4 = 1;	//TIM0的输出比较动作将不会发生在该通道管脚上---rookie_lxp
	TIM0_TIOS |= 0x10;				// 又设置一遍-------------------------------------------rookie_lxp
    TIM0_TIE_C4I = 1;					// 又设置一遍-------------------------------------------rookie_lxp
  	TIM0_TC4 = TIM0_TCNT + _1ms;  //又设置一遍-------------------------------------------rookie_lxp
 	
  	TIM0_OCPD_OCPD5 = 1;
	TIM0_TIOS |= 0x20;				// set IOC0_5 to be output compare ..... use for 1ms system tick.
    TIM0_TIE_C5I = 1;					// enable IOC0_5 interrupt
  	TIM0_TC5 = TIM0_TCNT + _1ms;  // SET INTERRUPT RATE for General Purpose System Tic

  	TIM0_TC0 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  // 上面是对TIM0_CH0,TIM_CH2的设置，怎么又成了TC1？？？？？
  	//TIM0_TC1 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  //
	TIM0_TC2 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  //

  	TIM1_TC0 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
  	//TIM1_TC1 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
	TIM1_TC2 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
  	//Init_pwm();
}


/******************************************************************************
@ Func:  步进电机控制器初始化
@ Brief: 控制0~3号步进电机
******************************************************************************/
void Init_MCPWM(void)
{
  	// Set up MCPWM Prescaler based on Desired BUSCLK and PWMCLK 

  	#if ( (BUSCLK / MCPWMCLK) == 2)
   	 	#define MCPWMPRESCALER 0
  	#endif  

  	#if ( (BUSCLK / MCPWMCLK) == 4)
    	#define MCPWMPRESCALER 1
  	#endif  

  	#if ( (BUSCLK / MCPWMCLK) == 8)
    	#define MCPWMPRESCALER 2
  	#endif  

  	#if ( (BUSCLK / MCPWMCLK) == 16)
   	 	#define MCPWMPRESCALER 3
  	#endif  

  //----------------------------
  //  Motor PWM setup code   
  //----------------------------

	// set up Motor Control PWMs

	/*
	|---------------------------------------------------------------------|
	|   0   | MCPRE1 | MCPRE0 | MCSWAI |  FAST  |  DITH  |   0   | MCTOIF |
	|---------------------------------------------------------------------|
    
    MCPRE[1:0] -->  00 = module clock is FBUS
    			          01 = module clock is FBUS/2
				            10 = module clock is FBUS/4
				            11 = module clock is FBUS/8
	
	  MCSWAI = 0 -->  1 = motor coils are released in WAIT mode
					          0 = motor coils are still driven during WAIT mode
	  FAST = 0   -->  1 =  7-bit PWM mode
					          0 = 11-bit PWM mode
	  DITH = 0   -->  1 = Dither enabled
					          0 = Dither disabled	
	*/
	
  	MCCTL0_MCPRE = MCPWMPRESCALER;												//fTC = fBUS=32MHz-------rookie_lxp
  	MCCTL0 &= 0x60;   // set MCSWAI, FAST, DITH, and MCTOIF = 0

	/*
	|---------------------------------------------------------------------|
	|  RECIRC  |   0   |   0   |   0   |   0   |   0   |   0   |  MCTOIE  |
	|---------------------------------------------------------------------|
	
	  RECIRC = 0 -->  0 = Recirc on high side (PWM active low) 
		  		          1 = Recirc on low side (PWM active high) 
	  MCTOIE = 0 -->  0 = Interrupt disabled
		  		          1 = PWM Timer Interrupt enabled	
	*/

	MCCTL1 = 0x00;

	/*
		FBUS  = 16MHz 
    ftc   = FBUS/4 = 4MHz 

		using 8-bits of resolution:
		250ns * 256 = 32us = PWM_PERIOD
	
		PWM Frequency = 1/PWM_PERIOD = 15.625 KHz	 (4MHz Timer clock)
		PWM Frequency = 1/PWM_PERIOD = 31.25 KHz   (8MHz Timer clock)
	*/

	MCPER = PWMPER;			//32MHz/0x400=31.25kHz-----------------------------------------rookie_lxp

	/*	 
	|---------------------------------------------------------------------|
	|   MCOM1  |  MCOM0  |  MCAM1  |  MCAM0  |  0  |  0  |  CD1  |  CD0   |
	|---------------------------------------------------------------------|
    
    MCOM[1:0] --> 00 = Half H-Bridge mode - PWM on MnCxM - MnCxP is released
                  01 = Half H-Bridge mode - PWM on MnCxP - MnCxM is released
			        	  10 = Full H-Bridge mode
		        		  11 = Dual Full H-Bridge mode

	  MCAM[1:0] --> 00 = Channel disabled
		        		  01 = left aligned
				          10 = right aligned
	        			  11 = center aligned
	
	  CD[1:0] ----> 00 = 0 timer clocks of delay
		        		  01 = 1 timer clocks of delay
				          10 = 2 timer clocks of delay
				          11 = 3 timer clocks of delay  				  	
	*/

  	MCCC0 = 0xF0;   // Motor 0
  	MCCC1 = 0xF0;
  	MCCC2 = 0xF0;	  // Motor 1
  	MCCC3 = 0xF0;
  	
  	MCCC4 = 0xF0;  	// Motor 2
  	MCCC5 = 0xF0;
  	MCCC6 = 0xF0;	  // Motor 3
  	MCCC7 = 0xF0;

	// Initial Duty cycle

  	MCDC0 = 0;		  // Motor 0
  	MCDC1 = 0;
  	MCDC2 = 0;		  // Motor 1
  	MCDC3 = 0;
  	
  	MCDC4 = 0;		  // Motor 2
  	MCDC5 = 0;
  	MCDC6 = 0;		  // Motor 3
  	MCDC7 = 0;
  	
}

/******************************************************************************
@ Func: PWM3:0 重映射
******************************************************************************/
void Motor_PWMRoute(void)
{
  PTPRRL = 0x0f;
}

/******************************************************************************
@ Func:  PWM初始化
@ Brief: 控制4号步进电机
******************************************************************************/
void Motor_PWMInit(void)
{
  PTPRRL = 0x0f;      // Route PWM3:0 from PP3:0 to PS7:4

  PWMPRCLK = 0x00;    // Value of A & B is bus clock.
  //PWMPOL = 0x10;      // Outputs are high at the beginning of the period, then go low when the duty count is reached.
  PWMPOL |= 0x0f;
  PWMCAE = 0x00;      // Left aligned output mode
  
  //PWMCLK = 0x10;
  //The above register setting should match the following truth table  .
  //PWM0: Center Aligned=0, Polarity=0
  //PWM1: Center Aligned=0, Polarity=1
  //PWM2: Center Aligned=1, Polarity=0
  //PWM3: Center Aligned=1, Polarity=1
  /*************************/  

  PWMPER0 = 0xff;     //Set chan 0 period length to the maximum.
  PWMPER1 = 0xff;     //Set chan 1 period length to the maximum.
  PWMPER2 = 0xff;     //Set chan 2 period length to the maximum.
  PWMPER3 = 0xff;     //Set chan 3 period length to the maximum.
  PWMDTY0 = 0x00;     //Set chan 0 duty cycle to half the maximum, 50%.
  PWMDTY1 = 0x00;     //Set chan 1 duty cycle to half the maximum, 50%.
  PWMDTY2 = 0x00;     //Set chan 2 duty cycle to half the maximum, 50%.
  PWMDTY3 = 0x00;     //Set chan 3 duty cycle to half the maximum, 50%.
  
  PWME = 0x0f;        // Eisable PWM channels 0 - 3
}

/******************************************************************************************
@ Func:  步进电机控制参数初始化
******************************************************************************************/
void Init_Motor_Params(void)
{
  //转速表
  Motor0.u32_poschange = MOTOR_ENGINE_SPEED_MAX_ANGLE * 12;
  Motor0.u32_pos = MOTOR_ENGINE_SPEED_MAX_ANGLE * 12;
  Motor0.u32_cmdpos = Motor0.u32_pos;
  Motor0.u8_pwmchan = 0;
  Motor0.u8_dir = CCW;
  Motor0.u8_cmddir = Motor0.u8_dir;
  Motor0.u8_run_status = 0;
  Motor0.u8_status_id = 1;
  Motor0.u32_step = 0;
  
  //冷却液温表
  Motor1.u32_poschange = MOTOR_COOLING_LIQUID_MAX_ANGLE * 12;
  Motor1.u32_pos = MOTOR_COOLING_LIQUID_MAX_ANGLE * 12;
  Motor1.u32_cmdpos = Motor1.u32_pos;
  Motor1.u8_pwmchan = 1;
  Motor1.u8_dir = CCW;
  Motor1.u8_cmddir = Motor1.u8_dir;
  Motor1.u8_run_status = 0;
  Motor1.u8_status_id = 1;
  Motor1.u32_step = 0;
  
  //油温表
  Motor2.u32_poschange = MOTOR_TRANS_OIL_TEMP_MAX_ANGEL * 12;
  Motor2.u32_pos = MOTOR_TRANS_OIL_TEMP_MAX_ANGEL * 12;
  Motor2.u32_cmdpos = Motor2.u32_pos;
  Motor2.u8_pwmchan = 2;
  Motor2.u8_dir = CCW;
  Motor2.u8_cmddir = Motor2.u8_dir;
  Motor2.u8_run_status = 0;
  Motor2.u8_status_id = 1;
  Motor2.u32_step = 0;
  
  //尿素液位表
  Motor3.u32_poschange = MOTOR_UREA_LEVEL_MAX_ANGLE * 12;
  Motor3.u32_pos = MOTOR_UREA_LEVEL_MAX_ANGLE * 12;
  Motor3.u32_cmdpos = Motor3.u32_pos;
  Motor3.u8_pwmchan = 3;
  Motor3.u8_dir = CCW;
  Motor3.u8_cmddir = Motor3.u8_dir;
  Motor3.u8_run_status = 0;
  Motor3.u8_status_id = 1;
  Motor3.u32_step = 0;

  //燃油液位表
  Motor4.u32_poschange = MOTOR_FUEL_LEVEL_MAX_ANGLE * 12;
  Motor4.u32_pos = MOTOR_FUEL_LEVEL_MAX_ANGLE * 12;
  Motor4.u32_cmdpos = Motor4.u32_pos;
  Motor4.u8_pwmchan = 4;
  Motor4.u8_dir = CCW;
  Motor4.u8_cmddir = Motor4.u8_dir;
  Motor4.u8_run_status = 0;
  Motor4.u8_status_id = 1;
  Motor4.u32_step = 0; 
}

/******************************************************************************************
@ Func:  步进电机模块初始化
******************************************************************************************/
void	Config_Motor_io(void)
{
	PTU = 0x00;      	// make sure motor pins are not driven if not used by MCPWM
	PTV = 0x00;  
	DDRU = 0xFF;  	// motor control pins are outputs by default
	DDRV = 0xFF;	// motor control pins are outputs by default
	Init_TIMx();
	Init_MCPWM();
	Motor_PWMInit();
	Init_Motor_Params();
	
	mtr0 = &Motor0;
 	mtr1 = &Motor1;
	mtr2 = &Motor2;
	mtr3 = &Motor3;
	mtr4 = &Motor4;
}

/******************************************************************************************
@ Func:  步进电机旋转到指定角度
******************************************************************************************/
void GoToPosition(motorparams *mtr, u8 position)
{
    Uint32_t cmdpos_data;
    
    cmdpos_data = position * 12;
    if(cmdpos_data > mtr->u32_cmdpos)
    {
        mtr->u8_direction = 1;
    }
    else
    {
         mtr->u8_direction = 2;
    } 
    mtr->u32_cmdpos = cmdpos_data;
} 

/******************************************************************************************
@ Func:  检查步进电机是否停止运转
******************************************************************************************/
u8 CheckIfStopped(motorparams *mtr) 
{
    if (mtr->u32_cmdpos != mtr->u32_pos)
      	return(FALSE);
    else
      	return(TRUE);
}

/******************************************************************************************
@ Func:  步进电机大幅度归零
******************************************************************************************/
void	motor_zero(void)
{
  Motor0.u32_pos = MOTOR_ENGINE_SPEED_MAX_ANGLE * 12;
  Motor1.u32_pos = MOTOR_COOLING_LIQUID_MAX_ANGLE * 12;
  Motor2.u32_pos = MOTOR_TRANS_OIL_TEMP_MAX_ANGEL * 12;
  Motor3.u32_pos = MOTOR_UREA_LEVEL_MAX_ANGLE * 12;  
  Motor4.u32_pos = MOTOR_FUEL_LEVEL_MAX_ANGLE * 12;
  
  GoToPosition(mtr0,0);
	GoToPosition(mtr1,0);
	GoToPosition(mtr2,0);
	GoToPosition(mtr3,0);
 	GoToPosition(mtr4,0);

 	while((CheckIfStopped(mtr0) == FALSE) || (CheckIfStopped(mtr1) == FALSE)
			  || (CheckIfStopped(mtr2) == FALSE) || (CheckIfStopped(mtr3) == FALSE)
			  || (CheckIfStopped(mtr4) == FALSE))
 	{
#ifdef WDT_ON
    __RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif
    DelayMS(10);
 	}
}

/******************************************************************************************
@ Func:  步进电机小幅度归零
******************************************************************************************/
void	small_motor_zero(void)
{
	Motor0.u32_pos = 5 * 12;
	Motor1.u32_pos = 5 * 12;
	Motor2.u32_pos = 5 * 12;
	Motor3.u32_pos = 5 * 12;
	Motor4.u32_pos = 5 * 12;

	GoToPosition(mtr0, 0);
	GoToPosition(mtr1, 0);
	GoToPosition(mtr2, 0);
	GoToPosition(mtr3, 0);
	GoToPosition(mtr4, 0);
	
 	while((CheckIfStopped(mtr0) == FALSE) || (CheckIfStopped(mtr1) == FALSE)
			  || (CheckIfStopped(mtr2) == FALSE) || (CheckIfStopped(mtr3) == FALSE)
			  || (CheckIfStopped(mtr4) == FALSE))
 	{
#ifdef	WDT_ON   
  	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif
    DelayMS(10);
 	}
}

/******************************************************************************************
@ Func:  步进电机归零
******************************************************************************************/
void	motor_set_zero(void)
{
 	GoToPosition(mtr0,0);
	GoToPosition(mtr1,0);
	GoToPosition(mtr2,0);
	GoToPosition(mtr3,0);
 	GoToPosition(mtr4,0);
 	
 	while((CheckIfStopped(mtr0) == FALSE) || (CheckIfStopped(mtr1) == FALSE)
			  || (CheckIfStopped(mtr2) == FALSE) || (CheckIfStopped(mtr3) == FALSE)
			  || (CheckIfStopped(mtr4) == FALSE))
 	{
#ifdef	WDT_ON   
  	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif
    DelayMS(10);
 	}
}

/******************************************************************************************
@ Func:  步进电机指到最大角度
******************************************************************************************/
void	motor_max(void)
{
	GoToPosition(mtr0, MOTOR_ENGINE_SPEED_MAX_ANGLE);	
	GoToPosition(mtr1, MOTOR_COOLING_LIQUID_MAX_ANGLE);	
	GoToPosition(mtr2, MOTOR_TRANS_OIL_TEMP_MAX_ANGEL);	
	GoToPosition(mtr3, MOTOR_UREA_LEVEL_MAX_ANGLE);
	GoToPosition(mtr4, MOTOR_FUEL_LEVEL_MAX_ANGLE);

 	while((CheckIfStopped(mtr0) == FALSE) || (CheckIfStopped(mtr1) == FALSE)
			  || (CheckIfStopped(mtr2) == FALSE) || (CheckIfStopped(mtr3) == FALSE)
			  || (CheckIfStopped(mtr4) == FALSE))
 	{
#ifdef	WDT_ON   
  	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif
    DelayMS(10);
 	}
}

/******************************************************************************
@ Func:  步进电机微步驱动（一步转动1/12°）
@ Brief: 采用正弦波形驱动，24步为一个周期。若左线圈电压超前右线圈电压π/3，则逆
         时针旋转，反之则顺时针旋转。
@ Note:  油温表和尿素表转动方向与其他表相反！
******************************************************************************/
static void Drive_Pwm(motorparams *mtr)
{
    unsigned int duty = 0;
    unsigned int dutyDelay = 0;
    
    if(4 == mtr->u8_pwmchan)
    {
      //NOP，因M4利用PWM通道驱动，设置寄存器较繁琐，占空比设置置于Switch-Case结构中
    }
    else
    {
      //设置左右线圈占空比及电流方向
      if(Motor_MicroStepDrivePulse[mtr->u32_step] >= 0)
      {
        duty = Motor_MicroStepDrivePulse[mtr->u32_step];
      }
      else
      {
        duty = (-Motor_MicroStepDrivePulse[mtr->u32_step]) + MCDC_SIGN;
      }
      
      if(Motor_MircoStepDrivePusleDelay60Degree[mtr->u32_step] >= 0)
      {
        dutyDelay = Motor_MircoStepDrivePusleDelay60Degree[mtr->u32_step];
      }
      else
      {
        dutyDelay = (-Motor_MircoStepDrivePusleDelay60Degree[mtr->u32_step]) + MCDC_SIGN;
      }
    }
    
    //逆时针旋转
    if(mtr->u8_dir == CCW)
    {     
        switch(mtr->u8_pwmchan)
        {
          case 0:
            MOTOR_0_COIL1 = duty;
            MOTOR_0_COIL2 = dutyDelay;
          break;
          
          case 1:
            MOTOR_1_COIL1 = duty;
            MOTOR_1_COIL2 = dutyDelay;
          break;
          
          case 2:
            MOTOR_2_COIL1 = dutyDelay;
            MOTOR_2_COIL2 = duty;
          break;
          
          case 3:
            MOTOR_3_COIL1 = duty;
            MOTOR_3_COIL2 = dutyDelay;
          break;
          
          case 4:
            if(Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step] >= 0)
            {
              MOTOR4_PIN1 = Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step];
              MOTOR4_PIN2 = 0;
            }
            else
            {
              MOTOR4_PIN1 = 0;
              MOTOR4_PIN2 = -Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step];
            }
            
            if(Motor4_MicroStepDrivePulse[mtr->u32_step] >= 0)
            {
              MOTOR4_PIN4 = Motor4_MicroStepDrivePulse[mtr->u32_step];
              MOTOR4_PIN3 = 0;
            }
            else
            {
              MOTOR4_PIN4 = 0;
              MOTOR4_PIN3 = -Motor4_MicroStepDrivePulse[mtr->u32_step];
            }
          break;
        }
        
        //逆时针步进一步，位置减1
    		if(mtr->u32_pos)
    		{
    			mtr->u32_pos--;
    		}
    }
    //顺时针旋转
    else
    {
        switch(mtr->u8_pwmchan)
        {
          case 0:
            MOTOR_0_COIL1 = dutyDelay;
            MOTOR_0_COIL2 = duty;
          break;
          
          case 1:
            MOTOR_1_COIL1 = dutyDelay;
            MOTOR_1_COIL2 = duty;
          break;
          
          case 2:
            MOTOR_2_COIL1 = duty;
            MOTOR_2_COIL2 = dutyDelay;
          break;
          
          case 3:
            MOTOR_3_COIL1 = dutyDelay;
            MOTOR_3_COIL2 = duty;
          break;
          
          case 4:
            if(Motor4_MicroStepDrivePulse[mtr->u32_step] >= 0)
            {
              MOTOR4_PIN1 = Motor4_MicroStepDrivePulse[mtr->u32_step];
              MOTOR4_PIN2 = 0;
            }
            else
            {
              MOTOR4_PIN1 = 0;
              MOTOR4_PIN2 = -Motor4_MicroStepDrivePulse[mtr->u32_step];
            }
            
            if(Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step] >= 0)
            {
              MOTOR4_PIN4 = Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step];
              MOTOR4_PIN3 = 0;
            }
            else
            {
              MOTOR4_PIN4 = 0;
              MOTOR4_PIN3 = -Motor4_MircoStepDrivePulseDelay60Degree[mtr->u32_step];
            }
            
          break;
        }

        //顺时针步进一步，位置加1
        mtr->u32_pos++;
    }
    
    //下一步微步脉冲索引
    ++mtr->u32_step;
    mtr->u32_step %= 24;
}

/******************************************************************************************
@ Func:  步进电机调度
******************************************************************************************/
static Uint32_t  Motor_Scheduler(motorparams *mtr)
{
  Uint32_t PositionDifference;

  if(mtr->u32_cmdpos > mtr->u32_pos)                                              //若当前位置小于目标位置，则朝位置增大的方向转动
  {
    mtr->u8_cmddir = CW;                                                          
    PositionDifference = mtr->u32_cmdpos - mtr->u32_pos;					
  }
  else
  {
    mtr->u8_cmddir = CCW;                                                          //若当前位置大于（含等于）目标位置，则朝位置减小的方向转动
  	PositionDifference = mtr->u32_pos - mtr->u32_cmdpos;
  }

  /*if(mtr->u8_status_id == 1)
  {
    mtr->u8_dir = mtr->u8_cmddir;
  }*/ 

  //若当前位置和目标位置不一致，则需要旋转
  if(mtr->u32_cmdpos != mtr->u32_pos)
  {
    mtr->u8_status_id = 0;                                                      //置正在转动标志

    if((motor_status == 0) || (motor_status == 0x5a))							              //最大值归零或初始化最大值，初始化完成后才是0xA5
    {
      if(mtr->u8_cmddir != mtr->u8_dir)													                //如果目标方向 ！= 当前方向
      {
        mtr->u16_vel += STEP_TIME_0 ;													                  //马达减速
        if(mtr->u16_vel > MAX_STEP_PERIOD_0)									                  //如果超过4000
        {
          mtr->u16_vel = MAX_STEP_PERIOD_0;							                        //中断频率1kHz---马达角速度83deg/sec
          //Potential Danger!!! 若马达正在高速运行，例如表针旋转到位置A，此时CAN通信发送转速要求表针停在A+1，则
          //可能因为减速步数不够而发生过冲，导致失步
          if((mtr->u32_step == 0)||(mtr->u32_step == 12))		                    //如果当前走完1deg
          {
            mtr->u8_dir = mtr->u8_cmddir;									                      //将当前方向设置为目标方向
          }
        }
      }
      else																								                      //方向相同
      {
      	//if ((PositionDifference <= DEC_STEP_TIME_0) && (motor_status == 0))	  //如果微步差值小于15，且是最大角度归零
      	if ((PositionDifference <= DEC_STEP_TIME_0) && (motor_status == 0x5A))	//离目标位置近，减速
        {
          mtr->u16_vel += STEP_TIME_0 ; 
          if(mtr->u16_vel > MAX_STEP_PERIOD_0)
          {
              mtr->u16_vel = MAX_STEP_PERIOD_0;
          }
        }
        else                                                                    //离目标位置远，加速
        {
          if(motor_status == 0)												                          //最大角度归零-------rookie_lxp
          {
            if(mtr->u16_vel > STEP_TIME_01)					                            //如果当前u16_vel大于300
            {
                mtr->u16_vel -= STEP_TIME_01;					                          //马达加速
            }
            if(mtr->u16_vel < MIN_STEP_PERIOD_01)		                            //如果 小于 750
            {
                mtr->u16_vel = MIN_STEP_PERIOD_01;		                          //中断频率5333Hz，马达角速度444deg/sec
            }
          }
          else																			                            //除最大归零外的所有同向驱动
          {
            mtr->u16_vel -= STEP_TIME_0;						                            //马达加速
            if(mtr->u16_vel < MIN_STEP_PERIOD_0)
            {
                mtr->u16_vel = MIN_STEP_PERIOD_0;	                              //中断频率4878Hz，马达角速度406deg/sec
            }
          }           																			                    //除最大归零外的所有同向驱动
        }
      }
    }
    //小幅度归零，转速不变
  	else if (motor_status == 0x10)
  	{
  		mtr->u16_vel = 3333;
  	}
  	//正常工作
    else																					                //初始化之后的驱动---------rookie_lxp
    {
        if(mtr->u8_cmddir != mtr->u8_dir)						              //方向不同
        {
            mtr->u16_vel += STEP_TIME;							              //马达减速
            if(mtr->u16_vel > MAX_STEP_PERIOD)
            {
                mtr->u16_vel = MAX_STEP_PERIOD;			              //中断频率1kHz---马达角速度83deg/sec
                //Potential Danger!!! 马达减速步数不足可能导致过冲
                if((mtr->u32_step == 0)||(mtr->u32_step == 12))
                {   
                    mtr->u8_dir = mtr->u8_cmddir;
                }
            }
        }                                         						    
        else																								      //方向相同
        {
            if(PositionDifference <= DEC_STEP_TIME)						    //如果微步差值小于10
            {
                mtr->u16_vel += STEP_TIME;										    //马达减速
                if(mtr->u16_vel > MAX_STEP_PERIOD)
                {
                    mtr->u16_vel = MAX_STEP_PERIOD;						    //中断频率1kHz---马达角速度83deg/sec
                }
            }
            else
            {
                mtr->u16_vel -= STEP_TIME;									  	  //马达加速
                if(mtr->u8_pwmchan == 0)										      //如果为转速马达
                {
                    if(mtr->u16_vel < MIN_STEP_PERIOD)
                    {
                        mtr->u16_vel = MIN_STEP_PERIOD;				    //中断频率为2kHz，马达角速度为166deg/sec
                    }
                }
                else
                {
                    if(mtr->u16_vel < (MIN_STEP_PERIOD * 3))				  //为啥区分马达？？if(mtr->u16_vel < MIN_STEP_PERIOD*3)
                    {
                        mtr->u16_vel = (MIN_STEP_PERIOD * 3);
                    }
                }
            }
        }
    }
        
    Drive_Pwm(mtr);                                               //驱动马达步进一步
  }
  //若当前位置和目标位置一致，则停止旋转
  //Potential Danger!!! 若表针高速转动中，指到位置A，而命令要求指到A，则表针应在此位置停止，但因为没有减速缓冲过程
  //可能发生过冲
  else
  {
      //表示电机停止转动
      mtr->u8_status_id = 1;	
      
      mtr->u8_dir = mtr->u8_cmddir;
      if((motor_status == 0) || (motor_status == 0x10) || (motor_status == 0x5A))
      {
          mtr->u16_vel = MAX_STEP_PERIOD_0;
      }
      else
      {
          mtr->u16_vel = MAX_STEP_PERIOD;
      }
  }
  
  return(mtr->u16_vel); 
}    

/******************************************************************************
@ Func:  步进电机自检
@ Brief: motor_status用于电机转速控制
******************************************************************************/
void Motor_SelfCheck(void)
{
 	//电锁上电归零（大幅度）----rookie_lxp
  motor_status = 0;
 	motor_zero();			
 	
 	//请求发动机时间
  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //发送设备信息
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
 
  //小幅度归零	
  motor_status = 0x10;	
  small_motor_zero();
	
 	DelayMS(200);					  
#ifdef	WDT_ON
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif

  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //发送设备信息
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
  
  //旋转到最大角度
 	motor_status = 0x5a;
 	motor_max();				  
 	
  DelayMS(200);
#ifdef	WDT_ON
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif

  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //发送设备信息
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);

  //回零
 	motor_set_zero();			  

 	DelayMS(200);
#ifdef	WDT_ON
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif

  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //发送设备信息
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);

  //自检完成
 	motor_status = 0xa5;		
}

#pragma CODE_SEG NON_BANKED
/******************************************************************************
@ Func:  定时器0通道0中断
@ Brief: 用于Motor0调度，燃油液位表
******************************************************************************/
__interrupt VectorNumber_Vtim0ch0 void _isr_M0__IOC_0_Interrupt (void)
{
   	TIM0_TC0 = TIM0_TC0 + Motor_Scheduler(&Motor0);     // set next step time        
  	TIM0_TFLG1 = 0x01;                                  // clear OC0 interrupt flag
}

/******************************************************************************
@ Func:  定时器0通道2中断
@ Brief: 用于Motor1调度，冷却液温表
******************************************************************************/
__interrupt VectorNumber_Vtim0ch2 void _isr_M1__IOC_1_Interrupt (void)
{
   	TIM0_TC2 = TIM0_TC2 + Motor_Scheduler(&Motor1);     // set next step time
  	TIM0_TFLG1 = 0x04;	                                // clear interrupt flag
} 

/******************************************************************************
@ Func:  定时器1通道0中断
@ Brief: 用于Motor2调度，油温表
******************************************************************************/
__interrupt VectorNumber_Vtim1ch0 void _isr_M2__IOC_0_Interrupt (void)
{   
   	TIM1_TC0 = TIM1_TC0 + Motor_Scheduler(&Motor2);     // set next step time        
  	TIM1_TFLG1 = 0x01;	                                // clear OC0 interrupt flag    
}

/******************************************************************************
@ Func:  定时器1通道2中断
@ Brief: 用于Motor3调度，转速表
******************************************************************************/
__interrupt VectorNumber_Vtim1ch2 void _isr_M3__IOC_1_Interrupt (void)
{
   	TIM1_TC2 = TIM1_TC2 + Motor_Scheduler(&Motor3);     // set next step time
  	TIM1_TFLG1 = 0x04;	                                // clear interrupt flag 
} 

/******************************************************************************
@ Func:  定时器0通道5中断
@ Brief: 用于Motor4调度，尿素液位表
******************************************************************************/
__interrupt VectorNumber_Vtim0ch5 void _isr_M4__IOC_1_Interrupt (void)
{
   	TIM0_TC5 = TIM0_TC5 + Motor_Scheduler(&Motor4);     // set next step time
   	TIM0_TFLG1 = 0x20;	                                // clear interrupt flag 
}

#pragma CODE_SEG DEFAULT

