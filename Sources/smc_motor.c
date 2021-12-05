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

//Motor0������λ�� Motor1����ˮ�±� Motor2�������±� Motor3����ת�ٱ� Motor4�������ر�
//���޸�Ϊ��Motor0����ת�ٱ� Motor1����ˮ�±� Motor2�������±� Motor3��������Һλ�� Motor4����ȼ��Һλ��
motorparams Motor0, Motor1, Motor2, Motor3,Motor4;
motorparams *mtr0, *mtr1, *mtr2, *mtr3, *mtr4;
Uint8_t	motor_status = 0;

#pragma CONST_SEG DEFAULT


//M0~M3΢����������                              
const int Motor_MicroStepDrivePulse[24] = { 265,  512,  724,  886,  989,  1014,  989,  886,  724,  512,  265, 0, 
                                           -265, -512, -724, -886, -989, -1014, -989, -886, -724, -512, -265, 0,};
//M0~M3΢���������壨��λ�ͺ��/3��
const int Motor_MircoStepDrivePusleDelay60Degree[24] = {-724, -512, -265, 0,  265,  512,  724,  886,  989,  1014,  989, 886,  
                                                         724,  512,  265, 0, -265, -512, -724, -886, -989, -1014, -989, -886,};
                                                    
//M4΢����������                              
const int Motor4_MicroStepDrivePulse[24] = { 66,  128,  182,  221,  247,  253,  247,  221,  182,  128,  66, 0, 
                                            -66, -128, -182, -221, -247, -253, -247, -221, -182, -128, -66, 0};
//M4΢���������壨��λ�ͺ��/3��
const int Motor4_MircoStepDrivePulseDelay60Degree[24] = {-182, -128, -66, 0,  66,   128,  182,  221,  247,  253,  247,  221,
                                                          182,  128,  66, 0, -66,  -128, -182, -221, -247, -253, -247, -221};
extern	Uint8_t		motor_status;
extern const unsigned char CAN_WorkingTimePGN[3];
extern const unsigned char CAN_DeviceInfoBuffer[8];

/******************************************************************************
@ Func: ��ʱ����ʼ��
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

    TIM0_TIOS  = 0x05;		//1,��Ӧͨ������Ϊ�Ա����-rookie_lxp
	TIM0_TCTL2 = 0x00;	// make sure IOC0_0 and IOC0_2 are set up for SW compare only - no action on pin.
	TIM0_TCTL4 = 0xFF;	// set up IOC0_0 - IOC0_3 to capture on either edge.  This could be used for stall detection on M0 and M1

    TIM1_TIOS  = 0x05;		//1,��Ӧͨ������Ϊ�Ա����-rookie_lxp
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

	TIM0_OCPD_OCPD4 = 1;	//TIM0������Ƚ϶��������ᷢ���ڸ�ͨ���ܽ���---rookie_lxp
	TIM0_TIOS |= 0x10;				// ������һ��-------------------------------------------rookie_lxp
    TIM0_TIE_C4I = 1;					// ������һ��-------------------------------------------rookie_lxp
  	TIM0_TC4 = TIM0_TCNT + _1ms;  //������һ��-------------------------------------------rookie_lxp
 	
  	TIM0_OCPD_OCPD5 = 1;
	TIM0_TIOS |= 0x20;				// set IOC0_5 to be output compare ..... use for 1ms system tick.
    TIM0_TIE_C5I = 1;					// enable IOC0_5 interrupt
  	TIM0_TC5 = TIM0_TCNT + _1ms;  // SET INTERRUPT RATE for General Purpose System Tic

  	TIM0_TC0 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  // �����Ƕ�TIM0_CH0,TIM_CH2�����ã���ô�ֳ���TC1����������
  	//TIM0_TC1 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  //
	TIM0_TC2 = TIM0_TCNT + (SAMPLE_RATE * TIMERCLK);  //

  	TIM1_TC0 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
  	//TIM1_TC1 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
	TIM1_TC2 = TIM1_TCNT + (SAMPLE_RATE * TIMERCLK);  //
  	//Init_pwm();
}


/******************************************************************************
@ Func:  ���������������ʼ��
@ Brief: ����0~3�Ų������
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
@ Func: PWM3:0 ��ӳ��
******************************************************************************/
void Motor_PWMRoute(void)
{
  PTPRRL = 0x0f;
}

/******************************************************************************
@ Func:  PWM��ʼ��
@ Brief: ����4�Ų������
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
@ Func:  ����������Ʋ�����ʼ��
******************************************************************************************/
void Init_Motor_Params(void)
{
  //ת�ٱ�
  Motor0.u32_poschange = MOTOR_ENGINE_SPEED_MAX_ANGLE * 12;
  Motor0.u32_pos = MOTOR_ENGINE_SPEED_MAX_ANGLE * 12;
  Motor0.u32_cmdpos = Motor0.u32_pos;
  Motor0.u8_pwmchan = 0;
  Motor0.u8_dir = CCW;
  Motor0.u8_cmddir = Motor0.u8_dir;
  Motor0.u8_run_status = 0;
  Motor0.u8_status_id = 1;
  Motor0.u32_step = 0;
  
  //��ȴҺ�±�
  Motor1.u32_poschange = MOTOR_COOLING_LIQUID_MAX_ANGLE * 12;
  Motor1.u32_pos = MOTOR_COOLING_LIQUID_MAX_ANGLE * 12;
  Motor1.u32_cmdpos = Motor1.u32_pos;
  Motor1.u8_pwmchan = 1;
  Motor1.u8_dir = CCW;
  Motor1.u8_cmddir = Motor1.u8_dir;
  Motor1.u8_run_status = 0;
  Motor1.u8_status_id = 1;
  Motor1.u32_step = 0;
  
  //���±�
  Motor2.u32_poschange = MOTOR_TRANS_OIL_TEMP_MAX_ANGEL * 12;
  Motor2.u32_pos = MOTOR_TRANS_OIL_TEMP_MAX_ANGEL * 12;
  Motor2.u32_cmdpos = Motor2.u32_pos;
  Motor2.u8_pwmchan = 2;
  Motor2.u8_dir = CCW;
  Motor2.u8_cmddir = Motor2.u8_dir;
  Motor2.u8_run_status = 0;
  Motor2.u8_status_id = 1;
  Motor2.u32_step = 0;
  
  //����Һλ��
  Motor3.u32_poschange = MOTOR_UREA_LEVEL_MAX_ANGLE * 12;
  Motor3.u32_pos = MOTOR_UREA_LEVEL_MAX_ANGLE * 12;
  Motor3.u32_cmdpos = Motor3.u32_pos;
  Motor3.u8_pwmchan = 3;
  Motor3.u8_dir = CCW;
  Motor3.u8_cmddir = Motor3.u8_dir;
  Motor3.u8_run_status = 0;
  Motor3.u8_status_id = 1;
  Motor3.u32_step = 0;

  //ȼ��Һλ��
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
@ Func:  �������ģ���ʼ��
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
@ Func:  ���������ת��ָ���Ƕ�
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
@ Func:  ��鲽������Ƿ�ֹͣ��ת
******************************************************************************************/
u8 CheckIfStopped(motorparams *mtr) 
{
    if (mtr->u32_cmdpos != mtr->u32_pos)
      	return(FALSE);
    else
      	return(TRUE);
}

/******************************************************************************************
@ Func:  �����������ȹ���
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
@ Func:  �������С���ȹ���
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
@ Func:  �����������
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
@ Func:  �������ָ�����Ƕ�
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
@ Func:  �������΢��������һ��ת��1/12�㣩
@ Brief: �������Ҳ���������24��Ϊһ�����ڡ�������Ȧ��ѹ��ǰ����Ȧ��ѹ��/3������
         ʱ����ת����֮��˳ʱ����ת��
@ Note:  ���±�����ر�ת���������������෴��
******************************************************************************/
static void Drive_Pwm(motorparams *mtr)
{
    unsigned int duty = 0;
    unsigned int dutyDelay = 0;
    
    if(4 == mtr->u8_pwmchan)
    {
      //NOP����M4����PWMͨ�����������üĴ����Ϸ�����ռ�ձ���������Switch-Case�ṹ��
    }
    else
    {
      //����������Ȧռ�ձȼ���������
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
    
    //��ʱ����ת
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
        
        //��ʱ�벽��һ����λ�ü�1
    		if(mtr->u32_pos)
    		{
    			mtr->u32_pos--;
    		}
    }
    //˳ʱ����ת
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

        //˳ʱ�벽��һ����λ�ü�1
        mtr->u32_pos++;
    }
    
    //��һ��΢����������
    ++mtr->u32_step;
    mtr->u32_step %= 24;
}

/******************************************************************************************
@ Func:  �����������
******************************************************************************************/
static Uint32_t  Motor_Scheduler(motorparams *mtr)
{
  Uint32_t PositionDifference;

  if(mtr->u32_cmdpos > mtr->u32_pos)                                              //����ǰλ��С��Ŀ��λ�ã���λ������ķ���ת��
  {
    mtr->u8_cmddir = CW;                                                          
    PositionDifference = mtr->u32_cmdpos - mtr->u32_pos;					
  }
  else
  {
    mtr->u8_cmddir = CCW;                                                          //����ǰλ�ô��ڣ������ڣ�Ŀ��λ�ã���λ�ü�С�ķ���ת��
  	PositionDifference = mtr->u32_pos - mtr->u32_cmdpos;
  }

  /*if(mtr->u8_status_id == 1)
  {
    mtr->u8_dir = mtr->u8_cmddir;
  }*/ 

  //����ǰλ�ú�Ŀ��λ�ò�һ�£�����Ҫ��ת
  if(mtr->u32_cmdpos != mtr->u32_pos)
  {
    mtr->u8_status_id = 0;                                                      //������ת����־

    if((motor_status == 0) || (motor_status == 0x5a))							              //���ֵ������ʼ�����ֵ����ʼ����ɺ����0xA5
    {
      if(mtr->u8_cmddir != mtr->u8_dir)													                //���Ŀ�귽�� ��= ��ǰ����
      {
        mtr->u16_vel += STEP_TIME_0 ;													                  //������
        if(mtr->u16_vel > MAX_STEP_PERIOD_0)									                  //�������4000
        {
          mtr->u16_vel = MAX_STEP_PERIOD_0;							                        //�ж�Ƶ��1kHz---�����ٶ�83deg/sec
          //Potential Danger!!! ��������ڸ������У����������ת��λ��A����ʱCANͨ�ŷ���ת��Ҫ�����ͣ��A+1����
          //������Ϊ���ٲ����������������壬����ʧ��
          if((mtr->u32_step == 0)||(mtr->u32_step == 12))		                    //�����ǰ����1deg
          {
            mtr->u8_dir = mtr->u8_cmddir;									                      //����ǰ��������ΪĿ�귽��
          }
        }
      }
      else																								                      //������ͬ
      {
      	//if ((PositionDifference <= DEC_STEP_TIME_0) && (motor_status == 0))	  //���΢����ֵС��15���������Ƕȹ���
      	if ((PositionDifference <= DEC_STEP_TIME_0) && (motor_status == 0x5A))	//��Ŀ��λ�ý�������
        {
          mtr->u16_vel += STEP_TIME_0 ; 
          if(mtr->u16_vel > MAX_STEP_PERIOD_0)
          {
              mtr->u16_vel = MAX_STEP_PERIOD_0;
          }
        }
        else                                                                    //��Ŀ��λ��Զ������
        {
          if(motor_status == 0)												                          //���Ƕȹ���-------rookie_lxp
          {
            if(mtr->u16_vel > STEP_TIME_01)					                            //�����ǰu16_vel����300
            {
                mtr->u16_vel -= STEP_TIME_01;					                          //������
            }
            if(mtr->u16_vel < MIN_STEP_PERIOD_01)		                            //��� С�� 750
            {
                mtr->u16_vel = MIN_STEP_PERIOD_01;		                          //�ж�Ƶ��5333Hz�������ٶ�444deg/sec
            }
          }
          else																			                            //���������������ͬ������
          {
            mtr->u16_vel -= STEP_TIME_0;						                            //������
            if(mtr->u16_vel < MIN_STEP_PERIOD_0)
            {
                mtr->u16_vel = MIN_STEP_PERIOD_0;	                              //�ж�Ƶ��4878Hz�������ٶ�406deg/sec
            }
          }           																			                    //���������������ͬ������
        }
      }
    }
    //С���ȹ��㣬ת�ٲ���
  	else if (motor_status == 0x10)
  	{
  		mtr->u16_vel = 3333;
  	}
  	//��������
    else																					                //��ʼ��֮�������---------rookie_lxp
    {
        if(mtr->u8_cmddir != mtr->u8_dir)						              //����ͬ
        {
            mtr->u16_vel += STEP_TIME;							              //������
            if(mtr->u16_vel > MAX_STEP_PERIOD)
            {
                mtr->u16_vel = MAX_STEP_PERIOD;			              //�ж�Ƶ��1kHz---�����ٶ�83deg/sec
                //Potential Danger!!! �����ٲ���������ܵ��¹���
                if((mtr->u32_step == 0)||(mtr->u32_step == 12))
                {   
                    mtr->u8_dir = mtr->u8_cmddir;
                }
            }
        }                                         						    
        else																								      //������ͬ
        {
            if(PositionDifference <= DEC_STEP_TIME)						    //���΢����ֵС��10
            {
                mtr->u16_vel += STEP_TIME;										    //������
                if(mtr->u16_vel > MAX_STEP_PERIOD)
                {
                    mtr->u16_vel = MAX_STEP_PERIOD;						    //�ж�Ƶ��1kHz---�����ٶ�83deg/sec
                }
            }
            else
            {
                mtr->u16_vel -= STEP_TIME;									  	  //������
                if(mtr->u8_pwmchan == 0)										      //���Ϊת�����
                {
                    if(mtr->u16_vel < MIN_STEP_PERIOD)
                    {
                        mtr->u16_vel = MIN_STEP_PERIOD;				    //�ж�Ƶ��Ϊ2kHz�������ٶ�Ϊ166deg/sec
                    }
                }
                else
                {
                    if(mtr->u16_vel < (MIN_STEP_PERIOD * 3))				  //Ϊɶ��������if(mtr->u16_vel < MIN_STEP_PERIOD*3)
                    {
                        mtr->u16_vel = (MIN_STEP_PERIOD * 3);
                    }
                }
            }
        }
    }
        
    Drive_Pwm(mtr);                                               //������ﲽ��һ��
  }
  //����ǰλ�ú�Ŀ��λ��һ�£���ֹͣ��ת
  //Potential Danger!!! ���������ת���У�ָ��λ��A��������Ҫ��ָ��A�������Ӧ�ڴ�λ��ֹͣ������Ϊû�м��ٻ������
  //���ܷ�������
  else
  {
      //��ʾ���ֹͣת��
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
@ Func:  ��������Լ�
@ Brief: motor_status���ڵ��ת�ٿ���
******************************************************************************/
void Motor_SelfCheck(void)
{
 	//�����ϵ���㣨����ȣ�----rookie_lxp
  motor_status = 0;
 	motor_zero();			
 	
 	//���󷢶���ʱ��
  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //�����豸��Ϣ
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
 
  //С���ȹ���	
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
  //�����豸��Ϣ
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
  
  //��ת�����Ƕ�
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
  //�����豸��Ϣ
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);

  //����
 	motor_set_zero();			  

 	DelayMS(200);
#ifdef	WDT_ON
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
    Feed_WatchDog();
#endif

  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
  //�����豸��Ϣ
  Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);

  //�Լ����
 	motor_status = 0xa5;		
}

#pragma CODE_SEG NON_BANKED
/******************************************************************************
@ Func:  ��ʱ��0ͨ��0�ж�
@ Brief: ����Motor0���ȣ�ȼ��Һλ��
******************************************************************************/
__interrupt VectorNumber_Vtim0ch0 void _isr_M0__IOC_0_Interrupt (void)
{
   	TIM0_TC0 = TIM0_TC0 + Motor_Scheduler(&Motor0);     // set next step time        
  	TIM0_TFLG1 = 0x01;                                  // clear OC0 interrupt flag
}

/******************************************************************************
@ Func:  ��ʱ��0ͨ��2�ж�
@ Brief: ����Motor1���ȣ���ȴҺ�±�
******************************************************************************/
__interrupt VectorNumber_Vtim0ch2 void _isr_M1__IOC_1_Interrupt (void)
{
   	TIM0_TC2 = TIM0_TC2 + Motor_Scheduler(&Motor1);     // set next step time
  	TIM0_TFLG1 = 0x04;	                                // clear interrupt flag
} 

/******************************************************************************
@ Func:  ��ʱ��1ͨ��0�ж�
@ Brief: ����Motor2���ȣ����±�
******************************************************************************/
__interrupt VectorNumber_Vtim1ch0 void _isr_M2__IOC_0_Interrupt (void)
{   
   	TIM1_TC0 = TIM1_TC0 + Motor_Scheduler(&Motor2);     // set next step time        
  	TIM1_TFLG1 = 0x01;	                                // clear OC0 interrupt flag    
}

/******************************************************************************
@ Func:  ��ʱ��1ͨ��2�ж�
@ Brief: ����Motor3���ȣ�ת�ٱ�
******************************************************************************/
__interrupt VectorNumber_Vtim1ch2 void _isr_M3__IOC_1_Interrupt (void)
{
   	TIM1_TC2 = TIM1_TC2 + Motor_Scheduler(&Motor3);     // set next step time
  	TIM1_TFLG1 = 0x04;	                                // clear interrupt flag 
} 

/******************************************************************************
@ Func:  ��ʱ��0ͨ��5�ж�
@ Brief: ����Motor4���ȣ�����Һλ��
******************************************************************************/
__interrupt VectorNumber_Vtim0ch5 void _isr_M4__IOC_1_Interrupt (void)
{
   	TIM0_TC5 = TIM0_TC5 + Motor_Scheduler(&Motor4);     // set next step time
   	TIM0_TFLG1 = 0x20;	                                // clear interrupt flag 
}

#pragma CODE_SEG DEFAULT

