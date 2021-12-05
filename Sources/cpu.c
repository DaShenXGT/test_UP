#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "cpu.h"
#include "board.h"//added by rookie_lxp 若不添加，否则看门狗不起作用！

/************************************************
*name:			Cofigure_Cpu_Clk
*describe:		system clock initialization
*parameter		no
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void Cofigure_Cpu_Clk(void)
{
	CPMUSYNR = 0x04;						//fVCO =2*fREF*(SYNDIV+1)=2*1*(4+1)=10MHz---rookie_lxp
	CPMUREFDIV = 0x07;					//fREF = fOSC/(REFDIV+1) = 1MHz---rookie_lxp
	CPMUPOSTDIV = 0;						//如果LOCK=1，则fPLL = fVCO/(POSTDIV+1)=64MHz; 
															//如果PLLSEL=1，则fBUS=fPLL/2=32MHz---rookie_lxp
	
	CPMUSYNR_SYNDIV = 32-1;			 //如果LOCK=1，则fVCO =2*fREF*(SYNDIV+1) =64MHz---rookie_lxp
	CPMUSYNR_VCOFRQ = 0x01;       // 01 = 48MHz < Fvco <= 64MHz---rookie_lxp

	CPMUOSC_OSCE = 1;					//OSCE=1-----使得fREF = fOSC/(REFDIV+1)---rookie_lxp
	CPMUOSC_OSCFILT = 4;            // Oscillator Filter = (VCOCLK/OSCCLK)/2 = (32MHz/4MHz)/2 = 4
	while((!CPMUFLG_UPOSC) && (!CPMUFLG_LOCK));//Fpll = Fvco / (PostDiv+1)---rookie_lxp
											// wait for ext osc to stabilize and pll to lock
	CPMUFLG = 0xFF;							// clear CMPMU int flags - not needed but good practice
	
#ifdef	WDT_ON	
	//CPMUCOP = 0x04;//0x07;				//WATCHDOG	2013-6-20 21:49:45
	CPMUCOP = 0x07;
	CPMUCLKS = 0x85; 			//WATCHDOG	2013-6-20 21:49:42//P240
#endif
}

/************************************************
*name:			Cofigure_Pwm_Io
*describe:		PWM module  initialization
*parameter		no
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void Cofigure_Pwm_Io(void)
{
	PTPRRL = 0x0F; // Route PWM3:0 from PP4:0 to PS7:4
	PWMCAE = 0x0C; //Set center alignment for channels 0 - 3
  	PWMPOL = 0x0A; //Set polarity for channels 0 - 3
  //The above register setting should match the following truth table  .
  //PWM0: Center Aligned=0, Polarity=0
  //PWM1: Center Aligned=0, Polarity=1
  //PWM2: Center Aligned=1, Polarity=0
  //PWM3: Center Aligned=1, Polarity=1
  /*************************/  
  
  	PWMPER0 = 0xFF; //Set chan 0 period length to the maximum.
  	PWMPER1 = 0xFF; //Set chan 1 period length to the maximum.
  	PWMPER2 = 0xFF; //Set chan 2 period length to the maximum.
  	PWMPER3 = 0xFF; //Set chan 3 period length to the maximum.

  	PWMDTY0 = 0x80; //Set chan 0 duty cycle to half the maximum, 50%.
  	PWMDTY1 = 0x80; //Set chan 1 duty cycle to half the maximum, 50%.
  	PWMDTY2 = 0x80; //Set chan 2 duty cycle to half the maximum, 50%.
  	PWMDTY3 = 0x80; //Set chan 3 duty cycle to half the maximum, 50%.

  	PWME = 0x0F;    //Enable PWM channels 0 - 3
}