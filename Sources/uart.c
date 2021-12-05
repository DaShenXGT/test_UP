#include <hidef.h>	
#include <mc9s12hy64.h>
#include "board.h"
#include "app.h"
#include "cpu.h"
#include "led.h"
#include "display.h"
//#include "motor.h"
//#include "flash.h"
#include "time.h"
#include "uart.h"
#include "vehicle.h"
#include "atd.h"


siocirqueue RTbuf_UART0;

void sensor_inf_tx(void)//发送传感器信息
{
   RTbuf_UART0.T_recvcrc = 0xffff;
   RTbuf_UART0.T_head = 0;
   

   
   switch(RTbuf_UART0.T_cmd) 
   {
      case 0:
      case 1:
        RTbuf_UART0.T_cmd = 2;
        break;    
      case 2: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(1,0,1);//数据长度为1 
				 Tbuf_putchar(0x03,0,1);//慕尼黑设备类型为3
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 3: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(1,0,1);//数据长度为1 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(munich_motor[3].new_motor_angle,0,1);//气压表   指针角度  0～90
				   // Tbuf_putchar(88,0,1);//气压表   指针角度  0～90
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 4: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(1,0,1);//数据长度为1 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(munich_motor[4].new_motor_angle,0,1);//油位表  指针角度  0～90
				    //Tbuf_putchar(55,0,1);  
				 
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 5: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(1,0,1);//数据长度为1 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(mnuich_atd[2].new_convert_value,0,1);//水温  角度
				    //Tbuf_putchar(66,0,1);
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 6: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(1,0,1);//数据长度为1 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(mnuich_atd[3].new_convert_value,0,1);//油温   角度
				    //Tbuf_putchar(77,0,1);//油温   角度
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 7: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(2,0,1);//数据长度为2 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(((engineSpeedParams.new_speed>>8) &0xff),0,1);//转速
				    Tbuf_putchar((engineSpeedParams.new_speed & 0xff),0,1);//转速
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 8: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(4,0,1);//数据长度为4 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(((working_time_h >>8) &0xff),0,1);//小时计
				    Tbuf_putchar((working_time_h & 0xff),0,1);//小时计
				    Tbuf_putchar(((working_time_l>>8) &0xff),0,1);//小时计
				    Tbuf_putchar((working_time_l & 0xff),0,1);//小时计
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0
				 } 
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
      }
      case 9: 
      {
         Tbuf_putchar(FRAME_START,0,1);
			 	 Tbuf_putchar(RTbuf_UART0.T_cmd,0,1);
				 Tbuf_putchar(4,0,1);//数据长度为4 
				 if(systemState.b.ign_on) 
				 {
				    Tbuf_putchar(((LED_OnOffBitsMap >>24) &0xff),0,1);//LED状态
				    Tbuf_putchar(((LED_OnOffBitsMap >>16) &0xff),0,1);//LED状态
				    Tbuf_putchar(((LED_OnOffBitsMap >>8) &0xff),0,1);//LED状态
				    Tbuf_putchar((LED_OnOffBitsMap & 0xff),0,1);//LED状态   
				 }  
				 else 
				 {
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0 
				    Tbuf_putchar(0,0,1);//关闭电锁后发送数据都是0   
				 }
				 Tbuf_putchar((RTbuf_UART0.T_recvcrc & 0xff),0,0);
			   Tbuf_putchar(((RTbuf_UART0.T_recvcrc>>8) & 0xff),0,0);
			   RTbuf_UART0.time_delay_count =2;
         break;
       }
       default: 
       {
         RTbuf_UART0.T_cmd = 2;
         RTbuf_UART0.time_delay_count =2;
         break;  
       }
   }
   if(RTbuf_UART0.T_head >0) 
   {
     RTbuf_UART0.T_tail = 0;
     SCICR2_TCIE = 1;
   } 
}
static void Tbuf_putchar(unsigned char x,unsigned char channel,unsigned char crc)
{
	  if(channel==0)
		{
				if(crc)
				{
						sendcrc0(x);
				}
				/*if(RTbuf_UART0.T_disabled)
				{	 
					   RTbuf_UART0.T_buf[RTbuf_UART0.T_head]=x;
						 RTbuf_UART0.T_head++;
						 RTbuf_UART0.T_head %= TBUF_SIZE;
					   RTbuf_UART0.T_disabled=0;
					   SCICR2_TIE = 1;
				}
				else */
				{
						 RTbuf_UART0.T_buf[RTbuf_UART0.T_head]=x;
						 RTbuf_UART0.T_head++;
						 RTbuf_UART0.T_head %= TBUF_SIZE;
				}
	  }
}



/************************************************
*
*************************************************/
void config_uart(void)
{

	SCIBD = BAUD9600;									// 19.2 kHz
	SCICR1 &= ~SCICR1_M_MASK & ~SCICR1_PE_MASK;		// 8,N,1
	SCICR2 |= SCICR2_TE_MASK | SCICR2_RE_MASK;		// SCI Tx/Rx enabled
	SCICR2_RIE = 1;	
//	SCICR2_TCIE = 1;								// enable Rx Data Ready interrupt
 /* if(run_mode == 0xa5) 
  {
     PTR_PTR7 = 0;//
     PTH_PTH4 = 0;
  }                                        //liu~gps
  else 
  {
     PTR_PTR7 = 1;//
     PTH_PTH4 = 0;  
  }*/
}
/************************************************
*
*************************************************/



void Rbuf_init(siocirqueue *RTbuf)
{
		RTbuf->R_head=0;
		RTbuf->R_tail=0;
		RTbuf->R_step=0;
		RTbuf->R_overflow=0;
}
void Tbuf_init(siocirqueue *RTbuf)
{
		RTbuf->T_head=0;
		RTbuf->T_tail=0;
		RTbuf->T_count=0;
		RTbuf->T_cmd=2;
		RTbuf->time_delay_count=3;
		RTbuf->tx_nub=0;
		RTbuf->T_disabled=1;
		
}

void Uart0_Proc(void)
{
  unsigned char m_tmpdata; 
	if (RTbuf_UART0.R_head != RTbuf_UART0.R_tail)	                 
	{
		m_tmpdata = RTbuf_UART0.R_buf[RTbuf_UART0.R_tail];
		RTbuf_UART0.R_tail ++;
		RTbuf_UART0.R_tail %= RBUF_SIZE;	
		switch(RTbuf_UART0.R_step)
		{
			  case 0:
						if (m_tmpdata == FRAME_START)
						{					
							RTbuf_UART0.R_step ++;
							RTbuf_UART0.R_recvcrc =0xffff;
							rxcrc0(m_tmpdata);
						}
						break;					
		    case 1:
						RTbuf_UART0.R_step ++;	
						rxcrc0(m_tmpdata);
						RTbuf_UART0.R_cmd = m_tmpdata;
						break;	
			  case 2:	
						{
								RTbuf_UART0.R_len = m_tmpdata;
								if (RTbuf_UART0.R_len <=40)
								{
										RTbuf_UART0.R_oldlen = RTbuf_UART0.R_len;
										RTbuf_UART0.R_step ++;
										rxcrc0(m_tmpdata);
								}
								else
								{
									  RTbuf_UART0.R_step = 0;
								}
						}
						break;
			  case 3:
						{
							if (RTbuf_UART0.R_oldlen >0)
							{
								RTbuf_UART0.R_data_buf[RTbuf_UART0.R_len - RTbuf_UART0.R_oldlen] = m_tmpdata;
								RTbuf_UART0.R_oldlen --;
								rxcrc0(m_tmpdata);
							}
							else
							{
								if((RTbuf_UART0.R_recvcrc & 0xff)==m_tmpdata)
								{
									 RTbuf_UART0.R_step ++;
								}
								else
								{
									 RTbuf_UART0.R_step =0;
								}
							}
						}
						break;
		   case 4:
		        if(((RTbuf_UART0.R_recvcrc>>8) & 0xff)==m_tmpdata)
						{
								ProcRx0();
						}	
						RTbuf_UART0.R_step =0;
						break;
		}
	}
}
void ProcRx0(void)
{
  	RTbuf_UART0.T_recvcrc = 0xffff;
  	if(RTbuf_UART0.R_cmd == RTbuf_UART0.T_cmd) 
  	{
  	    RTbuf_UART0.T_cmd++;
  	    RTbuf_UART0.tx_nub = 0;
  	    if(RTbuf_UART0.T_cmd>9) 
  	    {
  	        RTbuf_UART0.T_cmd = 2;
  	        RTbuf_UART0.time_delay_count = 30;//延时30秒 
  	    } 
  	    else 
  	    {
  	        RTbuf_UART0.time_delay_count = 5;//延时30秒 
  	    }
  	}
}
void  sendcrc0(unsigned char  crcbuf) 
{ 
   unsigned char   i; 
   RTbuf_UART0.T_recvcrc=RTbuf_UART0.T_recvcrc^crcbuf; 
   for(i=0;i<8;i++) 
	{ 
		unsigned char  TT; 
		TT=RTbuf_UART0.T_recvcrc&1; 
		RTbuf_UART0.T_recvcrc=RTbuf_UART0.T_recvcrc>>1; 
		RTbuf_UART0.T_recvcrc=RTbuf_UART0.T_recvcrc&0x7fff; 
		if(TT==1) 
		   RTbuf_UART0.T_recvcrc=RTbuf_UART0.T_recvcrc^0xa001; 
		RTbuf_UART0.T_recvcrc=RTbuf_UART0.T_recvcrc&0xffff; 
	} 
}
void  rxcrc0(unsigned char  crcbuf) 
{ 
   unsigned char   i; 
   RTbuf_UART0.R_recvcrc=RTbuf_UART0.R_recvcrc^crcbuf; 
   for(i=0;i<8;i++) 
	{ 
		unsigned char  TT; 
		TT=RTbuf_UART0.R_recvcrc&1; 
		RTbuf_UART0.R_recvcrc=RTbuf_UART0.R_recvcrc>>1; 
		RTbuf_UART0.R_recvcrc=RTbuf_UART0.R_recvcrc&0x7fff; 
		if(TT==1) 
		   RTbuf_UART0.R_recvcrc=RTbuf_UART0.R_recvcrc^0xa001; 
		RTbuf_UART0.R_recvcrc=RTbuf_UART0.R_recvcrc&0xffff; 
	} 
}

#pragma CODE_SEG NON_BANKED
/************************************************
*
*************************************************/
__interrupt VectorNumber_Vsci void _isr_uart_Interrupt(void)
{
	char data,flag;
	flag = SCISR1;
	if(flag & 0x20)
	{
		data = SCIDRL;
		RTbuf_UART0.R_buf[RTbuf_UART0.R_head] = data;
	  RTbuf_UART0.R_head++;
	  RTbuf_UART0.R_head %=RBUF_SIZE;
	} 
	else	if(flag & 0x40)
	{	
//	   	  busy_bit = 0;	
		    data = RTbuf_UART0.T_buf[RTbuf_UART0.T_tail];
		    RTbuf_UART0.T_tail++;
		    //RTbuf_UART0.T_tail %=TBUF_SIZE;
		    if (RTbuf_UART0.T_tail>=RTbuf_UART0.T_head)
  		  {
  				RTbuf_UART0.T_disabled=1;
  			  SCICR2_TCIE = 0;//禁止中断 ;
  		  }
  		  SCIDRL  =  data;
	}

}   
#pragma CODE_SEG DEFAULT
