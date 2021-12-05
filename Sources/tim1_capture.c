#include <mc9s12hy64.h>
#include "tim1_capture.h"
#include "board.h"

extern unsigned char CarSpeedDisFlag;
unsigned char PluseFlag = 0;

/*---------------------------------------------------------------------------------
@ Variables
---------------------------------------------------------------------------------*/
//脉冲边沿采样数目
unsigned int Pulse_EdgeSampleNum = PULSE_EDGE_MAX_NUM;
//脉冲宽度
volatile unsigned long pulse_width_difference[PULSE_EDGE_MAX_NUM - 1];
//车速变量
volatile struct loader car_speed_parameter =
{
  {0},
  {0},
  0,
  false,
  0,
  0
};

/*---------------------------------------------------------------------------------
@ Local Function
---------------------------------------------------------------------------------*/
//计算脉冲宽度
static unsigned long Pulse_PeriodCalculate(unsigned int nr);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* 函数名称:void tim1_ioc1_6_input_capture_init()  ;
*
* 函数功能描述:捕捉IOC1_6设置 ;
*              
* 输入参数:none ;
*
* 返回数据:none;
*
* 注意事项:频率测量范围;
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tim1_ioc1_6_input_capture_init()
{
  TIM1_TIOS_IOS6=0;//input capture

  TIM1_TCTL3_EDG6B=0;
  TIM1_TCTL3_EDG6A=1;//capture on rising edges only

  TIM1_TSCR2_PR2=0;
  TIM1_TSCR2_PR1=1;
  TIM1_TSCR2_PR0=1;//32M/8=4M
  TIM1_TSCR2_TOI=1;//timer overflow interrupt enable

  TIM1_TIE_C6I=1;//input capture interrupt enable

  TIM1_TSCR1_TEN=1;//timer enable
}

/******************************************************************************
@ Func:  使能TIM1 CH2捕获中断
******************************************************************************/
void restart_ioc1_6_capture()
{
  TIM1_TIE_C6I = 1;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* 函数名称:void bubble_sort(unsigned int length,unsigned int data[])  ;
*
* 函数功能描述:冒泡算法 ;
*              
* 输入参数:unsigned int length数据的个数 ,unsigned int data[]要排序的数组 ;
*
* 返回数据:none;
*
* 注意事项:none;
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void bubble_sort(unsigned int length,unsigned long data[]) 
{
  unsigned int i;
  unsigned int j;
  unsigned long m;
  unsigned int change=1; 

  for(i=1;i<length && change==1 ;i++) 
  {
    change=0;
    
    for(j=0;j<length-i;j++) 
    {
      if(data[j]>data[j+1]) 
      {
        m=data[j+1];
        data[j+1]=data[j];
        data[j]=m;
        change=1;
      }
    }
  }
}

/******************************************************************************
@ Func:  计算两脉冲边沿间隔
@ Param: nr--脉冲边沿序号
@ Note:  序号须大于0
******************************************************************************/
static unsigned long Pulse_PeriodCalculate(unsigned int nr)
{
  if(nr)
  {
    return ((car_speed_parameter.pulse_overflow[nr] * (unsigned long)65536 + car_speed_parameter.pulse_width[nr])
            - car_speed_parameter.pulse_width[nr - 1]);
  }
  
  return 0;
}

/******************************************************************************
@ Func: 车速计算
******************************************************************************/
void calculating_speed() 
{
  unsigned char i = 0;
  unsigned char pulseNum = 0;
  unsigned long tcnt_count = 0;
  static unsigned long ulFerqNew = 0;
  static unsigned long ulFerqOld	= 0;

  if(car_speed_parameter.pulse_sample_finish) 
  {
  	car_speed_parameter.pulse_sample_finish = false;
  	
  	//通过脉冲边沿时间戳计算脉冲宽度
  	for(i = 1; i < Pulse_EdgeSampleNum; ++i)
  	{
  	  pulse_width_difference[i - 1] = Pulse_PeriodCalculate(i);
  	}
  	
  	//脉冲数目为边沿数减1
  	pulseNum = Pulse_EdgeSampleNum - 1;
  	
  	//脉冲宽度数组排序
  	bubble_sort(pulseNum, pulse_width_difference);

    //记录上一次脉冲频率	
  	ulFerqOld = ulFerqNew;
  	
  	//若脉冲数目大于5个，则去掉头部2个和尾部2个
  	if(pulseNum >= 5)
  	{
  		for(i = 2; i < (pulseNum - 2); ++i)
  		{
  		  tcnt_count += pulse_width_difference[i];
  		}
  		
  		ulFerqNew = ((unsigned long)4000000 * (pulseNum - 4) * 10 / tcnt_count);
  	}
  	//脉冲数目小于5个，则全部参与计算，不滤波
  	else
  	{
  	  for(i = 0; i < pulseNum; ++i)
  	  {
  	    tcnt_count += pulse_width_difference[i];
  	  }
  	  
    	ulFerqNew = ((unsigned long)4000000 * pulseNum * 10 / tcnt_count);
  	}
  	
  	car_speed_parameter.pulse_frequency = (ulFerqOld + ulFerqNew) / 2 + 1; //+1是为了修正误差
  	
    //此处按照20km/h 对应108.24HZ来计算，此时的车速正好放大了10倍
    //car_speed_parameter.loader_speed=car_speed_parameter.pulse_frequency*20/108.24;
    car_speed_parameter.loader_speed=(car_speed_parameter.pulse_frequency*2000/SPEEDRATIO);

    //大于20公里，每小时加6%  加50是为了四舍五入 By周文熙 20200730
    //在18.5与20之间加入缓冲，以确保车速显示不会实际增加，显示减小 By周文熙 20200108
    if(car_speed_parameter.loader_speed>=200)  
    {
      car_speed_parameter.loader_speed+=((car_speed_parameter.loader_speed*6+50)/100);
    } 
    else if(car_speed_parameter.loader_speed >= 185)
    {
      car_speed_parameter.loader_speed = 200 + ((car_speed_parameter.loader_speed - 185) * 81 + 50) / 100;
    }   
    else 
    {
      car_speed_parameter.loader_speed+=((car_speed_parameter.loader_speed*8+50)/100);
    }

    //添加最大车速显示限制--rookie_lxp
    if (car_speed_parameter.loader_speed > 599)
    {
      car_speed_parameter.loader_speed = 600;
    }
  
    //重新打开捕获中断
    restart_ioc1_6_capture();
  }
  //根据是否存在车速置车速显示标志 By周文熙 20200108

  
  if(PluseFlag == 0)
    CarSpeedDisFlag = 0;  
  else {
    
  if(car_speed_parameter.loader_speed < 2)
    CarSpeedDisFlag = 0;
  else
    CarSpeedDisFlag = 1;      
  //无法接收到车速脉冲
  PluseFlag--;   
  }
}

#pragma CODE_SEG NON_BANKED

/******************************************************************************
@ Func: TIM1 CH6捕获中断处理
******************************************************************************/
__interrupt VectorNumber_Vtim1ch6 void _isr_pulse_input_Interrupt (void)
{
  static unsigned long periodCheck = 0;
  static unsigned char i = 0;
  
  //记录时间戳
	car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] = TIM1_TC6;
	
	//若检测到定时器溢出标志置位，则说明在记录时间戳的过程中定时器发生了溢出，而溢出中断尚未执行。
	//此时，需要根据记录的时间戳判断该时间戳是在溢出标志置位之前还是置位之后。
	//若是溢出标志置位之后记录的时间戳，将时间戳置为定时器最大值65535，误差为：65535-捕获中断标志置位时的定时器计数值
	//若是溢出标志置位之前记录的时间戳，合法，不做处理。
	if(TIM1_TFLG2 & (1 << 7))
	{
	  if((car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] <= 10)
	     && (car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] >= 0))
	  {
	    car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] = 65535;
	  }
	}
	
	//开始捕获时清溢出计数
	if(0 == car_speed_parameter.signal_pulse_count)
	{
	  for(i = 0; i < PULSE_EDGE_MAX_NUM; ++i)
	  {
	    car_speed_parameter.pulse_overflow[i] = 0;
	  }
	}
	
	if(car_speed_parameter.signal_pulse_count)
	{
  	
  	//计算当前脉冲宽度
	  periodCheck = Pulse_PeriodCalculate(car_speed_parameter.signal_pulse_count);
	  
	  //≥61Hz
	  if(periodCheck <= 65536)
	  {
	    Pulse_EdgeSampleNum = 50;
	  }
	  //31Hz~61Hz
	  else if(periodCheck <= (65536 * 2))
	  {
	    if(car_speed_parameter.signal_pulse_count > 30)
	    {
	      Pulse_EdgeSampleNum = car_speed_parameter.signal_pulse_count;
	    }
	    else
	    {
  	    Pulse_EdgeSampleNum = 30;
	    }
	  }
	  //15Hz~31Hz
	  else if(periodCheck <= (65536 * 4))
	  {
	    if(car_speed_parameter.signal_pulse_count > 15)
	    {
	      Pulse_EdgeSampleNum = car_speed_parameter.signal_pulse_count;
	    }
	    else
	    {
  	    Pulse_EdgeSampleNum = 15;
	    }
	  }
	  //8Hz~15Hz
	  else if(periodCheck <= (65536 * 8))
	  {
	    if(car_speed_parameter.signal_pulse_count > 8)
	    {
	      Pulse_EdgeSampleNum = car_speed_parameter.signal_pulse_count;
	    }
	    else
	    {
  	    Pulse_EdgeSampleNum = 8;
	    }
	  }
	  //4Hz~8Hz
	  else if(periodCheck <= (65536 * 16))
	  {
	    if(car_speed_parameter.signal_pulse_count > 8)
	    {
	      Pulse_EdgeSampleNum = car_speed_parameter.signal_pulse_count;
	    }
	    else
	    {
  	    Pulse_EdgeSampleNum = 8;
	    }
	  }
	  //4Hz以下
	  else
	  {
	    if(car_speed_parameter.signal_pulse_count > 4)
	    {
	      Pulse_EdgeSampleNum = car_speed_parameter.signal_pulse_count;
	    }
	    else
	    {
  	    Pulse_EdgeSampleNum = 4;
	    }
	  }
	}
	
	car_speed_parameter.signal_pulse_count++;

	if(car_speed_parameter.signal_pulse_count >= Pulse_EdgeSampleNum)
	{
		TIM1_TIE_C6I = 0; 
		car_speed_parameter.signal_pulse_count = 0;
		car_speed_parameter.pulse_sample_finish = true;
	}


  PluseFlag = 2;
  //写1清零标志位   C6F	
  TIM1_TFLG1 |= 0X40; 
}

/**************************************************************************
@ Func:  TIM1溢出中断处理
@ Brief: 记录某个脉冲到下一个脉冲之间定时器溢出次数
         定时器溢出122次——0.5Hz
         定时器溢出61次——1Hz
         定时器溢出1次——61Hz
**************************************************************************/
__interrupt VectorNumber_Vtim1ovf void time_overflow_Interrupt (void)
{
  //clear the tof
  TIM1_TFLG2_TOF = 1;   
  
  car_speed_parameter.pulse_overflow[car_speed_parameter.signal_pulse_count]++;
  
  if(car_speed_parameter.signal_pulse_count > 0)
  {
    if(car_speed_parameter.pulse_overflow[car_speed_parameter.signal_pulse_count] > 122)
    {
      car_speed_parameter.signal_pulse_count = 0;
      car_speed_parameter.loader_speed = 0; 
    }
  }
}

#pragma CODE_SEG DEFAULT