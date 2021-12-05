#include <mc9s12hy64.h>
#include "tim1_capture.h"
#include "board.h"

extern unsigned char CarSpeedDisFlag;
unsigned char PluseFlag = 0;

/*---------------------------------------------------------------------------------
@ Variables
---------------------------------------------------------------------------------*/
//������ز�����Ŀ
unsigned int Pulse_EdgeSampleNum = PULSE_EDGE_MAX_NUM;
//������
volatile unsigned long pulse_width_difference[PULSE_EDGE_MAX_NUM - 1];
//���ٱ���
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
//����������
static unsigned long Pulse_PeriodCalculate(unsigned int nr);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* ��������:void tim1_ioc1_6_input_capture_init()  ;
*
* ������������:��׽IOC1_6���� ;
*              
* �������:none ;
*
* ��������:none;
*
* ע������:Ƶ�ʲ�����Χ;
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
@ Func:  ʹ��TIM1 CH2�����ж�
******************************************************************************/
void restart_ioc1_6_capture()
{
  TIM1_TIE_C6I = 1;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* ��������:void bubble_sort(unsigned int length,unsigned int data[])  ;
*
* ������������:ð���㷨 ;
*              
* �������:unsigned int length���ݵĸ��� ,unsigned int data[]Ҫ��������� ;
*
* ��������:none;
*
* ע������:none;
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
@ Func:  ������������ؼ��
@ Param: nr--����������
@ Note:  ��������0
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
@ Func: ���ټ���
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
  	
  	//ͨ���������ʱ�������������
  	for(i = 1; i < Pulse_EdgeSampleNum; ++i)
  	{
  	  pulse_width_difference[i - 1] = Pulse_PeriodCalculate(i);
  	}
  	
  	//������ĿΪ��������1
  	pulseNum = Pulse_EdgeSampleNum - 1;
  	
  	//��������������
  	bubble_sort(pulseNum, pulse_width_difference);

    //��¼��һ������Ƶ��	
  	ulFerqOld = ulFerqNew;
  	
  	//��������Ŀ����5������ȥ��ͷ��2����β��2��
  	if(pulseNum >= 5)
  	{
  		for(i = 2; i < (pulseNum - 2); ++i)
  		{
  		  tcnt_count += pulse_width_difference[i];
  		}
  		
  		ulFerqNew = ((unsigned long)4000000 * (pulseNum - 4) * 10 / tcnt_count);
  	}
  	//������ĿС��5������ȫ��������㣬���˲�
  	else
  	{
  	  for(i = 0; i < pulseNum; ++i)
  	  {
  	    tcnt_count += pulse_width_difference[i];
  	  }
  	  
    	ulFerqNew = ((unsigned long)4000000 * pulseNum * 10 / tcnt_count);
  	}
  	
  	car_speed_parameter.pulse_frequency = (ulFerqOld + ulFerqNew) / 2 + 1; //+1��Ϊ���������
  	
    //�˴�����20km/h ��Ӧ108.24HZ�����㣬��ʱ�ĳ������÷Ŵ���10��
    //car_speed_parameter.loader_speed=car_speed_parameter.pulse_frequency*20/108.24;
    car_speed_parameter.loader_speed=(car_speed_parameter.pulse_frequency*2000/SPEEDRATIO);

    //����20���ÿСʱ��6%  ��50��Ϊ���������� By������ 20200730
    //��18.5��20֮����뻺�壬��ȷ��������ʾ����ʵ�����ӣ���ʾ��С By������ 20200108
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

    //����������ʾ����--rookie_lxp
    if (car_speed_parameter.loader_speed > 599)
    {
      car_speed_parameter.loader_speed = 600;
    }
  
    //���´򿪲����ж�
    restart_ioc1_6_capture();
  }
  //�����Ƿ���ڳ����ó�����ʾ��־ By������ 20200108

  
  if(PluseFlag == 0)
    CarSpeedDisFlag = 0;  
  else {
    
  if(car_speed_parameter.loader_speed < 2)
    CarSpeedDisFlag = 0;
  else
    CarSpeedDisFlag = 1;      
  //�޷����յ���������
  PluseFlag--;   
  }
}

#pragma CODE_SEG NON_BANKED

/******************************************************************************
@ Func: TIM1 CH6�����жϴ���
******************************************************************************/
__interrupt VectorNumber_Vtim1ch6 void _isr_pulse_input_Interrupt (void)
{
  static unsigned long periodCheck = 0;
  static unsigned char i = 0;
  
  //��¼ʱ���
	car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] = TIM1_TC6;
	
	//����⵽��ʱ�������־��λ����˵���ڼ�¼ʱ����Ĺ����ж�ʱ�������������������ж���δִ�С�
	//��ʱ����Ҫ���ݼ�¼��ʱ����жϸ�ʱ������������־��λ֮ǰ������λ֮��
	//���������־��λ֮���¼��ʱ�������ʱ�����Ϊ��ʱ�����ֵ65535�����Ϊ��65535-�����жϱ�־��λʱ�Ķ�ʱ������ֵ
	//���������־��λ֮ǰ��¼��ʱ������Ϸ�����������
	if(TIM1_TFLG2 & (1 << 7))
	{
	  if((car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] <= 10)
	     && (car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] >= 0))
	  {
	    car_speed_parameter.pulse_width[car_speed_parameter.signal_pulse_count] = 65535;
	  }
	}
	
	//��ʼ����ʱ���������
	if(0 == car_speed_parameter.signal_pulse_count)
	{
	  for(i = 0; i < PULSE_EDGE_MAX_NUM; ++i)
	  {
	    car_speed_parameter.pulse_overflow[i] = 0;
	  }
	}
	
	if(car_speed_parameter.signal_pulse_count)
	{
  	
  	//���㵱ǰ������
	  periodCheck = Pulse_PeriodCalculate(car_speed_parameter.signal_pulse_count);
	  
	  //��61Hz
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
	  //4Hz����
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
  //д1�����־λ   C6F	
  TIM1_TFLG1 |= 0X40; 
}

/**************************************************************************
@ Func:  TIM1����жϴ���
@ Brief: ��¼ĳ�����嵽��һ������֮�䶨ʱ���������
         ��ʱ�����122�Ρ���0.5Hz
         ��ʱ�����61�Ρ���1Hz
         ��ʱ�����1�Ρ���61Hz
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