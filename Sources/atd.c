#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "board.h"
#include "vehicle.h"
#include "atd.h"

vehicle_ad		mnuich_atd[ADC_MAX_CHANNEL/*+1*/];
Uint8_t			atd_channel_id = 4;
unsigned long	adc_temp;
//Uint16_t		vcc_atd[3];
Uint8_t			atd_delay_time;
extern	queue_list	systemTask;
extern vehicle_motor munich_motor[5];


/*******************************************************************************
@ Func: ATD初始化
*******************************************************************************/
static void ATD_Init(void)
{
	PER1AD 	= 0x00;					// Port AD Pull Enable  
	PT1AD 	= 0x00;					// Port AD I/O Register
	DDR1AD 	= 0x00;					// Port AD Data Direction Register	
	PIE1AD	= 0x00;					// disable Port AD interrupts on PT1AD[7:0]
	PIF1AD	= 0x00;					// clear spurious interrupts
	ATDDIENL= 0xFF;					// enable PT1AD[7:0] as AD input

  DDRP |= (1 << 3);
  PTP &= ~(1 << 3);
}
/************************************************
*name:			average_data
*describe:		initialize paramter of atd
*parameter		no				
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void	Init_Atd_Paramter(void)
{
	Uint8_t	atd_mi;
	
	for(atd_mi = 0;atd_mi < ADC_MAX_CHANNEL;atd_mi++)
	{
		mnuich_atd[atd_mi].Atd_id = atd_mi;
		mnuich_atd[atd_mi].samp_time = 0;
		mnuich_atd[atd_mi].old_adc_value = 0xffff;
		mnuich_atd[atd_mi].new_adc_value = 0xffff;
		mnuich_atd[atd_mi].samp_value[0] = 0xffff;
		mnuich_atd[atd_mi].samp_value[1] = 0xffff;
		mnuich_atd[atd_mi].samp_value[2] = 0xffff;
		mnuich_atd[atd_mi].samp_value[3] = 0xffff;
		mnuich_atd[atd_mi].samp_value[4] = 0xffff;
//		mnuich_atd[atd_mi].vcc_atd[0] =0;
//		mnuich_atd[atd_mi].vcc_atd[1] =0;
		mnuich_atd[atd_mi].old_convert_value = 0xffff;
		mnuich_atd[atd_mi].old_convert_value = 0xffff;
	}
	
	mnuich_atd[4].old_adc_value = 1000;
    mnuich_atd[4].new_adc_value = 1000;
    
	ATD_Init();
}

/************************************************
*name:			average_data
*describe:		initialize paramter of atd
*parameter		no				
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
unsigned int	average_data(Uint16_t * source_point,Uint8_t data_time)
{
	Uint16_t	data_sum = 0;
	Uint16_t	reback_data = 0;
	Uint8_t		mi;
	
	for(mi = 0;mi < data_time;mi ++ )
		data_sum += * source_point++;
	if(data_sum >= data_time)
		reback_data = data_sum/data_time;
	else
		reback_data =0;
//	if(reback_data > 2000)
//		mi++;
	return reback_data;
}
/************************************************
*name:			sort
*describe:		Sort Data
*parameter		data_arrey---data
*				length-------the length of data			
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void	sort(Uint16_t data_arrey[],Uint8_t length)
{
	Uint8_t mi,mj;
	Uint16_t temp;
	
	for(mi = 0;mi < length-1;mi ++)
	{
		for(mj = mi+1;mj < length;mj ++)
		{
			if(data_arrey[mi] < data_arrey[mj]) 
   		{
    		temp = data_arrey[mi];
    		data_arrey[mi] = data_arrey[mj];
    		data_arrey[mj] = temp; 
   		}
		}
	}
}
/************************************************
*name:			set_atd_ch
*describe:		set atd channel
*parameter		adc_id----channel id
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang
*************************************************/
static	void set_atd_ch(Uint8_t  adc_id)
{
	ATDDIENL = (0x01<<adc_id);
	ATDCTL1 = 0xb8;//0x20;					// 10-bit results
	ATDCTL1 |= adc_id;						//select atd channel
	ATDCTL3 = 0xA0;							//	right justify results, 4 conversions per sequence
	ATDCTL4 = 0x03;							// fATDCLK = 2.67MHz, 4 clks per sample
	ATDCTL5 = 0x00;							// single conversion
	ATDCTL5 |= adc_id;
	ATDSTAT0_SCF = 0x01;					// clear conversion complete flag
}
/************************************************
*name:			adc_convert
*describe:		atd convert 
*parameter		atd_id-----	channel id
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
static Uint16_t adc_convert(Uint8_t atd_id) 
{
	Uint8_t		adc_mi;
	Uint16_t	reback;
	Uint16_t	adc_buff[ADC_CONVERT_TIME];
	Uint16_t    *adc_point;
		
	reback = 0xffff;
	
	if(atd_id > ADC_MAX_CHANNEL)
	{
		return reback;
	}
	
	for(adc_mi = 0; adc_mi < ADC_CONVERT_TIME; adc_mi++)
	{
		set_atd_ch(atd_id);
		atd_delay_time = 0;
		while((!ATDSTAT0_SCF)&&(atd_delay_time < ADC_HOLD_TIME));
		
		if(atd_channel_id == 4)
		{
			adc_buff[adc_mi] = (ATDDR0+ATDDR1+ATDDR2+ATDDR3)/4;
		}
		else
		{
			//adc_buff[adc_mi] = (ATDDR0+ATDDR1+ATDDR2+ATDDR3)/4;//*(adc_point + atd_id);
			adc_temp = (ATDDR0+ATDDR1+ATDDR2+ATDDR3)/4;
			/*if(mnuich_atd[ATD_CHANNEL_AVDD].new_adc_value != 0)
			{
			  adc_temp *= 1023;
			  adc_temp /= mnuich_atd[ATD_CHANNEL_AVDD].new_adc_value;
			} */
			adc_buff[adc_mi] = adc_temp;
			/*if(mnuich_atd[4].supply_value != 0)
			{	
				adc_temp = adc_buff[adc_mi];
				adc_temp = adc_temp*mnuich_atd[4].old_adc_value;///mnuich_atd[4].new_adc_value;
				adc_temp = adc_temp/mnuich_atd[4].new_adc_value;
				adc_buff[adc_mi] = adc_temp;
			} */
		}
	}
	
	sort(adc_buff,ADC_CONVERT_TIME);
	adc_point = adc_buff;
	if(ADC_CONVERT_TIME > 3)
		reback = average_data((adc_point+1),ADC_CONVERT_TIME-2);
	else
		reback = average_data(adc_point,ADC_CONVERT_TIME);
	
	return reback;
}
/************************************************
*name:			atd_handle
*describe:		atd handle 
*parameter		no				
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void	atd_handle(void)
{
	static Uint8_t test;

	if(systemTask.b.Atd_active)
	{
	  mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time] = adc_convert(atd_channel_id);
	}
		
	if((mnuich_atd[atd_channel_id].samp_time == 0)&&(mnuich_atd[atd_channel_id].samp_value[1] == 0xffff))
	{

		mnuich_atd[atd_channel_id].samp_value[1] = mnuich_atd[atd_channel_id].samp_value[0];
		mnuich_atd[atd_channel_id].samp_value[2] = mnuich_atd[atd_channel_id].samp_value[0];
		mnuich_atd[atd_channel_id].samp_value[3] = mnuich_atd[atd_channel_id].samp_value[0];
		mnuich_atd[atd_channel_id].samp_value[4] = mnuich_atd[atd_channel_id].samp_value[0];
	}

  //处理AVDD时，将5个值都赋为同一个值，是为了跟得上该电压的波动？？？
	if(atd_channel_id == 4)
	{	
		mnuich_atd[atd_channel_id].samp_value[0] = mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time];
		mnuich_atd[atd_channel_id].samp_value[1] = mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time];
		mnuich_atd[atd_channel_id].samp_value[2] = mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time];
		mnuich_atd[atd_channel_id].samp_value[3] = mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time];
		mnuich_atd[atd_channel_id].samp_value[4] = mnuich_atd[atd_channel_id].samp_value[mnuich_atd[atd_channel_id].samp_time];
	}

	mnuich_atd[atd_channel_id].new_adc_value = (mnuich_atd[atd_channel_id].samp_value[0] + mnuich_atd[atd_channel_id].samp_value[1] 
	                                            + mnuich_atd[atd_channel_id].samp_value[2] + mnuich_atd[atd_channel_id].samp_value[3]
	                                            + mnuich_atd[atd_channel_id].samp_value[4]) / 5;

	if(mnuich_atd[atd_channel_id].samp_time >= 4)
	{
	  mnuich_atd[atd_channel_id].samp_time = 0;
	}	
	else
	{
	  mnuich_atd[atd_channel_id].samp_time++;
	}
			
	systemTask.b.Atd_active = FALSE;
	
	ATDDIENL = 0;	
}

