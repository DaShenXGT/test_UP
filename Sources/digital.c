#include <hidef.h>	
#include <mc9s12hy64.h>
#include "board.h"
#include "app.h"
#include "cpu.h"
#include "atd.h"
#include "time.h"
#include "vehicle.h"
#include "digital.h"
#ifdef	CAN_COMM
#include "can.h"
#endif

vehicle_digital	digital_status;

extern Engine_State_t Engine_State;

/************************************************
*name:			Init_Digital_Params
*describe:		initialize paramter of digital handle
*parameter		no
*ver: 			1.0
*date:			2012-08-27
*author:		lijingxiang		
*************************************************/
void Init_Digital_Params(void)
{
	Uint8_t	digital_mi;
	
	digital_status.check_time = 0;
	digital_status.old_status = 0;
	digital_status.new_status = 0;
	
	for(digital_mi = 0;digital_mi < ENGINE_SPEED_SAMPLE_NUM;digital_mi++)
		Engine_SpeedSampleBuffer[digital_mi] = 0;
	
	Engine_SpeedSampleCnt = 0;
}

/**************************************************************************************
@ Func:  �����������ʼ��
@ Brief: 74HC165���Ŷ�ӦIO
         CLK   <--> PH6
         SH/LD <--> PH5
         QH    <--> PH4
         ����������� <--> PT6
**************************************************************************************/
void Config_Ditigal_Io(void)
{
  DDRH |= (1 << 6) + (1 << 5);
  DDRH &= ~(1 << 4);
  
  //IGN_IN����Ϊ��������
  DDRP &= ~(1 << 6);
  PERT &= ~(1 << 6);
}

/**************************************************************************************
@ Func:  �����ź�����
@ Brief: ͨ��74HC165�ɼ����������������ɼ�˳��Ϊ
         [0]      [0]      [0]      [0]      [0]      [0]      [0]      [NC]                                                     
         [DI_IN32][NC]     [DI_IN30][DI_IN29][DI_IN28][DI_IN27][DI_IN26][DI_IN8]
         [DI_IN9] [DI_IN10][DI_IN11][DI_IN4] [DI_IN5] [DI_IN6] [DI_IN7] [DI_IN3]
         [DI_IN22][DI_IN21][DI_IN19][DI_IN14][DI_IN13][DI_IN12][DI_IN1] [DI_IN2]
         
         DI_IN32  ���� NC
         DI_IN30  ���� ���ָʾ
         DI_IN29  ���� NC
         DI_IN28  ���� NC
         DI_IN27  ���� ��ת��
         DI_IN26  ���� ��ת��
         DI_IN8   ���� Զ��
         DI_IN9   ���� K2
         DI_IN10  ���� �����л�
         DI_IN11  ���� NC
         DI_IN4   ���� פ���ƶ�
         DI_IN5   ���� K1
         DI_IN6   ���� ������ѹ
         DI_IN7   ���� NC
         DI_IN3   ���� K0
         DI_IN22  ���� ����
         DI_IN21  ���� ����
         DI_IN19  ���� NC
         DI_IN14  ���� NC
         DI_IN13  ���� Ԥ��
         DI_IN12  ���� K3
         DI_IN1   ���� �ƶ�ѹ��
         DI_IN2   ���� NC
**************************************************************************************/
Uint32_t Data_In(void)
{
	Uint32_t	data_buff = 0;
	Uint8_t		data_mi = 0;
	
	DITIGAL_LOAD = 0;
	DITIGAL_CLK = 0;
	DITIGAL_LOAD = 1;

	for(data_mi = 0; data_mi < 25; data_mi++)    
	{	
	  data_buff<<=1;
	  
		DITIGAL_CLK = 1;
		if(DITIGAL_DATA)
		{
	  	data_buff |= 1;
		}	
		DITIGAL_CLK = 0;
	}

	return data_buff;
}

/**************************************************************************************
@ Func:  ��ȡ�������������߼�λ
@ Brief: ��������������
         [0]      [0]      [0]      [0]      [0]      [0]      [0]      [NC]                                                     
         [DI_IN32][NC]     [DI_IN30][DI_IN29][DI_IN28][DI_IN27][DI_IN26][DI_IN8]
         [DI_IN9] [DI_IN10][DI_IN11][DI_IN4] [DI_IN5] [DI_IN6] [DI_IN7] [DI_IN3]
         [DI_IN22][DI_IN21][DI_IN19][DI_IN14][DI_IN13][DI_IN12][DI_IN1] [DI_IN2]
**************************************************************************************/
Uint8_t	Digital_Drive(void)
{
  static bool firstReadFlag = true;
  unsigned long temp = 0;
  unsigned char i = 0;
  
  digital_status.new_status = Data_In();       
                                               	   
#ifdef CAN_COMM

  //����⵽����ѹ����
  if((ENGINE_STOPPED == Engine_State) || CAN_AlarmFlag.engineOilPressureLowFlag) 
  {
    digital_status.new_status |= LOGICAL_OIL_PRESSURE_LOW_BIT;    
  }
  else
  {
    digital_status.new_status &= ~LOGICAL_OIL_PRESSURE_LOW_BIT;
  }
   
  //����⵽ȼ�ͺ�ˮ  
  if(can_data.fuel_water == 1)
  {  
    digital_status.new_status |= LOGICAL_FUEL_CONTAIN_WATER_BIT;
  }
  else
  {
    digital_status.new_status &= ~LOGICAL_FUEL_CONTAIN_WATER_BIT;
  }
  
#endif 

  //����
  if(PTT & (1 << 6))
  {
    digital_status.new_status |= IGN_ON;
  }
  else
  {
    digital_status.new_status &= ~IGN_ON;
  }
 
  //���ֲɼ�λͼ��������
  //��һ�ζ�ȡ����������ʼ����ر���
  if(firstReadFlag)
  {
    firstReadFlag = false;
    
    digital_status.old_status = digital_status.new_status;
    digital_status.validBits = digital_status.new_status;
  }
  
  //�ݴ�����������
  temp = digital_status.new_status;
  
  for(i = 0; i < 32; ++i)
  {
    if((digital_status.new_status & ((unsigned long)1 << i)) == (digital_status.old_status & ((unsigned long)1 << i)))
    {
      //����������ȡ��ͬ������Ϊ��Ч��������Ч��¼
      digital_status.validBits &= ~((unsigned long)1 << i);
      digital_status.validBits |= (digital_status.new_status & ((unsigned long)1 << i));
    }
    else
    {
      //����������ȡ����ͬ��ʹ����Ч��¼ֵ
      digital_status.new_status &= ~((unsigned long)1 << i);
      digital_status.new_status |= (digital_status.validBits & ((unsigned long)1 << i));
    }
  }
  
  //������ʷλͼ
  digital_status.old_status = temp;
}



