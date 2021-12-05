#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "board.h"
#include "atd.h"
#include "digital.h"
#include "lcd.h"
#include "time.h"
#include "led.h"
#include "smc_motor.h"
//#include "flash.h"
#include "vehicle.h"
#include "CAN.h"
#include "buzzer.h"
#include "algorithm.h"

//任务标志，置位表示需要处理对应任务
queue_list systemTask;

//电锁状态
Elock_State_t Elock_State = ELOCK_OFF;

extern TYPE_of_Engine_t TYPE_of_Engine;
extern unsigned char NoconversionUreaLevel;
extern unsigned char EN_YC_Urea_Function;
//报警标志集
vehicle_warning systemState; 

//变速油压
double Vehicle_TransOilPressure = 0;
bool Vehicle_TransOilPressureLowFlag = false;       //检测到变速油压低标志
unsigned int Vehicle_TransOilPressureLowCnt = 0;    //变速油压低计数器
bool Vehicle_TransOilPressureHighFlag = false;      //检测到变速油压高标志
unsigned int Vehicle_TransOilPressureHighCnt = 0;   //变速油压高计数器
bool Vehicle_HasTransOilPressureSensorFlag = false; //车辆是否带有变速油压传感器

//燃油液位、冷却液温、变矩器油温、尿素液位、变速油压在第一次处理过后才能用于报警判断
bool Meter_FuelLevelFirstProcessFlag = true;
bool Meter_CoolingLiquidTempFirstProcessFlag = true;
bool Meter_OilTempFirstProcessFlag = true;
bool Meter_UreaLevelFirstProcessFlag = true;
bool transOilPressureFirstProcessFlag = true;

//LED控制相关变量
Uint32_t            LED_OnOffBitsMap;
unsigned long LED_CentralAlarmMap;
unsigned long LED_OldState = 0;

Uint16_t park_delay_time;

//发动机转速
Uint8_t	Engine_SpeedSampleCnt; 
Uint16_t Engine_SpeedSampleBuffer[ENGINE_SPEED_SAMPLE_NUM];
engineSpeedParams_t engineSpeedParams;


//车速比设置
unsigned long SPEEDRATIO = 0;

//发动机状态
Engine_State_t Engine_State = ENGINE_STOPPED;


//步进电机控制组
vehicle_motor       munich_motor[5];

//阻值-燃油液位-角度查找表，数组元素格式：[ADC Value Min][ADC Value Typ][ADC Value Max][液位][指针角度]
//液位范围为0~100%，为了精确到十分位，液位*10，范围为0~1000%
const unsigned int LUT_FuelLevel[6][5] =
{
  {491, 488, 485, 0,   0},     //109.4Ω
  {414, 410, 407, 125,  18},   //80.3Ω
  {328, 324, 319, 250,  32},   //55.5Ω
  {221, 215, 210, 500,  46},   //32Ω
  {147, 140, 133, 750,  74},   //19Ω
  {57,  49,  40,  1000, 90},   //6Ω
};

//阻值-油温-角度查找表，数组元素格式：[ADC Value Max][ADC Value Typ][ADC Value Min][油温][指针角度]
const unsigned int LUT_TransOilTemp[11][5] =
{
  {933, 928, 923, 40, 0},     //第一刻度
  {899, 894, 889, 50, 13},
  {857, 851, 845, 60, 26},    //第二刻度
  {808, 802, 796, 70, 33},
  {752, 746, 739, 80, 39},    //
  {691, 685, 679, 90, 46},
  {627, 623, 617, 100, 52},
  {565, 558, 552, 110, 65},   //第三刻度
  {503, 496, 489, 120, 104},  //第四刻度
  {444, 436, 429, 130, 117},
  {391, 380, 373, 140, 130},  //第五刻度
};

//冷却液温度-指针角度查找表，数组元素格式：[冷却液温度][指针角度]
const unsigned char LUT_CoolingLiquid[7][2] =
{
  {40, 0},     //第一刻度
  {60, 26},    //第二刻度
  {80, 46},
  {90, 65},    //第三刻度
  {100, 85},
  {103, 104},  //第四刻度
  {120, 130},  //第五刻度
};

//尿素液位-指针角度查找表，数组元素格式：[液位百分比][指针角度]
const unsigned char LUT_UreaLevel[7][2] =
{
  {0, 0},     //0
  {12, 18},   //1/8
  {25, 27},   //1/4
  {37, 37},
  {50, 46},   //1/2
  {75, 74},   //3/4
  {100, 90},  //1
};

//External Vars
extern Uint8_t      motor_status;
extern motorparams  *mtr0,*mtr1,*mtr2,*mtr3,*mtr4;
extern	vehicle_ad		mnuich_atd[ADC_MAX_CHANNEL/*+1*/];
extern const unsigned char CAN_WorkingTimePGN[3];
extern bool CAN_RecvEngineHourReqAckFlag;
extern unsigned int CAN_EngineSpeedRecvTimeout; 
extern unsigned int CAN_CoolingLiquidTempRecvTimeout;
extern unsigned int CAN_UreaLevelRecvTimeout;
extern Uint32_t CAN_FaultBitMap;
/************************************************
*name: 		Init_Vehicle_Params
*describe: 	parameter initialize
*parameter: no
*ver: 		1.0
*date:		2012-08-27
*author:	
*************************************************/
void Init_Vehicle_Params(void)
{
	Uint8_t	vehicle_mi;
	
	LED_OnOffBitsMap = 0;
	
	systemState.std = 0;
	systemTask.b.Flash_active = 0;	
	
	for(vehicle_mi = 0;vehicle_mi < 5;vehicle_mi++)
	{
		munich_motor[vehicle_mi].motor_dir = 0;
		munich_motor[vehicle_mi].old_motor_angle = 0;
		munich_motor[vehicle_mi].new_motor_angle = 0;
		//speed_data[vehicle_mi] = 0;
	}
}

/***************************************************************************************
@ Func: 燃油液位处理
***************************************************************************************/
static void	ATD_FuelLevelProcess(void)
{	
	static unsigned long sum = 0;
	static unsigned char sampleNum = 0;
	static unsigned char totalSampleNum = 0;
	static unsigned lastSample = 0;
	static unsigned currSample = 0;
	
	unsigned char i = 0;
	double temp = 0;
	unsigned int coolingLiquidTemp = 0;
	
	if(Meter_FuelLevelFirstProcessFlag)
	{
	  Meter_FuelLevelFirstProcessFlag = false;
	  
	  totalSampleNum = 1;
	  lastSample = currSample = mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value;
	}
	else
	{
	  totalSampleNum = 3;
  	lastSample = currSample;
  	currSample = mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value;
  	//若燃油晃动严重则重新采样，已采样值无效
    if(Algo_ValueChangeCheck(currSample, lastSample, 80))
    {
      sum = 0;
      sampleNum = 0;
      
      return;
    }
	}
  
	sum += mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value;
	++sampleNum;
	if(sampleNum == totalSampleNum)
	{
	  mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value = sum / sampleNum;
	  sum = 0;
	  sampleNum = 0;
	  
  	//AD值大于表中最大值，则燃油液位置为0
  	if(mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value > LUT_FuelLevel[0][0])
  	{
  	  //mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value = 0;
  	  coolingLiquidTemp = LUT_FuelLevel[0][3];
  	  munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle = LUT_FuelLevel[0][4];
  	}
  	//AD值小于表中最小值，则置为140℃
  	else if(mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value < LUT_FuelLevel[5][2])
  	{
  	  //mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value = 100;
  	  coolingLiquidTemp = LUT_FuelLevel[5][3];
  	  munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle = LUT_FuelLevel[5][4];
  	}
  	else
  	{
  	  //首先查找是否在刻度线上（满足典型电阻值误差范围时，指针必须指在刻度线上）
  	  for(i = 0; i < 6; ++i)
  	  {
  	    if((mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value <= LUT_FuelLevel[i][0])
  	       && (mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value >= LUT_FuelLevel[i][2]))
  	    {
  	      //mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value = LUT_FuelLevel[i][3];
  	      coolingLiquidTemp = LUT_FuelLevel[i][3];
      	  munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle = LUT_FuelLevel[i][4];
      	  
      	  break;
  	    }
  	  }
  	  
  	  //阻值不在刻度线上，则进行区间线性划分
  	  if(6 == i)
  	  {
  	    for(i = 0; i < 5; ++i)
  	    {
  	      if((mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value < LUT_FuelLevel[i][2])
  	         && (mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value > LUT_FuelLevel[i + 1][0]))
  	      {
  	        temp = (double)(mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value - LUT_FuelLevel[i + 1][0]) 
    	             / (LUT_FuelLevel[i][2] - LUT_FuelLevel[i + 1][0]);
    	             
    	      coolingLiquidTemp = LUT_FuelLevel[i + 1][3] - temp * (LUT_FuelLevel[i + 1][3] - LUT_FuelLevel[i][3]) + 0.5;
    	      //coolingLiquidTemp /= 10;
        	  munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle = LUT_FuelLevel[i + 1][4] 
    	                                                       - temp * (LUT_FuelLevel[i + 1][4] - LUT_FuelLevel[i][4]) + 0.5;
        	  
        	  break;
  	      }
  	    }
  	  }
  	}
  	
  	
  	//当采样值≥2时更新电机角度，防止抖动
  	if(Algo_ValueChangeCheck(mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value, mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].old_adc_value, 2))
  	{
  	  mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].old_adc_value = mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value;
  	  
  	  mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value = coolingLiquidTemp;
  	  
    	//指针角度变化则需要调整电机
    	if(munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle != munich_motor[MOTOR_FUEL_LEVEL].old_motor_angle)
    	{
    		systemTask.b.motor_active = TRUE;
    	}
  	}
	}
}

/********************************************************************************
@ Func: 油温处理
********************************************************************************/
void	ATD_TransOilTempProcess(void)
{
	static unsigned long sum = 0;
	static unsigned char sampleNum = 0;
	static unsigned char totalSampleNum = 0;
	
	unsigned char i = 0;
	double temp = 0;
	
	if(Meter_OilTempFirstProcessFlag)
	{
	  Meter_OilTempFirstProcessFlag = false;
	  
	  totalSampleNum = 1;
	}
	else
	{
	  totalSampleNum = 3;
	}
	
	sum += mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value;
	++sampleNum;
	if(sampleNum == totalSampleNum)
	{
	  mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value = sum / sampleNum;
	  sum = 0;
	  sampleNum = 0;
	  
  	//AD值大于表中最大值，则置为40℃
  	if(mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value > LUT_TransOilTemp[0][0])
  	{
  	  mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value = LUT_TransOilTemp[0][3];
  	  munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle = LUT_TransOilTemp[0][4];
  	}
  	//AD值小于表中最小值，则置为140℃
  	else if(mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value < LUT_TransOilTemp[10][2])
  	{
  	  mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value = LUT_TransOilTemp[10][3];
  	  munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle = LUT_TransOilTemp[10][4];
  	}
  	else
  	{
  	  //首先查找是否在刻度线上（满足典型电阻值误差范围时，指针必须指在刻度线上）
  	  for(i = 0; i < 11; ++i)
  	  {
  	    if((mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value <= LUT_TransOilTemp[i][0])
  	       && (mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value >= LUT_TransOilTemp[i][2]))
  	    {
  	      mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value = LUT_TransOilTemp[i][3];
      	  munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle = LUT_TransOilTemp[i][4];
      	  
      	  break;
  	    }
  	  }
  	  
  	  //阻值不在刻度线上，则进行区间线性划分
  	  if(11 == i)
  	  {
  	    for(i = 0; i < 10; ++i)
  	    {
  	      if((mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value < LUT_TransOilTemp[i][2])
  	         && (mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value > LUT_TransOilTemp[i + 1][0]))
  	      {
  	        temp = (double)(mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value - LUT_TransOilTemp[i + 1][0]) 
    	             / (LUT_TransOilTemp[i][2] - LUT_TransOilTemp[i + 1][0]);
    	             
    	      mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value = LUT_TransOilTemp[i + 1][3] 
    	                                                                 - temp * (LUT_TransOilTemp[i + 1][3] - LUT_TransOilTemp[i][3]);
        	  munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle = LUT_TransOilTemp[i + 1][4] 
    	                                                           - temp * (LUT_TransOilTemp[i + 1][4] - LUT_TransOilTemp[i][4]);
        	  
        	  break;
  	      }
  	    }
  	  }
  	}
  	
  	//当采样值≥2时更新电机角度，防止抖动
  	if(Algo_ValueChangeCheck(mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value, mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].old_adc_value, 2))
  	{
  	  mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].old_adc_value = mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_adc_value;
    	//指针角度变化则需要调整电机
    	if(munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle != munich_motor[MOTOR_TRANS_OIL_TEMP].old_motor_angle)
    	{
    		systemTask.b.motor_active = TRUE;
    	}
  	}
	}
}

/********************************************************************************
@ Func:  冷却液温处理
********************************************************************************/
void Engine_CoolingLiquidTempProcess(void)
{
  unsigned char i = 0;
  double percent = 0;
  
  if(Meter_CoolingLiquidTempFirstProcessFlag)
  {
    Meter_CoolingLiquidTempFirstProcessFlag = false;
  }
  
  if(0 == CAN_CoolingLiquidTempRecvTimeout)
  {
    can_data.currWaterTemp = 40;
    can_data.oldWaterTemp = can_data.currWaterTemp;
    CAN_CoolingLiquidTempRecvTimeout = CAN_RECV_TIMEOUT;
  }
  
  //检查是否超出表头范围
  if(can_data.currWaterTemp > 120)
  {
    munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle = LUT_CoolingLiquid[6][1];
  }
  else if(can_data.currWaterTemp < 40)
  {
    munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle = LUT_CoolingLiquid[0][1];
  }
  else
  {
    //检查液位是否在刻度点上
    for(i = 0; i < 7; ++i)
    {
      if(can_data.currWaterTemp == LUT_CoolingLiquid[i][0])
      {
        munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle = LUT_CoolingLiquid[i][1];
        break;
      }
    }
    
    //不在刻度点上的则进行区间内线性划分
    if(7 == i)
    {
      for(i = 0; i < 6; ++i)
      {
        if((can_data.currWaterTemp >= LUT_CoolingLiquid[i][0]) && (can_data.currWaterTemp < LUT_CoolingLiquid[i + 1][0]))
        {
          percent = (double)(can_data.currWaterTemp - LUT_CoolingLiquid[i][0]) / (LUT_CoolingLiquid[i + 1][0] - LUT_CoolingLiquid[i][0]); 
          munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle =  LUT_CoolingLiquid[i][1] + percent * (LUT_CoolingLiquid[i + 1][1] - LUT_CoolingLiquid[i][1]);
          break;
        }
      }
    }
  }
  
  if(munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle != munich_motor[MOTOR_COOLING_LIQUID_TEMP].old_motor_angle)
  {
    systemTask.b.motor_active = true;
  }
}

/***************************************************************************************
@ Func:  尿素液位处理
@ Brief: 尿素液位为0~100%
***************************************************************************************/
void Engine_UreaLevelProcess()
{
  unsigned char i = 0;
  double percent = 0;
  
  if(Meter_UreaLevelFirstProcessFlag)
  {
    Meter_UreaLevelFirstProcessFlag = false;
  }
  
  if(0 == CAN_UreaLevelRecvTimeout)
  {
    CAN_UreaLevelRecvTimeout = CAN_RECV_TIMEOUT;
    can_data.currUreaLevel = 0;
    can_data.oldUreaLevel = can_data.currUreaLevel;
    NoconversionUreaLevel = 0; 
  }
  
  //检查是否超出100%  Added on 2019.4.15 after testing on the real vehicle.
  if(can_data.currUreaLevel > LUT_UreaLevel[6][0])
  {
    munich_motor[MOTOR_UREA_LEVEL].new_motor_angle = LUT_UreaLevel[6][1];
  }
  else
  {
    //检查液位是否在刻度点上
    for(i = 0; i < 7; ++i)
    {
      if(can_data.currUreaLevel == LUT_UreaLevel[i][0])
      {
        munich_motor[MOTOR_UREA_LEVEL].new_motor_angle = LUT_UreaLevel[i][1];
        break;
      }
    }
    
    //不在刻度点上的则进行区间内线性划分
    if(7 == i)
    {
      for(i = 0; i < 6; ++i)
      {
        if((can_data.currUreaLevel >= LUT_UreaLevel[i][0]) && (can_data.currUreaLevel < LUT_UreaLevel[i + 1][0]))
        {
          percent = (double)(can_data.currUreaLevel - LUT_UreaLevel[i][0]) / (LUT_UreaLevel[i + 1][0] - LUT_UreaLevel[i][0]); 
          munich_motor[MOTOR_UREA_LEVEL].new_motor_angle =  LUT_UreaLevel[i][1] + percent * (LUT_UreaLevel[i + 1][1] - LUT_UreaLevel[i][1]) + 0.5;
          break;
        }
      }
    }
  }
  
  
  if(munich_motor[MOTOR_UREA_LEVEL].new_motor_angle != munich_motor[MOTOR_UREA_LEVEL].old_motor_angle)
  {
    systemTask.b.motor_active = true;
  }
}

/********************************************************************************
@ Func:  变速油压处理
@ Brief: 当油压下降到0.9MPa（9Bar）时油压低报警，对应传感器输出电压为1.94V；
         当油压上升到1MPa（10Bar）时油压正常，对应传感器输出电压为2.1V。
         传感器输出电压与油压计算公式：
         Output = 0.5 + 4 * (Pressure / 25)，单位：V
         Pressure = (Output - 0.5) * 25 / 4，单位：Bar
********************************************************************************/
unsigned char globalHasSensorSemaphore = 0;
void ATD_TransOilPressureProcess(void)
{
  static unsigned int sum = 0;
  static unsigned char sampleNum = 0;
  static unsigned char totalSampleNum = 0;
  double volt = 0;
  
  static unsigned char hasSensorSemaphore = 0;
  
  if(transOilPressureFirstProcessFlag)
  {
    transOilPressureFirstProcessFlag = false;
    totalSampleNum = 1;
  }
  else
  {
    totalSampleNum = 3;
  }
  
  sum += mnuich_atd[ATD_CHANNEL_TRANS_OIL_LOW].new_adc_value;
  ++sampleNum;
  if(sampleNum == totalSampleNum)
  {
    volt = ((double)sum / sampleNum) / 1023.0 * 5.0;
    
    //对信号量操作以判断车辆是否带有油压传感器
    if(volt < 0.1)
    {
      if(hasSensorSemaphore)
      {
        --hasSensorSemaphore;
      }
    }
    else
    {
      if(hasSensorSemaphore < 15)
      {
        ++hasSensorSemaphore;
      }
    }
    
    if(hasSensorSemaphore >= 5)
    {
      Vehicle_HasTransOilPressureSensorFlag = true;
    }
    else
    {
      Vehicle_HasTransOilPressureSensorFlag = false;
    }
    
    //计算油压
    if(volt < 0.5)
    {
      volt = 0.5;
    }
    else if(volt > 4.5)
    {
      volt = 4.5;
    }
    Vehicle_TransOilPressure = (volt - 0.5) * 25 / 4;
    
    sampleNum = 0;
    sum = 0;
  }
  
  globalHasSensorSemaphore = hasSensorSemaphore;
}

/************************************************
*name: 		ATD_SupplyVoltageProcess
*describe: 	Pressure pointer angle processing
*parameter: no
*ver: 		1.0
*date:		2012-08-27
*author:	lijingxiang
*************************************************/
static void	ATD_SupplyVoltageProcess(void)
{
	if((mnuich_atd[4].old_adc_value == 0xffff)||(mnuich_atd[4].new_adc_value > 1010))
	{	
		if(mnuich_atd[4].new_adc_value > 1010)
		{
			mnuich_atd[4].old_adc_value = mnuich_atd[4].new_adc_value;
			mnuich_atd[4].supply_value = 0;
		}
		else
			mnuich_atd[4].old_adc_value = 1010;
	}
//	if(mnuich_atd[4].new_adc_value >= 1000)
//	{	
//		mnuich_atd[4].old_adc_value = mnuich_atd[4].new_adc_value;
//	}

	if((mnuich_atd[4].new_adc_value > mnuich_atd[4].old_adc_value)&&(mnuich_atd[4].new_adc_value < 1010))
	{	
		if((mnuich_atd[4].new_adc_value-mnuich_atd[4].old_adc_value) >= 10)
			mnuich_atd[4].supply_value = mnuich_atd[4].new_adc_value-mnuich_atd[4].old_adc_value;
		else
			mnuich_atd[4].supply_value = 0;
	}

}

/********************************************************************************************
@ Func:  各通道AD转换处理
********************************************************************************************/
static void	atd_manage(void)
{
  switch(atd_channel_id)
  {
    case ATD_CHANNEL_TRANS_OIL_LOW:       //变速油压
      ATD_TransOilPressureProcess(); 
    break;
      
    case ATD_CHANNEL_FUEL_LEVEL:          //油位
      ATD_FuelLevelProcess();		      
    break;
   
    case ATD_CHANNEL_CHARGING_INDICATOR:  //充电指示，Reserved
    break;
    
    case ATD_CHANNEL_TRANS_OIL_TEMP:
      ATD_TransOilTempProcess();		      //油温
    break;
    
    case ATD_CHANNEL_AVDD:
      ATD_SupplyVoltageProcess();		      //参考电压
    break;            
    
    case ATD_CHANNEL_AI_IN5:              //Reserved
    break;
                                      
    case ATD_CHANNEL_AN06:                //Reserved
    break;
    
    case ATD_CHANNEL_BATTERY_VOLTAGE:     //蓄电池电压，Reserved
    break;
            
    default:
    break;
  }
  
  atd_channel_id++;  
  if(atd_channel_id >= ADC_MAX_CHANNEL)
  {
    atd_channel_id = 0;
  }
}

/*****************************************************************************************************************
@ Func:  5V输出初始化
@ Brief: 当三极管Q28导通时，5V输出断开；反之，5V对外输出。
******************************************************************************************************************/
void voltageOutInit()
{
  DDRP |= (1 << 0);
  PTP |= (1 << 0);
}

/*****************************************************************************************************************
@ Func:  使能5V输出
@ Brief: 使三极管Q28关闭
******************************************************************************************************************/
void voltageOutputEnable()
{
  PTP &= ~(1 << 0);
}

/*****************************************************************************************************************
@ Func:  禁止5V输出
@ Brief: 使三极管Q28导通  
******************************************************************************************************************/
void voltageOutputDisable()
{
  PTP |= (1 << 0);
}



/*****************************************************************************************************************
@ Func:  车速比设置检查
@ Brief: 自检时检查车速比
******************************************************************************************************************/
unsigned char testratio = 0;
void speedRatioCheck(void) 
{
    unsigned char speed_ratio = 0x00;
    
    if(digital_status.new_status & SPEED_RATIO_K0_BIT)
      speed_ratio |= 0x01;
    else 
      speed_ratio &= 0xfe;
    
    if(digital_status.new_status & SPEED_RATIO_K1_BIT)
      speed_ratio |= 0x02;
    else
      speed_ratio &= 0xfd;
    
    if(digital_status.new_status & SPEED_RATIO_K2_BIT)
      speed_ratio |=0x04;
    else
      speed_ratio &= 0xfb;
    
    if(digital_status.new_status & SPEED_RATIO_K3_BIT)
      speed_ratio |=0x08;
    else
      speed_ratio &= 0xf7;
     
    testratio = speed_ratio;  
    switch(speed_ratio) 
    {
      //to do 未来在这里修改或增加车速比，为确保计算精度，所有20km/h对应的频率都要乘100，小数点后省略
      //例如20km/h对应频率为200Hz,参数就应设置为20000    By周文熙 20200731
                                           // K0     K1    K2    K3
      case 0x0f: SPEEDRATIO = 10860;      //悬空   悬空  悬空  悬空
                 break;

      
      //默认车速比，第二版PCBA未焊接电路，在这里设置默认车速比即可
      default:   SPEEDRATIO = 10860;
                 break;
    }
}



/*****************************************************************************************************************
@ Func:  报警标志设置
@ Brief: 根据数字量设置各报警标志
         警告字各Bit含义：
         [NC]         [NC]             [NC]         [NC]         [NC]            [box_press][brake_press]    [oil_press]
         [engineGeneralFault][net_block]      [oil_level_l][drive_block][oil_block]     [oil_water][power_charge]   [water_temp_h]
         [flicher_bit][backlight_on]   [ign_on]     [hand_brake] [right_turn]    [left_turn][speed_err]      [engineSevereFault]
         [can_warn]   [oil_level_limit][chinese]    [mute]       [motor_zero_bit][pulse_bit][lcd_flicher_bit][buzz_bit]
******************************************************************************************************************/
void	warningStateCheck(void)
{
  unsigned int i = 0;
  
    //电锁开的条件下
    if(ELOCK_ON == Elock_State)
    {
      //驻车制动报警检测（接地，常亮）
      //驻车制动报警需延时3秒
      if(digital_status.new_status & DIGITAL_PARKING_BRAKE_BIT)
      {
        park_delay_time = 3; 
      }
      
      if(0 == (digital_status.new_status & DIGITAL_PARKING_BRAKE_BIT))
      {
        LED_OnOffBitsMap |= LED_PARKING_BREAK;
        //发动机转速＞1000并且延时3秒之后报警
        if((engineSpeedParams.new_speed > 1000) && (0 == park_delay_time))
        {
          systemState.b.hand_brake = TRUE; 
        }
        else
        {
          systemState.b.hand_brake = false;
        }
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_PARKING_BREAK;
        systemState.b.hand_brake = false;
      }
      //手刹通过CAN方式
      /*if(1 == can_data.handbrake)
      {
        LED_OnOffBitsMap |= LED_PARKING_BREAK;
        if(engineSpeedParams.new_speed > 1000)
        {
          systemState.b.hand_brake = TRUE; 
        }
        else
        {
          systemState.b.hand_brake = false;
        }
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_PARKING_BREAK;
        systemState.b.hand_brake = false;
      }*/
      
      //左右转向灯
      /*if(digital_status.new_status & DIGITAL_LEFT_TURN_BIT)
      {
        //LED_OnOffBitsMap |= LED_LEFT;
      }
      else
      {
        //LED_OnOffBitsMap &= ~LED_LEFT;
      }
      
      if(digital_status.new_status & DIGITAL_RIGHT_TURN_BIT)
      {
        //LED_OnOffBitsMap |= LED_RIGHT;
      }
      else
      {
        //LED_OnOffBitsMap &= ~LED_RIGHT;
      }*/
      
      //远光
      if(digital_status.new_status & DIGITAL_DISTANT_LIGHT_BIT)
      {
        LED_OnOffBitsMap |= LED_HIGH_BEAM;
        systemState.b.distantLight = true;
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_HIGH_BEAM;
        systemState.b.distantLight = false;
      }
      
    	//静音报警检测（接地，常亮）
      if(0 == (digital_status.new_status & DIGITAL_MUTE_BIT))
      {
        systemState.b.mute = TRUE;
        LED_OnOffBitsMap |= LED_MUTE;
      }
      else
      {
        systemState.b.mute = false;
        LED_OnOffBitsMap &= ~LED_MUTE;
      }
          
      //机油压力低报警检测（逻辑，闪烁，发动机启动瞬间不检测） 
      if((digital_status.new_status & LOGICAL_OIL_PRESSURE_LOW_BIT) && ((ENGINE_STARTING != Engine_State) && (ENGINE_STOPPING != Engine_State))/* && (!munich_time.enging_on_time)*/)
      {
        LED_OnOffBitsMap |= LED_LOW_OIL_P;
        if(engineSpeedParams.new_speed >= ENGINE_STARTUP_SPEED)
        {
          systemState.b.oil_press = true;
        }
        else
        {
          systemState.b.oil_press = false;
        }
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_LOW_OIL_P;
        systemState.b.oil_press = false;
      } 
                
      //充电指示报警（接地，常亮）
      if(0 == (digital_status.new_status & DIGITAL_CHARGING_INDICATOR_BIT))
      {
        LED_OnOffBitsMap |= LED_BATTERY;
        if((!munich_time.enging_on_time) && (engineSpeedParams.new_speed >= ENGINE_STARTUP_SPEED))
        {
          systemState.b.power_charge = true;
        }
        else
        {
          systemState.b.power_charge = false;
        }
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_BATTERY;
        systemState.b.power_charge = false;
      }
      
      //发动机故障报警检测（逻辑，常亮）
      //if(CAN_AlarmFlag.engineFaultFlag)
      if (CAN_FaultBitMap)
      {
        for (i = 0; i < CAN_FAULT_RECORD_MAX_NUM; ++i)
        {
          if (CAN_FaultRecordBuffer[i].can_spn)
          {
            //潍柴状态下发动机故障灯点亮方式
            if (TYPE_of_Engine == Weichai_engine)
            {
              if ((CAN_FaultRecordBuffer[i].lamp_status & 0x03) == 0x01)
              {
                LED_OnOffBitsMap |= LED_ENGINE_FAULT_BIT;
                systemState.b.engineGeneralFault = true;
                break;
              }
            }
            else //玉柴状态下发动机故障灯报警方式
            {
              if ((CAN_FaultRecordBuffer[i].flash_lamp_status & 0x30) == 0x10)                             
              {
                LED_OnOffBitsMap |= LED_ENGINE_FAULT_BIT;
                systemState.b.engineSevereFault = true;
                break;
              }
              else if(
                      ((CAN_FaultRecordBuffer[i].lamp_status & 0x30) == 0x10)
                      &&((CAN_FaultRecordBuffer[i].flash_lamp_status & 0x30) != 0x10)
                     )
              {
                LED_OnOffBitsMap |= LED_ENGINE_FAULT_BIT;
                systemState.b.engineGeneralFault = true;
                systemState.b.engineSevereFault = false;
                break;
              }

            }
          }
        }
        //若已搜索玩整个故障数组未发现SVS Lamp点亮设置，则复位变量
        if (CAN_FAULT_RECORD_MAX_NUM == i)
        {
          LED_OnOffBitsMap &= ~LED_ENGINE_FAULT_BIT;
          systemState.b.engineGeneralFault = false;
          systemState.b.engineSevereFault = false;
        }
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_ENGINE_FAULT_BIT;
        systemState.b.engineGeneralFault = false;
        systemState.b.engineSevereFault = false;
      }
      //后处理故障灯（驾驶员警告灯、只在玉柴发动机中点亮)
      if (TYPE_of_Engine == Yuchai_engine)
      {
        for (i = 0; i < CAN_FAULT_RECORD_MAX_NUM; ++i)
        {
          if ((CAN_FaultRecordBuffer[i].lamp_status & 0x0C) == 0x04)
          {
            LED_OnOffBitsMap |= LED_POST_PROCESSING_BIT;
            if (CAN_FaultBitMap == 0)
            {
              if (CAN_FaultRecordBuffer[i].error_time == 0)
              {
                CAN_FaultRecordBuffer[i].lamp_status = 0;
              }
            }
            break;
          }
        }
        if (CAN_FAULT_RECORD_MAX_NUM == i)
        {
          LED_OnOffBitsMap &= ~LED_POST_PROCESSING_BIT;
        }
      }
      //预热，CAN方式
      if(0x01 == can_data.can_preheat)
      {
        LED_OnOffBitsMap |= LED_PREHEAT;
        systemState.b.preheat = true;
      }
      else if(0 == can_data.can_preheat)
      {
        LED_OnOffBitsMap &= ~LED_PREHEAT;
        systemState.b.preheat = false;    
      }
      
      //冷却液温高报警检测（逻辑，闪烁）  
      if((can_data.currWaterTemp >= WATER_WARN_VALUE) && (!Meter_CoolingLiquidTempFirstProcessFlag))
      {
        LED_OnOffBitsMap |= LED_WATER_TEMP;
        systemState.b.water_temp_h = TRUE;
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_WATER_TEMP;
        systemState.b.water_temp_h = FALSE;
      }
     
    	//油水分离故障报警检测（逻辑，常亮）
    	if(digital_status.new_status & LOGICAL_FUEL_CONTAIN_WATER_BIT)
    	{
    	  systemState.b.oil_water = TRUE;
    	  LED_OnOffBitsMap |= LED_FUEL_WATER;
    	}
    	else
    	{
    	  systemState.b.oil_water = false;
    	  LED_OnOffBitsMap &= ~LED_FUEL_WATER;
    	}
		      
      //制动压力低报警检测（PA_6，接地，闪烁）
      if(0 == (digital_status.new_status & DIGITAL_BRAKE_PRESSURE_BIT))
      {
        LED_OnOffBitsMap |= LED_BREAK_ERR;
        systemState.b.brake_press = true;
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_BREAK_ERR;
        systemState.b.brake_press = false;
      }
      
	    //变速油压低报警检测（逻辑，闪烁，需要先检测车辆是否带有油压传感器）
    	if((ENGINE_STARTED == Engine_State) && (Vehicle_TransOilPressure <= 9) && /*(!transOilPressureFirstProcessFlag) && */Vehicle_HasTransOilPressureSensorFlag)
    	{
    	  Vehicle_TransOilPressureLowFlag = true;
    	  if(Vehicle_TransOilPressureLowCnt > 10000)
    	  {
  	      systemState.b.box_press = true;
  	      LED_OnOffBitsMap |= LED_TRANS_OIL_P;
    	  }
    	}
    	else
    	{
    	  Vehicle_TransOilPressureLowFlag = false;
    	  Vehicle_TransOilPressureLowCnt = 0;
    	}
    	
      if((Vehicle_TransOilPressure >= 10))
      {
        Vehicle_TransOilPressureHighFlag = true;
        if(Vehicle_TransOilPressureHighCnt > 10000)
        {
  	      systemState.b.box_press = false;
  	      LED_OnOffBitsMap &= ~LED_TRANS_OIL_P;
        }
      }
      else
      {
        Vehicle_TransOilPressureHighFlag = false;
        Vehicle_TransOilPressureHighCnt = 0;
      }
      
      //若无变速油压传感器或发动机未启动，则不报警
    	if((ENGINE_STOPPED == Engine_State) || (!Vehicle_HasTransOilPressureSensorFlag))
    	{
    	  systemState.b.box_press = false;
	      LED_OnOffBitsMap &= ~LED_TRANS_OIL_P;
    	}

      //潍柴状态下再生指示灯
      if (TYPE_of_Engine == Weichai_engine)
      {
        if ((REGENERATE_STATE_IN_PROCESS == CAN_PostprocessingState.regenerateState)   //再生服务状态下，LED灯不显示，只显示需服务再生字样
             ||(REGENERATE_STATE_PARKING_PROMPT== CAN_PostprocessingState.regenerateState))
             //根据临工要求，屏蔽潍柴在再生服务时，再生灯亮
             //||(REGENERATE_STATE_SERVICE_PROMPT== CAN_PostprocessingState.regenerateState))
        {
          LED_OnOffBitsMap |= LED_RECOVERY_BIT;
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_RECOVERY_BIT;
        }
      }
      else    //玉柴状态下再生指示灯 if_else语句将两种发动机分开，逻辑比较清晰
      {
        if((REGENERATE_STATE_IN_PROCESS == CAN_PostprocessingState_YC.regenerateRemind)
         ||(REGENERATE_STATE_PARKING_PROMPT == CAN_PostprocessingState_YC.regenerateRemind)
         ||(REGENERATE_STATE_SERVICE_PROMPT == CAN_PostprocessingState_YC.regenerateRemind)
         ||(0x04 == CAN_PostprocessingState_YC.regenerateProcess))
        {
          LED_OnOffBitsMap |= LED_RECOVERY_BIT;
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_RECOVERY_BIT;
        }
      }
      //潍柴状态下再生抑制指示灯
      if (TYPE_of_Engine == Weichai_engine)
      {
        if (REGENERATE_INHIBIT_STATE_IN_PROCESS == CAN_PostprocessingState.regenerateInhibitState)
        {
          LED_OnOffBitsMap |= LED_RECOVERY_INHIBITION_BIT;
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_RECOVERY_INHIBITION_BIT;
        }
      }
      else    //玉柴状态下再生抑制指示灯,用if-else分开逻辑比较明确
      {
        if (0X04 == CAN_PostprocessingState_YC.regenerateInhibitState)
        {
          LED_OnOffBitsMap |= LED_RECOVERY_INHIBITION_BIT;
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_RECOVERY_INHIBITION_BIT;
        }
      }
      //玉柴排气高温报警灯
      if (0X04==CAN_PostprocessingState_YC.exhaust_high_temperature)
      {
        LED_OnOffBitsMap |= LED_RECOVERY_TEMP_BIT;
      }
      else if (0X00==CAN_PostprocessingState_YC.exhaust_high_temperature)
      {
        LED_OnOffBitsMap &= ~LED_RECOVERY_TEMP_BIT;
      }
      //燃油液位报警检测（逻辑，闪烁）
      //当引脚悬空时，不报警
      if(mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_adc_value < OIL_WARN_LIMIT_VALUE)   
     	{
     	  systemState.b.oil_level_limit = TRUE;
     	}	  	  
      else
      {
        systemState.b.oil_level_limit = FALSE;
      } 

                    
      if((mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value <= 125) && (systemState.b.oil_level_limit) && (!Meter_FuelLevelFirstProcessFlag))     //liu~
      {
        systemState.b.oil_level_l = TRUE;
     	  LED_OnOffBitsMap |= LED_FUEL_LOW;
      }
      else
      {
        systemState.b.oil_level_l = FALSE;	
        LED_OnOffBitsMap &= ~LED_FUEL_LOW;
      }
            
      //NCD故障
      //if(((NCD_STATE_INACTIVE != ((CAN_PostprocessingState.ncdState & 0x38) >> 3)) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x07)) 
      //   || ((NCD_DRIVER_ALARM_INACTIVE != (CAN_PostprocessingState.ncdState & 0x07)) && ((CAN_PostprocessingState.ncdState & 0x07) != 0x07)))
      //潍柴发动机状态下，NCD灯的报警方式
      if (TYPE_of_Engine == Weichai_engine)
      {
        if (((CAN_PostprocessingState.ncdState & 0x07) == NCD_DRIVER_ALARM_ACTIVE) || ((CAN_PostprocessingState.ncdState & 0x07) == NCD_DRIVER_ALARM_UREA_LEVEL_LOW))
        {
          LED_OnOffBitsMap |= LED_UREA_LEVEL;
          //檢測是否是尿素液位低報警
          if ((CAN_PostprocessingState.ncdState & 0x07) == NCD_DRIVER_ALARM_UREA_LEVEL_LOW)
          {
            systemState.b.ureaLevelLow = true;
          }
          else
          {
            systemState.b.ureaLevelLow = false;
          }
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_UREA_LEVEL;
          systemState.b.ureaLevelLow = false;
        }
      }
      else //玉柴发动机状态下，NCD灯的报警方式，尿素小于1/8时，且使能玉柴发动机的尿素功能，NCD灯才闪烁
      {
        if ((NoconversionUreaLevel <= 31) && (!Meter_UreaLevelFirstProcessFlag) && (EN_YC_Urea_Function == true))
        {
          LED_OnOffBitsMap |= LED_UREA_LEVEL;
          systemState.b.ureaLevelLow = true;
        }
        else
        {
          LED_OnOffBitsMap &= ~LED_UREA_LEVEL;
          systemState.b.ureaLevelLow = false;
        }
      }
      //背光灯报警检测（接电源）
      if(digital_status.new_status & DIGITAL_BACKLIGHT_BIT)
      {
        systemState.b.backlight_on = true;
      }
      else
      {
        systemState.b.backlight_on = false;
      }
      
      //根据报警条件设置中央报警灯
      if(systemState.b.brake_press || systemState.b.oil_press || systemState.b.box_press || systemState.b.oil_water
         || systemState.b.hand_brake || systemState.b.water_temp_h || systemState.b.oil_level_l || systemState.b.power_charge
         || systemState.b.engineGeneralFault || systemState.b.ureaLevelLow || systemState.b.engineSevereFault 
         //|| ((CAN_PostprocessingState.ncdState & 0x07) == NCD_DRIVER_ALARM_ACTIVE) || ((CAN_PostprocessingState.ncdState & 0x07) == NCD_DRIVER_ALARM_UREA_LEVEL_LOW))
         || ((NCD_STATE_INACTIVE != ((CAN_PostprocessingState.ncdState & 0x38) >> 3)) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x01)
             && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x06) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x07))
         || ((NCD_DRIVER_ALARM_INACTIVE != (CAN_PostprocessingState.ncdState & 0x07)) && ((CAN_PostprocessingState.ncdState & 0x07) != 0x07))
         || ((CAN_GPSWarningBitsMap & GPS_ALARM_51_DAYS_NO_SIGNAL) || (CAN_GPSWarningBitsMap & GPS_ALARM_58_DAYS_NO_SIGNAL)) 
             || (CAN_GPSWarningBitsMap & GPS_ALARM_61_DAYS_NO_SIGNAL) || (CAN_GPSWarningBitsMap & GPS_ALARM_WORK_ABNORMAL)
             || (CAN_GPSWarningBitsMap & GPS_ALARM_WEB_LOCK_CMD)
             || (CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT) || (CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT_LOCKED))
      {
        LED_OnOffBitsMap |= LED_WARN;
      }
      else
      {
        LED_OnOffBitsMap &= ~LED_WARN;
      }
      
      //根据报警条件设置蜂鸣器    
      if(!systemState.b.mute)
      {
        if(systemState.b.brake_press || systemState.b.box_press || systemState.b.hand_brake || systemState.b.water_temp_h 
           || systemState.b.engineGeneralFault || systemState.b.ureaLevelLow || systemState.b.engineSevereFault
           || ((NCD_STATE_INACTIVE != ((CAN_PostprocessingState.ncdState & 0x38) >> 3)) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x01)
               && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x06) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x07))
           || ((NCD_DRIVER_ALARM_INACTIVE != (CAN_PostprocessingState.ncdState & 0x07)) && ((CAN_PostprocessingState.ncdState & 0x07) != 0x07))
           || ((CAN_GPSWarningBitsMap & GPS_ALARM_51_DAYS_NO_SIGNAL) || (CAN_GPSWarningBitsMap & GPS_ALARM_58_DAYS_NO_SIGNAL)) 
               || (CAN_GPSWarningBitsMap & GPS_ALARM_61_DAYS_NO_SIGNAL) || (CAN_GPSWarningBitsMap & GPS_ALARM_WORK_ABNORMAL)
               || (CAN_GPSWarningBitsMap & GPS_ALARM_WEB_LOCK_CMD)
               || (CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT) || (CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT_LOCKED))   
        {                                                                             
          systemState.b.buzz_bit = TRUE;
        }
        else
        {
          systemState.b.buzz_bit = FALSE;
          Buzzer_Close();
        }
      }
      else
      {
        systemState.b.buzz_bit = FALSE;
        Buzzer_Close();
      }
    }	 
    
    //LED状态字改变时需要刷新状态
    if(LED_OnOffBitsMap != LED_OldState)
    {
      LED_OldState = LED_OnOffBitsMap;
      systemTask.b.Led_active = true;
    }
}

/*******************************************************************************
@ Func: 检测电锁状态
@ Param: None
@ Ret: ELOCK_ON/ELOCK_OFF/ELCOK_JUST_TURN_ON/ELOCK_JUST_TURN_OFF
*******************************************************************************/
Elock_State_t Elock_CheckState(void)
{
  static unsigned int Elock_LoopCnt = 0;
  
  if((ELOCK_OFF == Elock_State) || (ELOCK_JUST_TURN_OFF == Elock_State))
  {
    while(digital_status.new_status & IGN_ON)
    {
      ++Elock_LoopCnt;
      DelayMS(15);
      if(Elock_LoopCnt >= 2)
      {
        Elock_LoopCnt = 0;
        return ELOCK_JUST_TURN_ON;
      }
    }

    Elock_LoopCnt = 0;
    return ELOCK_OFF;
  }
  else if((ELOCK_ON == Elock_State) || (ELOCK_JUST_TURN_ON == Elock_State))
  {
    while(!(digital_status.new_status & IGN_ON))
    {
      ++Elock_LoopCnt;
      DelayMS(15);
      if(Elock_LoopCnt >= 2)
      {
        Elock_LoopCnt = 0;
        return ELOCK_JUST_TURN_OFF;
      }
    }

    Elock_LoopCnt = 0;
    return ELOCK_ON;
  }
}

/********************************************************************************************
@ Func:  LED控制
********************************************************************************************/
void Led_Handle(void)
{
    Uint32_t	led_buff;
    unsigned long bitMask = 0;
    
    led_buff = LED_OnOffBitsMap;
    //控制再生指示灯慢闪 by_徐光同  2021/4/26
    if ((slow_flicher_flag == 0) &&
        (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_SERVICE_PROMPT))
    {
      led_buff &= ~LED_RECOVERY_BIT;
    }
    //控制LED闪烁
    if(!systemState.b.flicher_bit)
    {
      bitMask = LED_BLINK_MASK;
      
      //油壓低報警需要閃爍
      if(systemState.b.oil_press)
      {
        bitMask |= LED_LOW_OIL_P;
      }
      
      //尿素液位低報警需要閃爍
      if(systemState.b.ureaLevelLow)
      {
        bitMask |= LED_UREA_LEVEL;
      }
      //玉柴状态下发动机故障灯闪烁
      if(systemState.b.engineSevereFault)
      {
        bitMask |= LED_ENGINE_FAULT_BIT;
      }      
      //驻车再生需要闪烁       //完善玉柴快闪功能 by_徐光同 2021/4/16
      if((REGENERATE_STATE_PARKING_PROMPT == CAN_PostprocessingState.regenerateState)||
         (REGENERATE_STATE_PARKING_PROMPT == CAN_PostprocessingState_YC.regenerateRemind))
      {
        bitMask |= LED_RECOVERY_BIT;
      }
            
      led_buff &= ~bitMask;
    }
    else
    {
      //NOP
    }
    
    Led_Drive(led_buff);
    
    //背光
    if(systemState.b.backlight_on)
    {
      LED_BacklightOnOff(LED_ON);
    }
    else
    {
      LED_BacklightOnOff(LED_OFF);
    }
}

/********************************************************************************************
@ Func:  步进电机控制
@ Param: motor_id --
         0      <--> 所有电机回零
         其他值 <--> 各步进电机旋转到指定角度
********************************************************************************************/
void	Motor_Handle(Uint8_t motor_id)
{
	if(!motor_id)
	{
		motor_status = 0;
		
		Motor0.u8_status_id = 0;
		Motor1.u8_status_id = 0;
		Motor2.u8_status_id = 0;
		Motor3.u8_status_id = 0;
		Motor4.u8_status_id = 0;
		
		Motor0.u16_vel = MAX_STEP_PERIOD_0;
		Motor1.u16_vel = MAX_STEP_PERIOD_0;
		Motor2.u16_vel = MAX_STEP_PERIOD_0;
		Motor3.u16_vel = MAX_STEP_PERIOD_0;
		Motor4.u16_vel = MAX_STEP_PERIOD_0;

		motor_set_zero();

		Motor0.u8_status_id = 1;
		Motor1.u8_status_id = 1;
		Motor2.u8_status_id = 1;
		Motor3.u8_status_id = 1;
		Motor4.u8_status_id = 1;
	}
	else
	{
	  //转速表
		if(engineSpeedParams.old_motor_angle != engineSpeedParams.new_motor_angle)
		{
			engineSpeedParams.old_motor_angle = engineSpeedParams.new_motor_angle;
			
			GoToPosition(mtr0, engineSpeedParams.new_motor_angle);
			
			if(!munich_motor[MOTOR_ROTATION_SPEED].motor_first)
			{
				munich_motor[MOTOR_ROTATION_SPEED].motor_first = 1;
			}
		}	
		//燃油液位表	
		else if(munich_motor[MOTOR_FUEL_LEVEL].old_motor_angle != munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle)
		{
			munich_motor[MOTOR_FUEL_LEVEL].old_motor_angle = munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle;
			
			GoToPosition(mtr4, munich_motor[MOTOR_FUEL_LEVEL].new_motor_angle);
			
			if(!munich_motor[MOTOR_FUEL_LEVEL].motor_first)
			{
				munich_motor[MOTOR_FUEL_LEVEL].motor_first = 1;
			}
		}
		//冷却液温表
		else if(munich_motor[MOTOR_COOLING_LIQUID_TEMP].old_motor_angle != munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle)
		{	
			munich_motor[MOTOR_COOLING_LIQUID_TEMP].old_motor_angle = munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle;
			
			GoToPosition(mtr1, munich_motor[MOTOR_COOLING_LIQUID_TEMP].new_motor_angle);
			
			if(!munich_motor[MOTOR_COOLING_LIQUID_TEMP].motor_first)
			{
				munich_motor[MOTOR_COOLING_LIQUID_TEMP].motor_first = 1;
			}
		}	
		//传动油温表
		else if(munich_motor[MOTOR_TRANS_OIL_TEMP].old_motor_angle != munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle)
		{
			munich_motor[MOTOR_TRANS_OIL_TEMP].old_motor_angle = munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle;
			
			GoToPosition(mtr2, munich_motor[MOTOR_TRANS_OIL_TEMP].new_motor_angle);
			
			if(!munich_motor[MOTOR_TRANS_OIL_TEMP].motor_first)
			{
				munich_motor[MOTOR_TRANS_OIL_TEMP].motor_first = 1;
			}
		}
		//尿素表
		else if(munich_motor[MOTOR_UREA_LEVEL].old_motor_angle != munich_motor[MOTOR_UREA_LEVEL].new_motor_angle)
		{	
			munich_motor[MOTOR_UREA_LEVEL].old_motor_angle = munich_motor[MOTOR_UREA_LEVEL].new_motor_angle;
			
			GoToPosition(mtr3, munich_motor[MOTOR_UREA_LEVEL].new_motor_angle);
			
			if(!munich_motor[MOTOR_UREA_LEVEL].motor_first)
			{
				munich_motor[MOTOR_UREA_LEVEL].motor_first = 1;
			}
		}	
		else
		{
			if((CheckIfStopped(mtr0) == TRUE) && (CheckIfStopped(mtr1) == TRUE) && (CheckIfStopped(mtr2) == TRUE) 
			   && (CheckIfStopped(mtr3) == TRUE) && (CheckIfStopped(mtr4) == TRUE))
			{
				systemTask.b.motor_active = FALSE;
			}
		}
	}
}

/********************************************************************************************
@ Func:  转速表角度计算
********************************************************************************************/
void tachometerPointerAngleCalculate(void)
{
	if(engineSpeedParams.new_speed > MAX_SPEED)
	{
		engineSpeedParams.new_motor_angle = MOTOR_ENGINE_SPEED_MAX_ANGLE;
	}
	else
	{
	  engineSpeedParams.new_motor_angle = (double)((unsigned long)engineSpeedParams.new_speed * MOTOR_ENGINE_SPEED_MAX_ANGLE) / 3000.0;
	  if(engineSpeedParams.new_speed >= 3000)
	  {
	    engineSpeedParams.new_motor_angle -= 2;
	  }
	  else if(engineSpeedParams.new_speed >= 2000)
	  {
	    engineSpeedParams.new_motor_angle -= 1;
	  }
	}
	
	if(engineSpeedParams.new_motor_angle != engineSpeedParams.old_motor_angle)
	{
	  systemTask.b.motor_active = true;
	}
}

/********************************************************************************************
@ Func:  发动机转速计算
********************************************************************************************/
void	Speed_handle(void)
{
  Uint32_t  speed_temp = 0;
  Uint8_t speed_mi;

  for(speed_mi = 0; speed_mi < ENGINE_SPEED_SAMPLE_NUM; speed_mi++)
  {
    speed_temp += Engine_SpeedSampleBuffer[speed_mi];
  }

  engineSpeedParams.new_speed = speed_temp / ENGINE_SPEED_SAMPLE_NUM;
  
  //若转速接收超时，则转速置零，表针回到零位
  if(0 == CAN_EngineSpeedRecvTimeout)
  {
    engineSpeedParams.new_speed = 0;
    CAN_EngineSpeedRecvTimeout = CAN_RECV_TIMEOUT;
  }
  
  
  //发动机转速变化控制
  if(Algo_ValueChangeCheck(engineSpeedParams.new_speed, engineSpeedParams.old_speed, /*11*/20))
  {
    //转速表角度计算
    tachometerPointerAngleCalculate();
    
    engineSpeedParams.old_speed = engineSpeedParams.new_speed;
    LCD_NormalDisplayCtrl.engineSpeedRefreshFlag = true;
  }
}

/*******************************************************************************
@ Func: 检测发动机状态
@ Param: None
@ Ret: ENGINE_STARTED/ENGINE_STOPPED/ENGINE_STARTING/ENGINE_STOPPING
*******************************************************************************/
void Engine_CheckState(void)
{
  bool isStartStopAction = false;
  static bool startActionFlag = false;
  static bool stopActionFlag = false;
  static unsigned long oldSpeed = 0;
  
  //首先判断发动机是否启停
  if((oldSpeed < ENGINE_STARTUP_SPEED) && (engineSpeedParams.new_speed >= ENGINE_STARTUP_SPEED)
     && (!munich_time.enging_on_time))
  {
    Engine_State = ENGINE_STARTING;
    startActionFlag = true;
    stopActionFlag = false;
    munich_time.enging_on_time = ENGINE_STARTSTOP_DELAY;
    
    oldSpeed = engineSpeedParams.new_speed;
    
    return;
  } 
  else if((oldSpeed >= ENGINE_STARTUP_SPEED) && (engineSpeedParams.new_speed < ENGINE_STARTUP_SPEED)
          && (!munich_time.enging_on_time))
  {
    Engine_State = ENGINE_STOPPING;
    stopActionFlag = true;
    startActionFlag = false;
    munich_time.enging_on_time = ENGINE_STARTSTOP_DELAY;
    
    oldSpeed = engineSpeedParams.new_speed;
    
    return;
  }
  
  if((!munich_time.enging_on_time) && (engineSpeedParams.new_speed >= ENGINE_STARTUP_SPEED))
  {
    Engine_State = ENGINE_STARTED;
  }
  else if((!munich_time.enging_on_time) && (engineSpeedParams.new_speed < ENGINE_STARTUP_SPEED))
  {
    Engine_State = ENGINE_STOPPED;
  }
}

/********************************************************************************************
@ Func:  仪表自检
********************************************************************************************/
void System_SelfCheckWhenElockOn(void)
{
  Led_Drive(0xffffffff);
  LED_BacklightOnOff(LED_ON);
  LED_UreaLevelBacklightOff();
  LED_WaterTempBacklightOff();
  LED_OilTempBacklightOff();
  LED_OilLevelBacklightOff();
  LED_LeftTurnLightOpen();
  LED_RightTurnLightOpen();
	Motor_SelfCheck(); 
	Led_Drive(0); 
	LED_LeftTurnLightClose();
	LED_RightTurnLightClose();
}

/********************************************************************************************
@ Func:  主要任务处理
********************************************************************************************/
void systemTasksProc(void)     
{
  //报警处理，设置LED状态及报警标志
  warningStateCheck();
	  
  if(!systemTask.w)
  {
    return;
  }
    
	if(systemTask.b.Ditigal_active)
	{
    //读入开关信号
    Digital_Drive();
    
    //电锁开关检测
    Elock_State = Elock_CheckState();

    if((ELOCK_JUST_TURN_OFF == Elock_State) || (ELOCK_OFF == Elock_State))
    {
      systemState.b.ign_on = FALSE;
      
      Buzzer_Close();                     //关闭蜂鸣器
      systemState.b.buzz_bit = FALSE;
      
      Motor_Handle(0);                    //电机归零
      
      Led_Drive(0);                       //关闭指示灯
      LCD_BACK_LIGHT_CLOSE();             //关闭LCD背光
      LCD_ClearScreen();                  //清屏，防止残压导致LCD屏幕残影
      LED_BacklightOnOff(LED_OFF);        //关闭仪表刻度盘背光
      LED_UreaLevelBacklightOff();
      LED_WaterTempBacklightOff();
      LED_OilTempBacklightOff();
      LED_OilLevelBacklightOff();
      DisableInterrupts;     
      PTP &= ~(1 << 2);                   //切断5V供电
      EnableInterrupts;
      DelayMS(200);
      //等待200MS后，容性负载已经放电完毕，
      //如果后面的程序还能执行说明电锁又重新打开，
      //因为已经进入了主程序，所以后面进行复位。
      if (PTT & (1 << 6))
      {
//        CPMUCOP = 0x47;
//        CPMUCLKS = 0x85;
        CPMUARMCOP = 0xbb; //写入非法值，使看门狗重启
        while (TRUE);
      }
    }
    else if(ELOCK_ON == Elock_State)
    {
      systemState.b.ign_on = TRUE;
    }
    else if(ELOCK_OFF == Elock_State)
    {
      systemState.b.ign_on = false;
    }
	}
	
	//更新车速比
	speedRatioCheck();
	
  if(ELOCK_ON == Elock_State)
  {
      //AD转换
      if(systemTask.b.Atd_active)
      {
        atd_handle();
        atd_manage();
      }
      
      //LED控制
      else if(systemTask.b.Led_active)
      {
        systemTask.b.Led_active = FALSE;
        Led_Handle();
      }
      
      //Flash存储处理
  		/*else if (systemTask.b.Flash_active && (!systemTask.b.motor_active))
  		{
  			Flash_Handle();
  		}*/
  		
  		//LCD显示处理
      else if(systemTask.b.Lcd_active)
      {  
        systemTask.b.Lcd_active = 0;
        LCD_Task();
      }
      
      //发动机转速计算
      if(systemTask.b.Pulse_active || (0 == CAN_EngineSpeedRecvTimeout))
      {
        systemTask.b.Pulse_active = false;        
        Speed_handle();
        Engine_CheckState();
      }

      
      
      //冷却液温处理
      if(systemTask.b.coolingLiquidTempProcessFlag || (0 == CAN_CoolingLiquidTempRecvTimeout))
      {
        systemTask.b.coolingLiquidTempProcessFlag = false;
        Engine_CoolingLiquidTempProcess();
      }
      
      //尿素液位处理
      if(systemTask.b.ureaLevelProcessFlag || (0 == CAN_UreaLevelRecvTimeout))
      {
        systemTask.b.ureaLevelProcessFlag = false;
        Engine_UreaLevelProcess();
      }
  }

  //步进电机旋转控制
  if(systemTask.b.motor_active)	
  {
    Motor_Handle(1);
  }
}


#ifdef OUT_WDT
/********************************************************************************************
@ 作用：使能外部看门狗,以及喂狗引脚
@ 说明：-
********************************************************************************************/
void En_out_WatchDog(void)
{
  DDRP |= 0x01;
  PTP &= 0xFE;
  DDRR |=0x10;
}
/********************************************************************************************
@ 作用：喂狗的实现
@ 说明：-
********************************************************************************************/
void Feed_WatchDog(void) 
{	
  unsigned i = 0;
  PTR |= 0x10;
  for(i=0;i<10;i++);
  PTR &= 0xEF;   
}
#endif
