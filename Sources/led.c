#include <hidef.h>	
#include <mc9s12hy64.h>
#include "board.h"
#include "app.h"
#include "cpu.h"
#include "led.h"

extern Uint32_t LED_OnOffBitsMap;

/************************************************************************************
@ Func:  LED初始化
@ Brief: LED驱动引脚与IO对应如下
         SIN   <--> PR0 
         SCLK  <--> PR1
         LAT   <--> PR2
         BLANK <--> PR3
         LED驱动电源开关控制          <--> PR4（新版中该控制引脚取消）
         表盘刻度背光灯控制引脚       <--> PT0 & PT1
         冷却液温指示符背光灯控制引脚 <--> PT4
         油温指示符背光灯控制引脚     <--> PT5
         燃油液位指示符背光灯控制引脚 <--> PT7
         尿素液位指示符背光灯控制引脚 <--> PH7
         左转向灯控制引脚 <--> PP5
         右转向灯控制引脚 <--> PP6
************************************************************************************/
void LED_IOInit(void)
{
	DDRR |= (1 << 0) + (1 << 1) + (1 << 2) + (1 << 3) + (1 << 4);
	PTR &= ~(1 << 4);
	
  DDRT |= (1 << 0) + (1 << 1) + (1 << 4) + (1 << 5) + (1 << 7);
  PTT &= ~((1 << 0) + (1 << 1)+ (1 << 4) + (1 << 5) + (1 << 7));
  
  DDRH |= (1 << 7);
  PTH &= ~(1 << 7);
  
  DDRP |= (1 << 5) + (1 << 6);
  PTP &= ~((1 << 5) + (1 << 6));	
	
	LED_BLANK = 1;
	
	Led_Drive(0);
}

/************************************************************************************
@ Func:  LED点亮与熄灭驱动
@ Brief: LED控制长字位对应的LED顺序如下
         [NC]   [NC]   [NC]   [LED49][LED48][NC]   [LED46][LED45]
         [NC]   [LED43][LED42][NC]   [LED40][LED39][LED38][LED37]
         [NC]   [NC]   [LED34][NC]   [NC]   [LED31][NC]   [LED29]
         [LED28][LED27][LED26][LED25][NC]   [LED23][LED50][LED21]
         各LED指示灯名称如下
         LED21——充电指示
         LED50——油温
         LED23——中央报警灯#1（中央报警灯由LED23、LED29两个组成）
         LED25——制动压力
         LED26——后处理故障
         LED27——驻车制动
         LED28——远光
         LED29——中央报警灯#2
         LED31——油水分离
         LED34——再生
         LED37——再生抑制
         LED38——变速油压
         LED39——水温高
         LED40——静音
         LED42——发动机故障
         LED43——燃油液位低
         LED45——再生温度
         LED46——预热
         LED48——尿素液位低
         LED49——机油压力低
************************************************************************************/
void Led_Drive(Uint32_t val)
{
	Uint8_t		led_time;
	Uint32_t	led_buff;
	
	LED_BLANK = 1;
	LED_LAT = 1;
	LED_LAT = 0;
	led_buff = 0x80000000;
	LED_CLK = 0;
	for(led_time = 0;led_time < 32;led_time++)
	{
		if(val&led_buff)
			LED_DATA = 1;
		else
			LED_DATA = 0;
		LED_CLK = 1;
		led_buff >>= 1;
		LED_CLK = 0;
	}
	LED_LAT = 1;
	LED_LAT = 0;
	LED_BLANK = 0; 
}

/************************************************************************************
@ Func: 打开左转向灯
************************************************************************************/
void LED_LeftTurnLightOpen(void)
{
  PTP |= (1 << 5);
}

/************************************************************************************
@ Func: 关闭左转向灯
************************************************************************************/
void LED_LeftTurnLightClose(void)
{
  PTP &= ~(1 << 5);
}

/************************************************************************************
@ Func: 打开右转向灯
************************************************************************************/
void LED_RightTurnLightOpen(void)
{
  PTP |= (1 << 6);
}

/************************************************************************************
@ Func: 关闭右转向灯
************************************************************************************/
void LED_RightTurnLightClose(void)
{
  PTP &= ~(1 << 6);
}

/************************************************************************************
@ Func: 仪表刻度盘背光灯开关
************************************************************************************/
void LED_BacklightOnOff(LED_Action_t action)
{
  switch(action)
  {
    case LED_ON:
      PTT |= (1 << 0) + (1 << 1);
      
      //尿素报警灯、水温报警灯、油温报警灯、燃油液位报警灯报警时，其白色背光灯应熄灭
      if(!(LED_OnOffBitsMap & LED_UREA_LEVEL))
      {
        LED_UreaLevelBacklightOn();
      }
      else
      {
        LED_UreaLevelBacklightOff();
      }
      
      if(!(LED_OnOffBitsMap & LED_WATER_TEMP))
      {
        LED_WaterTempBacklightOn();
      }
      else
      {
        LED_WaterTempBacklightOff();
      }
      
      if(!(LED_OnOffBitsMap & LED_OIL_TEMP))
      {
        LED_OilTempBacklightOn();
      }
      else
      {
        LED_OilTempBacklightOff();
      }
      
      if(!(LED_OnOffBitsMap & LED_FUEL_LOW))
      {
        LED_OilLevelBacklightOn();
      }
      else
      {
        LED_OilLevelBacklightOff();
      }
    break;
    
    case LED_OFF:
      PTT &= ~((1 << 0) + (1 << 1));
      LED_UreaLevelBacklightOff();
      LED_WaterTempBacklightOff();
      LED_OilTempBacklightOff();
      LED_OilLevelBacklightOff();
    break;
    
    case LED_TOGGLE:
    break;
    
    default:
    break;
  }
}

/************************************************************************************
@ Func: 尿素表指示符背光打开
************************************************************************************/
void LED_UreaLevelBacklightOn(void)
{
  PTH |= (1 << 7); 
}

/************************************************************************************
@ Func: 尿素表指示符背光关闭
************************************************************************************/
void LED_UreaLevelBacklightOff(void)
{
  PTH &= ~(1 << 7);
}

/************************************************************************************
@ Func: 水温表指示符背光打开
************************************************************************************/
void LED_WaterTempBacklightOn(void)
{
  PTT |= (1 << 4);
}

/************************************************************************************
@ Func: 水温表指示符背光关闭
************************************************************************************/
void LED_WaterTempBacklightOff(void)
{
  PTT &= ~(1 << 4);
}

/************************************************************************************
@ Func: 油温表指示符背光打开
************************************************************************************/
void LED_OilTempBacklightOn(void)
{
  PTT |= (1 << 5);
}

/************************************************************************************
@ Func: 油温表指示符背光关闭
************************************************************************************/
void LED_OilTempBacklightOff(void)
{
  PTT &= ~(1 << 5);
}

/************************************************************************************
@ Func: 燃油液位表指示符背光打开
************************************************************************************/
void LED_OilLevelBacklightOn(void)
{
  PTT |= (1 << 7);
}

/************************************************************************************
@ Func: 燃油液位表指示符背光关闭
************************************************************************************/
void LED_OilLevelBacklightOff(void)
{
  PTT &= ~(1 << 7);
}
