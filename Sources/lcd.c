#include <hidef.h>	
#include <mc9s12hy64.h>
#include <math.h>
#include "app.h"
#include "board.h"
#include "CAN.h"
#include "vehicle.h"
#include "can_error.h"
#include "time.h"
#include "lcd.h"
#include "lcd_zimo.h"
#include "digital.h"
#include "tim1_capture.h"
#include "fontImage.h"
#include "character.h"

extern Uint32_t CAN_FaultBitMap;
extern Uint32_t LED_OnOffBitsMap;
extern queue_list systemTask; 
extern Engine_State_t Engine_State;

//CAN_GPSWarningBitsMap各Bit含义
//[I don't know]  [I don't know]  [NC]           [NC]             [NC][NC]                   [NC][GPRS61天无信号]
//[GPRS58天无信号][GPRS51天无信号][Web端强制锁车][ECU无法正常工作][NC][锁车状态下KEY值不正确][NC][KEY值验证不正确]
Uint16_t  CAN_GPSWarningBitsMap;
Uint16_t  CAN_GPSWarningBitsMapOld;


//新增一个车速显示标志位，如果存在车速信号切换则置1.old标志位用于是否更新
unsigned char  CarSpeedDisFlag    = 0;
unsigned char  CarSpeedDisFlagOld = 0;

unsigned long LCD_OldCanFaultSpnFmi = 0;

//LCD显示控制 
LCD_DisplayCtrl_t LCD_DisplayCtrl =
{
  LCD_DISPLAY_LOGO,     //显示界面类型
  LCD_DISPLAY_LOGO,     //上次显示界面类型
  false,                //显示类型改变标志，用于切换不同显示界面时
  200,                  //定时刷新计数器
  false,                //刷新标志
  0,                    //任务执行周期
};

//沙漏闪烁控制
LCD_HourglassCtrl_t LCD_HourglassCtrl =
{
  false,
  0,
  false,
  LCD_GLASSHOUR_SHOW,
};

//LCD正常工作界面显示控制
LCD_NormalDisplayCtrl_t LCD_NormalDisplayCtrl =
{
  false,
  0,
  false,
  0,
  false,
  false,
};

//LCD GPS报警类型
LCD_GPSWarningDisplayCtrl_t LCD_GPSWarningDisplayCtrl =
{
  LCD_GPS_WARNING_NONE,
  LCD_GPS_WARNING_NONE,
  ENGINE_STOPPED,
};

//DTC显示控制
CAN_FaultDisplayCtrl_t CAN_FaultDisplayCtrl =
{
  0,        //故障总数
  0,        //上次故障总数
  
  0,        //起始搜索位置
  
  0,        //刷新超时计数器
  false,    //刷新标志
  
  true,     //第一次显示故障界面标志
  
  {0},      //要显示的DTC在缓冲区中的位置
  {0},      //当前显示的3个DTC缓冲区
  {0},      //上次显示的3个DTC缓冲区
};

//NCD报警控制
LCD_NCDWarningDisplayCtrl_t LCD_NCDWarningDisplayCtrl =
{
  LCD_NCD_WARNING_NONE,
  LCD_NCD_WARNING_NONE,
  false,
};

//LCD再生提醒控制
LCD_REGENERATE_DisplayCtrl_t LCD_REGENERATE_DisplayCtrl=
{
  LCD_NO_REMIND,
  LCD_NO_REMIND,
//  FALSE,
};


//故障显示界面，NCD报警显示标志（无DTC时，在LCD中央显示，小时计在LCD下方显示；有DTC时，NCD在LCD下方显示，小时计不显示）
bool ncdChangeDisplayAreaFlag;
//故障显示界面，需显示小时计标志
bool hourmeterNeedShowFlag;
//再生服务提醒消失后，刷新油耗和温度界面显示
bool Refresh_fuelConsumption_and_ambientAirTemp = FALSE;



/*********************************************************************************
@ Func:  LCD初始化
@ Brief: 8088接口
		     D0~D7 	<--> PA0~PA7
		     RWR    <--> PB1
		     EDR    <--> PB2
         A0     <--> PB3
         RST    <--> PB4
         CS2    <--> PB5
         CS1    <--> PB6
*********************************************************************************/
void LCD_GPIOInit(void)
{
  //相应IO口设置为输出
  DDRA = 0xff;
  //DDRB = 0xff;
  DDRB |= 0xfe;
  
  //初始化需要关闭LCD背光，写入开机Logo后再打开背光
  LCD_BACK_LIGHT_CLOSE();
}

/*********************************************************************************
@ Func: LCD配置
*********************************************************************************/
void LCD_Config(void)
{
	LCD_HardwareReset();
	LCD_InterfaceEnable();

	LCD_InverseDisplay(LCD_INVERSE_DISPLAY_NORMAL);
	//LCD_SetAllPixelDisplay(LCD_ALL_PIXEL_NORMAL);

	LCD_SetSegDirection(LCD_SEG_DIRECTION_REVERSE);
	LCD_SetComDirection(LCD_COM_DIRECTION_NORMAL);

	//设置DDRAM第20行映射到COM0
	LCD_SetStartLine(0);
	//第20个Pad输出COM0信号，该液晶有112个COM，而DDRAM有132行，132-112=20
	LCD_SetCOM0(0x14);
	LCD_SetDisplayLine(112);
  LCD_SetDriveMode(LCD_DRIVE_MODE_DMP_OFF, LCD_DRIVE_MODE_NFLG_NORMAL);
	LCD_SetFrameRate(9, 7, 6, 6);
	LCD_SetNLine(1);

	LCD_WriteCmd(0xf9);
  LCD_TCOn();
	LCD_SetTCCurve(4, 4, 3, 3, 2, 2, 1, 0, 0, 1, 2, 3, 4, 7, 8, 8);
	LCD_SetTCFlag(0x00, 0x00);
	LCD_SetTempAToC(0x1e, 0x28, 0x32);
	LCD_WriteCmd(0xf8);

	LCD_OscillatorOn();
	LCD_SetTCSensorSpeed(LCD_TC_SENSOR_SPEED_NORMAL);


	LCD_SetBooster(LCD_BOOSTER_4_VDD2);

	//LCD_SetVop(180); 
  LCD_SetVop(0XC3);         

	LCD_BiasSelect(LCD_BIAS_ONE_SIXTH);
	//LCD_PowerControl(LCD_POWER_ALL_EXTERNAL);
	LCD_PowerControl(LCD_POWER_ALL_INTERNAL);
	
	LCD_ClearScreen();
	LCD_DisplaySDLGLogo();
	LCD_DisplayOn();
	LCD_BACK_LIGHT_OPEN();
}


/*************************************************************************
@ Func: LCD发送命令
*************************************************************************/
void LCD_WriteCmd(unsigned char cmd)
{
	LCD_CMD();							//发送命令

#ifdef MPU_8080	
	LCD_EDR_SET();
	LCD_RWR_CLR();					//Set RWR low.
#else
	LCD_RWR_SET();
	LCD_EDR_SET();
#endif
	
	PORTA = cmd;

#ifdef MPU_8080	
	LCD_RWR_SET();					//Signals on D[7:0] will be latched at the rising edge of /WR signal.
#else
	LCD_RWR_CLR();
	LCD_EDR_CLR();
#endif
}

/*************************************************************************
@ Func: LCD发送数据
*************************************************************************/
void LCD_WriteData(unsigned char data)
{
	LCD_DATA();							//发送命令
	
#ifdef MPU_8080	
	LCD_EDR_SET();
	LCD_RWR_CLR();					//Set RWR low.
#else
	LCD_RWR_SET();
	LCD_EDR_SET();
#endif
	
	PORTA = data;
	
#ifdef MPU_8080	
	LCD_RWR_SET();					//Signals on D[7:0] will be latched at the rising edge of /WR signal.
#else
	LCD_RWR_CLR();
	LCD_EDR_CLR();
#endif
}

/****************************************************************************
@ Func:  使能通信接口D7~D0
@ Brief: PC5 <--> CS1
         PC4 <--> CS2
         Interface is enabled when both CS1B is “L” and CS2 is “H”. If chip 
         select pins are not active (CS1B=“H” or CS2=“L”), D0 to D7 become 
         high impedance.
****************************************************************************/
void LCD_InterfaceEnable(void)
{
	LCD_CS1_CLR();
	LCD_CS2_SET();
}

/****************************************************************************
@ Func:  禁止通信接口D7~D0
@ Brief: PC5 <--> CS1
         PC4 <--> CS2
         Interface is enabled when both CS1B is “L” and CS2 is “H”. If chip 
         select pins are not active (CS1B=“H” or CS2=“L”), D0 to D7 become 
         high impedance.
****************************************************************************/
void LCD_InterfaceDisable(void)
{
	LCD_CS1_SET();
	LCD_CS1_CLR();
}

/****************************************************************************
@ Func:  设置页地址
@ Brief: 页地址范围为0~16，第16页D0~D3有效
****************************************************************************/
void LCD_SetPageAddr(unsigned char page)
{
	LCD_WriteCmd(0x7c);
	LCD_WriteCmd(page);
}

/****************************************************************************
@ Func:  设置页地址
@ Brief: 页地址范围为0~159
****************************************************************************/
void LCD_SetColumnAddr(unsigned char column)
{
	LCD_WriteCmd(0x10 | ((column & 0xf0) >> 4));
	LCD_WriteCmd(column & 0x0f);
}

/****************************************************************************
@ Func:  设置内置电源发生器
@ Brief: NVDD Generator ON，V3 Generator OFF，V1 Generator OFF，MV3 & MV1 
         Generator OFF
****************************************************************************/
void LCD_PowerControl(LCD_VopGenerateMethod_t method)
{
	switch(method)
	{
		case LCD_POWER_ALL_INTERNAL:
			LCD_WriteCmd(0x98);
			DelayMS(20);
			LCD_WriteCmd(0x9c);
			DelayMS(50);
			LCD_WriteCmd(0x9e);
			DelayMS(20);
			LCD_WriteCmd(0x9f);
			DelayMS(40);
			break;
		
		case LCD_POWER_EXTERNAL_V3:
			LCD_WriteCmd(0x98);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			LCD_WriteCmd(0x9b);
			DelayMS(40);
			break;
		
		case LCD_POWER_EXTERNAL_V3_V1:
			LCD_WriteCmd(0x98);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			LCD_WriteCmd(0x99);
			DelayMS(40);
			break;
		
		case LCD_POWER_ALL_EXTERNAL:
			LCD_WriteCmd(0x98);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			DelayMS(20);
			break;
		
		default:
			break;
	}
}

/****************************************************************************
@ Func:  硬件复位
****************************************************************************/
void LCD_HardwareReset(void)
{
	LCD_RST_CLR();
	DelayMS(20);
	LCD_RST_SET();
	DelayMS(10);
}

/****************************************************************************
@ Func:  软件复位
****************************************************************************/
void LCD_SoftwareReset(void)
{
	LCD_WriteCmd(0xe1);
	DelayMS(10);
}

/****************************************************************************
@ Func:  打开内部晶振
****************************************************************************/
void LCD_OscillatorOn(void)
{
	LCD_WriteCmd(0xab);
}

/****************************************************************************
@ Func:  关闭内部晶振
****************************************************************************/
void LCD_OscillatorOff(void)
{
	LCD_WriteCmd(0xaa);
}

/****************************************************************************
@ Func:  打开反显
****************************************************************************/
void LCD_InverseDisplayOn(void)
{
	LCD_WriteCmd(0xa7);
}

/****************************************************************************
@ Func:  关闭反显
****************************************************************************/
void LCD_InverseDisplayOff(void)
{
	LCD_WriteCmd(0xa6);
}

/****************************************************************************
@ Func:  设置DDRAM起始行，该行将映射到COM0
****************************************************************************/
void LCD_SetStartLine(unsigned char lineNbr)
{
	LCD_WriteCmd(0xd0);
	LCD_WriteCmd(lineNbr);
}

/****************************************************************************
@ Func:  设置COM0在第几个Pad输出
****************************************************************************/
void LCD_SetCOM0(unsigned char padNbr)
{
	LCD_WriteCmd(0xd3);
	LCD_WriteCmd(padNbr);
}

/****************************************************************************
@ Func:  设置显示行数
****************************************************************************/
void LCD_SetDisplayLine(unsigned char num)
{
	LCD_WriteCmd(0xdc);
	LCD_WriteCmd(num);
}

/****************************************************************************
@ Func:  偏置电压设置
****************************************************************************/
void LCD_BiasSelect(LCD_BiasSelect_t bias)
{
	switch(bias)
	{
		case LCD_BIAS_ONE_THIRD:
			LCD_WriteCmd(0x50);
		break;

		case LCD_BIAS_ONE_FORTH:
			LCD_WriteCmd(0x52);
		break;

		case LCD_BIAS_ONE_FIFTH:
			LCD_WriteCmd(0x53);
		break;

		case LCD_BIAS_ONE_SIXTH:
			LCD_WriteCmd(0x54);
		break;

		case LCD_BIAS_ONE_SEVENTH:
			LCD_WriteCmd(0x55);
		break;

		default:
		break;
	}
}

void LCD_SetBooster(LCD_BoosterSelect_t booster)
{
	switch(booster)
	{
		case LCD_BOOSTER_4_VDD2:
			LCD_WriteCmd(0x64);
		break;

		case LCD_BOOSTER_6_VDD2:
			LCD_WriteCmd(0x65);
		break;

		case LCD_BOOSTER_8_VDD2:
			LCD_WriteCmd(0x66);
		break;

		default:
		break;
	}
}

void LCD_SetVop(unsigned char vop)
{
	LCD_WriteCmd(0x81);
	LCD_WriteCmd(vop);
}

/****************************************************************************
@ Func:  列扫描方向设置
****************************************************************************/
void LCD_SetSegDirection(LCD_SegDirection_t direction)
{
	if(LCD_SEG_DIRECTION_NORMAL == direction)
	{
		LCD_WriteCmd(0xa0);
	}
	else if(LCD_SEG_DIRECTION_REVERSE == direction)
	{
		LCD_WriteCmd(0xa1);
	}
}

/****************************************************************************
@ Func:  设置所有像素显示模式
****************************************************************************/
void LCD_SetAllPixelDisplay(LCD_AllPixelDisplayMode_t mode)
{
	if(LCD_ALL_PIXEL_NORMAL == mode)
	{
		LCD_WriteCmd(0xa4);	
	}
	else if(LCD_ALL_PIXEL_ON == mode)
	{
		LCD_WriteCmd(0xa5);
	}
}

/****************************************************************************
@ Func:  反显设置
****************************************************************************/
void LCD_InverseDisplay(LCD_InverseDisplayMode_t inverse)
{
	if(LCD_INVERSE_DISPLAY_NORMAL == inverse)
	{
		LCD_WriteCmd(0xa6);
	}
	else if(LCD_INVERSE_DISPLAY_INVERSE == inverse)
	{
		LCD_WriteCmd(0xa7);	
	}
}

/****************************************************************************
@ Func:  使能显示
****************************************************************************/
void LCD_DisplayOn(void)
{
	LCD_WriteCmd(0xa3);	
}

/****************************************************************************
@ Func:  关闭显示
****************************************************************************/
void LCD_DisplayOff(void)
{
	LCD_WriteCmd(0xa2);	
}

/****************************************************************************
@ Func:  省电模式设置
****************************************************************************/
void LCD_SetPowerSave(LCD_PowerSaveMode_t mode)
{
	if(LCD_POWER_SAVE_NORMAL == mode)
	{
		LCD_WriteCmd(0x58);
	}
	else if(LCD_POWER_SAVE_STADBY == mode)
	{
		LCD_WriteCmd(0x59);	
	}
}

/****************************************************************************
@ Func:  电容放电设置
****************************************************************************/
void LCD_DischargeOn(void)
{
	LCD_WriteCmd(0x36);	
}

void LCD_DischargeOff(void)
{
	LCD_WriteCmd(0x37);	
}

/****************************************************************************
@ Func:  行扫描方向设置
****************************************************************************/
void LCD_SetComDirection(LCD_ComDirection_t direction)
{
	if(LCD_COM_DIRECTION_NORMAL == direction)
	{
		LCD_WriteCmd(0xc0);
	}
	else if(LCD_COM_DIRECTION_INVERSE == direction)
	{
		LCD_WriteCmd(0xc8);
	}
}

/****************************************************************************
@ Func:  进入写时列地址递增模式
****************************************************************************/
void LCD_EnterReadModifyWriteMode(void)
{
	LCD_WriteCmd(0xe0);
}

/****************************************************************************
@ Func:  退出写时列地址递增模式
****************************************************************************/
void LCD_ExitReadModifyWriteMode(void)
{
	LCD_WriteCmd(0xee);
}


void LCD_SetFrameRate(unsigned char fa, unsigned char fb, unsigned char fc, unsigned char fd)
{
	LCD_WriteCmd(0x30);
	LCD_WriteCmd((fb << 4) + fa);
	LCD_WriteCmd(0x31);
	LCD_WriteCmd((fd << 4) + fc);
}


void LCD_SetNLine(unsigned char lineNum)
{
	LCD_WriteCmd(0x4c);
	LCD_WriteCmd(1);
}
/****************************************************************************
@ Func:  驱动模式设置
****************************************************************************/
void LCD_SetDriveMode(LCD_DriveModeDMPSelect_t dummy, LCD_DriveModeNFLGSelect_t nflg)
{
	LCD_WriteCmd(0xfd);
	LCD_WriteCmd(0x71);
	if(LCD_DRIVE_MODE_DMP_ON == dummy)
	{
		LCD_WriteCmd(0x69);
	}
	else if(LCD_DRIVE_MODE_DMP_OFF == dummy)
	{
		LCD_WriteCmd(0x79);
	}
	LCD_WriteCmd(0x77);
	if(LCD_DRIVE_MODE_NFLG_NORMAL == nflg)
	{
		LCD_WriteCmd(0x62);
	}
	else if(LCD_DRIVE_MODE_NFLG_RESET_AFTER_SCANNING == nflg)
	{
		LCD_WriteCmd(0xe2);
	}
	LCD_WriteCmd(0xf8);
}

void LCD_TCOn(void)
{
	//LCD_WriteCmd(0x61);
  LCD_WriteCmd(0x69);
}

void LCD_TCOff(void)
{
	LCD_WriteCmd(0x60);
}

void LCD_SetTCCurve(unsigned char mt0, unsigned char mt1, unsigned char mt2, unsigned char mt3, unsigned char mt4,
                    unsigned char mt5, unsigned char mt6, unsigned char mt7, unsigned char mt8, unsigned char mt9,
					          unsigned char mta, unsigned char mtb, unsigned char mtc, unsigned char mtd, unsigned char mte,
					          unsigned char mtf)
{
	LCD_WriteCmd(0x30);
	LCD_WriteCmd((mt1 << 4) + mt0);
	LCD_WriteCmd(0x31);
	LCD_WriteCmd((mt3 << 4) + mt2);
	LCD_WriteCmd(0x32);
	LCD_WriteCmd((mt5 << 4) + mt4);
	LCD_WriteCmd(0x33);
	LCD_WriteCmd((mt7 << 4) + mt6);
	LCD_WriteCmd(0x34);
	LCD_WriteCmd((mt9 << 4) + mt8);
	LCD_WriteCmd(0x35);
	LCD_WriteCmd((mtb << 4) + mta);
	LCD_WriteCmd(0x36);
	LCD_WriteCmd((mtd << 4) + mtc);
	LCD_WriteCmd(0x37);
	LCD_WriteCmd((mtf << 4) + mte);
}

void LCD_SetTCFlag(unsigned char fmt0To7, unsigned char fmt8ToF)
{
	LCD_WriteCmd(0x3e);
	LCD_WriteCmd(fmt0To7);
	LCD_WriteCmd(0x3f);
	LCD_WriteCmd(fmt8ToF);
}

void LCD_SetTempAToC(unsigned char ta, unsigned char tb, unsigned char tc)
{
	LCD_WriteCmd(0xa0);
	LCD_WriteCmd(ta);
	LCD_WriteCmd(0xa1);
	LCD_WriteCmd(tb);
	LCD_WriteCmd(0xa2);
	LCD_WriteCmd(tc);
}

void LCD_SetTCSensorSpeed(LCD_TCSensorSpeedMode_t mode)
{
	LCD_WriteCmd(0xfd);
	LCD_WriteCmd(0x70);
	if(LCD_TC_SENSOR_SPEED_NORMAL == mode)
	{
		LCD_WriteCmd(0x21);
	}
	else if(LCD_TC_SENSOR_SPEED_FAST == mode)
	{
		LCD_WriteCmd(0x29);
	}
	LCD_WriteCmd(0xf8);
}

/****************************************************************************
@ Func:  清屏
@ Brief: DDRAM从SEG17开始显示在屏幕上，所以只要写DDRAM的SEG16~SEG143即可（索
         引从0开始）
****************************************************************************/
void LCD_ClearScreen(void)
{
	unsigned char i = 0;
	unsigned char j = 0;

	for(i = 0; i < 14; ++i)
	{
		LCD_SetPageAddr(i);
		for(j = 0; j < 128; ++j)
		{
			LCD_SetColumnAddr(j + 16);
			LCD_WriteData(0);
		}
	}
}

/****************************************************************************
@ Func:  点亮所有像素
@ Brief: DDRAM从SEG17开始显示在屏幕上，所以只要写DDRAM的SEG16~SEG143即可（索
         引从0开始）
****************************************************************************/
void LCD_SetScreen(void)
{
	unsigned char i = 0;
	unsigned char j = 0;

	for(i = 0; i < 14; ++i)
	{
		LCD_SetPageAddr(i);
		for(j = 0; j < 128; ++j)
		{
			LCD_SetColumnAddr(j + 16);
			LCD_WriteData(0xff);
		}
	}
}

/****************************************************************************
@ Func:  显示一个图案
@ Brief: 图案可以是字母、数字、汉字及图片等
****************************************************************************/
void LCD_DisplayItem(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn, const unsigned char *p)
{
	unsigned char i = 0;
	unsigned char j = 0;
	
	for(i = 0; i < (comSize / 8); ++i)
	{
	  LCD_SetPageAddr(i + startPage);
	  for(j = 0; j < segSize; ++j)
	  {
	    LCD_SetColumnAddr(16 + j + startColumn);   //16为DRAM无效的长度
	    LCD_WriteData(*(p + i * segSize + j));
	  }
	}
}

/****************************************************************************
@ Func:  显示一个图案
@ Brief: 图案可以是字母、数字、汉字及图片等，并且可以上下平移
****************************************************************************/
void LCD_DisplayItemX(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn, const unsigned char *p,
                      char shift)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char temp = 0;
	
	if(shift >= 0)
	{
  	for(i = 0; i < (comSize / 8); ++i)
  	{
  	  LCD_SetPageAddr(i + startPage);
  	  for(j = 0; j < segSize; ++j)
  	  {
  	    LCD_SetColumnAddr(16 + j + startColumn);   //16为DRAM无效的长度
  	    if(i >= 1)
  	    {
  	      temp = ((*(p + i * segSize + j)) << shift) + ((*(p + (i - 1) * segSize + j) >> (8 - shift))& (0xff >> (8 - shift)));
  	    }
  	    else
  	    {
  	      temp = *(p + i * segSize + j) << shift;
  	    }
  	    LCD_WriteData(temp);
  	  }
  	}
	}
	else
	{
	  shift = -shift;
	  for(i = 0; i < (comSize / 8); ++i)
	  {
	    LCD_SetPageAddr(i + startPage);
	    for(j = 0; j < segSize; ++j)
	    {
  	    LCD_SetColumnAddr(16 + j + startColumn);   //16为DRAM无效的长度
  	    if(i < (comSize / 8 - 1))
  	    {
  	      temp = ((*(p + i * segSize + j)) >> shift) + ((*(p + (i + 1) * segSize + j) << (8 - shift)) & (0xff << (8 - shift)));
  	    }
  	    else
  	    {
  	      temp = *(p + i * segSize + j) >> shift;
  	    }
  	    LCD_WriteData(temp);
	    }
	  }
	}
}

/****************************************************************************
@ Func:  清除一个图案
@ Param: segSize     <-> 列宽
         comSize     <-> 行高
         startPage   <-> 起始页
         startColumn <-> 起始列
****************************************************************************/
void LCD_ClearItem(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn)
{
	unsigned char i = 0;
	unsigned char j = 0;
	
	for(i = 0; i < (comSize / 8); ++i)
	{
	  LCD_SetPageAddr(i + startPage);
	  for(j = 0; j < segSize; ++j)
	  {
	    LCD_SetColumnAddr(16 + j + startColumn);   //16为DRAM无效的长度
	    LCD_WriteData(0);
	  }
	}
}

/****************************************************************************
@ Func:  显示开机Logo与软件版本号
@ Brief: DDRAM从SEG17开始显示在屏幕上，所以只要写DDRAM的SEG16~SEG143即可（索
         引从0开始）
****************************************************************************/
void LCD_DisplaySDLGLogo(void)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char temp = 0;
	unsigned char startPage = 0;
	unsigned char startColumn = 0;
	
	for(i = 0; i < 8; ++i)
	{
		LCD_SetPageAddr(i + 2);
		for(j = 0; j < 64; ++j)
		{
			LCD_SetColumnAddr(j + 16 + 32);
			temp = reverseByteOrder(gImage_SDLGLogo[i * 64 + j]);
			LCD_WriteData(temp);
		}
	}
	
	startColumn = 100;
	startPage = 12;
  LCD_DisplayItemX(7, 16, startPage, startColumn, LCD_CharArray_7Row16Column_NumberSix_V, 2);
  startColumn += 7;
  LCD_DisplayItemX(7, 16, startPage, startColumn, LCD_DigitArray_16Row7Column_NumberSix_1, 2);
  startColumn += 7;
  LCD_DisplayItemX(7, 16, startPage, startColumn, LCD_CharArray_16Row7Column_Dot, -1);
  startColumn += 4;
  LCD_DisplayItemX(7, 16, startPage, startColumn, LCD_DigitArray_16Row7Column_NumberSix_0, 2);
  	
}




/***********************************************************************************
@ Func:  仅显示发动机转速，不显示车速
@ Brief: 在无法收取到车速脉冲时不显示车速，仅显示转速，由于字体不同，所以单独列出
***********************************************************************************/
void LCD_DisplayEngineSpeedOnly(unsigned int speed)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char k = 0;
	unsigned char temp = 0;
	unsigned char engineSpeedDigit = 0;
	unsigned char column = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char page = 0;
	
	unsigned int spd = speed;
  //异常数据检查，当转速超过3000时，转速显示3000  By周文熙 20200804
  if(spd > 3000)
     spd = 3000;
  
  
	//显示转速xxxx
	height = 32;
	width = 16;
	page = 1;
	column = 9;
	
	for(i = 0; i < 4; ++i)
	{
    engineSpeedDigit = (spd % (unsigned long)pow(10, 4 - i)) / (unsigned long)pow(10, 3 - i);
    
	  for(j = page; j < (page + height / 8); ++j)
	  {
	    LCD_SetPageAddr(j);
	    for(k = 0; k < width; ++k)
	    {
	      LCD_SetColumnAddr(k + 16 + column);
	      temp = LCD_DigitArray_16Seg_32Com[engineSpeedDigit][(j - page) * width + k];
	      if(0 == i)
	      {
	        if(!engineSpeedDigit)
	        {
    	      temp = 0;
	        }
	      }
	      LCD_WriteData(temp);
	    }
	  }
	  
	  column += width;
	  column += 1;
	}
	
	//显示转速单位n/M
	height = 16;
	width = 8;
	page = 2;
	column += 5;

  LCD_DisplayItem(width, height, page, column, LCD_CharArray_8Seg_16Com_n);	
	
	column += width;
	column += 1;
  LCD_DisplayItem(width, height, page, column, LCD_CharArray_16Row8Column_Slash);	
	
	column += width;
	column += 1;
  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_m, 3);	

	column += width; 
  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_i, 3);	

	column += width;
	LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_n, 3);
}

/***********************************************************************************
@ Func:  显示发动机转速
@ Brief: 显示格式<-->xxxx n/M，发动机转速最大值为65535/8=8192rpm，所以LCD足以容纳最
         大转速显示
***********************************************************************************/
void LCD_DisplayEngineSpeed(unsigned int speed)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char k = 0;
	unsigned char temp = 0;
	unsigned char engineSpeedDigit = 0;
	unsigned char column = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char page = 0;
	
	unsigned int spd = speed;
  //异常数据检查，当转速超过3000时，转速显示3000  By周文熙 20200804
  if(spd > 3000)
     spd = 3000;


	//显示转速xxxx
	height = 24;
	width = 12;
	page = 1;
	column = 9;
	
	for(i = 0; i < 4; ++i)
	{
    engineSpeedDigit = (spd % (unsigned long)pow(10, 4 - i)) / (unsigned long)pow(10, 3 - i);
    
	  for(j = page; j < (page + height / 8); ++j)
	  {
	    LCD_SetPageAddr(j);
	    for(k = 0; k < width; ++k)
	    {
	      LCD_SetColumnAddr(k + 16 + column);
	      temp = LCD_DigitArray_12Seg_24Com[engineSpeedDigit][(j - page) * width + k];
	      if(0 == i)
	      {
	        if(!engineSpeedDigit)
	        {
    	      temp = 0;
	        }
	      }
	      LCD_WriteData(temp);
	    }
	  }
	  
	  column += width;
	  column += 1;
	}
	
	//显示转速单位n/M
	height = 16;
	width = 8;
	page = 1;
	column += 5;

  LCD_DisplayItem(width, height, page, column, LCD_CharArray_8Seg_16Com_n);	
	
	column += width;
	column += 1;
  LCD_DisplayItem(width, height, page, column, LCD_CharArray_16Row8Column_Slash);	
	
	column += width;
	column += 1;
  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_m, 3);	

	column += width; 
  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_i, 3);	

	column += width;
	LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_n, 3);
}



/***********************************************************************************
@ Func:  显示整车车速
@ Brief: 显示格式<-->xx.x km/h，有效值为0-60km/h，超出60则显示60
***********************************************************************************/
void LCD_DisplayVehicleSpeed(unsigned int spd)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char k = 0;
	unsigned char temp = 0;
	unsigned char vehicleSpeedDigit = 0;
	unsigned char column = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char page = 0;
	
	//显示车速xx.x
	height = 24;
	width = 12;
	page = 4;
	column = 22;
	
	//显示车速
	for(i = 0; i < 3; i++)
	{
    vehicleSpeedDigit = (spd % (unsigned long)pow(10, 3 - i)) / (unsigned long)pow(10, 2 - i);
    
	  for(j = page; j < (page + height / 8); ++j)
	  {
	    LCD_SetPageAddr(j);
	    for(k = 0; k < width; ++k)
	    {
	      LCD_SetColumnAddr(k + 16 + column);
	      temp = LCD_DigitArray_12Seg_24Com[vehicleSpeedDigit][(j - page) * width + k];
	      LCD_WriteData(temp);
	    }
	  }
	  column += width;
	  column += 1;
	  //显示小数点
	  if(i==1) 
	  {	  
	    LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	    column += 8;
	  }
	}
	
	//显示转速单位km/h
	height = 16;
	width = 8;
	page = 4;
	column += 5;

  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_8Seg_16Com_K,1);	
  	
  column += width;
	column += 1;
  LCD_DisplayItemX(width, height, page, column, LCD_CharArray_16Row8Column_m,3);
	
	column += width;
	column += 1;
  LCD_DisplayItem(width, height, page, column, LCD_CharArray_16Row8Column_Slash);		
	
	column += width; 
 	LCD_DisplayItem(width, height, page, column, LCD_CharArray_16Row8Column_h);
}
/***********************************************************************************
@ Func:  沙漏显示
***********************************************************************************/
void LCD_DisplayHourglass(unsigned char page, unsigned char column, LCD_HourglassState_t state)
{
	if(LCD_GLASSHOUR_SHOW == state)
	{
	  LCD_DisplayItem(9, 16, page, column, Image_Hourglass_9Seg16Row);
	}
	else
	{
	  LCD_ClearItem(9, 16, page, column);
	}
}

void LCD_DisplayHourglassX(unsigned char page, unsigned char column, LCD_HourglassState_t state, int shift)
{
	if(LCD_GLASSHOUR_SHOW == state)
	{
	  LCD_DisplayItemX(9, 16, page, column, Image_Hourglass_9Seg16Row, shift);
	}
	else
	{
	  LCD_ClearItem(9, 16, page, column);
	}
}

/***********************************************************************************
@ Func:  显示总工作小时数
@ Brief: 从Page11开始显示，占用2个Page，显示格式为“沙漏符号 + XXXXXX.Xh”
***********************************************************************************/
void LCD_ShowTotalWorkTime(unsigned char page, unsigned char column)
{
	unsigned char i = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char workingHourDigit = 0;
	
  height = 16;
  width = 9;

  //显示沙漏符号
	LCD_DisplayHourglass(page, column, LCD_HourglassCtrl.displayState);
	
	column += width;
	column += 5;
	height = 16;
	width = 8;
	//显示小数点之前六位数字
	for(i = 0; i < 5; ++i)
	{
    workingHourDigit = (can_data.engineWorkingTime % (unsigned long)pow(10, 6 - i)) / (unsigned long)pow(10, 5 - i);
    LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[workingHourDigit]);
	  
	  column += width;
	  column += 1;
	}
	
	height = 16;
	width = 7;
	//显示小数点
	LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	
  column += width;
  column += 1;
	
	height = 16;
	width = 8;
	//显示小数点之后一位数字
	LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[can_data.engineWorkingTime % 10]);
  
  column += width;
  column += 1;
  column += 2;
  
  height = 16;
  width = 8;
  //width += 4;
  
  //显示字母“ｈ”
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_16Row8Column_h);
}

void LCD_ShowTotalWorkTimeX(unsigned char page, unsigned char column, int shift)
{
	unsigned char i = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char workingHourDigit = 0;
	
  height = 16;
  width = 9;

  //显示沙漏符号
	LCD_DisplayHourglassX(page, column, LCD_HourglassCtrl.displayState, shift);
	
	column += width;
	column += 5;
	height = 16;
	width = 8;
	//显示小数点之前六位数字
	for(i = 0; i < 5; ++i)
	{
    workingHourDigit = (can_data.engineWorkingTime % (unsigned long)pow(10, 6 - i)) / (unsigned long)pow(10, 5 - i);
    LCD_DisplayItemX(8, 16, page, column, LCD_DigitArray_16Row8Column[workingHourDigit], shift);
	  
	  column += width;
	  column += 1;
	}
	
	height = 16;
	width = 7;
	//显示小数点
	LCD_DisplayItemX(7, 16, page, column, LCD_CharArray_16Row7Column_Dot, shift);
	
  column += width;
  column += 1;
	
	height = 16;
	width = 8;
	//显示小数点之后一位数字
	LCD_DisplayItemX(8, 16, page, column, LCD_DigitArray_16Row8Column[can_data.engineWorkingTime % 10], shift);
  
  column += width;
  column += 1;
  column += 2;
  
  height = 16;
  width = 8;
  //width += 4;
  
  //显示字母“ｈ”
	LCD_DisplayItemX(8, 16, page, column, LCD_CharArray_16Row8Column_h, shift);
}


/***********************************************************************************
@ Func: 显示直线
***********************************************************************************/
void LCD_DisplayLine(unsigned char page, unsigned char len)
{
	unsigned char i = 0;

	LCD_SetPageAddr(page);
	for(i = 0; i < len; ++i)
	{
		LCD_SetColumnAddr(i + 16 + ((128 - len) / 2));
		if(i % 4)
		{
			LCD_WriteData(0x04);
		}
		else 
		{
			LCD_WriteData(0x00);	
		}
	}		
}

/***********************************************************************************
@ Func:  显示油耗
@ Brief: 显示格式<-->xxx.xL/h，油耗用2个字节表示，分辨率为0.05，最大值为65535*0.05
         =3276.75，因LCD最多显示3位整数，当油耗值超过1000时，显示999.9L/h
         宽度：(8+1)+(8+1)+(8+1)+(7+1)+(8+1)+(8)+(8)+(8)=68个column，占用0~69列，显
         示区域为2~69列，左边留出2个像素（0~1列）防止压框挡住显示内容
***********************************************************************************/
void LCD_DisplayOilConsumption(double fuelRate)
{
	unsigned char i = 0;
	unsigned char column = 0;
	unsigned char width = 0;
	unsigned char page = 0;
	unsigned int uiFuelRate = 0;
	unsigned char fuelRateDigit = 0;
	bool previousDigitUndisplayedFlag = true;
	
	//为显示一位小数，油耗值扩大10倍
  uiFuelRate = fuelRate;//(fuelRate + 0.05) * 10 ;
  if(uiFuelRate >= 1000 * 10)
  {
    uiFuelRate = 9999;
  }
  
	width = 8;
	if(CarSpeedDisFlag == 0)
	  page = 7;
	else
	  page = 8;
  
  //显示区域清空	
  LCD_ClearItem(70, 16, page, 0);
  
  //根据显示位数定义显示起始列
  if(uiFuelRate >= (unsigned int)100 * 10)
  {
    column = 2;
  }
  else if(uiFuelRate >= 10 * 10)
  {
    column = 11;
  }
  else
  {
    column = 20;
  }
	
	for(i = 0; i < 4; ++i)
	{
    fuelRateDigit = (uiFuelRate % (unsigned long)pow(10, 4 - i)) / (unsigned long)pow(10, 3 - i);
    
    if(previousDigitUndisplayedFlag && (i < 2))
    {
      if(fuelRateDigit)
      {
        previousDigitUndisplayedFlag = false;
        LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[fuelRateDigit]);
        column += width;
    	  column += 1;
      }
    }
    else
    {
      LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[fuelRateDigit]);
      column += width;
  	  column += 1;
    }
	  
	  //预留小数点位置
	  if(2 == i)
	  {
	    column += 8;
	  }
	}
	
	column -= 17;
	
	width = 7;
	//显示小数点
	LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	
	//显示油耗单位 L/h
  width = 8;	
	column = 46;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg_16Com_L);
	
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_16Row8Column_Slash);
	
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_16Row8Column_h);
}

/***********************************************************************************
@ Func:  显示温度
@ Brief: 温度格式<--> -xxx℃
         占用宽度 (8+1)+(8+1)+(8+1)+(8+1)+(8*2)=52，占用74列到127列共54列，右边缘留
         出2个像素防止压框挡住显示。大气温度用2个字节表示，分辨率为0.03125℃/bit，偏
         移为-273℃，表示范围为-273~1775℃
***********************************************************************************/
void LCD_DisplayTemp(double temperature)
{
	unsigned char i = 0;
	unsigned char column = 0;
	unsigned char width = 0;
	unsigned char page = 0;
	unsigned char tempDigit = 0;
	unsigned int uiTemperature = 0;
	bool previousDigitUndisplayedFlag = true;
	
	width = 8;
	
	if(CarSpeedDisFlag == 0)
	  page = 7;
	else
	  page = 8;
	
	//首先清除待显示器区域
	//column = 74;
  //增加再生提醒后，如果从column = 74处清除，再生提醒的内容会清除不干净，故改为 column = 71;
  column = 71;
	LCD_ClearItem(54, 16, page, column);
	
	//若温度为负值则显示负号
	if(temperature < 0)
	{
	  if(temperature > -10)
	  {
	    column = 92;
	  }
	  else if(temperature > -100)
	  {
	    column = 83;
	  }
	  else
	  {
	    column = 74;
	  }
	  
	  LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg16Com_Minus);
	}
	
	//若温度为负则取其绝对值用于显示
	if(temperature < 0)
	{
  	uiTemperature = -(temperature - 0.5);
	}
	else
	{
  	uiTemperature = temperature + 0.5;
	} 
	
	column = 74;
	
	for(i = 0; i < 4; ++i)
	{
    tempDigit = (uiTemperature % (unsigned long)pow(10, 4 - i)) / (unsigned long)pow(10, 3 - i);
    if(previousDigitUndisplayedFlag && (i < 3))
    {
      if(tempDigit)
      {
        previousDigitUndisplayedFlag = false;
        LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[tempDigit]);
      }
    }
    else
    {
      LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[tempDigit]);
    }
	  
	  column += width;
	  column += 1;
	}
	
  //显示℃
	column = 110;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg16Com_Celsius_Left);
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg16Com_Celsius_Right);
}

/***********************************************************************************
@ Func: 显示警告
***********************************************************************************/
void LCD_DisplayWarningHeader(void)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char temp = 0;
	unsigned char column = 50;
	unsigned char height = 16;
	unsigned char width = 13;
	unsigned char page = 1;

	LCD_DisplayItem(13, 16, 1, column, LCD_ChineseCharArray_16Row13Column_Jing);
	column += width;
	LCD_DisplayItem(13, 16, 1, column, LCD_ChineseCharArray_16Row13Column_Gao_1);
	LCD_DisplayLine(4, 128);
}

/***********************************************************************************
@ Func: LCD显示"GPRS已xx天无信号"
***********************************************************************************/
void LCD_DisplayGPRSNoSignalPrompt(unsigned char days)
{
  unsigned char column = 0;
  unsigned char width = 0;
  
  column = 10;
  
  width = 7;
  LCD_DisplayItem(7, 16, 6, column, LCD_CharArray_16Row7Column_G);
  column += width;
  LCD_DisplayItem(7, 16, 6, column, LCD_CharArray_16Row7Column_P);
  column += width;
  LCD_DisplayItem(7, 16, 6, column, LCD_CharArray_16Row7Column_R);
  column += width;
  LCD_DisplayItem(7, 16, 6, column, LCD_CharArray_16Row7Column_S);
  column += width;
  
  width = 13;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Yi_2);
  column += width;
  
  width = 7;
  LCD_DisplayItem(7, 16, 6, column, LCD_DigitArray_16Row7Column[days % 100 / 10]);
  column += width;
  LCD_DisplayItem(7, 16, 6, column, LCD_DigitArray_16Row7Column[days % 10]);
  column += width;
  
  width = 13;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Tian);
  column += width;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Wu_1);
  column += width;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Xin);
  column += width;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Hao);
}

/***********************************************************************************
@ Func: LCD显示"请移到有信号的地方"
***********************************************************************************/
void LCD_DisplayMoveToGoodSignalPlacePrompt(void)
{
  unsigned char column = 0;
  unsigned char page = 9;
  
  column = 5;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Qing);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Yi_3);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Dao);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_You_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Xin);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Hao);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_De);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Di);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Fang);
}

/***********************************************************************************
@ Func: LCD显示"通信异常"
***********************************************************************************/
void LCD_DisplayCommErrorPrompt(void)
{
  unsigned char column = 0;
  
  column = 38;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Tong);
  column += 13;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Xin);
  column += 13;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Yi_4);
  column += 13;
  LCD_DisplayItem(13, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Chang);
}

/***********************************************************************************
@ Func: LCD显示"请移动至安全地带"
***********************************************************************************/
void LCD_DisplayMoveToSafePlacePrompt(void)
{
  unsigned char column = 0;
  unsigned char width = 0;
  
  column = 12;
  width = 13;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Qing);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Yi_3);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Dong);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Zhi_1);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_An);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Quan);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Di);
  column += width;
  LCD_DisplayItem(width, 16, 6, column, LCD_ChineseCharArray_16Row13Column_Dai_1);
}

/***********************************************************************************
@ Func: LCD显示"整车已无法启动"
***********************************************************************************/
void LCD_DisplayCanNotStartPrompt(void)
{
  unsigned char column = 0;
  unsigned char page = 9;
  
  column = 16;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Zheng_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Che);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Yi_2);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Wu_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Fa);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Qi_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Dong);
}

/***********************************************************************************
@ Func: LCD显示"停车后无法启动"
***********************************************************************************/
void LCD_DisplayCanNotStartOnceStopPrompt(void)
{
  unsigned char column = 0;
  unsigned char page = 9;
  
  column = 16;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Ting);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Che);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Hou_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Wu_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Fa);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Qi_1);
  column += 13;
  LCD_DisplayItem(13, 16, page, column, LCD_ChineseCharArray_16Row13Column_Dong);
}

/***********************************************************************************
@ Func: LCD显示"即将限扭"
***********************************************************************************/
void LCD_DisplayWillLimitTorquePrompt(unsigned char startPage, unsigned char startColumn)
{
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Ji_2);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jiang);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xian);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Niu);
}

/***********************************************************************************
@ Func: LCD显示"已经限扭"
***********************************************************************************/
void LCD_DisplayTorqueLimitedPrompt(unsigned char startPage, unsigned char startColumn)
{
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Yi_2);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jing_1);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xian);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Niu);
}

/***********************************************************************************
@ Func: LCD显示"即将限速"
***********************************************************************************/
void LCD_DisplayWillLimitSpeedPrompt(unsigned char startPage, unsigned char startColumn)
{
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Ji_2);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jiang);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xian);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Su);
}

/***********************************************************************************
@ Func: LCD显示"已经限速"
***********************************************************************************/
void LCD_DisplaySpeedLimitedPrompt(unsigned char startPage, unsigned char startColumn)
{
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Yi_2);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jing_1);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xian);
  startColumn += 13;
  LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Su);
}

/***********************************************************************************
@ Func: LCD显示"需进行再生"
***********************************************************************************/
void LCD_Display_NEED_TAKE_REGENERATE(unsigned char startPage, unsigned char startColumn)
{
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xu);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jin);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xing);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Zai_1);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Sheng);
}

/***********************************************************************************
@ Func: LCD显示"需立即再生"
***********************************************************************************/
void LCD_Display_NEED_RIGHT_REGENERATE(unsigned char startPage, unsigned char startColumn)
{
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xu);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Li);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Ji_2);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Zai_1);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Sheng);
}

/***********************************************************************************
@ Func: LCD显示"需服务再生"
***********************************************************************************/
void LCD_Display_NEED_SERVER_REGENERATE(unsigned char startPage, unsigned char startColumn)
{
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xu);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Fu);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Wu_2);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Zai_1);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Sheng);
}

/***********************************************************************************
@ Func: LCD显示"再生进行中"
***********************************************************************************/
void LCD_Display_TAKEING_REGENERATE(unsigned char startPage, unsigned char startColumn)
{
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Zai_1);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Sheng);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Jin);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Xing);
    startColumn += 13;
    LCD_DisplayItem(13, 16, startPage, startColumn, LCD_ChineseCharArray_16Row13Column_Zhong);
}

/***********************************************************************************
@ Func: 根据再生提醒显示类型，在LCD屏幕上显示相关描述   
***********************************************************************************/
void LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayType_t displayType)
{
  unsigned char clearPage = 0;
  unsigned char writePage = 0;
  if (CarSpeedDisFlag == 0) //此处用来判断再生再生提醒的显示位置
  {
    clearPage = 7;
    writePage = 8;   
  }
  else
  {
    clearPage = 8;
    writePage = 9; 
  }
  if (LCD_REGENERATE_DisplayCtrl.displayTypeOld == LCD_NO_REMIND)
  {
    //显示油耗和温度的区域清空
    LCD_ClearItem(128, 16, clearPage, 0);
    //清空再生提醒显示区域
    LCD_ClearItem(128, 16, writePage, 0);
  }
  else
  {
    //清空再生提醒显示区域
    LCD_ClearItem(128, 16, writePage, 0);
  }
  switch (displayType)
  {
  case LCD_NEED_TAKE_REGENERATE:
    LCD_Display_NEED_TAKE_REGENERATE(writePage, 32);
    break;
  case LCD_NEED_RIGHT_REGENERATE:
    LCD_Display_NEED_RIGHT_REGENERATE(writePage, 32);
    break;
  case LCD_NEED_SERVER_REGENERATE:
    LCD_Display_NEED_SERVER_REGENERATE(writePage, 32);
    break;
  case LCD_TAKEING_REGENERATE:
    LCD_Display_TAKEING_REGENERATE(writePage, 32);
    break;
  default:
    break;
  }
}

/***********************************************************************************
@ Func: LCD显示DTC SPN & FMI
***********************************************************************************/
void LCD_DisplaySpnFmi(unsigned long spn, unsigned char fmi, unsigned char page)
{
  unsigned char i = 0;
  unsigned char width = 0;
  unsigned char height = 0;
  unsigned char column = 0;
  
  bool spnFirstNonZeroDigitFlag = false;
  unsigned char temp = 0;
  unsigned char numDigit = 0;
  
  //显示SPN
  width = 8;
  height = 16;
  //column = 0;
  column = 1;
  LCD_DisplayItem(width, height, page, column, LCD_CharArray_16Row8Column_S);
  column += (width);
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row8Column_P);
  column += (width);
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row8Column_N);
  column += (width);
  width = 6;
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row6Column_Colon);
  column += (width);
  
  width = 8;
  for(i = 0; i < 6; ++i)
	{
    numDigit = (spn % (unsigned long)pow(10, 6 - i)) / (unsigned long)pow(10, 5 - i);
    if(numDigit || (5 == i))
    {
      spnFirstNonZeroDigitFlag = true;
    }
    
    if(spnFirstNonZeroDigitFlag)
    {
      LCD_DisplayItem(width, 16, page, column, LCD_DigitArray_16Row8Column[numDigit]);
      column += (width);
    }
	}
	
	//显示FMI
	column = 81;
	LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row8Column_F);
  column += (width);
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row8Column_M);
  column += (width);
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row8Column_I);
  column += (width);
  width = 6;
  LCD_DisplayItem(width, 16, page, column, LCD_CharArray_16Row6Column_Colon);
  column += (width);
  
  width = 8;
  temp = fmi % 100 / 10;
  if(temp)
  {
    LCD_DisplayItem(width, 16, page, column, LCD_DigitArray_16Row8Column[temp]);
    column += (width);
  }
  
  temp = fmi % 10;
  LCD_DisplayItem(width, 16, page, column, LCD_DigitArray_16Row8Column[temp]);
  column += (width);
}

/***********************************************************************************
@ Func: LCD显示“【三角警告符号】警告 故障总数：XXX”
***********************************************************************************/
void LCD_DisplayDTCWarningHeader(unsigned char dtcTotalNum)
{
  unsigned char column = 0;
  unsigned char i = 0;
  unsigned char numDigit = 0;
  bool nonZeroDigitFlag = false;
  
  column = 3;
  //显示三角报警符号
  LCD_DisplayItemX(13, 16, 0, column, Image_WarningTriangle_16_13, 2);
  
  column += 13;
  column += 2;
  LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Jing, 3);
  column += 13;
	LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Gao_1, 3);
	LCD_DisplayLine(2, 128);
	
	//显示“警告 故障总数:”
	column = 53;
  LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Gu, 3);
	LCD_DisplayItemX(13, 16, 0, column + 13, LCD_ChineseCharArray_16Row13Column_Zhang, 3);
  LCD_DisplayItemX(13, 16, 0, column + 13 * 2, LCD_ChineseCharArray_16Row13Column_Zong, 3);
	LCD_DisplayItemX(13, 16, 0, column + 13 * 3, LCD_ChineseCharArray_16Row13Column_Shu, 3);
	LCD_DisplayItemX(7, 16, 0, column + 13 * 4, LCD_CharArray_16Row7Column_Colon, 3);
	
	//显示故障数目
	column = column + 13 * 4 + 7;
  for(i = 0; i < 3; ++i)
	{
    numDigit = (dtcTotalNum % (unsigned long)pow(10, 3 - i)) / (unsigned long)pow(10, 2 - i);
    if(numDigit || (2 == i))
    {
      nonZeroDigitFlag = true;
    }
    
    if(nonZeroDigitFlag)
    {
      LCD_DisplayItemX(7, 16, 0, column, LCD_DigitArray_16Row7Column[numDigit], 3);
      column += 7;
    }
	}
}

/***********************************************************************************
@ Func: LCD轮询显示DTC
***********************************************************************************/
void LCD_CANFaultDisplay(void)
{
  unsigned char i = 0;
  unsigned char j = 0;
  unsigned char dtcPosInx = 0;
  unsigned char pageFrameNbr = 0;
  
  //static bool firstDisplayFlag = true;
  
  //从上次显示的页框的下一位置开始搜索
  for(i = CAN_FaultDisplayCtrl.startPosForSearch; i < (CAN_FAULT_RECORD_MAX_NUM / 3); ++i)
  {
    for(j = 0; j < 3; ++j)
    {
      if(CAN_FaultRecordBuffer[i * 3 + j].can_spn)
      {
        CAN_FaultDisplayCtrl.dtcPos[dtcPosInx++] = i;  //记录页框序号（索引从1开始）
        CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_spn = CAN_FaultRecordBuffer[i * 3 + j].can_spn;
        CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_fmi = CAN_FaultRecordBuffer[i * 3 + j].can_fmi;
      }
    }
    
    //页框中有DTC则停止搜索
    if(dtcPosInx >= 1)
    {
      break;
    }
  }
  
  //若搜索到缓冲区末尾仍未发现DTC，则从头开始搜索，直到本次开始搜索位置的前一位置
  if(0 == dtcPosInx)
  {
    for(i = 0; i < CAN_FaultDisplayCtrl.startPosForSearch; ++i)
    {
      for(j = 0; j < 3; ++j)
      {
        if(CAN_FaultRecordBuffer[i * 3 + j].can_spn)
        {
          CAN_FaultDisplayCtrl.dtcPos[dtcPosInx++] = i;
          CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_spn = CAN_FaultRecordBuffer[i * 3 + j].can_spn;
          CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_fmi = CAN_FaultRecordBuffer[i * 3 + j].can_fmi;
        }
      }
      
      if(dtcPosInx >= 1)
      {
        break;
      }
    }
  }
  
  //若DTC数目＜3，则将未用到的缓冲区清零
  switch(dtcPosInx)
  {
    case 0:
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[0], 0, sizeof(CAN_Fault_t));
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[1], 0, sizeof(CAN_Fault_t));
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[2], 0, sizeof(CAN_Fault_t));
      
      CAN_FaultDisplayCtrl.startPosForSearch = 0;
    break;
    
    case 1:
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[1], 0, sizeof(CAN_Fault_t));
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[2], 0, sizeof(CAN_Fault_t));
    break;
    
    case 2:
      memset(&CAN_FaultDisplayCtrl.currFaultInfor[2], 0, sizeof(CAN_Fault_t));
    break;
    
    case 3:
    break;
    
    default:
    break;
  }
  
  //记录下一次搜索DTC开始位置
  CAN_FaultDisplayCtrl.startPosForSearch = CAN_FaultDisplayCtrl.dtcPos[dtcPosInx - 1] + 1;
  if((CAN_FAULT_RECORD_MAX_NUM / 3) == CAN_FaultDisplayCtrl.startPosForSearch)
  {
    CAN_FaultDisplayCtrl.startPosForSearch = 0;
  }
  
  //显示“【三角警告符号】警告   故障总数：xxx”
  CAN_FaultDisplayCtrl.totalNum = CAN_FaultNumStats();
  if((CAN_FaultDisplayCtrl.oldTotalNum != CAN_FaultDisplayCtrl.totalNum) || CAN_FaultDisplayCtrl.firstDisplayFlag/*firstDisplayFlag*/
     || CAN_FaultDisplayCtrl.refreshFlag) //即使没有DTC也需要显示警告，所以第一次执行该函数需要刷新警告显示
  {
    CAN_FaultDisplayCtrl.refreshFlag = false;
    
    //当DTC出现时需要刷新NCD和小时计显示
    if(0 == CAN_FaultDisplayCtrl.oldTotalNum)
    {
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = true;
      hourmeterNeedShowFlag = true;
    } 
    
    CAN_FaultDisplayCtrl.oldTotalNum = CAN_FaultDisplayCtrl.totalNum;
    
    //清除2个Page，因为故障总数是变化的，所以需要擦除后重写
    LCD_ClearItem(128, 16, 0, 0);
    LCD_DisplayDTCWarningHeader(CAN_FaultDisplayCtrl.totalNum);    
  }
    
  //显示故障DTC
  //比较和上次显示内容是否有变化
  for(i = 0; i < 3; ++i)
  {
    if((CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn != CAN_FaultDisplayCtrl.oldFaultInfor[i].can_spn) 
       || (CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi != CAN_FaultDisplayCtrl.oldFaultInfor[i].can_fmi))
    {
      break;
    }
  }
  
  //要显示的DTC有变化
  if(3 != i)
  {
    //记录本次显示内容
    for(i = 0; i < 3; ++i)
    {
      CAN_FaultDisplayCtrl.oldFaultInfor[i].can_spn = CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn;
      CAN_FaultDisplayCtrl.oldFaultInfor[i].can_fmi = CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi;
    }
    
    //清空显示区域
    for(i = 0; i < 3; ++i)
    {
      LCD_ClearItem(128, 16, 3 * i + 3, 0);
    }
    
    //在相应区域显示DTC
    for(i = 0; i < dtcPosInx; ++i)
    {
      if(CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn)
      {
        LCD_DisplaySpnFmi(CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn, CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi, 3 * i + 3);
      }
    }
  }
  
  //NCD提示顯示
  if(LCD_NCD_WARNING_NONE != LCD_NCDWarningDisplayCtrl.displayType)
  {
    if((LCD_NCDWarningDisplayCtrl.displayTypeOld != LCD_NCDWarningDisplayCtrl.displayType) || LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag)
    {
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = false;
            
      LCD_NCDWarningDisplayCtrl.displayTypeOld = LCD_NCDWarningDisplayCtrl.displayType;
      
      //Add newly.
      LCD_ClearItem(128, 16, 0, 0);
      LCD_DisplayDTCWarningHeader(CAN_FaultDisplayCtrl.totalNum);
      
      //若無DTC故障，則在屏幕中間顯示NCD提示，屏幕下方顯示小時計
      if(0 == CAN_FaultDisplayCtrl.totalNum)
      {
        LCD_ClearItem(128, 72, 3, 0);
        
        switch(LCD_NCDWarningDisplayCtrl.displayType)
        {
          case LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SLIGHT:
            LCD_DisplayWillLimitTorquePrompt(6, 38);
          break;
          
          case LCD_NCD_WARNING_TORQUE_LIMITED_SLIGHT:
            LCD_DisplayTorqueLimitedPrompt(6, 38);
          break;
          
          case LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SEVERE:
            LCD_DisplayWillLimitSpeedPrompt(6, 38);
          break;
          
          case LCD_NCD_WARNING_TORQUE_LIMITED_SEVERE:
            LCD_DisplaySpeedLimitedPrompt(6, 38);
          break;
          
          default:
          break;
        }
        
      }
      //若存在DTC，則LCD下方顯示NCD提示，小時計不顯示
      else
      {
        LCD_ClearItem(128, 16, 12, 0);
        switch(LCD_NCDWarningDisplayCtrl.displayType)
        {
          case LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SLIGHT:
            LCD_DisplayWillLimitTorquePrompt(12, 38);
          break;
          
          case LCD_NCD_WARNING_TORQUE_LIMITED_SLIGHT:
            LCD_DisplayTorqueLimitedPrompt(12, 38);
          break;
          
          case LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SEVERE:
            LCD_DisplayWillLimitSpeedPrompt(12, 38);
          break;
          
          case LCD_NCD_WARNING_TORQUE_LIMITED_SEVERE:
            LCD_DisplaySpeedLimitedPrompt(12, 38);
          break;
          
          default:
          break;
        }
      }
    }
  }
  
  //小时计显示条件包括：1）无NCD警告；2）有NCD警告但无DTC
  if((LCD_NCD_WARNING_NONE == LCD_NCDWarningDisplayCtrl.displayType)
     || ((LCD_NCD_WARNING_NONE != LCD_NCDWarningDisplayCtrl.displayType) && (0 == CAN_FaultDisplayCtrl.totalNum)))
  {
    if((can_data.engineWorkingTime != can_data.engineWorkingTimeOld) || CAN_FaultDisplayCtrl.firstDisplayFlag/*firstDisplayFlag*/ || hourmeterNeedShowFlag)
    {
      can_data.engineWorkingTimeOld = can_data.engineWorkingTime;
      hourmeterNeedShowFlag = false;
      
      LCD_ClearItem(128, 16, 12, 0);
      LCD_ShowTotalWorkTimeX(12, LCD_TIME_SHOW_START_COLUMN, -3);
    }   
  }
  
  //firstDisplayFlag = false;
  CAN_FaultDisplayCtrl.firstDisplayFlag = false;
}

/***********************************************************************************
@ Func: 判斷NCD駕駛系統性能限制類型
***********************************************************************************/
void LCD_JudgeNCDType(unsigned char ncdByte)
{
  switch((ncdByte & 0x38) >> 3)
  {
    case NCD_STATE_WILL_LIMIT_TORQUE_SLIGHTLY:
      LCD_NCDWarningDisplayCtrl.displayType = LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SLIGHT;
    break;
    
    case NCD_STATE_TORQUE_LIMITED_SLIGHTLY:
      LCD_NCDWarningDisplayCtrl.displayType = LCD_NCD_WARNING_TORQUE_LIMITED_SLIGHT;
    break;
    
    case NCD_STATE_WILL_LIMIT_TORQUE_SEVERELY:
      LCD_NCDWarningDisplayCtrl.displayType = LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SEVERE;
    break;
    
    case NCD_STATE_TORQUE_LIMITED_SEVERELY:
      LCD_NCDWarningDisplayCtrl.displayType = LCD_NCD_WARNING_TORQUE_LIMITED_SEVERE;
    break;
    
    default:
    break;
  }
}
/***********************************************************************************
@ Func: 判断再生提醒显示类型
***********************************************************************************/
void LCD_REGENERATE_Type(void)
{
  if (TYPE_of_Engine == Weichai_engine)
  {
    if(REGENERATE_STATE_SERVICE_PROMPT != CAN_PostprocessingState.regenerateState)
    {
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NO_REMIND;
    }
    else
    {
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_SERVER_REGENERATE;
    }
  }
  else //此处为玉柴发动机类型，玉柴发动机的量比较少，故放到后面
  {
    if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_IN_PROCESS)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_TAKE_REGENERATE;
    else if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_PARKING_PROMPT)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_SERVER_REGENERATE;
    else if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_SERVICE_PROMPT)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_RIGHT_REGENERATE;
    //判断再生过程显示
    else if (CAN_PostprocessingState_YC.regenerateProcess == 0x04)
    {
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_TAKEING_REGENERATE;
    }
    else
    {
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NO_REMIND;
    }
  }
}

/***********************************************************************************
@ Func: LCD显示任务
***********************************************************************************/
void LCD_Task(void)
{
  unsigned char i = 0;
  
  //设置显示界面类型
  if(CAN_GPSWarningBitsMap)
  {
    LCD_DisplayCtrl.LCD_DisplayType = LCD_DISPLAY_GPS_WARNING;
  }
  else if(CAN_FaultBitMap)
  {
    LCD_DisplayCtrl.LCD_DisplayType = LCD_DISPLAY_FAULT;
    if((((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x00) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x01)
       && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x06) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x07))
    {
      LCD_JudgeNCDType(CAN_PostprocessingState.ncdState);
    }
  }
  else if((((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x00) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x01)
          && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x06) && (((CAN_PostprocessingState.ncdState & 0x38) >> 3) != 0x07))
  {
    LCD_DisplayCtrl.LCD_DisplayType = LCD_DISPLAY_FAULT;
    LCD_JudgeNCDType(CAN_PostprocessingState.ncdState);
  }
  else if((!CAN_RecvEngineHourReqAckFlag) && (CAN_RecvEngineHourReqAckTimeout < 6))
  {
    LCD_DisplayCtrl.LCD_DisplayType = LCD_DISPLAY_LOGO;
  }
  else
  {
    LCD_DisplayCtrl.LCD_DisplayType = LCD_DISPLAY_NORMAL;
    LCD_REGENERATE_Type();
  }
  
  //若无GPS报警，复位界面显示控制变量
  if(!CAN_GPSWarningBitsMap)
  {
    LCD_GPSWarningDisplayCtrl.displayTypeOld = LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_NONE;
  }
  
  //若无CAN DTC则复位CAN DTC显示控制变量
  if(!CAN_FaultBitMap)
  {
    if(CAN_FaultDisplayCtrl.oldTotalNum)
    {
      memset(&CAN_FaultDisplayCtrl, 0, sizeof(CAN_FaultDisplayCtrl_t));
      CAN_FaultDisplayCtrl.firstDisplayFlag = true;
      
      //故障警告数目需要更新，NCD显示位置可能需要刷新（若存在NCD），小时计需要刷新（存在NCD时是未显示的）
      CAN_FaultDisplayCtrl.refreshFlag = true;
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = true;
      hourmeterNeedShowFlag = true;
    }
  }
  
  //无NCD故障，复位NCD故障显示控制变量
  if((((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x00) || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x01)
     || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x06) || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x07))
  {
    //当NCD消失时需要刷新小时计
    if(LCD_NCD_WARNING_NONE != LCD_NCDWarningDisplayCtrl.displayTypeOld)
    {
      hourmeterNeedShowFlag = true;
      CAN_FaultDisplayCtrl.refreshTimeout = 0;
      
      LCD_NCDWarningDisplayCtrl.displayType = LCD_NCD_WARNING_NONE;
      LCD_NCDWarningDisplayCtrl.displayTypeOld = LCD_NCD_WARNING_NONE;
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = false;
      CAN_FaultDisplayCtrl.firstDisplayFlag = true;
    }
    
  }
  
  //判断是否允许沙漏闪烁
  if(LCD_DISPLAY_NORMAL == LCD_DisplayCtrl.LCD_DisplayType)
  {
    if((ENGINE_STARTED == Engine_State) || (ENGINE_STARTING == Engine_State))
    {
      LCD_HourglassCtrl.blinkEnable = true;
    }
    else
    {
      LCD_HourglassCtrl.blinkEnable = false;
      LCD_HourglassCtrl.displayState = LCD_GLASSHOUR_SHOW;
    }
  }
  else
  {
    LCD_HourglassCtrl.blinkEnable = false;
    LCD_HourglassCtrl.displayState = LCD_GLASSHOUR_SHOW;
  }

  //复位LCD再生提醒显示类型
  if (LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
  {
    if (LCD_REGENERATE_DisplayCtrl.displayTypeOld != LCD_NO_REMIND)
    {
      LCD_REGENERATE_DisplayCtrl.displayTypeOld = LCD_NO_REMIND;
      Refresh_fuelConsumption_and_ambientAirTemp = TRUE;
    }
  }

  //根据显示界面类型显示相应内容
  switch(LCD_DisplayCtrl.LCD_DisplayType)
  {
    //开机Logo显示
    case LCD_DISPLAY_LOGO:
      if(LCD_DISPLAY_LOGO != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_LOGO;
        LCD_ClearScreen();
        LCD_DisplaySDLGLogo();
      }
    break;
    
    //正常工作界面显示
    case LCD_DISPLAY_NORMAL:
      if(LCD_DISPLAY_NORMAL != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_NORMAL; 
        LCD_ClearScreen();
        
        //车速和转速显示在第一次显示时初始化
        CarSpeedDisFlag = 0;
        CarSpeedDisFlagOld = 0;
        LCD_DisplayEngineSpeedOnly(engineSpeedParams.old_speed);
        //LCD_DisplayEngineSpeed(engineSpeedParams.old_speed);
        //LCD_DisplayVehicleSpeed(car_speed_parameter.loader_speed);
        LCD_DisplayLine(5, 128);
        if (LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
        {
          LCD_DisplayOilConsumption(can_data.fuelConsumption);
          LCD_DisplayTemp(can_data.ambientAirTemp);
        }
        else
        {
          LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayCtrl.displayType);
          LCD_REGENERATE_DisplayCtrl.displayTypeOld = LCD_REGENERATE_DisplayCtrl.displayType;
        }
        LCD_ShowTotalWorkTime(11, LCD_TIME_SHOW_START_COLUMN);
        break;
      }
      
      if(CarSpeedDisFlagOld != CarSpeedDisFlag) 
      {
        LCD_ClearScreen();
        CarSpeedDisFlagOld = CarSpeedDisFlag;
        if(CarSpeedDisFlag == 0)
        {
          LCD_DisplayEngineSpeedOnly(engineSpeedParams.old_speed);
          LCD_DisplayLine(5, 128);
        }
        else 
        {
          LCD_DisplayVehicleSpeed(car_speed_parameter.loader_speed);
          LCD_DisplayEngineSpeed(engineSpeedParams.old_speed);
          LCD_DisplayLine(7, 128);
        }  
        if (LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
        {   
        LCD_DisplayOilConsumption(can_data.fuelConsumption);
        LCD_DisplayTemp(can_data.ambientAirTemp);
        }
        else
        {
          LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayCtrl.displayType);
          LCD_REGENERATE_DisplayCtrl.displayTypeOld = LCD_REGENERATE_DisplayCtrl.displayType;
        }
        LCD_ShowTotalWorkTime(11, LCD_TIME_SHOW_START_COLUMN); 
        break; 
      }
      
      //发动机转速
      if(LCD_NormalDisplayCtrl.engineSpeedRefreshFlag && (LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout > 1000))
      {
        LCD_NormalDisplayCtrl.engineSpeedRefreshFlag = false;
        LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout = 0;
        if(CarSpeedDisFlag == 0)
          LCD_DisplayEngineSpeedOnly(engineSpeedParams.old_speed);
        else
          LCD_DisplayEngineSpeed(engineSpeedParams.old_speed);
      }
      
      //只有车速不为0时才进行显示，没有车速信号时不显示
      if( CarSpeedDisFlag == 1)
        LCD_DisplayVehicleSpeed(car_speed_parameter.loader_speed);

      //有再生提醒时，在LCD屏幕上显示提醒内容，如果没有再生提醒则显示油耗和温度  
      if(LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
      {
        //清除油耗、温度以及再生提醒区域
        if (Refresh_fuelConsumption_and_ambientAirTemp == TRUE)
        {
          if (CarSpeedDisFlag == 0)
          {
            LCD_ClearItem(128, 16, 8, 0);
          }
          else
          {
            LCD_ClearItem(128, 16, 9, 0);
          }
        }
        //燃油消耗
        if((((can_data.fuelConsumption - can_data.fuelConsumptionOld) > 1) || (can_data.fuelConsumptionOld > (1 + can_data.fuelConsumption)))
          && (LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout > 1000)||(Refresh_fuelConsumption_and_ambientAirTemp == TRUE))
        {
          LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout = 0;
          can_data.fuelConsumptionOld = can_data.fuelConsumption;
          LCD_DisplayOilConsumption(can_data.fuelConsumption);
        }
        
        //大气温度
        if(((can_data.ambientAirTemp - can_data.ambientAirTempOld) > 1.0) || ((can_data.ambientAirTemp - can_data.ambientAirTempOld) < -1.0)
             ||(Refresh_fuelConsumption_and_ambientAirTemp == TRUE))
        {
          can_data.ambientAirTempOld = can_data.ambientAirTemp;
          LCD_DisplayTemp(can_data.ambientAirTemp);
        }
          Refresh_fuelConsumption_and_ambientAirTemp = FALSE;
        }
      else
      {
        if(LCD_REGENERATE_DisplayCtrl.displayType != LCD_REGENERATE_DisplayCtrl.displayTypeOld)
        {
        LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayCtrl.displayType);
        LCD_REGENERATE_DisplayCtrl.displayTypeOld = LCD_REGENERATE_DisplayCtrl.displayType;
        }
      }
      //工作时间
      if(can_data.engineWorkingTime != can_data.engineWorkingTimeOld)
      {
        can_data.engineWorkingTimeOld = can_data.engineWorkingTime;
        LCD_ShowTotalWorkTime(11, LCD_TIME_SHOW_START_COLUMN);
      }
      
      //沙漏闪烁控制
      if(LCD_HourglassCtrl.blinkEnable)
      {
        LCD_DisplayHourglass(11, LCD_TIME_SHOW_START_COLUMN, LCD_HourglassCtrl.displayState);
      }
      else
      {
        LCD_DisplayHourglass(11, LCD_TIME_SHOW_START_COLUMN, LCD_GLASSHOUR_SHOW);
      }
    break;
        
    //故障界面显示
    case LCD_DISPLAY_FAULT:
      if(LCD_DisplayCtrl.LCD_DisplayTypeOld != LCD_DisplayCtrl.LCD_DisplayType)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DisplayCtrl.LCD_DisplayType;
        LCD_NCDWarningDisplayCtrl.displayTypeOld = LCD_NCD_WARNING_NONE;  //如果存在NCD故障的话，用来刷新出汉显
        CAN_FaultDisplayCtrl.firstDisplayFlag = true;  //目的是界面切换后，快速刷新 故障码总数、故障码以及小时计
        CAN_FaultDisplayCtrl.refreshTimeout = 8;       //防止界面切换后进行两次刷新
        LCD_ClearScreen();
        LCD_CANFaultDisplay();
        //LCD_ShowTotalWorkTimeX(12, LCD_TIME_SHOW_START_COLUMN, -3);
      }
      
      //当NCD驾驶性能限制改变时，立刻刷新界面
      if(LCD_NCDWarningDisplayCtrl.displayType != LCD_NCDWarningDisplayCtrl.displayTypeOld)
      {
        CAN_FaultDisplayCtrl.refreshTimeout = 0;
      }
      
      //定时刷新界面，约5秒
      if(0 == CAN_FaultDisplayCtrl.refreshTimeout)
      {
        CAN_FaultDisplayCtrl.refreshTimeout = 8;
        
        LCD_CANFaultDisplay();
      }
    break;
    
    //GPS报警显示
    case LCD_DISPLAY_GPS_WARNING:
      if(LCD_DISPLAY_GPS_WARNING != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_GPS_WARNING;
      }
      
      //判断警告类型
      if(CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_COMM_ERR_UNLOCKED;
      }
      else if(CAN_GPSWarningBitsMap & EECU_ALARM_KEY_INCORRECT_LOCKED)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_COMM_ERR_LOCKED;
      }
      else if(CAN_GPSWarningBitsMap & GPS_ALARM_WORK_ABNORMAL)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_COMM_ERR_GPS_WORK_ABNORMAL;
      }
      else if(CAN_GPSWarningBitsMap & GPS_ALARM_WEB_LOCK_CMD)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_WEB_LOCK;
      }
      else if(CAN_GPSWarningBitsMap & GPS_ALARM_61_DAYS_NO_SIGNAL)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_61_DAYS_NO_SIGNAL;
      }
      else if(CAN_GPSWarningBitsMap & GPS_ALARM_58_DAYS_NO_SIGNAL)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_58_DAYS_NO_SIGNAL;
      }
      else if(CAN_GPSWarningBitsMap & GPS_ALARM_51_DAYS_NO_SIGNAL)
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_51_DAYS_NO_SIGNAL;
      }
      else
      {
        LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_NONE;
      }
      
      //根据GPS报警类型显示相应界面
      if((LCD_GPSWarningDisplayCtrl.displayTypeOld != LCD_GPSWarningDisplayCtrl.displayType) || (LCD_GPSWarningDisplayCtrl.oldEngineState != Engine_State))
      {
        LCD_GPSWarningDisplayCtrl.displayTypeOld = LCD_GPSWarningDisplayCtrl.displayType;
        LCD_GPSWarningDisplayCtrl.oldEngineState = Engine_State;
        
        LCD_ClearScreen();
        LCD_DisplayWarningHeader();
        switch(LCD_GPSWarningDisplayCtrl.displayType)
        {
          case LCD_GPS_WARNING_COMM_ERR_UNLOCKED:
            LCD_DisplayCommErrorPrompt();
            LCD_DisplayCanNotStartOnceStopPrompt();
          break;
          
          case LCD_GPS_WARNING_COMM_ERR_LOCKED:
            LCD_DisplayCommErrorPrompt();
            LCD_DisplayCanNotStartPrompt();
          break;
          
          case LCD_GPS_WARNING_COMM_ERR_GPS_WORK_ABNORMAL:
            LCD_DisplayCommErrorPrompt();
            LCD_DisplayCanNotStartOnceStopPrompt();
          break;
          
          case LCD_GPS_WARNING_WEB_LOCK:
            LCD_DisplayMoveToSafePlacePrompt();
            LCD_DisplayCanNotStartPrompt();
          break;
          
          case LCD_GPS_WARNING_61_DAYS_NO_SIGNAL:
            LCD_DisplayGPRSNoSignalPrompt(61);
            LCD_DisplayCanNotStartOnceStopPrompt();
          break;
          
          case LCD_GPS_WARNING_58_DAYS_NO_SIGNAL:
            LCD_DisplayGPRSNoSignalPrompt(58);
            LCD_DisplayMoveToGoodSignalPlacePrompt();
          break;
          
          case LCD_GPS_WARNING_51_DAYS_NO_SIGNAL:
            LCD_DisplayGPRSNoSignalPrompt(51);
            LCD_DisplayMoveToGoodSignalPlacePrompt();
          break;
          
          default:
          break;
        }
      }
    break;
    
    default:
    break;
  }
}