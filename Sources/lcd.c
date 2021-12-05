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

//CAN_GPSWarningBitsMap��Bit����
//[I don't know]  [I don't know]  [NC]           [NC]             [NC][NC]                   [NC][GPRS61�����ź�]
//[GPRS58�����ź�][GPRS51�����ź�][Web��ǿ������][ECU�޷���������][NC][����״̬��KEYֵ����ȷ][NC][KEYֵ��֤����ȷ]
Uint16_t  CAN_GPSWarningBitsMap;
Uint16_t  CAN_GPSWarningBitsMapOld;


//����һ��������ʾ��־λ��������ڳ����ź��л�����1.old��־λ�����Ƿ����
unsigned char  CarSpeedDisFlag    = 0;
unsigned char  CarSpeedDisFlagOld = 0;

unsigned long LCD_OldCanFaultSpnFmi = 0;

//LCD��ʾ���� 
LCD_DisplayCtrl_t LCD_DisplayCtrl =
{
  LCD_DISPLAY_LOGO,     //��ʾ��������
  LCD_DISPLAY_LOGO,     //�ϴ���ʾ��������
  false,                //��ʾ���͸ı��־�������л���ͬ��ʾ����ʱ
  200,                  //��ʱˢ�¼�����
  false,                //ˢ�±�־
  0,                    //����ִ������
};

//ɳ©��˸����
LCD_HourglassCtrl_t LCD_HourglassCtrl =
{
  false,
  0,
  false,
  LCD_GLASSHOUR_SHOW,
};

//LCD��������������ʾ����
LCD_NormalDisplayCtrl_t LCD_NormalDisplayCtrl =
{
  false,
  0,
  false,
  0,
  false,
  false,
};

//LCD GPS��������
LCD_GPSWarningDisplayCtrl_t LCD_GPSWarningDisplayCtrl =
{
  LCD_GPS_WARNING_NONE,
  LCD_GPS_WARNING_NONE,
  ENGINE_STOPPED,
};

//DTC��ʾ����
CAN_FaultDisplayCtrl_t CAN_FaultDisplayCtrl =
{
  0,        //��������
  0,        //�ϴι�������
  
  0,        //��ʼ����λ��
  
  0,        //ˢ�³�ʱ������
  false,    //ˢ�±�־
  
  true,     //��һ����ʾ���Ͻ����־
  
  {0},      //Ҫ��ʾ��DTC�ڻ������е�λ��
  {0},      //��ǰ��ʾ��3��DTC������
  {0},      //�ϴ���ʾ��3��DTC������
};

//NCD��������
LCD_NCDWarningDisplayCtrl_t LCD_NCDWarningDisplayCtrl =
{
  LCD_NCD_WARNING_NONE,
  LCD_NCD_WARNING_NONE,
  false,
};

//LCD�������ѿ���
LCD_REGENERATE_DisplayCtrl_t LCD_REGENERATE_DisplayCtrl=
{
  LCD_NO_REMIND,
  LCD_NO_REMIND,
//  FALSE,
};


//������ʾ���棬NCD������ʾ��־����DTCʱ����LCD������ʾ��Сʱ����LCD�·���ʾ����DTCʱ��NCD��LCD�·���ʾ��Сʱ�Ʋ���ʾ��
bool ncdChangeDisplayAreaFlag;
//������ʾ���棬����ʾСʱ�Ʊ�־
bool hourmeterNeedShowFlag;
//��������������ʧ��ˢ���ͺĺ��¶Ƚ�����ʾ
bool Refresh_fuelConsumption_and_ambientAirTemp = FALSE;



/*********************************************************************************
@ Func:  LCD��ʼ��
@ Brief: 8088�ӿ�
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
  //��ӦIO������Ϊ���
  DDRA = 0xff;
  //DDRB = 0xff;
  DDRB |= 0xfe;
  
  //��ʼ����Ҫ�ر�LCD���⣬д�뿪��Logo���ٴ򿪱���
  LCD_BACK_LIGHT_CLOSE();
}

/*********************************************************************************
@ Func: LCD����
*********************************************************************************/
void LCD_Config(void)
{
	LCD_HardwareReset();
	LCD_InterfaceEnable();

	LCD_InverseDisplay(LCD_INVERSE_DISPLAY_NORMAL);
	//LCD_SetAllPixelDisplay(LCD_ALL_PIXEL_NORMAL);

	LCD_SetSegDirection(LCD_SEG_DIRECTION_REVERSE);
	LCD_SetComDirection(LCD_COM_DIRECTION_NORMAL);

	//����DDRAM��20��ӳ�䵽COM0
	LCD_SetStartLine(0);
	//��20��Pad���COM0�źţ���Һ����112��COM����DDRAM��132�У�132-112=20
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
@ Func: LCD��������
*************************************************************************/
void LCD_WriteCmd(unsigned char cmd)
{
	LCD_CMD();							//��������

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
@ Func: LCD��������
*************************************************************************/
void LCD_WriteData(unsigned char data)
{
	LCD_DATA();							//��������
	
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
@ Func:  ʹ��ͨ�Žӿ�D7~D0
@ Brief: PC5 <--> CS1
         PC4 <--> CS2
         Interface is enabled when both CS1B is ��L�� and CS2 is ��H��. If chip 
         select pins are not active (CS1B=��H�� or CS2=��L��), D0 to D7 become 
         high impedance.
****************************************************************************/
void LCD_InterfaceEnable(void)
{
	LCD_CS1_CLR();
	LCD_CS2_SET();
}

/****************************************************************************
@ Func:  ��ֹͨ�Žӿ�D7~D0
@ Brief: PC5 <--> CS1
         PC4 <--> CS2
         Interface is enabled when both CS1B is ��L�� and CS2 is ��H��. If chip 
         select pins are not active (CS1B=��H�� or CS2=��L��), D0 to D7 become 
         high impedance.
****************************************************************************/
void LCD_InterfaceDisable(void)
{
	LCD_CS1_SET();
	LCD_CS1_CLR();
}

/****************************************************************************
@ Func:  ����ҳ��ַ
@ Brief: ҳ��ַ��ΧΪ0~16����16ҳD0~D3��Ч
****************************************************************************/
void LCD_SetPageAddr(unsigned char page)
{
	LCD_WriteCmd(0x7c);
	LCD_WriteCmd(page);
}

/****************************************************************************
@ Func:  ����ҳ��ַ
@ Brief: ҳ��ַ��ΧΪ0~159
****************************************************************************/
void LCD_SetColumnAddr(unsigned char column)
{
	LCD_WriteCmd(0x10 | ((column & 0xf0) >> 4));
	LCD_WriteCmd(column & 0x0f);
}

/****************************************************************************
@ Func:  �������õ�Դ������
@ Brief: NVDD Generator ON��V3 Generator OFF��V1 Generator OFF��MV3 & MV1 
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
@ Func:  Ӳ����λ
****************************************************************************/
void LCD_HardwareReset(void)
{
	LCD_RST_CLR();
	DelayMS(20);
	LCD_RST_SET();
	DelayMS(10);
}

/****************************************************************************
@ Func:  �����λ
****************************************************************************/
void LCD_SoftwareReset(void)
{
	LCD_WriteCmd(0xe1);
	DelayMS(10);
}

/****************************************************************************
@ Func:  ���ڲ�����
****************************************************************************/
void LCD_OscillatorOn(void)
{
	LCD_WriteCmd(0xab);
}

/****************************************************************************
@ Func:  �ر��ڲ�����
****************************************************************************/
void LCD_OscillatorOff(void)
{
	LCD_WriteCmd(0xaa);
}

/****************************************************************************
@ Func:  �򿪷���
****************************************************************************/
void LCD_InverseDisplayOn(void)
{
	LCD_WriteCmd(0xa7);
}

/****************************************************************************
@ Func:  �رշ���
****************************************************************************/
void LCD_InverseDisplayOff(void)
{
	LCD_WriteCmd(0xa6);
}

/****************************************************************************
@ Func:  ����DDRAM��ʼ�У����н�ӳ�䵽COM0
****************************************************************************/
void LCD_SetStartLine(unsigned char lineNbr)
{
	LCD_WriteCmd(0xd0);
	LCD_WriteCmd(lineNbr);
}

/****************************************************************************
@ Func:  ����COM0�ڵڼ���Pad���
****************************************************************************/
void LCD_SetCOM0(unsigned char padNbr)
{
	LCD_WriteCmd(0xd3);
	LCD_WriteCmd(padNbr);
}

/****************************************************************************
@ Func:  ������ʾ����
****************************************************************************/
void LCD_SetDisplayLine(unsigned char num)
{
	LCD_WriteCmd(0xdc);
	LCD_WriteCmd(num);
}

/****************************************************************************
@ Func:  ƫ�õ�ѹ����
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
@ Func:  ��ɨ�跽������
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
@ Func:  ��������������ʾģʽ
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
@ Func:  ��������
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
@ Func:  ʹ����ʾ
****************************************************************************/
void LCD_DisplayOn(void)
{
	LCD_WriteCmd(0xa3);	
}

/****************************************************************************
@ Func:  �ر���ʾ
****************************************************************************/
void LCD_DisplayOff(void)
{
	LCD_WriteCmd(0xa2);	
}

/****************************************************************************
@ Func:  ʡ��ģʽ����
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
@ Func:  ���ݷŵ�����
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
@ Func:  ��ɨ�跽������
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
@ Func:  ����дʱ�е�ַ����ģʽ
****************************************************************************/
void LCD_EnterReadModifyWriteMode(void)
{
	LCD_WriteCmd(0xe0);
}

/****************************************************************************
@ Func:  �˳�дʱ�е�ַ����ģʽ
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
@ Func:  ����ģʽ����
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
@ Func:  ����
@ Brief: DDRAM��SEG17��ʼ��ʾ����Ļ�ϣ�����ֻҪдDDRAM��SEG16~SEG143���ɣ���
         ����0��ʼ��
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
@ Func:  ������������
@ Brief: DDRAM��SEG17��ʼ��ʾ����Ļ�ϣ�����ֻҪдDDRAM��SEG16~SEG143���ɣ���
         ����0��ʼ��
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
@ Func:  ��ʾһ��ͼ��
@ Brief: ͼ����������ĸ�����֡����ּ�ͼƬ��
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
	    LCD_SetColumnAddr(16 + j + startColumn);   //16ΪDRAM��Ч�ĳ���
	    LCD_WriteData(*(p + i * segSize + j));
	  }
	}
}

/****************************************************************************
@ Func:  ��ʾһ��ͼ��
@ Brief: ͼ����������ĸ�����֡����ּ�ͼƬ�ȣ����ҿ�������ƽ��
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
  	    LCD_SetColumnAddr(16 + j + startColumn);   //16ΪDRAM��Ч�ĳ���
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
  	    LCD_SetColumnAddr(16 + j + startColumn);   //16ΪDRAM��Ч�ĳ���
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
@ Func:  ���һ��ͼ��
@ Param: segSize     <-> �п�
         comSize     <-> �и�
         startPage   <-> ��ʼҳ
         startColumn <-> ��ʼ��
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
	    LCD_SetColumnAddr(16 + j + startColumn);   //16ΪDRAM��Ч�ĳ���
	    LCD_WriteData(0);
	  }
	}
}

/****************************************************************************
@ Func:  ��ʾ����Logo������汾��
@ Brief: DDRAM��SEG17��ʼ��ʾ����Ļ�ϣ�����ֻҪдDDRAM��SEG16~SEG143���ɣ���
         ����0��ʼ��
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
@ Func:  ����ʾ������ת�٣�����ʾ����
@ Brief: ���޷���ȡ����������ʱ����ʾ���٣�����ʾת�٣��������岻ͬ�����Ե����г�
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
  //�쳣���ݼ�飬��ת�ٳ���3000ʱ��ת����ʾ3000  By������ 20200804
  if(spd > 3000)
     spd = 3000;
  
  
	//��ʾת��xxxx
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
	
	//��ʾת�ٵ�λn/M
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
@ Func:  ��ʾ������ת��
@ Brief: ��ʾ��ʽ<-->xxxx n/M��������ת�����ֵΪ65535/8=8192rpm������LCD����������
         ��ת����ʾ
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
  //�쳣���ݼ�飬��ת�ٳ���3000ʱ��ת����ʾ3000  By������ 20200804
  if(spd > 3000)
     spd = 3000;


	//��ʾת��xxxx
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
	
	//��ʾת�ٵ�λn/M
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
@ Func:  ��ʾ��������
@ Brief: ��ʾ��ʽ<-->xx.x km/h����ЧֵΪ0-60km/h������60����ʾ60
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
	
	//��ʾ����xx.x
	height = 24;
	width = 12;
	page = 4;
	column = 22;
	
	//��ʾ����
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
	  //��ʾС����
	  if(i==1) 
	  {	  
	    LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	    column += 8;
	  }
	}
	
	//��ʾת�ٵ�λkm/h
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
@ Func:  ɳ©��ʾ
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
@ Func:  ��ʾ�ܹ���Сʱ��
@ Brief: ��Page11��ʼ��ʾ��ռ��2��Page����ʾ��ʽΪ��ɳ©���� + XXXXXX.Xh��
***********************************************************************************/
void LCD_ShowTotalWorkTime(unsigned char page, unsigned char column)
{
	unsigned char i = 0;
	unsigned char height = 0;
	unsigned char width = 0;
	unsigned char workingHourDigit = 0;
	
  height = 16;
  width = 9;

  //��ʾɳ©����
	LCD_DisplayHourglass(page, column, LCD_HourglassCtrl.displayState);
	
	column += width;
	column += 5;
	height = 16;
	width = 8;
	//��ʾС����֮ǰ��λ����
	for(i = 0; i < 5; ++i)
	{
    workingHourDigit = (can_data.engineWorkingTime % (unsigned long)pow(10, 6 - i)) / (unsigned long)pow(10, 5 - i);
    LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[workingHourDigit]);
	  
	  column += width;
	  column += 1;
	}
	
	height = 16;
	width = 7;
	//��ʾС����
	LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	
  column += width;
  column += 1;
	
	height = 16;
	width = 8;
	//��ʾС����֮��һλ����
	LCD_DisplayItem(8, 16, page, column, LCD_DigitArray_16Row8Column[can_data.engineWorkingTime % 10]);
  
  column += width;
  column += 1;
  column += 2;
  
  height = 16;
  width = 8;
  //width += 4;
  
  //��ʾ��ĸ���衱
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

  //��ʾɳ©����
	LCD_DisplayHourglassX(page, column, LCD_HourglassCtrl.displayState, shift);
	
	column += width;
	column += 5;
	height = 16;
	width = 8;
	//��ʾС����֮ǰ��λ����
	for(i = 0; i < 5; ++i)
	{
    workingHourDigit = (can_data.engineWorkingTime % (unsigned long)pow(10, 6 - i)) / (unsigned long)pow(10, 5 - i);
    LCD_DisplayItemX(8, 16, page, column, LCD_DigitArray_16Row8Column[workingHourDigit], shift);
	  
	  column += width;
	  column += 1;
	}
	
	height = 16;
	width = 7;
	//��ʾС����
	LCD_DisplayItemX(7, 16, page, column, LCD_CharArray_16Row7Column_Dot, shift);
	
  column += width;
  column += 1;
	
	height = 16;
	width = 8;
	//��ʾС����֮��һλ����
	LCD_DisplayItemX(8, 16, page, column, LCD_DigitArray_16Row8Column[can_data.engineWorkingTime % 10], shift);
  
  column += width;
  column += 1;
  column += 2;
  
  height = 16;
  width = 8;
  //width += 4;
  
  //��ʾ��ĸ���衱
	LCD_DisplayItemX(8, 16, page, column, LCD_CharArray_16Row8Column_h, shift);
}


/***********************************************************************************
@ Func: ��ʾֱ��
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
@ Func:  ��ʾ�ͺ�
@ Brief: ��ʾ��ʽ<-->xxx.xL/h���ͺ���2���ֽڱ�ʾ���ֱ���Ϊ0.05�����ֵΪ65535*0.05
         =3276.75����LCD�����ʾ3λ���������ͺ�ֵ����1000ʱ����ʾ999.9L/h
         ��ȣ�(8+1)+(8+1)+(8+1)+(7+1)+(8+1)+(8)+(8)+(8)=68��column��ռ��0~69�У���
         ʾ����Ϊ2~69�У��������2�����أ�0~1�У���ֹѹ��ס��ʾ����
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
	
	//Ϊ��ʾһλС�����ͺ�ֵ����10��
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
  
  //��ʾ�������	
  LCD_ClearItem(70, 16, page, 0);
  
  //������ʾλ��������ʾ��ʼ��
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
	  
	  //Ԥ��С����λ��
	  if(2 == i)
	  {
	    column += 8;
	  }
	}
	
	column -= 17;
	
	width = 7;
	//��ʾС����
	LCD_DisplayItem(7, 16, page, column, LCD_CharArray_16Row7Column_Dot);
	
	//��ʾ�ͺĵ�λ L/h
  width = 8;	
	column = 46;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg_16Com_L);
	
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_16Row8Column_Slash);
	
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_16Row8Column_h);
}

/***********************************************************************************
@ Func:  ��ʾ�¶�
@ Brief: �¶ȸ�ʽ<--> -xxx��
         ռ�ÿ�� (8+1)+(8+1)+(8+1)+(8+1)+(8*2)=52��ռ��74�е�127�й�54�У��ұ�Ե��
         ��2�����ط�ֹѹ��ס��ʾ�������¶���2���ֽڱ�ʾ���ֱ���Ϊ0.03125��/bit��ƫ
         ��Ϊ-273�棬��ʾ��ΧΪ-273~1775��
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
	
	//�����������ʾ������
	//column = 74;
  //�����������Ѻ������column = 74��������������ѵ����ݻ�������ɾ����ʸ�Ϊ column = 71;
  column = 71;
	LCD_ClearItem(54, 16, page, column);
	
	//���¶�Ϊ��ֵ����ʾ����
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
	
	//���¶�Ϊ����ȡ�����ֵ������ʾ
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
	
  //��ʾ��
	column = 110;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg16Com_Celsius_Left);
	column += width;
	LCD_DisplayItem(8, 16, page, column, LCD_CharArray_8Seg16Com_Celsius_Right);
}

/***********************************************************************************
@ Func: ��ʾ����
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
@ Func: LCD��ʾ"GPRS��xx�����ź�"
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
@ Func: LCD��ʾ"���Ƶ����źŵĵط�"
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
@ Func: LCD��ʾ"ͨ���쳣"
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
@ Func: LCD��ʾ"���ƶ�����ȫ�ش�"
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
@ Func: LCD��ʾ"�������޷�����"
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
@ Func: LCD��ʾ"ͣ�����޷�����"
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
@ Func: LCD��ʾ"������Ť"
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
@ Func: LCD��ʾ"�Ѿ���Ť"
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
@ Func: LCD��ʾ"��������"
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
@ Func: LCD��ʾ"�Ѿ�����"
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
@ Func: LCD��ʾ"���������"
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
@ Func: LCD��ʾ"����������"
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
@ Func: LCD��ʾ"���������"
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
@ Func: LCD��ʾ"����������"
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
@ Func: ��������������ʾ���ͣ���LCD��Ļ����ʾ�������   
***********************************************************************************/
void LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayType_t displayType)
{
  unsigned char clearPage = 0;
  unsigned char writePage = 0;
  if (CarSpeedDisFlag == 0) //�˴������ж������������ѵ���ʾλ��
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
    //��ʾ�ͺĺ��¶ȵ��������
    LCD_ClearItem(128, 16, clearPage, 0);
    //�������������ʾ����
    LCD_ClearItem(128, 16, writePage, 0);
  }
  else
  {
    //�������������ʾ����
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
@ Func: LCD��ʾDTC SPN & FMI
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
  
  //��ʾSPN
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
	
	//��ʾFMI
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
@ Func: LCD��ʾ�������Ǿ�����š����� ����������XXX��
***********************************************************************************/
void LCD_DisplayDTCWarningHeader(unsigned char dtcTotalNum)
{
  unsigned char column = 0;
  unsigned char i = 0;
  unsigned char numDigit = 0;
  bool nonZeroDigitFlag = false;
  
  column = 3;
  //��ʾ���Ǳ�������
  LCD_DisplayItemX(13, 16, 0, column, Image_WarningTriangle_16_13, 2);
  
  column += 13;
  column += 2;
  LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Jing, 3);
  column += 13;
	LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Gao_1, 3);
	LCD_DisplayLine(2, 128);
	
	//��ʾ������ ��������:��
	column = 53;
  LCD_DisplayItemX(13, 16, 0, column, LCD_ChineseCharArray_16Row13Column_Gu, 3);
	LCD_DisplayItemX(13, 16, 0, column + 13, LCD_ChineseCharArray_16Row13Column_Zhang, 3);
  LCD_DisplayItemX(13, 16, 0, column + 13 * 2, LCD_ChineseCharArray_16Row13Column_Zong, 3);
	LCD_DisplayItemX(13, 16, 0, column + 13 * 3, LCD_ChineseCharArray_16Row13Column_Shu, 3);
	LCD_DisplayItemX(7, 16, 0, column + 13 * 4, LCD_CharArray_16Row7Column_Colon, 3);
	
	//��ʾ������Ŀ
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
@ Func: LCD��ѯ��ʾDTC
***********************************************************************************/
void LCD_CANFaultDisplay(void)
{
  unsigned char i = 0;
  unsigned char j = 0;
  unsigned char dtcPosInx = 0;
  unsigned char pageFrameNbr = 0;
  
  //static bool firstDisplayFlag = true;
  
  //���ϴ���ʾ��ҳ�����һλ�ÿ�ʼ����
  for(i = CAN_FaultDisplayCtrl.startPosForSearch; i < (CAN_FAULT_RECORD_MAX_NUM / 3); ++i)
  {
    for(j = 0; j < 3; ++j)
    {
      if(CAN_FaultRecordBuffer[i * 3 + j].can_spn)
      {
        CAN_FaultDisplayCtrl.dtcPos[dtcPosInx++] = i;  //��¼ҳ����ţ�������1��ʼ��
        CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_spn = CAN_FaultRecordBuffer[i * 3 + j].can_spn;
        CAN_FaultDisplayCtrl.currFaultInfor[dtcPosInx - 1].can_fmi = CAN_FaultRecordBuffer[i * 3 + j].can_fmi;
      }
    }
    
    //ҳ������DTC��ֹͣ����
    if(dtcPosInx >= 1)
    {
      break;
    }
  }
  
  //��������������ĩβ��δ����DTC�����ͷ��ʼ������ֱ�����ο�ʼ����λ�õ�ǰһλ��
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
  
  //��DTC��Ŀ��3����δ�õ��Ļ���������
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
  
  //��¼��һ������DTC��ʼλ��
  CAN_FaultDisplayCtrl.startPosForSearch = CAN_FaultDisplayCtrl.dtcPos[dtcPosInx - 1] + 1;
  if((CAN_FAULT_RECORD_MAX_NUM / 3) == CAN_FaultDisplayCtrl.startPosForSearch)
  {
    CAN_FaultDisplayCtrl.startPosForSearch = 0;
  }
  
  //��ʾ�������Ǿ�����š�����   ����������xxx��
  CAN_FaultDisplayCtrl.totalNum = CAN_FaultNumStats();
  if((CAN_FaultDisplayCtrl.oldTotalNum != CAN_FaultDisplayCtrl.totalNum) || CAN_FaultDisplayCtrl.firstDisplayFlag/*firstDisplayFlag*/
     || CAN_FaultDisplayCtrl.refreshFlag) //��ʹû��DTCҲ��Ҫ��ʾ���棬���Ե�һ��ִ�иú�����Ҫˢ�¾�����ʾ
  {
    CAN_FaultDisplayCtrl.refreshFlag = false;
    
    //��DTC����ʱ��Ҫˢ��NCD��Сʱ����ʾ
    if(0 == CAN_FaultDisplayCtrl.oldTotalNum)
    {
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = true;
      hourmeterNeedShowFlag = true;
    } 
    
    CAN_FaultDisplayCtrl.oldTotalNum = CAN_FaultDisplayCtrl.totalNum;
    
    //���2��Page����Ϊ���������Ǳ仯�ģ�������Ҫ��������д
    LCD_ClearItem(128, 16, 0, 0);
    LCD_DisplayDTCWarningHeader(CAN_FaultDisplayCtrl.totalNum);    
  }
    
  //��ʾ����DTC
  //�ȽϺ��ϴ���ʾ�����Ƿ��б仯
  for(i = 0; i < 3; ++i)
  {
    if((CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn != CAN_FaultDisplayCtrl.oldFaultInfor[i].can_spn) 
       || (CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi != CAN_FaultDisplayCtrl.oldFaultInfor[i].can_fmi))
    {
      break;
    }
  }
  
  //Ҫ��ʾ��DTC�б仯
  if(3 != i)
  {
    //��¼������ʾ����
    for(i = 0; i < 3; ++i)
    {
      CAN_FaultDisplayCtrl.oldFaultInfor[i].can_spn = CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn;
      CAN_FaultDisplayCtrl.oldFaultInfor[i].can_fmi = CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi;
    }
    
    //�����ʾ����
    for(i = 0; i < 3; ++i)
    {
      LCD_ClearItem(128, 16, 3 * i + 3, 0);
    }
    
    //����Ӧ������ʾDTC
    for(i = 0; i < dtcPosInx; ++i)
    {
      if(CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn)
      {
        LCD_DisplaySpnFmi(CAN_FaultDisplayCtrl.currFaultInfor[i].can_spn, CAN_FaultDisplayCtrl.currFaultInfor[i].can_fmi, 3 * i + 3);
      }
    }
  }
  
  //NCD��ʾ�@ʾ
  if(LCD_NCD_WARNING_NONE != LCD_NCDWarningDisplayCtrl.displayType)
  {
    if((LCD_NCDWarningDisplayCtrl.displayTypeOld != LCD_NCDWarningDisplayCtrl.displayType) || LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag)
    {
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = false;
            
      LCD_NCDWarningDisplayCtrl.displayTypeOld = LCD_NCDWarningDisplayCtrl.displayType;
      
      //Add newly.
      LCD_ClearItem(128, 16, 0, 0);
      LCD_DisplayDTCWarningHeader(CAN_FaultDisplayCtrl.totalNum);
      
      //���oDTC���ϣ��t����Ļ���g�@ʾNCD��ʾ����Ļ�·��@ʾС�rӋ
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
      //������DTC���tLCD�·��@ʾNCD��ʾ��С�rӋ���@ʾ
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
  
  //Сʱ����ʾ����������1����NCD���棻2����NCD���浫��DTC
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
@ Func: �Д�NCD�{�ϵ�y�����������
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
@ Func: �ж�����������ʾ����
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
  else //�˴�Ϊ��񷢶������ͣ���񷢶��������Ƚ��٣��ʷŵ�����
  {
    if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_IN_PROCESS)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_TAKE_REGENERATE;
    else if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_PARKING_PROMPT)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_SERVER_REGENERATE;
    else if (CAN_PostprocessingState_YC.regenerateRemind == REGENERATE_STATE_SERVICE_PROMPT)
      LCD_REGENERATE_DisplayCtrl.displayType = LCD_NEED_RIGHT_REGENERATE;
    //�ж�����������ʾ
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
@ Func: LCD��ʾ����
***********************************************************************************/
void LCD_Task(void)
{
  unsigned char i = 0;
  
  //������ʾ��������
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
  
  //����GPS��������λ������ʾ���Ʊ���
  if(!CAN_GPSWarningBitsMap)
  {
    LCD_GPSWarningDisplayCtrl.displayTypeOld = LCD_GPSWarningDisplayCtrl.displayType = LCD_GPS_WARNING_NONE;
  }
  
  //����CAN DTC��λCAN DTC��ʾ���Ʊ���
  if(!CAN_FaultBitMap)
  {
    if(CAN_FaultDisplayCtrl.oldTotalNum)
    {
      memset(&CAN_FaultDisplayCtrl, 0, sizeof(CAN_FaultDisplayCtrl_t));
      CAN_FaultDisplayCtrl.firstDisplayFlag = true;
      
      //���Ͼ�����Ŀ��Ҫ���£�NCD��ʾλ�ÿ�����Ҫˢ�£�������NCD����Сʱ����Ҫˢ�£�����NCDʱ��δ��ʾ�ģ�
      CAN_FaultDisplayCtrl.refreshFlag = true;
      LCD_NCDWarningDisplayCtrl.ncdChangeDisplayAreaFlag = true;
      hourmeterNeedShowFlag = true;
    }
  }
  
  //��NCD���ϣ���λNCD������ʾ���Ʊ���
  if((((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x00) || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x01)
     || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x06) || (((CAN_PostprocessingState.ncdState & 0x38) >> 3) == 0x07))
  {
    //��NCD��ʧʱ��Ҫˢ��Сʱ��
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
  
  //�ж��Ƿ�����ɳ©��˸
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

  //��λLCD����������ʾ����
  if (LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
  {
    if (LCD_REGENERATE_DisplayCtrl.displayTypeOld != LCD_NO_REMIND)
    {
      LCD_REGENERATE_DisplayCtrl.displayTypeOld = LCD_NO_REMIND;
      Refresh_fuelConsumption_and_ambientAirTemp = TRUE;
    }
  }

  //������ʾ����������ʾ��Ӧ����
  switch(LCD_DisplayCtrl.LCD_DisplayType)
  {
    //����Logo��ʾ
    case LCD_DISPLAY_LOGO:
      if(LCD_DISPLAY_LOGO != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_LOGO;
        LCD_ClearScreen();
        LCD_DisplaySDLGLogo();
      }
    break;
    
    //��������������ʾ
    case LCD_DISPLAY_NORMAL:
      if(LCD_DISPLAY_NORMAL != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_NORMAL; 
        LCD_ClearScreen();
        
        //���ٺ�ת����ʾ�ڵ�һ����ʾʱ��ʼ��
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
      
      //������ת��
      if(LCD_NormalDisplayCtrl.engineSpeedRefreshFlag && (LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout > 1000))
      {
        LCD_NormalDisplayCtrl.engineSpeedRefreshFlag = false;
        LCD_NormalDisplayCtrl.engineSpeedRefreshTimeout = 0;
        if(CarSpeedDisFlag == 0)
          LCD_DisplayEngineSpeedOnly(engineSpeedParams.old_speed);
        else
          LCD_DisplayEngineSpeed(engineSpeedParams.old_speed);
      }
      
      //ֻ�г��ٲ�Ϊ0ʱ�Ž�����ʾ��û�г����ź�ʱ����ʾ
      if( CarSpeedDisFlag == 1)
        LCD_DisplayVehicleSpeed(car_speed_parameter.loader_speed);

      //����������ʱ����LCD��Ļ����ʾ�������ݣ����û��������������ʾ�ͺĺ��¶�  
      if(LCD_REGENERATE_DisplayCtrl.displayType == LCD_NO_REMIND)
      {
        //����ͺġ��¶��Լ�������������
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
        //ȼ������
        if((((can_data.fuelConsumption - can_data.fuelConsumptionOld) > 1) || (can_data.fuelConsumptionOld > (1 + can_data.fuelConsumption)))
          && (LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout > 1000)||(Refresh_fuelConsumption_and_ambientAirTemp == TRUE))
        {
          LCD_NormalDisplayCtrl.engineFuelRateRefreshTimeout = 0;
          can_data.fuelConsumptionOld = can_data.fuelConsumption;
          LCD_DisplayOilConsumption(can_data.fuelConsumption);
        }
        
        //�����¶�
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
      //����ʱ��
      if(can_data.engineWorkingTime != can_data.engineWorkingTimeOld)
      {
        can_data.engineWorkingTimeOld = can_data.engineWorkingTime;
        LCD_ShowTotalWorkTime(11, LCD_TIME_SHOW_START_COLUMN);
      }
      
      //ɳ©��˸����
      if(LCD_HourglassCtrl.blinkEnable)
      {
        LCD_DisplayHourglass(11, LCD_TIME_SHOW_START_COLUMN, LCD_HourglassCtrl.displayState);
      }
      else
      {
        LCD_DisplayHourglass(11, LCD_TIME_SHOW_START_COLUMN, LCD_GLASSHOUR_SHOW);
      }
    break;
        
    //���Ͻ�����ʾ
    case LCD_DISPLAY_FAULT:
      if(LCD_DisplayCtrl.LCD_DisplayTypeOld != LCD_DisplayCtrl.LCD_DisplayType)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DisplayCtrl.LCD_DisplayType;
        LCD_NCDWarningDisplayCtrl.displayTypeOld = LCD_NCD_WARNING_NONE;  //�������NCD���ϵĻ�������ˢ�³�����
        CAN_FaultDisplayCtrl.firstDisplayFlag = true;  //Ŀ���ǽ����л��󣬿���ˢ�� �������������������Լ�Сʱ��
        CAN_FaultDisplayCtrl.refreshTimeout = 8;       //��ֹ�����л����������ˢ��
        LCD_ClearScreen();
        LCD_CANFaultDisplay();
        //LCD_ShowTotalWorkTimeX(12, LCD_TIME_SHOW_START_COLUMN, -3);
      }
      
      //��NCD��ʻ�������Ƹı�ʱ������ˢ�½���
      if(LCD_NCDWarningDisplayCtrl.displayType != LCD_NCDWarningDisplayCtrl.displayTypeOld)
      {
        CAN_FaultDisplayCtrl.refreshTimeout = 0;
      }
      
      //��ʱˢ�½��棬Լ5��
      if(0 == CAN_FaultDisplayCtrl.refreshTimeout)
      {
        CAN_FaultDisplayCtrl.refreshTimeout = 8;
        
        LCD_CANFaultDisplay();
      }
    break;
    
    //GPS������ʾ
    case LCD_DISPLAY_GPS_WARNING:
      if(LCD_DISPLAY_GPS_WARNING != LCD_DisplayCtrl.LCD_DisplayTypeOld)
      {
        LCD_DisplayCtrl.LCD_DisplayTypeOld = LCD_DISPLAY_GPS_WARNING;
      }
      
      //�жϾ�������
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
      
      //����GPS����������ʾ��Ӧ����
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