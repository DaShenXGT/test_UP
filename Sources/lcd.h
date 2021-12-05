#ifndef _LCD_H
#define _LCD_H

#include "can.h"
#include "vehicle.h"

//通信接口选择
//#define MPU_6800 1
#define MPU_8080 1

#define LCD_WRITE_DELAY_MS 1

#define LCD_BACK_LIGHT_OPEN() (PORTB |= (1 << 7))
#define LCD_BACK_LIGHT_CLOSE() (PORTB &= ~(1 << 7))

#define LCD_CMD() (PORTB &= ~(1 << 3))
#define LCD_DATA() (PORTB |= (1 << 3))

#define LCD_EDR_SET() (PORTB |= (1 << 2))
#define LCD_EDR_CLR() (PORTB &= ~(1 << 2))

#define LCD_RWR_SET() (PORTB |= (1 << 1))
#define LCD_RWR_CLR() (PORTB &= ~(1 << 1))

#define LCD_RST_SET() (PORTB |= (1 << 4))
#define LCD_RST_CLR() (PORTB &= ~(1 << 4))

#define LCD_CS1_SET() (PORTB |= (1 << 6))
#define LCD_CS1_CLR() (PORTB &= ~(1 << 6))

#define LCD_CS2_SET() (PORTB |= (1 << 5))
#define LCD_CS2_CLR() (PORTB &= ~(1 << 5))

#define LCD_TASK_PERIOD 100
#define LCD_HOURGLASS_BLINK_PERIOD 1250

//LCD Vop产生方式
typedef enum
{
	LCD_POWER_ALL_INTERNAL,
	LCD_POWER_EXTERNAL_V3,
	LCD_POWER_EXTERNAL_V3_V1,
	LCD_POWER_ALL_EXTERNAL,
	
} LCD_VopGenerateMethod_t;

typedef enum
{
	LCD_BOOSTER_4_VDD2,
	LCD_BOOSTER_6_VDD2,
	LCD_BOOSTER_8_VDD2,

} LCD_BoosterSelect_t;

typedef enum
{
	LCD_BIAS_ONE_THIRD,
	LCD_BIAS_ONE_FORTH,
	LCD_BIAS_ONE_FIFTH,
	LCD_BIAS_ONE_SIXTH,
	LCD_BIAS_ONE_SEVENTH,

} LCD_BiasSelect_t;

typedef enum
{
	LCD_SEG_DIRECTION_NORMAL,
	LCD_SEG_DIRECTION_REVERSE,

} LCD_SegDirection_t;

typedef enum
{
	LCD_ALL_PIXEL_NORMAL,
	LCD_ALL_PIXEL_ON,

} LCD_AllPixelDisplayMode_t;

typedef enum
{
	LCD_INVERSE_DISPLAY_NORMAL,
	LCD_INVERSE_DISPLAY_INVERSE,

} LCD_InverseDisplayMode_t;

typedef enum
{
	LCD_POWER_SAVE_NORMAL,
	LCD_POWER_SAVE_STADBY,

} LCD_PowerSaveMode_t;

typedef enum
{
	LCD_COM_DIRECTION_NORMAL,
	LCD_COM_DIRECTION_INVERSE,

} LCD_ComDirection_t;

typedef enum
{
	LCD_DRIVE_MODE_DMP_ON,	 	//dummy period ON	
	LCD_DRIVE_MODE_DMP_OFF,		//dummy period OFF

} LCD_DriveModeDMPSelect_t;

typedef enum
{
	LCD_DRIVE_MODE_NFLG_NORMAL,
	LCD_DRIVE_MODE_NFLG_RESET_AFTER_SCANNING,

} LCD_DriveModeNFLGSelect_t;

typedef enum
{
	LCD_TC_SENSOR_SPEED_NORMAL,
	LCD_TC_SENSOR_SPEED_FAST,

} LCD_TCSensorSpeedMode_t;


#define LCD_TIME_SHOW_START_COLUMN 21

//LCD界面刷新控制 
//LCD显示界面类型
typedef enum _LCD_DisplayType
{
  LCD_DISPLAY_LOGO,
  LCD_DISPLAY_NORMAL,
  LCD_DISPLAY_FAULT,
  LCD_DISPLAY_GPS_WARNING,
  LCD_DISPLAY_DRIVE_SYSTEM_LIMIT,
  
} LCD_DisplayType_t;

typedef struct _LCD_DisplayCtrl
{
  LCD_DisplayType_t LCD_DisplayType;
  LCD_DisplayType_t LCD_DisplayTypeOld;
  bool displayTypeChangeFlag;
  unsigned int refreshTimeout;
  bool refreshFlag;
  unsigned long taskPeriod;
  
} LCD_DisplayCtrl_t;

//LCD正常工作界面显示控制
typedef struct _LCD_NormalDisplayCtrl
{
  bool engineSpeedRefreshFlag;
  unsigned long engineSpeedRefreshTimeout;
  bool engineFuelRateRefreshFlag;
  unsigned long engineFuelRateRefreshTimeout;
  bool ambientTempRefreshFlag;
  bool hourmeterRefreshFlag;
  
} LCD_NormalDisplayCtrl_t;

//沙漏显示控制
typedef enum
{
  LCD_GLASSHOUR_SHOW,
  LCD_GLASSHOUR_HIDDEN,
  
} LCD_HourglassState_t;

typedef struct _LCD_HourMeterDisplayCtrl
{
  bool blinkEnable;
  unsigned long blinkTimeout;
  bool refreshFlag;
  unsigned char displayState;
  
} LCD_HourglassCtrl_t;

//GPS报警显示控制
typedef enum _LCD_GPSWarningDisplayType
{
  LCD_GPS_WARNING_NONE,
  LCD_GPS_WARNING_51_DAYS_NO_SIGNAL,
  LCD_GPS_WARNING_58_DAYS_NO_SIGNAL,
  LCD_GPS_WARNING_61_DAYS_NO_SIGNAL,
  LCD_GPS_WARNING_WEB_LOCK,
  LCD_GPS_WARNING_COMM_ERR_GPS_WORK_ABNORMAL,
  LCD_GPS_WARNING_COMM_ERR_UNLOCKED,
  LCD_GPS_WARNING_COMM_ERR_LOCKED,

} LCD_GPSWarningDisplayType_t;

typedef struct _LCD_GPSWarningDisplayCtrl
{
  LCD_GPSWarningDisplayType_t displayType;
  LCD_GPSWarningDisplayType_t displayTypeOld;
  Engine_State_t oldEngineState;
  
} LCD_GPSWarningDisplayCtrl_t;

//NCD报警显示控制
typedef enum _LCD_NCDDisplayType
{
  LCD_NCD_WARNING_NONE,
  LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SLIGHT,
  LCD_NCD_WARNING_TORQUE_LIMITED_SLIGHT,
  LCD_NCD_WARNING_WILL_LIMIT_TORQUE_SEVERE,
  LCD_NCD_WARNING_TORQUE_LIMITED_SEVERE,
  
} LCD_NCDWarningDisplayType_t;

typedef struct _LCD_NCDDisplayCtrl
{
  LCD_NCDWarningDisplayType_t displayType;
  LCD_NCDWarningDisplayType_t displayTypeOld;
  
  bool ncdChangeDisplayAreaFlag;
  
} LCD_NCDWarningDisplayCtrl_t;

extern bool ncdChangeDisplayAreaFlag;
extern bool hourmeterNeedShowFlag;
extern bool Refresh_fuelConsumption_and_ambientAirTemp;


//再生提醒显示类型               //by_徐光同_2021/7/29
typedef enum _LCD_REGENERATE_DisplayType
{
  LCD_NO_REMIND,               //无需显示再生提醒
  LCD_NEED_TAKE_REGENERATE,    //需进行再生
  LCD_NEED_RIGHT_REGENERATE,   //需立即再生
  LCD_NEED_SERVER_REGENERATE,  //需服务再生
  LCD_TAKEING_REGENERATE,      //再生进行中
} LCD_REGENERATE_DisplayType_t;

typedef struct _LCD_REGENERATE_DisplayCtrl
{
  LCD_REGENERATE_DisplayType_t displayType;
  LCD_REGENERATE_DisplayType_t displayTypeOld;
//  bool  Position_Flag;
} LCD_REGENERATE_DisplayCtrl_t;

//CAN DTC显示
//CAN故障显示切换时间
#define CAN_DISPLAY_REFRESH_TIMEOUT 250

//CAN故障切换控制结构体
typedef struct __CAN_FaultDisplayCtrl_t
{
	unsigned char totalNum;
	unsigned char oldTotalNum;
	
	unsigned char startPosForSearch;
	
	unsigned int refreshTimeout;
	bool refreshFlag;
	
	bool firstDisplayFlag;
	
	unsigned char dtcPos[3];
	CAN_Fault_t currFaultInfor[3];
	CAN_Fault_t oldFaultInfor[3];
	
	
} CAN_FaultDisplayCtrl_t;


void LCD_GPIOInit(void);
void LCD_Config(void);

void LCD_InterfaceEnable(void);
void LCD_InterfaceDisable(void);
void LCD_HardwareReset(void);

void LCD_WriteData(unsigned char data);
void LCD_WriteCmd(unsigned char cmd);

void LCD_SetPageAddr(unsigned char page);
void LCD_SetColumnAddr(unsigned char column);
void LCD_PowerControl(LCD_VopGenerateMethod_t method);
void LCD_OscillatorOn(void);
void LCD_OscillatorOff(void);
void LCD_SetStartLine(unsigned char lineNbr);
void LCD_SetCOM0(unsigned char padNbr);
void LCD_SetDisplayLine(unsigned char num);
void LCD_BiasSelect(LCD_BiasSelect_t bias);
void LCD_SetBooster(LCD_BoosterSelect_t booster);
void LCD_SetVop(unsigned char vop);
void LCD_SetSegDirection(LCD_SegDirection_t direction);
void LCD_SetAllPixelDisplay(LCD_AllPixelDisplayMode_t mode);
void LCD_InverseDisplay(LCD_InverseDisplayMode_t inverse);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_SetPowerSave(LCD_PowerSaveMode_t mode);
void LCD_DischargeOn(void);
void LCD_DischargeOff(void);
void LCD_SetComDirection(LCD_ComDirection_t direction);
void LCD_EnterReadModifyWriteMode(void);
void LCD_ExitReadModifyWriteMode(void);
void LCD_SoftwareReset(void);
void LCD_SetFrameRate(unsigned char fa, unsigned char fb, unsigned char fc, unsigned char fd);
void LCD_SetNLine(unsigned char lineNum);
void LCD_SetDriveMode(LCD_DriveModeDMPSelect_t dummy, LCD_DriveModeNFLGSelect_t nflg);
void LCD_TCOn(void);
void LCD_TCOff(void);
void LCD_SetTCCurve(unsigned char mt0, unsigned char mt1, unsigned char mt2, unsigned char mt3, unsigned char mt4,
                    unsigned char mt5, unsigned char mt6, unsigned char mt7, unsigned char mt8, unsigned char mt9,
					          unsigned char mta, unsigned char mtb, unsigned char mtc, unsigned char mtd, unsigned char mte,
					          unsigned char mtf);
void LCD_SetTCFlag(unsigned char fmt0To7, unsigned char fmt8ToF);
void LCD_SetTempAToC(unsigned char ta, unsigned char tb, unsigned char tc);
void LCD_SetTCSensorSpeed(LCD_TCSensorSpeedMode_t mode);
void LCD_ClearScreen(void);

void LCD_InverseDisplayOn(void);
void LCD_InverseDisplayOff(void);

void LCD_DisplayItem(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn, const unsigned char *p);
void LCD_DisplayItemX(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn, const unsigned char *p,
                      char shift);
void LCD_ClearItem(unsigned char segSize, unsigned char comSize, unsigned char startPage, unsigned char startColumn);

void LCD_DisplaySDLGLogo(void);
void LCD_DisplayAlarmImage(void);
void LCD_DisplayEngineSpeed(unsigned int speed);
void LCD_ShowTotalWorkTime(unsigned char page, unsigned char column);
void LCD_ShowTotalWorkTimeX(unsigned char page, unsigned char column, int shift);
void LCD_DisplayHourglass(unsigned char page, unsigned char column, LCD_HourglassState_t state);
void LCD_DisplayHourglassX(unsigned char page, unsigned char column, LCD_HourglassState_t state, int shift);
void LCD_DisplayLine(unsigned char page, unsigned char len);
void LCD_DisplayOilConsumption(void);
void LCD_DisplayTemp(double temperature);
void LCD_DisplayWarningHeader(void);
void LCD_DisplayGPRSNoSignalPrompt(unsigned char days);
void LCD_DisplayMoveToGoodSignalPlacePrompt(void);
void LCD_DisplayCommErrorPrompt(void);
void LCD_DisplayMoveToSafePlacePrompt(void);
void LCD_DisplayCanNotStartPrompt(void);
void LCD_DisplayCanNotStartOnceStopPrompt(void);
void LCD_DisplayWillLimitTorquePrompt(unsigned char startPage, unsigned char startColumn);
void LCD_DisplayTorqueLimitedPrompt(unsigned char startPage, unsigned char startColumn);
void LCD_DisplayWillLimitSpeedPrompt(unsigned char startPage, unsigned char startColumn);
void LCD_DisplaySpeedLimitedPrompt(unsigned char startPage, unsigned char startColumn);
void LCD_DisplayDTCWarningHeader(unsigned char dtcTotalNum);

void LCD_Display_NEED_TAKE_REGENERATE(unsigned char startPage, unsigned char startColumn);
void LCD_Display_NEED_RIGHT_REGENERATE(unsigned char startPage, unsigned char startColumn);
void LCD_Display_NEED_SERVER_REGENERATE(unsigned char startPage, unsigned char startColumn);
void LCD_Display_TAKEING_REGENERATE(unsigned char startPage, unsigned char startColumn);
void LCD_Display_REGENERATE_DESCRIPTION(LCD_REGENERATE_DisplayType_t displayType);
void LCD_DisplaySDLGLogo(void);
void LCD_CANFaultDisplay(void);
void LCD_DisplaySpnFmi(unsigned long spn, unsigned char fmi, unsigned char page);
void LCD_Task(void);

extern Uint16_t  CAN_GPSWarningBitsMap;
extern Uint16_t  CAN_GPSWarningBitsMapOld;
extern LCD_DisplayCtrl_t LCD_DisplayCtrl;
extern LCD_NormalDisplayCtrl_t LCD_NormalDisplayCtrl;
extern LCD_HourglassCtrl_t LCD_HourglassCtrl;
extern CAN_FaultDisplayCtrl_t CAN_FaultDisplayCtrl; 
extern LCD_REGENERATE_DisplayCtrl_t LCD_REGENERATE_DisplayCtrl;

#endif
